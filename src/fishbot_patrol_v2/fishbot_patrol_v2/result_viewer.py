#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡检结果看板：把一次巡检的每个点位结果实时打在终端上。

它只订阅、不参与控制，随时可以开、随时可以关，不影响巡检本身。

数据来源（全部是现成话题，不改任何别的节点）：
  /patrol/execution/status   std_msgs/String   JSON：state/message/point_id/
                                               current_index/current_loop/time
  /patrol/task/result        PatrolTaskResult  任务管理器给出的点位终态
  /patrol/task/external_result PatrolTaskResult 视觉的原始回执（可选显示）

输出：
  终端实时表格（tty 时原地刷新；重定向时逐行输出）
  ~/patrol_viewer/patrol_<execution_id>.jsonl   一行一个事件，便于事后分析
  ~/patrol_viewer/patrol_<execution_id>.csv     表格版，Excel 可直接打开

用法：
  ros2 run fishbot_patrol_v2 patrol_viewer
  ros2 run fishbot_patrol_v2 patrol_viewer --ros-args -p show_external:=true
  ros2 run fishbot_patrol_v2 patrol_viewer --ros-args -p live:=false | tee run.log
"""

from __future__ import annotations

import csv
import json
import os
import sys
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Float32, String

from fishbot_patrol_interfaces.msg import PatrolTaskResult

# PatrolTaskResult.status 的取值（见 interfaces/msg/PatrolTaskResult.msg）
STATUS_TEXT = {
    PatrolTaskResult.UNKNOWN: ('未知', ''),
    PatrolTaskResult.RUNNING: ('进行中', '\x1b[36m'),
    PatrolTaskResult.SUCCEEDED: ('成功', '\x1b[32m'),
    PatrolTaskResult.FAILED: ('失败', '\x1b[31m'),
    PatrolTaskResult.CANCELED: ('取消', '\x1b[33m'),
}

RESET = '\x1b[0m'
DIM = '\x1b[2m'
BOLD = '\x1b[1m'
CLEAR = '\x1b[H\x1b[2J'
GREEN = '\x1b[32m'
YELLOW = '\x1b[33m'
RED = '\x1b[31m'


def _fmt_clock(epoch: Optional[float]) -> str:
    if not epoch:
        return '--:--:--'
    return time.strftime('%H:%M:%S', time.localtime(epoch))


def _clip(text: str, width: int) -> str:
    """按显示宽度裁剪（中文按 2 列算），超长用 … 结尾。"""
    if width <= 0:
        return ''
    out = []
    used = 0
    for char in text:
        width_char = 2 if ord(char) > 0x2E80 else 1
        if used + width_char > width - 1:
            out.append('…')
            break
        out.append(char)
        used += width_char
    return ''.join(out)


def _pad(text: str, width: int) -> str:
    """按显示宽度补空格（中文占 2 列）。"""
    used = sum(2 if ord(c) > 0x2E80 else 1 for c in text)
    return text + ' ' * max(0, width - used)


class ResultViewer(Node):
    def __init__(self) -> None:
        super().__init__('patrol_viewer')

        self.declare_parameter('status_topic', '/patrol/execution/status')
        self.declare_parameter('task_result_topic', '/patrol/task/result')
        self.declare_parameter(
            'external_result_topic', '/patrol/task/external_result')
        self.declare_parameter('out_dir', '~/patrol_viewer')
        self.declare_parameter('live', True)
        self.declare_parameter('show_external', False)
        # 电池电压：ESP32 通过 micro-ROS 直接发 /battery/voltage（std_msgs/Float32）
        self.declare_parameter('battery_topic', '/battery/voltage')
        self.declare_parameter('battery_warn_v', 11.8)
        self.declare_parameter('battery_crit_v', 11.5)
        self.declare_parameter('battery_stale_sec', 10.0)

        self._status_topic = str(
            self.get_parameter('status_topic').value)
        self._task_result_topic = str(
            self.get_parameter('task_result_topic').value)
        self._external_topic = str(
            self.get_parameter('external_result_topic').value)
        self._out_dir = os.path.expanduser(
            str(self.get_parameter('out_dir').value))
        self._live = bool(self.get_parameter('live').value) and \
            sys.stdout.isatty()
        self._show_external = bool(
            self.get_parameter('show_external').value)
        self._battery_topic = str(self.get_parameter('battery_topic').value)
        self._battery_warn = float(self.get_parameter('battery_warn_v').value)
        self._battery_crit = float(self.get_parameter('battery_crit_v').value)
        self._battery_stale = float(
            self.get_parameter('battery_stale_sec').value)

        # 状态话题和任务结果都用 latched QoS（与发布端一致），
        # 否则后启动的看板收不到「当前状态」这条历史消息。
        latched = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            String, self._status_topic, self._on_status, latched)
        self.create_subscription(
            PatrolTaskResult, self._task_result_topic,
            self._on_task_result, latched)
        # 视觉的 external_result 是**VOLATILE** 发布的（yolo_detector.py:183-191
        # 只设了 RELIABLE），订阅端如果要求 TRANSIENT_LOCAL 就是 DURABILITY
        # 不兼容，rmw 会打 "requesting incompatible QoS ... No messages will be
        # sent to it"，然后一条都收不到。这里必须跟发布端一致。
        self.create_subscription(
            PatrolTaskResult, self._external_topic,
            self._on_external_result,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        # 电压走 BEST_EFFORT：丢一两帧无所谓，重发反而让 1 Hz 的显示滞后
        self.create_subscription(
            Float32, self._battery_topic, self._on_battery,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

        # ---- 内部状态 ----
        self._execution_id = ''
        self._route_name = ''
        self._state = 'IDLE'
        self._status_message = 'waiting for a patrol to start'
        self._current_index = -1
        self._current_loop = 0
        self._completed_points = 0
        self._point_started: Dict[Tuple[int, int], float] = {}
        # (loop, index) -> 行数据
        self._rows: Dict[Tuple[int, int], dict] = {}
        self._row_order: list = []
        self._last_event = ''
        self._last_event_time = 0.0
        self._events = 0
        self._jsonl = None
        self._jsonl_path = ''
        self._started_at = time.time()
        # 电池：None 表示还没收到过任何一帧
        self._battery_v: Optional[float] = None
        self._battery_at = 0.0
        self._battery_level = 0

        self._render()
        self.get_logger().info(
            'patrol_viewer started: status=%s, result=%s, external=%s, '
            'live=%s, out=%s'
            % (self._status_topic, self._task_result_topic,
               self._external_topic, self._live, self._out_dir))

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning(
                'status is not JSON, ignored: %r' % msg.data[:80])
            return

        execution_id = str(payload.get('execution_id', ''))
        if execution_id and execution_id != self._execution_id:
            self._begin_run(execution_id, str(payload.get('route_name', '')))

        self._state = str(payload.get('state', self._state))
        self._status_message = str(payload.get('message', ''))
        self._current_index = int(payload.get('current_index', -1))
        new_loop = int(payload.get('current_loop', 0))
        if new_loop != self._current_loop:
            self._finalize_stale_rows(new_loop)
            self._current_loop = new_loop
        self._completed_points = int(payload.get('completed_points', 0))
        point_id = str(payload.get('point_id', ''))

        # 进入某个点位的导航阶段 = 这个点位的计时起点。
        # 用本地 monotonic 而不是消息里的 time：执行器和看板可能不在同一台
        # 机器上、时钟也未必同步，跨机算差值会得到负数或几十秒的假耗时。
        if self._state in ('NAVIGATING', 'WAITING_TASK') and point_id:
            key = (self._current_loop, self._current_index)
            self._point_started.setdefault(key, time.monotonic())
            row = self._row(key, self._current_index, point_id)
            row.setdefault('point_id', point_id)
            if row.get('status') == PatrolTaskResult.RUNNING:
                row['phase'] = '等视觉' if self._state == 'WAITING_TASK' \
                    else '导航中'

        self._render()

    def _on_task_result(self, msg: PatrolTaskResult) -> None:
        """任务管理器给出的点位终态：表格里用它作为最终结论。"""
        self._accept_result(msg, source='task')

    def _on_external_result(self, msg: PatrolTaskResult) -> None:
        """视觉的原始回执：默认只在事件流里提一句，避免和终态重复。"""
        if self._show_external or msg.status == PatrolTaskResult.FAILED:
            label, color = STATUS_TEXT.get(msg.status, ('未知', ''))
            self._note('视觉回执 %s %s%s%s：%s'
                       % (msg.point_id, color, label, RESET, msg.message))
        self._accept_result(msg, source='vision', render=self._show_external)

    # ------------------------------------------------------------------
    # 电池电压：只显示，不参与调度
    # ------------------------------------------------------------------
    def _on_battery(self, msg: Float32) -> None:
        self._battery_v = float(msg.data)
        self._battery_at = time.time()

        level = 0
        if self._battery_v <= self._battery_crit:
            level = 2
        elif self._battery_v <= self._battery_warn:
            level = 1
        # 只在跨档时记一条事件，1 Hz 的电压不该把事件流刷满
        if level != self._battery_level:
            if level == 2:
                self._note('电池严重亏电 %.2fV（< %.2fV），建议立即回充'
                           % (self._battery_v, self._battery_crit))
            elif level == 1:
                self._note('电池偏低 %.2fV（< %.2fV），建议本轮结束后回充'
                           % (self._battery_v, self._battery_warn))
            elif self._battery_level != 0:
                self._note('电池电压恢复 %.2fV' % self._battery_v)
            self._battery_level = level
        self._render()

    def _accept_result(self, msg: PatrolTaskResult,
                       source: str, render: bool = True) -> None:
        # key = (轮次, 点位序号)。**优先挂到该点位最近一条还没出结果的行**：
        # 执行器的 current_loop 在“本轮最后一条结果发出之后”才加一，如果直接
        # 用当前轮次做 key，收尾那条结果会被记到下一轮，上一轮永远缺一个点。
        loop = self._current_loop
        key = None
        for candidate in reversed(self._row_order):
            row = self._rows[candidate]
            if (candidate[1] == int(msg.point_index)
                    and row.get('status') == PatrolTaskResult.RUNNING):
                key = candidate
                break
        if key is None:
            key = (loop, int(msg.point_index))
        row = self._row(key, int(msg.point_index), msg.point_id)
        row['point_id'] = msg.point_id or row.get('point_id', '')
        row['status'] = int(msg.status)
        row['message'] = msg.message
        row['source'] = source
        row['finished_at'] = time.time()

        started = self._point_started.get(key)
        if started:
            row['seconds'] = max(0.0, time.monotonic() - started)
        row['phase'] = None

        label, color = STATUS_TEXT.get(msg.status, ('未知', ''))
        self._note('%s %s%s%s：%s'
                   % (msg.point_id, color, label, RESET, msg.message))
        self._write_jsonl('result', {
            'execution_id': msg.execution_id,
            'route_name': msg.route_name,
            'point_index': int(msg.point_index),
            'point_id': msg.point_id,
            'task_id': msg.task_id,
            'status': int(msg.status),
            'status_text': label,
            'message': msg.message,
            'source': source,
            'seconds': row.get('seconds'),
        })
        if render:
            self._render()

    # ------------------------------------------------------------------
    # 表格数据
    # ------------------------------------------------------------------
    def _row(self, key: Tuple[int, int], index: int, point_id: str) -> dict:
        row = self._rows.get(key)
        if row is None:
            row = {
                'index': index,
                'point_id': point_id,
                'status': PatrolTaskResult.RUNNING,
                'message': '',
                'phase': '排队中',
                'seconds': None,
            }
            self._rows[key] = row
            self._row_order.append(key)
        return row

    def _finalize_stale_rows(self, new_loop: int) -> None:
        """轮次已经翻页：上一轮还挂着“没出结果”的行，就是这一轮压根没上报。

        标成「未上报」而不是留着 RUNNING，否则历史小结里会永远显示成一堆
        失败的假点数。
        """
        for key in self._row_order:
            if key[0] >= new_loop:
                continue
            row = self._rows[key]
            if row.get('status') == PatrolTaskResult.RUNNING:
                row['status'] = PatrolTaskResult.CANCELED
                row['phase'] = None
                row['message'] = row.get('message') or '本轮未上报结果'

    def _begin_run(self, execution_id: str, route_name: str) -> None:
        self._close_jsonl()
        self._execution_id = execution_id
        self._route_name = route_name
        self._rows.clear()
        self._row_order.clear()
        self._point_started.clear()
        self._started_at = time.time()
        os.makedirs(self._out_dir, exist_ok=True)
        name = 'patrol_%s' % execution_id[:8]
        self._jsonl_path = os.path.join(self._out_dir, name + '.jsonl')
        self._jsonl = open(self._jsonl_path, 'a', encoding='utf-8')
        self._note('新一轮巡检开始：route=%s execution=%s'
                   % (route_name, execution_id[:8]))

    def _note(self, text: str) -> None:
        self._last_event = text
        self._last_event_time = time.time()
        self._events += 1
        self._write_jsonl('note', {'text': text})
        if not self._live:
            print('[%s] %s' % (_fmt_clock(self._last_event_time), text),
                  flush=True)

    def _write_jsonl(self, kind: str, payload: dict) -> None:
        if self._jsonl is None:
            return
        record = {'ts': round(time.time(), 3), 'kind': kind}
        record.update(payload)
        try:
            self._jsonl.write(json.dumps(record, ensure_ascii=False) + '\n')
            self._jsonl.flush()
        except (OSError, ValueError) as error:
            self.get_logger().warning('jsonl write failed: %s' % error)
            self._jsonl = None

    def _close_jsonl(self) -> None:
        if self._jsonl is not None:
            try:
                self._jsonl.close()
            except OSError:
                pass
            self._jsonl = None

    # ------------------------------------------------------------------
    # 渲染
    # ------------------------------------------------------------------
    def _render(self) -> None:
        if not self._live:
            return
        loop = self._current_loop
        keys = [k for k in self._row_order if k[0] == loop]
        earlier = [k for k in self._row_order if k[0] < loop]

        lines = []
        title = '巡检结果看板'
        if self._route_name:
            title += '   路线 %s' % self._route_name
        if self._execution_id:
            title += '   执行 %s' % self._execution_id[:8]
        lines.append(BOLD + title + RESET)

        head = '状态 %s ｜ 第 %d 轮 ｜ 已完成 %d 个点 ｜ 运行 %s' % (
            self._state, loop + 1, self._completed_points,
            self._elapsed_text())
        lines.append(head)
        battery = self._battery_text()
        if battery:
            lines.append(battery)
        if self._status_message:
            lines.append(DIM + '最新消息：%s' % self._status_message + RESET)
        lines.append('')

        if not keys:
            lines.append(DIM + '（等第一个点位…）' + RESET)
        else:
            lines.append('┌────┬──────────┬────────┬%s┬────────┐'
                         % ('─' * 38))
            lines.append('│ #  │ 点位     │ 状态   │ 检测结果%-30s│ 耗时   │' % '')
            lines.append('├────┼──────────┼────────┼%s┼────────┤'
                         % ('─' * 38))
            for key in keys:
                row = self._rows[key]
                label, color = STATUS_TEXT.get(
                    row.get('status', 0), ('未知', ''))
                if row.get('phase') and row.get('status') == \
                        PatrolTaskResult.RUNNING:
                    label, color = row['phase'], DIM
                seconds = row.get('seconds')
                sec_text = '%.1f s' % seconds if seconds is not None else '—'
                lines.append(
                    '│ %-2d │ %s│ %s%s%s │ %s│ %s│'
                    % (row['index'],
                       _pad(row.get('point_id', ''), 8),
                       color, _pad(label, 4), RESET,
                       _pad(_clip(row.get('message', ''), 36), 38),
                       _pad(sec_text, 6)))
            lines.append('└────┴──────────┴────────┴%s┴────────┘'
                         % ('─' * 38))

        if earlier:
            summary = []
            for loop_index in sorted({k[0] for k in earlier}):
                rows = [self._rows[k] for k in earlier if k[0] == loop_index]
                ok = sum(1 for r in rows
                         if r.get('status') == PatrolTaskResult.SUCCEEDED)
                pending = sum(1 for r in rows
                              if r.get('status') == PatrolTaskResult.RUNNING)
                text = '第 %d 轮 %d/%d 成功' % (loop_index + 1, ok, len(rows))
                if pending:
                    text += '（%d 个未结束）' % pending
                summary.append(text)
            lines.append(DIM + '历史：' + '，'.join(summary) + RESET)

        if self._last_event:
            lines.append('')
            lines.append('%s  %s' % (
                DIM + _fmt_clock(self._last_event_time) + RESET,
                _clip(self._last_event, 72)))

        sys.stdout.write(CLEAR + '\n'.join(lines) + '\n')
        sys.stdout.flush()

    def _battery_text(self) -> str:
        """一行电池状态；从未收到过电压时返回空串，不占版面。"""
        if self._battery_v is None or self._battery_at <= 0.0:
            return ''
        age = time.time() - self._battery_at
        if age > self._battery_stale:
            return (DIM + '电池 —（%.0f s 未更新，检查 %s）'
                    % (age, self._battery_topic) + RESET)
        if self._battery_level == 2:
            colour, tag = RED, '严重亏电，请回充'
        elif self._battery_level == 1:
            colour, tag = YELLOW, '偏低，建议本轮结束后回充'
        else:
            colour, tag = GREEN, '正常'
        return '电池 %s%.2f V%s  %s' % (colour, self._battery_v, RESET, tag)

    def _elapsed_text(self) -> str:
        total = int(time.time() - self._started_at)
        return '%02d:%02d:%02d' % (total // 3600, total % 3600 // 60,
                                   total % 60)

    # ------------------------------------------------------------------
    # 收尾
    # ------------------------------------------------------------------
    def finish(self) -> None:
        """退出时把表格落成 CSV，并打一份小结。"""
        self._close_jsonl()
        if not self._rows:
            return
        os.makedirs(self._out_dir, exist_ok=True)
        name = 'patrol_%s.csv' % (self._execution_id[:8] or 'unknown')
        path = os.path.join(self._out_dir, name)
        try:
            with open(path, 'w', encoding='utf-8', newline='') as handle:
                writer = csv.writer(handle)
                writer.writerow(['loop', 'index', 'point_id', 'status',
                                 'status_text', 'message', 'seconds'])
                for key in sorted(self._rows):
                    row = self._rows[key]
                    label, _ = STATUS_TEXT.get(
                        row.get('status', 0), ('未知', ''))
                    writer.writerow([key[0], row['index'],
                                     row.get('point_id', ''), row.get('status'),
                                     label, row.get('message', ''),
                                     '' if row.get('seconds') is None
                                     else round(row['seconds'], 2)])
        except OSError as error:
            self.get_logger().warning('csv write failed: %s' % error)
            return

        ok = sum(1 for r in self._rows.values()
                 if r.get('status') == PatrolTaskResult.SUCCEEDED)
        bad = sum(1 for r in self._rows.values()
                  if r.get('status') == PatrolTaskResult.FAILED)
        print('\n=== 巡检小结 ===')
        print('路线      : %s' % (self._route_name or '-'))
        print('执行 ID   : %s' % (self._execution_id or '-'))
        print('点位数    : %d（成功 %d，失败 %d）'
              % (len(self._rows), ok, bad))
        print('表格      : %s' % path)
        print('事件流    : %s' % (self._jsonl_path or '-'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ResultViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finish()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

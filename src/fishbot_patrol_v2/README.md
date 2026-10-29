# fishbot_patrol_v2

巡检执行的第二版实现。对外接口与 `fishbot_patrol_executor`（v1）完全一致，
可以直接替换；两个包可以并存，随时切回。

## 一条命令跑起来

```bash
# 最省事：整条链路 + 发送任务 + 前台看反馈，Ctrl+C 一次全停
tools/patrol.sh                 # 路线 main，跑一轮
tools/patrol.sh main 1 3        # 路线 main，repeat 模式，3 轮

# 想自己控制：launch 默认就把三个节点全拉起来
ros2 launch fishbot_patrol_v2 patrol_v2.launch.py
ros2 action send_goal /execute_patrol \
  fishbot_patrol_interfaces/action/ExecutePatrol \
  "{route_name: 'main', mode: 0, repeat_count: 1}" --feedback
```

`patrol_v2.launch.py` 默认启动 **route_manager + patrol_task_manager +
patrol_executor_v2** 三个节点，任何一个挂掉都会 `respawn`。视觉和导航默认不启动
（通常跑在别的机器上），需要时加 `vision:=true` / `nav:=true`。

单独启动其中几个：

```bash
ros2 launch fishbot_patrol_v2 patrol_v2.launch.py route_manager:=false
ros2 launch fishbot_patrol_v2 patrol_v2.launch.py task_manager:=false
```

## 为什么要重写

v1 的 `executor_node.py` 是一个 798 行的 Node，`while` 循环串行跑完整轮巡检。
读下来有七个问题，按影响排序：

| 级别 | 问题 | v1 里的位置 |
|---|---|---|
| P0 | 单点失败直接中止整轮，前面跑完的点全部白费 | `_execute_route` 的 `if not ok: return self._finish(...'FAILED')` |
| P0 | 进度只在内存里，重启/断电/暂停后只能从头再来 | `self._current_index` 等局部状态 |
| P0 | 检测结果不落盘，事后查不到「哪个点、什么时候、拍到了什么」 | 结果只在 `/patrol/task/result` 的 message 里过一次 |
| P1 | 没有抢占：视觉报告火情时无法立即打断巡检 | 只有手动 `/patrol/pause` |
| P1 | 暂停等待逻辑散在 4 处，恢复后「重试当前点」的行为各处不同 | `_wait_for_resume` 被调 4 次 |
| P1 | 导航/任务/暂停/状态发布全混在一个类里，无法单独测试 | 整个文件 |
| P2 | 20 ms 级 `sleep` 轮询 future，回调线程里空转 | `_wait_future` |

## 现在长什么样

```
plan.py        纯逻辑  路线 → 作业队列；失败策略 stop/skip/retry；连续失败熔断
progress.py    纯逻辑  进度快照 + 原子写；resume 时跳过已完成的点
report.py      纯逻辑  JSONL 报告 + summary；可选存图
util.py        纯逻辑  到点校验、结果解析、四元数换算
navigation.py  适配层  Nav2 NavigateToPose 封装（超时/取消/中断/到点复核）
node.py        适配层  action server + 服务 + 状态发布，只做消息翻译
test/          49 个单测，本机 python3 直接跑，不需要 ROS
tools/         静态检查（抓「用了但没 import」的名字）
```

约 500 行是纯逻辑且有测试覆盖，`node.py` 只剩消息收发和主循环。

## 与 v1 的行为差异

1. **失败可跳过**：默认 `skip`，记录失败继续下一个点；连续失败 `max_consecutive_failures`（默认 3）次才中止整轮。`failure_policy=stop` 可退回 v1 行为。
2. **断点续跑**：进度写 `~/.ros/patrol_progress.json`（原子写），进程重启后 `resume=true` 从上次未完成的点继续。进度带 `指纹`（路线名+模式+轮数），换了路线不会错误续跑。
3. **落盘报告**：每轮写 `~/patrol_reports/patrol_<时间>_<id>.jsonl`，每个点一条（时间/点位/导航耗时/任务耗时/结果/检测到的目标），另写一份 `.summary.json` 汇总，可直接当「发现清单」。
4. **事件抢占**：订阅 `/patrol/preempt`（`std_msgs/String`），内容是点位 id 就先过去，是别的字符串就只记录。处理完按原进度继续。
5. **统一暂停门**：所有等待都走 `_gate()`，暂停不消耗任务超时预算。
6. **到点复核**：Nav2 返回成功后，再用 AMCL 位姿核对位置/朝向误差（默认 0.30 m / 0.60 rad），能抓住「以为到了其实没到」。

## 接口（与 v1 一致）

| 类型 | 名字 | 说明 |
|---|---|---|
| action | `/execute_patrol` | `ExecutePatrol`，goal 同 v1 |
| 服务 | `/patrol/pause`、`/patrol/resume` | `PausePatrol` / `ResumePatrol` |
| 话题 | `/patrol/task/request` | 发给 task_manager |
| 话题 | `/patrol/task/result` | 收 task_manager 结果 |
| 话题 | `/patrol/preempt` | 抢占（新增） |
| 话题 | `/patrol/execution/status` | JSON 状态（latched） |

## 在新机器上跑

```bash
# 1. 拷到工作空间
cp -r fishbot_patrol_v2 ~/fishbot_full_ws/src/

# 2. 编译（必须先有 fishbot_patrol_interfaces）
cd ~/fishbot_full_ws
colcon build --packages-select fishbot_patrol_interfaces fishbot_patrol_v2 --symlink-install
source install/setup.bash

# 3. 不起 v1 的 executor，起 v2
ros2 launch fishbot_patrol_v2 patrol_v2.launch.py
# 或指定策略：
ros2 launch fishbot_patrol_v2 patrol_v2.launch.py failure_policy:=stop

# 4. 触发一轮巡检（与 v1 相同）
ros2 action send_goal /execute_patrol fishbot_patrol_interfaces/action/ExecutePatrol \
  "{route_name: default, mode: 0, repeat_count: 1}" --feedback

# 5. 看报告
ls ~/patrol_reports/
```

## 单测

```bash
cd fishbot_patrol_v2
python3 test/test_plan.py
python3 test/test_progress.py
python3 test/test_report.py
python3 test/test_util.py
python3 tools/check_names.py fishbot_patrol_v2/*.py
```

不需要 ROS，本机 python3.10+ 即可。

## 参数

见 `config/patrol_v2.yaml`，常用几个：

| 参数 | 默认 | 说明 |
|---|---|---|
| `failure_policy` | `skip` | `stop` / `skip` / `retry` |
| `max_retries` | 1 | 仅 `retry` 生效 |
| `max_consecutive_failures` | 3 | 连续失败几次中止整轮 |
| `resume` | `true` | 是否断点续跑 |
| `task_result_timeout_s` | 130 | 单点任务超时 |
| `waypoint_timeout_s` | 180 | 单点导航超时 |
| `final_position_tolerance_m` | 0.30 | 到点复核容差 |
| `endpoint_max_xy_std_m` | 0.15 | 定位 xy 标准差上限，超过就跳过到点判定 |
| `endpoint_max_yaw_std_rad` | 0.15 | 定位 yaw 标准差上限 |
| `endpoint_stable_duration_s` | 0.5 | 到点复核采样窗口，取窗口内最坏残差 |
| `endpoint_require_localization` | `false` | `true` 时拿不到可信定位直接判导航失败 |
| `use_tf_endpoint` | `true` | 优先用 TF(`map->base_link`) 复核，退回 AMCL |
| `save_frames` | `false` | 是否存标注图（需要 cv_bridge + cv2） |
| `recheck_forward_m` | 0.4 | 主动复核每次前进距离；0 = 关闭该功能 |
| `recheck_max_rounds` | 2 | 最多走近几次 |
| `recheck_conf_low` | 0.25 | 低于它按「没看到」处理 |
| `recheck_conf_high` | 0.60 | 高于它直接确认，不再走近 |
| `recheck_on_miss` | `true` | 完全没检到时也走近确认 |

## 还没做的事

- 真机/仿真联调：本机没有 ROS，`node.py` 只做了静态检查和名字检查。
- `save_frames` 依赖 `cv_bridge`，在没装的机器上会打一条 error 然后跳过存图，不影响巡检。
- v1 包没有删，切换方式：改 launch 里起哪个可执行文件。

## 变更记录

### 0.3.2（主动视角复核：看不清就走近再看一眼）

- 新增 `recheck.py`（纯逻辑，19 个单测）：把视觉的 `fire_extinguisher x2 (0.92,0.91)`
  这类描述解析成 `(数量, 置信度)`，再按决策表决定「接受」还是「往前走」。
  认不出的格式一律不复核，宁可按原结果收，也不拿坏数据决定车往哪开。
- `node.py` 新增 `_run_task_with_recheck()`：置信度落在
  `[recheck_conf_low, recheck_conf_high)`（默认 0.25~0.60）或干脆没检到时，
  沿当前朝向前进 `recheck_forward_m`（默认 0.4 m）再检一次，最多
  `recheck_max_rounds`（默认 2）轮。已 >= 0.60 就直接确认，不折腾。
- 每次走近都换一个任务 `execution_id`（`-A%d` 用 `job.attempt + moves`）：
  不换的话 task_manager 会按 `(execution_id, point_index)` 去重，直接回放
  上一轮的结果，「再检」就白检了——这是实现时踩到的第一个坑。
- 走近是额外物理动作，所以在报告里留一条 `recheck` 事件
  （`moves` / `moved_m` / `top_conf` / `decision`），点位 message 末尾追加
  `[recheck: +0.80m x2, top=0.88]`。事后对不上耗时，这是唯一的证据。
- 挪不动（定位丢失/被取消/前方有障碍）不算点失败：按已有结果收，只打一条
  警告；被取消才向上抛中断。
- 关掉：`recheck_forward_m: 0.0`；只对「没检到」生效、不走近：
  `recheck_on_miss: false`。

### 0.3.1（修 external_result 的 QoS 不兼容）

- 看板订阅 `/patrol/task/external_result` 时要求 `TRANSIENT_LOCAL`，而视觉
  （`yolo_detector.py:183-191`）是 **VOLATILE** 发布，DURABILITY 不兼容，
  rmw 报 `requesting incompatible QoS ... No messages will be sent to it`，
  开 `show_external:=true` 也看不到任何视觉回执。订阅端改成和发布端一致的
  `RELIABLE + VOLATILE`（`/patrol/task/result` 与 `/patrol/execution/status`
  仍然是 latched，那两个的发布端本来就是 latched，没问题）。
- 只看不改：这条不兼容**不影响巡检本身**，task_manager 用的是 volatile，
  一直收得到；受影响的只有看板的“视觉原始回执”这一列。

### 0.3.0（看板加电池电压）

- `result_viewer.py` 订阅 `/battery/voltage`（`std_msgs/Float32`，ESP32 经
  micro-ROS 直接发），在表头下面多显示一行：
  `电池 12.34 V  正常` / `偏低，建议本轮结束后回充` / `严重亏电，请回充`；
  超过 `battery_stale_sec`（默认 10 s）没更新就显示 `电池 —（N s 未更新）`。
- 电压跨档时才写一条事件到 JSONL，1 Hz 的读数不会把事件流刷满。
- 新增参数：`battery_topic`、`battery_warn_v`（11.8）、`battery_crit_v`（11.5）、
  `battery_stale_sec`（10.0）。电池是可选话题，没有它看板照常工作。

### 0.2.6（`_non_negative` 缺失导致启动即死）

- 修 `node.py:96` 调用 `self._non_negative('endpoint_stable_duration_s')`
  但方法没实现，节点一构造就抛
  `AttributeError: 'PatrolExecutorV2' object has no attribute '_non_negative'`，
  `respawn=True` 下表现为无限重启、`ros2 node list` 里看不到执行器。
- 新增 `tools/check_attrs.py`：静态查找「`self._xxx()` 调用了但类里没定义」。
  `py_compile` 和单测都抓不到这类错（本机没装 rclpy 时 `node.py` 整个被跳过），
  只能在真机上暴露。现在 CI 前置检查能提前拦住。
- 单测 79 个不变。

### 0.2.5（`'bool' object is not callable` 收口）

- 导航层入口把 `cancel_check` / `interrupt_check` 统一归一成无参回调
  （`util.as_check()`）。调用方不管传回调还是传布尔值都不会再炸——
  rclpy 的 `ServerGoalHandle.is_cancel_requested` 是**属性**，误当回调传就会
  在每个点位开始导航时抛 `'bool' object is not callable`。
- 巡检异常分支补打完整栈（`traceback.format_exc()`），以后再出这类错
  能直接看到是哪一行。
- 单测 76 → 79（`as_check` 三个用例）。

> 如果你看到这个错，先确认跑的是不是旧构建：
> ```bash
> python3 -c "import fishbot_patrol_v2.node as m; print(m.__file__)"
> grep -c "cancel_check=lambda" $(python3 -c "import fishbot_patrol_v2.node as m; print(m.__file__)")
> ```
> 第二条输出 0 就是旧代码，重新 `colcon build --packages-select fishbot_patrol_v2` 后重启。

### 0.2.4（缺陷审查轮）

三个模块各自做了一轮逐行审查（plan/util/progress、navigation/report、node 并发），
确认的问题已修，并补了 21 个回归用例（单测从 55 个涨到 76 个）。

行为上会变的几处：

- **续跑指纹带上点位组成**。同名路线增删或重排点后，旧进度自动作废，
  不会再按「下标」去解释旧路线（旧快照 schema v1 一律作废，最多重跑一轮）。
- **坏快照不再能跑偏**：负游标被钳到 0，快照轮次超过本次请求轮数时直接判完成，
  `policy` 传字符串会在构造时就报错（以前是静默退化成 `skip`）。
- **进度写盘失败不再中断巡检**：`progress.save()` 返回布尔值，写不进只丢续跑能力。
- **`completed_points` / `completed_loops` 数字修正**：续跑时把已完成的点算进去，
  整轮跑完报 1 而不是停在 0（以前永远少一轮）。
- **`yaw_of` 先归一化四元数**：非单位四元数不再算出错误偏航（实测 90° 曾被算成 126.9°）。
- **到点复核拒绝 NaN**：以前 NaN 残差会被 `max()` 静默吞掉，变成「误差 0」的假到点。
- **Ctrl+C 能真正停下**：取消导航目标后再退出；`_finish` 里 `_goal_active` 最后放开，
  避免下一个 goal 在上一个收尾的窗口里被接住。
- **路线加载期间也响应 Ctrl+C**：`_wait_future` 加了 stop 检查（以前最长卡 10 s）。

已知未修（评估后认为收益不足或需要真机验证）：

- 任务等待期间不响应抢占（`_preempt` 只在点位之间处理）。
- 状态话题是 latched 的，跑完后订阅者仍会立刻收到最后一条状态。
- 进度快照只 fsync 文件不 fsync 目录，掉电极端情况下可能回退一份。

## 结果看板（patrol_viewer）

巡检跑起来之后，另开一个终端：

```bash
ros2 run fishbot_patrol_v2 patrol_viewer
```

它只订阅、不发控制指令，随时开随时关都不影响巡检：

```
巡检结果看板   路线 main2   执行 d1ef46f6
状态 RUNNING ｜ 第 1 轮 ｜ 已完成 1 个点 ｜ 运行 00:01:32
最新消息：waiting vision

┌────┬──────────┬────────┬──────────────────────────────────────┬────────┐
│ #  │ 点位     │ 状态   │ 检测结果                              │ 耗时   │
├────┼──────────┼────────┼──────────────────────────────────────┼────────┤
│ 0  │ p1      │ 成功   │ fire_extinguisher x2 (0.92, 0.86)     │ 4.6 s  │
│ 1  │ p2      │ 失败   │ no fresh detection (3.2s old)         │ 12.1 s │
│ 2  │ p3      │ 等视觉 │                                       │ —      │
└────┴──────────┴────────┴──────────────────────────────────────┴────────┘
历史：第 1 轮 2/3 成功
```

- 数据源：`/patrol/execution/status`（JSON）、`/patrol/task/result`、
  `/patrol/task/external_result`，三个都是现成话题，不新增任何接口。
- 轮次切换时自动清空重画，历史轮次折叠成一行小结。
- 跑完自动落两份文件到 `~/patrol_viewer/`：`patrol_<exec>.jsonl`
  （逐事件，便于事后分析）和 `patrol_<exec>.csv`（Excel 直接打开）。
- 常用参数：`-p live:=false`（不刷屏、逐行输出，适合 `| tee run.log`）、
  `-p show_external:=true`（把视觉的原始回执也显示出来）、
  `-p out_dir:=/tmp/patrol`。
- 耗时用**看板本机**的 monotonic 计算，不要求执行器和看板在同一台机器、
  时钟同步——跨机算时间差会得到负数或几十秒的假耗时。

## 多次运行汇总（tools/summarize_runs.py）

跑了几次之后，一行命令把所有报告合成一张表——面试/答辩时「稳不稳」靠它回答，
不靠记忆：

```bash
python3 ~/fishbot_full_ws/install/fishbot_patrol_v2/share/fishbot_patrol_v2/tools/summarize_runs.py
# 或直接跑源码里的
python3 tools/summarize_runs.py --last 4 --csv runs.csv
```

```
#  开始时间        路线   点位  成功  失败  成功率  总时长  单点P50  单点P95  检出
1  10-22 19:01:00  main3  30    30    0     100%    550s    23.0s    23.0s    30
2  10-22 19:01:00  main3  30    30    0     100%    550s    21.5s    25.2s    30

总体：4 次运行 / 120 个点位访问，成功 120（100.0%），整轮零失败的运行 4/4
     单点耗时 P50 22.5s ｜ P95 30.4s ｜ 平均 19.6s
```

- 只读 `~/patrol_reports/*.jsonl`，不依赖 ROS，在虚拟机/笔记本上都能跑。
- `--last N` 只看最近 N 次；`--csv` 另存一份给 Excel。
- 报告里的 `status` 是 `ok/skipped/failed/canceled`，成功率只按 `ok` 算。

## 更新记录

### 0.3.3

- **电量决策**（新增 attery.py）：订阅 /battery/voltage，中值滤波 + 迟滞
  （warn 只提示，crit 时**跑完当前点再中止**）。报告 summary 里带
  attery_v / attery_level，事件里记 attery_abort。
- **点位期望值**（新增 expect）：点位上写 present / bsent，执行器拿它和
  实际检出比对，自动产出 tp/fp/fn/tn；报告里多出 erdicts 与
  
ecall / precision / alse_alarm_rate 三个率。不填就完全不判定，
  行为与旧版一致。
- **接口变更**：PatrolPoint.msg 与 RecordPoint.srv 各加一个 string expect，
  ishbot_patrol_interfaces 升到 **0.2.0**，ishbot_patrol_v2 升到 **0.3.3**。
  两个包都要重编（旧 msg 与新 msg 的 md5 不同，混用会一收一发全丢）。
- 导航失败也按 detected=False 记判定，避免把「没到点」算成「没目标」让召回虚高。

---

## 网页控制台（v0.4.0）

不想背命令时用它：一个进程 = 一个 ROS 节点 + 一个 HTTP 服务，浏览器打开就能
开始/暂停巡检、看实时状态和历史报告。

```bash
ros2 run fishbot_patrol_v2 patrol_console
# 或随 launch 一起起（占用 8080 端口）
ros2 launch fishbot_patrol_v2 patrol_v2.launch.py console:=true
```

浏览器打开 `http://<本机IP>:8080`（手机连同一个热点也能开）。

> 只用标准库：不需要 rosbridge、不需要 Flask，也没有前端构建步骤。

| 页面能做什么 | 背后的命令 |
|---|---|
| 开始巡检（路线/模式/轮次） | `ros2 action send_goal /execute_patrol ...` |
| 暂停 / 恢复 | 服务 `/patrol/pause`、`/patrol/resume` |
| 抢占当前点位 | 发布 `/patrol/preempt` |
| 记录当前位置为点位（含 expect） | 服务 `/patrol_route/record_point` |
| 实时状态、电量、事件流 | 订阅 `/patrol/execution/status`、`/battery/voltage`、`/patrol/task/result` |
| 历史运行表格 + 点开看事件 | 读 `~/patrol_reports` |

端口和监听地址可以用环境变量改：

```bash
PATROL_CONSOLE_PORT=9000 PATROL_CONSOLE_HOST=127.0.0.1 \
  ros2 run fishbot_patrol_v2 patrol_console
```

注意：控制台只是转发命令，**真正的决策还在执行器里**。关掉网页不影响正在跑的巡检。

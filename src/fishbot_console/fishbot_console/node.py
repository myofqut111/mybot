#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制台入口：起 ROS 桥、起 HTTP 服务、Ctrl+C 干净退出。

端口默认 8080，手机跟机器人在同一个网里，浏览器打开
`http://<机器人IP>:8080` 就能用。
"""
import os
import signal
import sys
import threading
import traceback

import rclpy

from .ros_bridge import ConsoleBridge
from .server import ROSBRIDGE_PORT, ConsoleServer


def _settings(node):
    """AI 配置：参数优先，其次环境变量。没配就只走规则回答。"""
    def pick(param, env, default=''):
        value = node.get_parameter(param).value
        if value:
            return str(value)
        return os.environ.get(env, default)

    return {
        'base_url': pick('ai_base_url', 'OPENAI_BASE_URL'),
        'api_key': pick('ai_api_key', 'OPENAI_API_KEY'),
        'model': pick('ai_model', 'OPENAI_MODEL', 'gpt-4o-mini'),
        # 一键启停时是否顺带拉起 RViz。关掉是为了把 CPU 留给 Nav2 的控制循环
        # ——RViz 在弱虚拟机上会和 20 Hz 的控制周期抢核，表现为车一顿一顿。
        'stacks_rviz': bool(node.get_parameter('stacks_rviz').value),
    }


def main(args=None):
    rclpy.init(args=args)
    bridge = None
    httpd = None
    try:
        bridge = ConsoleBridge()
        host = str(bridge.get_parameter('http_host').value)
        port = int(bridge.get_parameter('http_port').value)
        bridge.start()

        server = ConsoleServer(bridge, _settings(bridge))
        httpd = server.serve(host, port)
        actual = int(server.port or port)
        if actual != port:
            bridge.get_logger().warn(
                '端口 %d 被占用（更可能是上一次的 socket 还没释放），'
                '已自动顺延到 %d。浏览器请打开 http://%s:%d'
                % (port, actual, host, actual))
        if actual == ROSBRIDGE_PORT:
            bridge.get_logger().warn(
                '%d 是 rosbridge 的端口，浏览器里实时流会连不上；'
                '控制台建议用 8080/8090。' % actual)
        bridge.get_logger().info(
            'console listening on http://%s:%d （把 0.0.0.0 换成机器人 IP 即可）'
            % (host, actual))

        stop = threading.Event()

        def _on_signal(_signum, _frame):
            stop.set()

        signal.signal(signal.SIGINT, _on_signal)
        signal.signal(signal.SIGTERM, _on_signal)

        thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        thread.start()
        while not stop.is_set():
            stop.wait(0.3)
        return 0
    except Exception:
        # launch 只会显示 "process has died [exit code 1]"，真正的栈在上面几行，
        # 用户经常只贴最后一行。这里把栈完整打到 stderr，并给一条能照做的提示。
        traceback.print_exc()
        if bridge is not None:
            bridge.get_logger().error(
                'console failed to start, see the traceback above')
        else:
            sys.stderr.write(
                '控制台启动失败。常见原因：端口被占用（换 port:=9000）、'
                'rosbridge/ROS 环境没 source、参数类型不对。\n')
        return 1
    finally:
        if bridge is not None:
            bridge.get_logger().info('console shutting down')
        if httpd is not None:
            try:
                httpd.shutdown()
            except Exception:
                pass
        if bridge is not None:
            bridge.shutdown()
            try:
                bridge.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

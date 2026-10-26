#!/usr/bin/env python3
"""把「雷达转接板 → TCP」转成虚拟串口 /tmp/laserport，供 ydlidar 节点读取。

和原版（别人写的）相比改了四处，都是围绕同一个现场问题：
**AP 抖一下、雷达 TCP 断掉之后，经常再也连不回来。**

1. 会话期间也监听 accept()：转接板重连时立刻接管，不等旧会话超时。
   原版只在 ``s.accept()`` 阻塞等待，旧会话半死（AP 重启过，收不到 RST/FIN）
   时会一直占着，新连接根本进不来 —— 这是「断了就回不来」的主因。
2. 客户端开 SO_KEEPALIVE（10 s 空闲 + 3 s 间隔 + 3 次）：对端真死能自己发现，
   不用一直干等。
3. 空闲超时 30 s → 10 s：雷达 10 Hz 连续出数，10 s 没一个字节就是死了；
   启动/重启时的几秒空窗仍然容得下。
4. 每次会话结束把 pty 里残留的字节读干净，避免上一轮的半帧喂给 ydlidar 解析。
"""

import os
import pty
import select
import socket
import time

import rclpy
from rclpy.node import Node


class TcpSocketServerNode(Node):

    def __init__(self):
        super().__init__('tcp_socket_server_node')

        self.declare_parameter('tcp_port', 8889)
        self.declare_parameter('serial_port', '/tmp/laserport')
        # 雷达停顿多久算“死了”。启动/恢复时会停几秒，但不会停 10 秒。
        self.declare_parameter('idle_timeout_sec', 10.0)
        # 新连接进来时是否抢占旧会话。现场一定要开：转接板重连比旧会话
        # 超时快得多，不抢占就要白等一个 idle_timeout。
        self.declare_parameter('preempt_on_new_client', True)
        # TCP keepalive：对端上电复位/断网时，最快约
        # idle + interval*count = 10 + 3*3 = 19 s 发现。
        self.declare_parameter('keepalive_idle_sec', 10)
        self.declare_parameter('keepalive_interval_sec', 3)
        self.declare_parameter('keepalive_count', 3)

        self.tcp_port = self.get_parameter(
            'tcp_port').get_parameter_value().integer_value
        self.serial_port = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        self.idle_timeout_sec = self.get_parameter(
            'idle_timeout_sec').get_parameter_value().double_value
        self.preempt = self.get_parameter(
            'preempt_on_new_client').get_parameter_value().bool_value
        self.keepalive_idle = self.get_parameter(
            'keepalive_idle_sec').get_parameter_value().integer_value
        self.keepalive_interval = self.get_parameter(
            'keepalive_interval_sec').get_parameter_value().integer_value
        self.keepalive_count = self.get_parameter(
            'keepalive_count').get_parameter_value().integer_value

    # ------------------------------------------------------------------
    def _tune_socket(self, client: socket.socket) -> None:
        """开 keepalive 并调参数；内核不支持某项就跳过（不因此中断会话）。"""
        try:
            client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            return
        for option, value in (
                (getattr(socket, 'TCP_KEEPIDLE', None), self.keepalive_idle),
                (getattr(socket, 'TCP_KEEPINTVL', None), self.keepalive_interval),
                (getattr(socket, 'TCP_KEEPCNT', None), self.keepalive_count),
        ):
            if option is None or value <= 0:
                continue
            try:
                client.setsockopt(socket.IPPROTO_TCP, option, value)
            except OSError:
                pass

    @staticmethod
    def _drain(fd: int) -> None:
        """把 pty 里残留的字节读掉，避免上一轮的半帧污染下一轮解析。"""
        try:
            while select.select([fd], [], [], 0)[0]:
                if not os.read(fd, 4096):
                    break
        except OSError:
            pass

    # ------------------------------------------------------------------
    def run(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(('0.0.0.0', self.tcp_port))
        listener.listen(1)

        master, slave = pty.openpty()
        if os.path.exists(self.serial_port):
            os.remove(self.serial_port)
        os.symlink(os.ttyname(slave), self.serial_port)

        self.get_logger().info(
            'TCP端口:%d，已映射到串口设备:%s (idle=%.0fs, keepalive=%d/%d/%d)'
            % (self.tcp_port, self.serial_port, self.idle_timeout_sec,
               self.keepalive_idle, self.keepalive_interval,
               self.keepalive_count))

        poller = select.poll()
        poller.register(master, select.POLLIN)
        listener_fd = listener.fileno()
        poller.register(listener_fd, select.POLLIN)

        client = None
        last_exchange = time.monotonic()
        self.get_logger().info('等待接受连接..')
        try:
            while True:
                events = dict(poller.poll(1000))

                # --- 新连接：抢断旧会话 ---
                if listener_fd in events:
                    try:
                        incoming, address = listener.accept()
                    except OSError as exc:
                        self.get_logger().warning('accept failed: %s' % exc)
                        incoming, address = None, None
                    if incoming is not None:
                        if client is not None:
                            if not self.preempt:
                                # 不抢占：直接拒掉新连接，保持旧会话
                                self.get_logger().warning(
                                    '已有会话，拒绝来自%s的新连接' % (address,))
                                incoming.close()
                                incoming = None
                            else:
                                self.get_logger().warning(
                                    '新连接来自%s，接管旧会话' % (address,))
                                try:
                                    poller.unregister(client.fileno())
                                except (KeyError, OSError):
                                    pass
                                try:
                                    client.close()
                                except OSError:
                                    pass
                                client = None
                                self._drain(master)
                        if incoming is not None:
                            self._tune_socket(incoming)
                            client = incoming
                            last_exchange = time.monotonic()
                            poller.register(client.fileno(),
                                            select.POLLIN | select.POLLHUP
                                            | select.POLLERR)
                            self.get_logger().info(
                                '来自%s的连接已建立' % (address,))

                if client is None:
                    continue

                # --- 数据转发 ---
                client_fd = client.fileno()
                dropped = False
                if client_fd in events:
                    event = events[client_fd]
                    if event & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                        dropped = True
                    else:
                        try:
                            data = client.recv(4096)
                        except OSError:
                            data = b''
                        if not data:
                            dropped = True
                        else:
                            last_exchange = time.monotonic()
                            try:
                                os.write(master, data)
                            except OSError as exc:
                                self.get_logger().warning(
                                    'pty write failed: %s' % exc)
                                dropped = True
                if master in events and not dropped:
                    try:
                        data = os.read(master, 4096)
                    except OSError:
                        data = b''
                    if data:
                        try:
                            client.sendall(data)
                        except OSError as exc:
                            self.get_logger().warning(
                                'client send failed: %s' % exc)
                            dropped = True
                    if data:
                        last_exchange = time.monotonic()

                if not dropped and self.idle_timeout_sec > 0.0:
                    if time.monotonic() - last_exchange > self.idle_timeout_sec:
                        self.get_logger().warning(
                            'no lidar traffic for %.0fs; dropping session'
                            % self.idle_timeout_sec)
                        dropped = True

                if dropped:
                    self.get_logger().warning('会话结束，等待重连')
                    try:
                        poller.unregister(client_fd)
                    except (KeyError, OSError):
                        pass
                    try:
                        client.close()
                    except OSError:
                        pass
                    client = None
                    self._drain(master)
        finally:
            try:
                listener.close()
            except OSError:
                pass


def main():
    rclpy.init()
    node = TcpSocketServerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

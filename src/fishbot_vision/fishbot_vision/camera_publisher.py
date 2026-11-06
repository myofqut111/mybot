"""ESP32 OV3660 WiFi 摄像头取流节点。

工作流程：
1. 用 OpenCV 的 VideoCapture 打开 ESP32 摄像头的 MJPEG over HTTP 流
   （固件是 Arduino CameraWebServer，流地址形如 http://<ip>:81/stream）。
2. 后台守护线程不停地 read() 最新一帧，只保留最新帧，避免网络抖动时
   画面在队列里堆积、延迟越来越大。
3. ROS 定时器按固定频率把最新帧用 cv_bridge 转成 sensor_msgs/Image 发布。

为什么读流要单独开线程：
    cv2.VideoCapture.read() 在等下一帧网络数据时会阻塞。如果直接在 ROS
    定时器回调里读，一旦网络卡顿，整个节点的回调队列都会被堵住。放到
    独立线程里，发布线程永远只拿“最新一帧”，互不影响。
"""

import http.client
import threading
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image


class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__('camera_publisher')

        # ---------------- 参数 ----------------
        # 视频流地址（ESP32 CameraWebServer 固定为 81 端口的 /stream）
        self.declare_parameter('stream_url', 'http://192.168.1.102:81/stream')
        # 图像消息里的坐标系名（要和 URDF 里的相机 link 对得上）
        self.declare_parameter('frame_id', 'camera_link')
        # 图像发布频率（Hz）。CPU 紧张可以调低
        self.declare_parameter('publish_rate', 15.0)
        # 压缩图发布频率（Hz）。原始图一帧 230KB，走 WiFi 5fps 就是 9Mbps，
        # 把整条链路（连路由器）压垮；压缩图一帧约 12KB，2fps 只有 0.2Mbps。
        # 设 <=0 关闭压缩图发布（回到只发原始图的老行为）。
        self.declare_parameter('compressed_rate', 2.0)
        # 压缩图的 JPEG 质量（1~100）。60 在 320x240 上肉眼几乎无损。
        self.declare_parameter('jpeg_quality', 60)
        # 画面翻转：-1=旋转180°(上下+左右)  0=上下翻转  1=左右镜像
        # 设为 2 表示不翻转（OpenCV 只认 -1/0/1，用 2 当作“关闭”）
        self.declare_parameter('flip_code', -1)
        # 断线后重连间隔（秒）
        self.declare_parameter('reconnect_interval', 2.0)
        # 打开/读取流的超时（毫秒，老版本 OpenCV 不支持时会被忽略）
        self.declare_parameter('io_timeout_ms', 5000)
        # 最后一帧的"保鲜期"（秒）。超过这个时间没有新帧，就**停止发布**。
        # 为什么必须要有它：read() 若是永久阻塞（WiFi 挂死），缓存里的最后一帧
        # 会一直留着，而定时器每 tick 都用**当前时间**给它打新戳 —— 下游
        # （YOLO/RViz/巡检）看到的是"时间戳一直很新的静止画面"，会对着冻结图
        # 一直输出检测结果，巡检甚至判"识别成功"。宁可停止发布，也不能发假的新帧。
        # 设 <=0 关闭该保护（回到老行为）。
        self.declare_parameter('max_frame_age_sec', 2.0)
        # 读流 socket 超时（秒）。超过这么久没收到任何字节就判定断流并重连。
        # 摄像头 15 FPS 下每帧间隔 ~67ms，3s 已经是非常宽松的上限。
        self.declare_parameter('stream_timeout_sec', 3.0)

        # ---- 摄像头参数自动设置（等效于"在代码里写死"）----
        # ESP32 的 CameraWebServer 网页改的参数只存在内存里，**断电就丢**（原版固件
        # 不写 NVS）。所以这里在每次连接（含断线重连、摄像头重启后）之前，用它的
        # HTTP 控制接口 /control?var=<名>&val=<值> 把参数设一遍 —— 比刷固件安全，
        # 也不会因为忘了开网页设置而"分辨率又变回去了"。
        self.declare_parameter('apply_camera_settings', True)
        self.declare_parameter(
            'camera_settings',
            'framesize=8,quality=12,lamp=0,face_detect=0,face_recognize=0')
        # 控制接口地址，留空 = 从 stream_url 自动推导出 http://<摄像头IP>/control
        self.declare_parameter('camera_control_base', '')
        self.declare_parameter('camera_setup_timeout', 1.5)
        # 设置失败后多久不再重试（秒），避免摄像头不在线时每次重连都卡几秒
        self.declare_parameter('camera_setup_retry_sec', 10.0)

        # ---- 取图方式：单张 /capture 轮询（默认） / 长连接 MJPEG 流 ----
        # 实测 ESP32 的 httpd 扛不住长连接：每个新连接 3~4 s 后必断一次，重连
        # 几次之后整个 httpd 不再 accept（TCP 通、:80/:81 全 000），只能断电。
        # /capture 是每次一个短连接，实测 5/5 成功、约 0.2 s/张、640x480，
        # 没有状态可残留，所以默认走这条路。巡检只需要 2 Hz 推理，5 Hz 够用。
        self.declare_parameter('capture_mode', True)
        self.declare_parameter('capture_rate', 5.0)
        self.declare_parameter('capture_url', '')

        self._stream_url = self.get_parameter('stream_url').value
        self._frame_id = self.get_parameter('frame_id').value
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._flip_code = int(self.get_parameter('flip_code').value)
        self._reconnect_interval = float(
            self.get_parameter('reconnect_interval').value)
        self.declare_parameter('max_reconnect_interval', 15.0)
        self.declare_parameter('settings_min_interval', 60.0)
        self._max_reconnect_interval = float(
            self.get_parameter('max_reconnect_interval').value)
        self._settings_min_interval = float(
            self.get_parameter('settings_min_interval').value)
        self._fail_count = 0
        self._settings_applied_at = 0.0
        self._io_timeout_ms = int(self.get_parameter('io_timeout_ms').value)
        self._max_frame_age = float(
            self.get_parameter('max_frame_age_sec').value)
        self._stream_timeout = float(
            self.get_parameter('stream_timeout_sec').value)
        self._apply_settings = bool(
            self.get_parameter('apply_camera_settings').value)
        self._camera_settings = str(
            self.get_parameter('camera_settings').value)
        self._camera_control_base = str(
            self.get_parameter('camera_control_base').value)
        self._setup_timeout = float(
            self.get_parameter('camera_setup_timeout').value)
        self._setup_retry_sec = float(
            self.get_parameter('camera_setup_retry_sec').value)
        self._capture_mode = bool(self.get_parameter('capture_mode').value)
        self._compressed_rate = float(
            self.get_parameter('compressed_rate').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._capture_rate = max(
            0.5, float(self.get_parameter('capture_rate').value))
        self._capture_url = (str(self.get_parameter('capture_url').value).strip()
                             or self._derive_capture_url())

        # ---------------- 发布者 ----------------
        # 图像数据量大、允许丢帧，用 BEST_EFFORT + 只留最近 5 帧的 QoS
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._image_pub = self.create_publisher(
            Image, '~/image_raw', image_qos)
        # 压缩图：给控制台/浏览器看的。原始图只在板上给 yolo_detector 用
        # （同一台机器，走共享内存，不过 WiFi），所以出板的只有这个小图。
        self._compressed_pub = self.create_publisher(
            CompressedImage, '~/image_raw/compressed', image_qos)
        self._last_compressed = 0.0

        self._bridge = CvBridge()

        # ---------------- 共享状态 ----------------
        self._lock = threading.Lock()       # 保护 latest_frame
        self._latest_frame = None           # 最新一帧（OpenCV BGR 格式）
        self._last_frame_time = 0.0         # 最近一次成功读到帧的时刻（monotonic）
        self._last_frame_stamp = None       # 那一帧到达时的 ROS 时间（发布时用）
        self._setup_failed_at = 0.0         # 上次摄像头参数设置失败的时刻
        self._stop_event = threading.Event()
        self._cap = None
        self._connected = False
        self._frame_count = 0               # 累计发布帧计数

        # ---------------- 启动后台读流线程 + 发布定时器 ----------------
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.create_timer(1.0 / self._publish_rate, self._publish_frame)

        self.get_logger().info(
            'camera_publisher started, url=%s, flip_code=%d, rate=%.1fHz'
            % (self._stream_url, self._flip_code, self._publish_rate))

    # ------------------------------------------------------------------
    # 摄像头参数：把 ESP32 网页里那些"一断电就丢"的设置自动设一遍
    # ------------------------------------------------------------------
    def _control_url(self) -> str:
        """推导出摄像头控制接口地址（http://<摄像头IP>/control）。"""
        if self._camera_control_base.strip():
            return self._camera_control_base.strip().rstrip('/') + '/control'
        try:
            parsed = urllib.parse.urlparse(self._stream_url)
            if not parsed.hostname:
                return ''
            return '%s://%s/control' % (parsed.scheme or 'http',
                                        parsed.hostname)
        except Exception:
            return ''

    def _apply_camera_settings(self) -> None:
        """每次连接摄像头前，把 camera_settings 里的参数设一遍。"""
        if not self._apply_settings or not self._camera_settings.strip():
            return
        now = time.monotonic()
        if (self._setup_failed_at > 0.0 and
                now - self._setup_failed_at < self._setup_retry_sec):
            return      # 刚失败过，先别再卡住重连流程
        # ESP32 的 httpd 只有 4~5 个 socket，每设一个参数要单开一次连接（共 5 次）。
        # 每次重连都设一遍会把它顶死，表现为"跑一会儿就再也连不上"。
        if (self._settings_applied_at > 0.0 and
                now - self._settings_applied_at < self._settings_min_interval):
            return
        self._settings_applied_at = now

        url = self._control_url()
        if not url:
            self.get_logger().warning(
                'cannot derive camera control url from %s; '
                'settings not applied' % self._stream_url)
            return

        applied = []
        failed = []
        for item in self._camera_settings.split(','):
            item = item.strip()
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            query = '%s?var=%s&val=%s' % (url, key.strip(), value.strip())
            try:
                request = urllib.request.Request(
                    query, headers={'User-Agent': 'camera_publisher'})
                with urllib.request.urlopen(
                        request, timeout=self._setup_timeout) as response:
                    response.read(64)       # 不关心内容，读掉即可
                applied.append('%s=%s' % (key.strip(), value.strip()))
            except Exception as error:
                failed.append('%s(%s)' % (item, error))

        if applied:
            self.get_logger().info(
                'camera settings applied: %s' % ', '.join(applied))
        if failed:
            self._setup_failed_at = time.monotonic()
            self.get_logger().warning(
                'camera settings failed: %s (参数名/数值以固件为准，'
                '可在网页按 F12 看它实际发的请求)' % '; '.join(failed))
        else:
            self._setup_failed_at = 0.0

    # ------------------------------------------------------------------
    # 后台线程：负责打开连接、不停读最新帧、断线重连
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # 后台线程：直接读 HTTP multipart（不再用 cv2.VideoCapture）
    # ------------------------------------------------------------------
    # 为什么换掉 cv2.VideoCapture(url, CAP_FFMPEG)：
    #   1) ffmpeg 内部有**无上限**的包队列。客户端一时跟不上，队列越积越长，
    #      read() 依旧秒返回，但拿到的是若干秒前的旧帧 —— 画面看着没断，延迟
    #      却一路飘高，只能靠"帧龄"这种间接指标晚期发现。
    #   2) 断流时 read() 会阻塞到 CAP_PROP_READ_TIMEOUT_MSEC（原来 5 s）才返回
    #      失败，再加 2 s 重连间隔，一次卡顿要 7 s 起步才恢复；实测日志里帧龄
    #      一路涨到 27 s，就是这个"慢发现"叠出来的。
    #   3) 拿不到"这一帧到底是什么时候采的"，只能用发布时刻冒充，下游无法判断
    #      这张图其实已经旧了。
    # 直接解析 HTTP body 之后：socket 超时可设 1~3 s，Content-Length 精确切帧，
    # 读到即知新鲜度，断了立刻知道。

    def _stream_once(self) -> None:
        """连一次流，一直读到断线/超时/节点退出为止。"""
        parsed = urllib.parse.urlparse(self._stream_url)
        host = parsed.hostname
        if not host:
            raise ValueError('bad stream_url: %s' % self._stream_url)
        port = parsed.port or 80
        path = parsed.path or '/'
        if parsed.query:
            path += '?' + parsed.query

        conn = http.client.HTTPConnection(host, port, timeout=self._stream_timeout)
        conn.connect()
        try:
            conn.request('GET', path, headers={'User-Agent': 'camera_publisher'})
            resp = conn.getresponse()
            if resp.status != 200:
                raise IOError('HTTP %d %s' % (resp.status, resp.reason))

            ctype = resp.getheader('Content-Type') or ''
            boundary = b''
            for part in ctype.split(';'):
                part = part.strip()
                if part.lower().startswith('boundary='):
                    boundary = part.split('=', 1)[1].strip('"').encode()
            if not boundary:
                raise IOError('no multipart boundary in %r' % ctype)

            self._connected = True
            self.get_logger().info(
                'Connected to camera stream (%s, boundary=%s).'
                % (ctype, boundary.decode(errors='replace')))

            delimiter = b'--' + boundary
            buf = b''

            def fill(min_bytes: int = 1) -> None:
                """从 socket 往 buf 里补数据，至少 min_bytes 字节或抛异常。"""
                nonlocal buf
                while len(buf) < min_bytes:
                    if self._stop_event.is_set():
                        raise IOError('node shutting down')
                    chunk = resp.read(8192)
                    if not chunk:
                        raise IOError('stream closed by peer')
                    buf += chunk
                    if len(buf) > (1 << 22):
                        raise IOError('buffer overflow (%d bytes), boundary missing'
                                      % len(buf))

            while not self._stop_event.is_set():
                # 1) 找分隔符
                idx = buf.find(delimiter)
                while idx < 0:
                    fill(len(buf) + 4096)
                    idx = buf.find(delimiter)
                buf = buf[idx + len(delimiter):]

                # 2) 解析这一帧的头部（到空行为止）
                fill()
                while b'\r\n\r\n' not in buf:
                    fill(len(buf) + 1024)
                head, buf = buf.split(b'\r\n\r\n', 1)

                length = None
                for line in head.split(b'\r\n'):
                    if line.lower().startswith(b'content-length:'):
                        try:
                            length = int(line.split(b':', 1)[1].strip())
                        except ValueError:
                            length = None
                        break
                if not length or length <= 0:
                    continue

                # 3) 读满这一帧的 JPEG
                fill(length)
                jpeg, buf = buf[:length], buf[length:]

                frame = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    self.get_logger().warning(
                        'decode failed (%d bytes), skip' % len(jpeg),
                        throttle_duration_sec=5.0)
                    continue

                # 记下"这一帧到达的时刻"：既是新鲜度依据，也是最终发布的时间戳
                with self._lock:
                    self._latest_frame = frame
                    self._last_frame_time = time.monotonic()
                    self._last_frame_stamp = self.get_clock().now()
        finally:
            try:
                conn.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 取图方式 B：单张 /capture 轮询（默认）
    # ------------------------------------------------------------------
    def _derive_capture_url(self) -> str:
        """从 stream_url 推导出 /capture：同主机、80 端口。"""
        try:
            parsed = urllib.parse.urlparse(self._stream_url)
            if not parsed.hostname:
                return ''
            return '%s://%s/capture' % (parsed.scheme or 'http',
                                        parsed.hostname)
        except Exception:
            return ''

    def _poll_capture(self) -> None:
        """按 capture_rate 反复取单张。每张一个短连接，不留长连接状态。"""
        if not self._capture_url:
            raise ValueError(
                'cannot derive capture url from %s' % self._stream_url)
        period = 1.0 / self._capture_rate
        announced = False
        while not self._stop_event.is_set():
            started = time.monotonic()
            request = urllib.request.Request(
                self._capture_url, headers={'User-Agent': 'camera_publisher'})
            with urllib.request.urlopen(
                    request, timeout=self._stream_timeout) as response:
                data = response.read()
            if not data:
                raise IOError('empty capture response')
            frame = cv2.imdecode(
                np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warning(
                    'capture decode failed (%d bytes), skip' % len(data),
                    throttle_duration_sec=5.0)
            else:
                with self._lock:
                    self._latest_frame = frame
                    self._last_frame_time = time.monotonic()
                    self._last_frame_stamp = self.get_clock().now()
                if not announced:
                    announced = True
                    self._connected = True
                    self.get_logger().info(
                        'camera capture polling %.1fHz (%s, %dx%d)'
                        % (self._capture_rate, self._capture_url,
                           frame.shape[1], frame.shape[0]))
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop_event.wait(remaining)

    def _capture_loop(self) -> None:
        """后台线程主循环：连流 -> 读帧 -> 断了就重连。"""
        while not self._stop_event.is_set():
            try:
                self._apply_camera_settings()
                if self._capture_mode:
                    self._poll_capture()
                else:
                    self._stream_once()
                self._fail_count = 0     # 连上过（哪怕是正常断开），退避清零
            except Exception as error:
                with self._lock:
                    self._latest_frame = None
                delay = min(
                    self._reconnect_interval * (2 ** min(self._fail_count, 4)),
                    self._max_reconnect_interval)
                self._fail_count += 1
                if self._connected:
                    self.get_logger().warning(
                        'camera stream lost (%s); reconnecting in %.1fs'
                        % (error, delay))
                else:
                    self.get_logger().warning(
                        'camera connect failed (%s); retry in %.1fs'
                        % (error, delay),
                        throttle_duration_sec=10.0)
                self._connected = False
                self._stop_event.wait(delay)

    # ------------------------------------------------------------------
    # ROS 定时器：把最新帧翻转、转消息、发布
    # ------------------------------------------------------------------
    def _publish_frame(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            frame_age = time.monotonic() - self._last_frame_time

        if frame is None or self._last_frame_stamp is None:
            # 断流时节流警告，每秒最多一条，避免刷屏
            self.get_logger().warning(
                'No frame available yet (waiting for camera)...',
                throttle_duration_sec=1.0)
            return

        # 帧太旧（read() 卡住 / WiFi 挂死）时**不发**：避免把冻结的旧帧当成新帧
        if self._max_frame_age > 0.0 and frame_age > self._max_frame_age:
            self.get_logger().warning(
                'Stale frame (%.1fs old, limit %.1fs); not publishing so that '
                'downstream never sees a frozen image with a fresh timestamp'
                % (frame_age, self._max_frame_age),
                throttle_duration_sec=5.0)
            return

        # 画面翻转（-1/0/1 才是合法的 cv2.flip 参数，2 = 不翻转）
        if self._flip_code in (-1, 0, 1):
            frame = cv2.flip(frame, self._flip_code)

        image_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # 用"这一帧到达的时刻"而不是"此刻"：下游据此能看出图到底多旧
        image_msg.header.stamp = self._last_frame_stamp.to_msg()
        image_msg.header.frame_id = self._frame_id

        self._image_pub.publish(image_msg)
        self._frame_count += 1

        # 压缩图（低频）：只在需要时才编码，省板子的 CPU
        if self._compressed_rate > 0.0:
            now = time.monotonic()
            if now - self._last_compressed >= 1.0 / self._compressed_rate:
                self._last_compressed = now
                quality = max(1, min(100, self._jpeg_quality))
                ok, buffer = cv2.imencode(
                    '.jpg', frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), quality])
                if ok:
                    msg = CompressedImage()
                    msg.header = image_msg.header
                    msg.format = 'jpeg'
                    msg.data = buffer.tobytes()
                    self._compressed_pub.publish(msg)

    def destroy_node(self) -> bool:
        # 优雅退出：通知线程停止、释放摄像头
        self._stop_event.set()
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

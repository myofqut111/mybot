# fishbot_safety — 速度指令看门狗

车在动、但已经没人下速度指令的时候，主动发零速把它停下来。

## 它防的是什么

真实事故形态是：导航或巡检进程中途崩了 / 卡死 / 被 kill，**而底盘只认"最后一条指令"**。
于是车按最后一帧的速度一直开下去——直到撞墙。这个窗口在演练里出现过两次
（`bool object is not callable` 之后车还在跑、以及收到 ABORTED 后继续滑行）。

## 覆盖边界（必须说清楚）

| 场景 | 能不能兜住 |
|---|---|
| nav2 / 巡检执行器 / 控制台进程崩溃 | ✅ 约 0.5 s 内刹停 |
| 发布者卡死（进程在、不发消息） | ✅ 同上 |
| 虚拟机整机死机 / 断电 | ❌ 没人发零速，需要 ESP32 固件侧兜底 |
| 底盘与虚拟机之间网络断 | ❌ 零速也发不过去 |

后两种只能靠固件里的"指令超时自停"（`millis() - last_cmd_ms > 500` 就清零），
大约 10 行代码。这里没改固件，所以别把它当成唯一防护。

## 只有两条判据

1. 里程计显示车在动（含原地转，`|v| > 0.02` 或 `|w| > 0.02`），
   但最近 `cmd_timeout_sec`（默认 0.5 s）内**没有任何非零速度指令** → 刹车。
2. 里程计本身超过 `odom_timeout_sec`（默认 1.0 s）没来 → 不知道车是否在动，
   按安全侧假设在动 → 刹车。

**不把"视觉节点死了"算进来**：视觉挂了只会让巡检报失败，不会让车乱跑；
在那里刹车只会制造假故障。看门狗只管"会不会动"。

## 为什么零速不算心跳

刹车时我们自己往 `cmd_vel` 发零速。如果把零速也当成"还活着"，
看门狗会被自己发的消息解除，等于没有。所以只有**非零**指令才算心跳，
解除刹车必须等来一条真的运动指令。

## 用法

```bash
ros2 launch fishbot_safety watchdog.launch.py
```

第一次上车建议先干跑，只看日志不动车：

```bash
ros2 launch fishbot_safety watchdog.launch.py \
  --ros-args -p dry_run:=true
```

手推一下车（`odom` 有速度、`cmd_vel` 没人发），日志里应该出现：

```
[safety_watchdog] BRAKING: 车在动（0.32 m/s）但已 620 ms 没有速度指令
```

状态话题：`ros2 topic echo /safety/watchdog`（JSON，含 `tripped/reason/trips/speed/brakes_sent`）。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `cmd_topic` | `cmd_vel` | 监听的速度指令 |
| `odom_topic` | `odom` | 里程计（判"是否在动"） |
| `brake_topic` | 空=同 `cmd_topic` | 刹车发到哪 |
| `cmd_timeout_sec` | 0.5 | 多久没有非零指令算"没人开" |
| `odom_timeout_sec` | 1.0 | 多久没有里程计算"看不见车" |
| `move_epsilon` | 0.02 | 小于这个速度当静止 |
| `brake_rate_hz` | 20 | 刹车重发频率 |
| `dry_run` | false | 只记日志不发零速 |

## 测试

```bash
python3 test/test_guard.py      # 15 个用例，不需要 ROS
```

覆盖的边界：刚启动宽限、里程计从没来过、停着不罚、原地转也算在动、
自己发的零速不能解除、真指令能解除、一次事件只记一次 trip。

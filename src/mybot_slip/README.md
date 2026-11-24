# mybot_slip · 差速轮打滑检测与恢复

按 `差速轮机器人打滑检测与恢复方案_v1.1.docx` 落地。**当前进度：判据/状态机 + ROS 节点 + 动作执行 + 速度仲裁 + 膨胀动作 + 协方差降权(0.5.1，不停 SLAM) + 冻结 progress_checker(0.8.0) + 脱困后位姿校验(0.5.1)，离线回放 0 误动作，86 个单测通过；真机只差一次真打滑复现。**

## 这一版有什么

| 文件 | 作用 | 对应方案 |
|---|---|---|
| `mybot_slip/windows.py` | 0.3 s **非重叠**窗口聚合：里程计积分、左右轮差分、陀螺积分、激光按时间取配对帧、指令覆盖门槛、起步 warmup | §5.1 / 5.4 |
| `mybot_slip/ranges.py` | 用两帧激光做**点对点 ICP 一步**，估出车体真实平移 `(dx, dy, dθ)`，带鲁棒剔除与可信度评分；dict 与 LaserScan 消息都能吃 | §4.1 / 7.3 的实测来源 |
| `mybot_slip/criteria.py` | P1 直行、P2 过位移/侧滑、P3 左右轮、P4 旋转、P5 堵转 + 传感器自相矛盾；自适应阈值 `min(thresh_abs, 0.5·d_enc)`；绝对差下限 `min_shortfall_m`；滞回 `r_high/r_low` | §5.2 / 5.3 / 6.1 |
| `mybot_slip/state.py` | `NORMAL → SLIP → RECOVERY → CHECK`，**按类型分流**：STALL 只转向不后退，其余后退 0.10~0.30 m 再判；连续失败升级为「调大膨胀」，用尽则 `FAILED` | §6.2 / 6.3 |
| `mybot_slip/slip_node.py` | ROS 节点：订阅五路输入，20 Hz 轮询窗口，发 `/slip/state`（JSON）；`wheel_joints` 缺失自动降级；采样率过低打 WARN | §3 |
| `config/slip.yaml` | 全部参数（分组注释） | §7 |
| `launch/slip.launch.py` | `ros2 launch fishbot_slip slip.launch.py` | — |
| `tools/replay_slip.py` | 拿 bag 离线回放：统计候选窗口与**确认次数**（发布门槛） | §8 验收

## 为什么不用「扫描变化百分比」

那是之前 `mybot_explore` 里的做法，会被转向、行人、回波噪声同时触发。这里的每条判据都要求**某个量按运动学应该怎么变、实际没怎么变**——有方向、有符号、有量纲。

## 关键设计取舍

1. **激光不用逐束比测距**。对着一面斜墙时，同一束打到的点会沿墙面滑动，`r'(θ) = r − (t·n)/(n·û)`，系数取决于墙面法向；直接套 `Δr = −(dx·cosθ + dy·sinθ)` 会系统性低估。所以改成点对点 ICP。
2. **自适应阈值**。0.1 m/s × 0.3 s 只有 3 cm，固定 5 cm 门槛会让「完全卡死」在数学上永远判不出来。
3. **STALL 绝不后退**。轮子本来转不动，后退指令同样发不出去，只会白等。
4. **滞回**。`r_high=0.60` 确认、`r_low=0.36` 解除，中间区域保持恢复状态，避免状态抖动。
5. **确认要"持续"**。`confirm_windows=2` 且两次间隔至少 `confirm_window_s=1.0` ——0.3 s 的窗口里出现一次疑似，不够动手。
6. **绝对差下限**。0.18 m/s × 0.3 s 只有 5.4 cm，ICP 噪声约 1.5 cm；短差不到 3 cm 时比例再高也不判，否则低速时全是误报。

## 离线回放结果（发布门槛）

```bash
python3 tools/replay_slip.py ~/bags/2026
# 288 s / 960 窗（422 窗无指令，跳过）/ 候选 5 / **确认 0**
python3 tools/replay_slip.py ~/bags/2020
# 51 s / 170 窗 / 候选 0 / 确认 0
```

已知局限：这两段包里**没有真的打滑**，所以回放只能证明"不误报"，证明不了"能报"。灵敏度是算出来的：0.3 s 窗、0.18 m/s 巡航时位移 5.4 cm，叠加 1.5 cm 的 ICP 噪声，能可靠分辨的是**≥50% 的滑移**；想抓 20% 这种轻滑，需要把窗口放到 1 s 以上（代价是反应慢 3 倍）。

## 跑测试

```bash
cd src/mybot_slip
python3 -m unittest discover -s test -v
# 55 例：窗口 7 / 激光 7 / 判据 15 / 状态机 18 / 动作达成 8
```

## 动作执行（0.3.0）

```
slip_node ──/slip/action (JSON)──> slip_executor ──/slip/report (JSON)──> slip_node
```

- `back`：直发 `/cmd_vel` 倒车，`TravelMonitor` 按里程计累计路程闭环，到位即停；
- `escape`：原地反向转，累计净转角闭环（来回摆不算进度）；
- 动作结束**一定发一帧 0 速度**；`timeout_s` 到点报失败并停；
- 执行器忙时收到新动作：不排队，直接回 `ok=false`（上层据此升级/放弃）。

为什么不直接用 Nav2 的 `BackUp`/`Spin`：那要走 `navigate_to_pose` 的 BT 和生命周期，
而打滑时车通常正在一次导航任务里，借 Nav2 的动作会和当前任务抢执行权。

```bash
# 只看动作序列，不发速度
ros2 launch fishbot_slip slip.launch.py scan_topic:=/scan recovery:=true executor_dry_run:=true
# 真动（确认周围安全后）
ros2 launch fishbot_slip slip.launch.py scan_topic:=/scan recovery:=true
```

### 和导航一起跑（速度仲裁）

导航在跑的时候会一直往 `/cmd_vel` 发速度，恢复动作会被它盖掉。所以有三步：

```bash
# 1) 导航的速度输出去 /cmd_vel_nav，别直接写 /cmd_vel
ros2 launch fishbot_navigation navigation.launch.py \
    map_yaml_file:=~/maps/maps.yaml cmd_vel_topic:=/cmd_vel_nav
# 2) slip 带仲裁起来：/cmd_vel_slip（恢复）> /cmd_vel_nav（导航）→ /cmd_vel
ros2 launch fishbot_slip slip.launch.py scan_topic:=/scan recovery:=true with_mux:=true
```

仲裁规则：恢复动作最后一条速度 0.5 秒内只听恢复的，之后交还导航；
两路都没在发就发零速。切换时日志会打「速度指令让给恢复动作 / 回到导航」。

### 整机一条命令（推荐）

上面两步已经接进控制台的整机 launch，加 `with_slip:=true` 就会自动把 Nav2 的速度
改到 `/cmd_vel_nav` 并起仲裁：

```bash
ros2 launch fishbot_console robot.launch.py with_slip:=true
# 自主建图（slam）时同样适用
ros2 launch fishbot_console robot.launch.py localization:=slam with_slip:=true
```

起来后 `/cmd_vel` 的发布者应该**只有 `cmd_mux` 一个**，Nav2 在 `/cmd_vel_nav` 上。

## 膨胀半径动作（0.4.1）

`inflate` 用参数服务临时把 `local_costmap`/`global_costmap` 的
`inflation_layer.inflation_radius` 调大（默认 0.55），保持 `inflate_hold_s`（默认 8 s）
后还原；退出、`abort`、超时都会兜底还原，不会把改过的值留在代价地图上。

```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius   # 0.40
# 触发一次后：动作期间 0.55，保持时间到 → 0.40
```

两个坑记在这里，省得下次再踩：
1. 节点名是 `/local_costmap/local_costmap`（中间有下划线），按 `%s%scostmap` 拼会拼错；
2. 别在定时器回调里 `spin_until_future_complete` 等参数回应——executor 正忙着跑这个
   回调，回应永远等不到。要 `call_async` + `add_done_callback`。

## 对齐方案 §9.4 / §12 的四条（0.9.0）

这四条之前只是"写了参数、没接逻辑"，0.9.0 补齐：

| 方案条款 | 落点 | 做法 |
|---|---|---|
| §9.4 后退前查后方 0.3 m | `slip_executor._rear_blocked()` | 只在激光后向 ±30° 扇区里找最近点；挡住就不发速度，回执 `blocked=True` |
| 后方被挡怎么继续 | `slip_node._on_report()` | 同一轮恢复内换成 `escape`（原地转向），不额外计一次恢复 |
| §12 `Tstall 0.6 s` | `state._stall_ready()` | 堵转要连续 0.6 s 才确认，避免"刹一下"被当成堵转 |
| §12 `k_turn 1.5` / `d_floor 0.03` / IMU 超时 / tilt 10° | `criteria.py` | 转向时阈值 ×1.5；位移下限 3 cm；IMU 超 100 ms 或倾角 >10° 整窗不判 |

```bash
# 默认值即方案值，需要改时在命令行覆盖
ros2 launch fishbot_slip slip.launch.py ... imu_timeout_s:=0.10 max_tilt_deg:=10.0
```

横向精度：ICP 从点对点改点对线（`ranges._normals` + `_solve` 法向残差）后，
真包 `~/bags/2026` 1031 窗实测 `dy` 中位 0.2 mm、95 分位 3.8 mm、最大 12.3 mm，
对方案给的 `thresh_lat 0.03 m` 有约 8 倍余量——所以不动方案数字。

## 保护地图：只放大协方差，不碰 SLAM（0.8.0）

方案 §10.1 写得很明确：「协方差放大 10~100 倍……**而不是停止 SLAM 节点或切断激光输入**」。
0.4.2 当年用 `/slam_toolbox/pause_new_measurements` 直接把建图摁住，属于自造做法，
0.8.0 已删除（`slam_guard.py` 整个文件移除），改为：

- `odom_guard` 把 `/odom` 转发成 `/odom_guarded`，打滑窗口里把位姿/速度协方差放大
  （`covariance_position_factor` / `covariance_yaw_factor`，默认 100 倍），
  EKF 的 `odom0` 换成 `/odom_guarded` 后就会自动降权。
- 回 `NORMAL` 立刻还原协方差。
- `/slip/state` 的 `map_paused` 字段保留但恒为 `false`（旧页面兼容），新增
  `progress_frozen` 表示有没有冻结 Nav2 的 progress_checker。

实测教训（保留备查）：`/slam_toolbox/pause_new_measurements` 是 **toggle**、
返回的 `bool status` 恒为 true，只能自己记账；但更重要的是——**方案不允许停 SLAM**。

## 协方差降权（0.5.1，方案 §4）

方案要的是"放大里程计协方差、降低融合权重，同时 SLAM/TF 不中断"。做法是原样转发
`/odom`，但在异常窗口里把 pose/twist 的协方差乘 100（标准差约 ×10）：

```bash
ros2 launch fishbot_slip slip.launch.py scan_topic:=/scan recovery:=true with_mux:=true
ros2 launch fishbot_navigation navigation.launch.py ... ekf_odom_topic:=/odom_guarded
```

- 节点把 `/odom` 转到 `/odom_guarded`，正常时一字不改；打滑时只改协方差。
- EKF 的 `odom0` 指向 `/odom_guarded` 即可——**数据还是那条，权重变了**，TF 照常输出，
  所以 SLAM 不会因为"没有里程计"而断掉。
- 底盘协方差常发全 0，乘多少还是 0，所以有下限（`min_position_var/min_yaw_var=1e-4`）。
- 整机走 `robot.launch.py with_slip:=true` 时这几个参数会自动接好（`cmd_vel_topic` 与
  `ekf_odom_topic` 一起改），不用手传。

## 脱困后的位姿校验（0.5.1，方案 §6）

恢复完成时比较"进恢复前"和"恢复后"的位姿（取 `/odometry/filtered`，不取轮子里程——
轮子自己算不出自己滑了多少）：

- 平移 ≤ `pose_jump_m`（0.15 m）且转角 ≤ `pose_jump_rad`（0.20 rad）→ 记一行 info，继续。
- 超了 → 打 ERROR、清局部/全局代价地图；有 `/reinitialize_global_localization`
  （AMCL 模式）就再撒一次粒子，slam 模式没有这个服务会自动跳过。

```bash
ros2 launch fishbot_slip slip.launch.py ... pose_jump_m:=0.15 pose_jump_rad:=0.20
```

## 还没做（下一步）

- [x] `slip_node.py`：订阅 `/odom` `/wheel_joints` `/imu` `/scan` `/cmd_vel`，发布 `/slip/state`（std_msgs/String JSON）
- [x] 动作执行：`back`（`/cmd_vel` + 激光闭环）、`escape`（原地转）
- [x] Nav2 协作：`inflate` 临时把 `inflation_radius` 调大，保持后还原（`call_async`，退出兜底）
- [x] 与 Nav2 抢 `/cmd_vel` 的仲裁（`cmd_mux`：恢复优先 0.5 s，之后交还导航）
- [x] 用 `~/bags/2026` 离线回放验证「不误动作」（0 确认）
- [ ] 真机复现一次真打滑（垫子/斜坡），确认能报出来
- [x] 固件烧完后接上真实 `/wheel_joints`（`sensor_msgs/JointState`，左右累计米）


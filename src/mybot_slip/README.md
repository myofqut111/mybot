# mybot_slip · 差速轮打滑检测与恢复

按 `差速轮机器人打滑检测与恢复方案_v1.1.docx` 落地。**当前进度：判据层与状态机的纯逻辑（42 例单测全过），ROS 节点尚未接。**

## 这一版有什么

| 文件 | 作用 | 对应方案 |
|---|---|---|
| `mybot_slip/windows.py` | 0.3 s **非重叠**窗口聚合：里程计积分、左右轮差分、陀螺积分、激光按时间取配对帧、指令覆盖门槛、起步 warmup | §5.1 / 5.4 |
| `mybot_slip/ranges.py` | 用两帧激光做**点对点 ICP 一步**，估出车体真实平移 `(dx, dy, dθ)`，带鲁棒剔除与可信度评分 | §4.1 / 7.3 的实测来源 |
| `mybot_slip/criteria.py` | P1 直行、P2 过位移/侧滑、P3 左右轮、P4 旋转、P5 堵转 + 传感器自相矛盾；自适应阈值 `min(thresh_abs, 0.5·d_enc)`；滞回 `r_high/r_low` | §5.2 / 5.3 / 6.1 |
| `mybot_slip/state.py` | `NORMAL → SLIP → RECOVERY → CHECK`，**按类型分流**：STALL 只转向不后退，其余后退 0.10~0.30 m 再判；连续失败升级为「调大膨胀」，用尽则 `FAILED` | §6.2 / 6.3 |
| `config/slip.yaml` | 全部参数（分组注释） | §7 |

## 为什么不用「扫描变化百分比」

那是之前 `mybot_explore` 里的做法，会被转向、行人、回波噪声同时触发。这里的每条判据都要求**某个量按运动学应该怎么变、实际没怎么变**——有方向、有符号、有量纲。

## 关键设计取舍

1. **激光不用逐束比测距**。对着一面斜墙时，同一束打到的点会沿墙面滑动，`r'(θ) = r − (t·n)/(n·û)`，系数取决于墙面法向；直接套 `Δr = −(dx·cosθ + dy·sinθ)` 会系统性低估。所以改成点对点 ICP。
2. **自适应阈值**。0.1 m/s × 0.3 s 只有 3 cm，固定 5 cm 门槛会让「完全卡死」在数学上永远判不出来。
3. **STALL 绝不后退**。轮子本来转不动，后退指令同样发不出去，只会白等。
4. **滞回**。`r_high=0.60` 确认、`r_low=0.36` 解除，中间区域保持恢复状态，避免状态抖动。

## 跑测试

```bash
cd src/mybot_slip
python3 -m unittest discover -s test -v
# 42 例：窗口 7 / 激光 7 / 判据 14 / 状态机 14
```

## 还没做（下一步）

- [ ] `slip_node.py`：订阅 `/odom` `/wheel_joints` `/imu` `/scan` `/cmd_vel`，发布 `/slip/state`（std_msgs/String JSON）
- [ ] 动作执行：后退走 `nav2_msgs/BackUp` 或直接发 `cmd_vel` + 激光闭环；脱困走 `nav2_msgs/Spin`
- [ ] Nav2 协作：`/slip/inflate` 时临时把局部 `inflation_radius` 调大（`costmap` 参数服务）
- [ ] 用 `~/bags/2026` 离线回放验证三类滑移各能报一次
- [ ] 固件烧完后接上真实 `/wheel_joints`（`sensor_msgs/JointState`，左右累计米）

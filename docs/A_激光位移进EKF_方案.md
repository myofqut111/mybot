# A 方案：激光位移进 EKF（打滑时位姿不跟着编码器跑）

> 目标：让 `base_link` 的位姿在打滑期间**不再由编码器单独决定**，而是由激光的帧间位移
> 与编码器共同约束。不需要先"判出打滑"——判据再快也有 0.4 s 延迟，位姿早跑掉了。

---

## 0. 为什么现在必错（先把事实摆出来）

| 事实 | 数据 | 出处 |
|---|---|---|
| 位姿只由两路决定 | `odom0`（X/Y/yaw/vx/vyaw）+ `imu0`（vyaw） | `config/ekf.yaml:38-56` |
| `/odom` 的协方差不为 0，但很松 | pose X/Y/yaw = 0.020（σ≈0.14 m） | 实测 `manual_1127_1451` 首帧 |
| EKF 输出同样松 | `/odometry/filtered` pose X=0.0435、yaw=0.0311 | 同上 |
| 打滑时轮子在转、车没动 | t=247.6：车体位移 0.141 m/0.8 s，扫描只变 0.010 m | `bag_tail` 合成 map 位姿 |
| 冻结总是迟到 | 423.4 确认 → 424.4 才冻结，期间位姿前进 0.326 m | 2030 包逐秒 |

结论：**位移只有编码器一路来源**，打滑时它撒谎，位姿必然跟着错；
我们所有判据都是"事后补救"，最多把错误**变短**，不可能变没有。

---

## 1. 验收标准（先定标准，再动手）

1. 打滑段：`/odometry/filtered` 的位移误差 ≤ **0.05 m**（今天实测 0.14 ~ 0.33 m）。
2. 正常行驶段：filtered 与 `/odom` 的位移差 ≤ **3%**，yaw 差 ≤ 2°。
3. 不引入抖动：巡航行进时 filtered 的 vx 抖动的 1σ ≤ 0.05 m/s。
4. 可一键回退：`use_scan_twist:=false` 时行为与今天**完全一致**。

---

## 2. 数据流

```
/scan ──► scan_odom(ICP 点对线) ──► /scan_twist ─┐
                                                ├─► EKF ──► /odometry/filtered + TF odom→base_link
/odom ──► odom_guard(协方差地板/冻结) ──► /odom_guarded ─┤
                                                │
/imu ───────────────────────────────────────────┘
```

- `scan_odom` 只做几何，不做判定；配准不可用就**不发**（宁可不给证据，也不给错证据）。
- 用 `twist` 而不是 `pose`：pose 观测会和 slam 的 `map→odom` 打架；twist 是局部速度观测，只影响运动学。

---

## 3. 节点接口（`mybot_slip/scan_odom.py`，已起草）

| 参数 | 默认 | 说明 |
|---|---|---|
| `scan_topic` | `/scan` | 用原始 `/scan`（门控后的 `/scan_safe` 会丢帧，不适合做位姿估计） |
| `out_topic` | `/scan_twist` | `geometry_msgs/TwistWithCovarianceStamped`，`frame_id: base_link` |
| `min_dt / max_dt` | 0.05 / 1.0 | 跨度太小噪声大，太大跟不上 |
| `quality_min` | 0.30 | 低于此值不发 |
| `coverage_min` | 0.60 | 重叠率下限 |
| `max_speed / max_yaw_rate` | 0.6 / 2.5 | 超过当配准错误，丢弃 |
| `pos_sigma_base / yaw_sigma_base` | 0.010 / 0.010 | 最好情况下的 σ |
| `use_odom_init` | true | 用 `/odom` 增量当 ICP 初值（走廊里避免局部极小） |

发布：`vx = dx/dt`、`wz = dtheta/dt`，协方差 `cov[0]`(vx)、`cov[7]`(vy，给大)、`cov[35]`(vyaw)。
诊断：每 2 s 打一行"配准不可用：q=… 覆盖=… 原因=…"。

> ⚠️ 草稿里 `_odom_delta()` 现在是占位（恒返回 0），落地时要真做成"用 `/odom` 增量当初值"。

---

## 4. EKF 改动（`config/ekf.yaml`，默认关闭）

```yaml
    # —— A 方案新增 ——
    twist0: /scan_twist
    twist0_config: [false, false, false,
                    false, false, false,
                    true,  false, false,   # vx
                    false, false, true,    # vyaw
                    false, false, false]
    twist0_differential: false
    twist0_relative: false
    twist0_queue_size: 5
    twist0_twist_rejection_threshold: 1.5
```

**关键改动（这条不做，A 就无效）**：`odom0_config` 从"X/Y/yaw + vx/vyaw"降成"只 vx/vyaw"。
原因：20 Hz 的绝对位姿观测会压过 10 Hz 的速度观测，激光拉不回来。
代价：失去绝对 yaw 参考 → 由 slam / AMCL 的 `map→odom` 修（这本来就是它的职责）。

---

## 5. 协方差怎么给（这是全方案的胜负手）

```
σ_v = pos_sigma_base + (1 - q) * gain        q = ICP 质量（0~1）
σ_w = yaw_sigma_base + (1 - q) * gain
```

取 `base=0.02`、`gain=0.15`：

| ICP 质量 q | σ_v | 相对 `/odom`（σ≈0.14）的含义 |
|---|---|---|
| 0.80 | 0.050 | 激光权重约为 odom 的 8 倍 |
| 0.65 | 0.073 | 约 4 倍 |
| 0.50 | 0.095 | 约 2 倍 |
| 0.35 | 0.118 | 与 odom 相当（顶箱子场景，仍然能拉） |
| < 0.30 | 不发 | 退化到只看 odom |

---

## 6. 三个退化场景（必须先想清楚，否则会把正常行驶搞坏）

| 场景 | 现象 | 处理 |
|---|---|---|
| **长走廊直行** | 沿墙平移，单帧激光对位移几乎无约束（孔径问题），dx 可能偏小 | 用 `/odom` 增量做 ICP 初值；正常行驶时两路本来就一致，**不冲突就不干预** |
| **顶箱子（打滑）** | 轮子转、车不动，ICP 的 dx≈0 但质量只有 0.35 | σ 抬到 0.118 ≈ odom，两路权重相当 → EKF 不会被 odom 单独带走；再配合 B 的门控保住地图 |
| **纯原地旋转堵转** | wz 有指令、轮子不转 | 激光对旋转最敏感（角点结构），σ_w 可以给到 0.05；同时保留角速度判据 |

**冲突时的策略（建议）**：当 `|v_scan − v_odom| > 3σ` 连续 2 帧，判定"两路冲突"，
此时**放大 `/odom_guarded` 的协方差**（我们已有这个机制），而不是抬高 scan 的 σ——
相当于"冲突时相信激光"。这条要作为 A2 配置单独评测。

---

## 7. 离线验证（不碰真车，用今天的两个包）

工具：`tools/replay_ekf.py` —— 读包 → 按时间戳把 `/scan`、`/odom`、`/imu` 发给
`ekf_node`（加载待测参数）→ 记录 `/odometry/filtered`。

| 配置 | 内容 |
|---|---|
| **baseline** | 今天的 `ekf.yaml`（odom 位姿+速度，无 twist0） |
| **A1** | odom 只取速度 + twist0（σ 按上表） |
| **A2** | A1 + 冲突时放大 odom 协方差 |

评估包与真值区间（车体真位移由 `map→odom` ∘ `odom→base` 合成，逐 0.8 s）：

- `manual_1127_1451`：t = 247.6 / 248.4 / 249.2（打滑，真位移 0.141 / 0.110 / 0.072 m）
- `manual_1126_1934`：t = 24.0 ~ 30.0（顶箱子）
- `manual_1127_1449`：全程（负样本，要求"不干预"）

输出一张表：每段 `误差 = |filtered 位移 − 真位移|`，以及正常段的 `filtered−odom` 差。

**通过条件**：打滑段误差 A2 < A1 < baseline，且 1449 全程 A1/A2 与 baseline 差 < 3%。

---

## 8. 风险与回退

| 风险 | 应对 |
|---|---|
| 位移被"双计"（odom 与 scan 都观测位移）→ 过冲 | odom0 只留速度；σ 按上表标定；回放先验 |
| 时间戳不同源（板子时钟 vs VM） | 用 `header.stamp`；两路差 > 0.2 s 直接丢弃并告警 |
| 走廊里 ICP 给出错误位移 | `/odom` 初值 + 质量门 + A2 冲突策略 |
| 上线后反而更差 | `use_scan_twist:=false` 一键回退，行为与今天一致 |

---

## 9. 工期与顺序

| 阶段 | 内容 | 工期 |
|---|---|---|
| S0 | **B 方案**（不确定就不写图）——独立，先上线兜住地图 | 1 天 |
| S1 | `scan_odom` 节点 + 单测（纯函数，不依赖 ROS） | 0.5 天 |
| S2 | `replay_ekf.py` + 三组配置离线对比 | 1 天 |
| S3 | EKF 接线（launch 参数 `use_scan_twist`，默认 false） | 0.5 天 |
| S4 | 真机 A/B（同一路线跑两遍，对比地图与 filtered 轨迹） | 0.5 天 |

---

## 10. 需要你拍板的三件事

1. **是否接受把 `odom0` 从"位姿+速度"降为"只速度"？**
   这是 A 生效的前提；副作用是长距离 yaw 要靠 slam/AMCL 兜（AMCL 场景下影响不大，但需要真机确认）。
2. **冲突时要不要"相信激光"（A2）？**
   即冲突 2 帧后临时放大 `/odom` 协方差到 1.0 量级，把权重交给激光。激进但有效；不喜欢就只做 A1。
3. **B 是否先做？**
   我的建议是先做 B（1 天，风险低，立刻让"图不脏"），再做 A。

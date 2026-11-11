# 离线任务卡

> 现场没有外网时，AI 不在线。这份文件的作用是：**出门前把"要做什么"固定成一张卡，回来把"发生了什么"装进一个包。**

## 怎么用

1. 出门前，我在这份文件里**填一张卡**（下一节的样子）
2. 你照着卡上的命令跑，只看卡上写的"看什么"
3. 卡住就停，拍照，回来
4. 回来把卡上写的"带回来"那几个文件给我

**现场不用 debug，不用猜。** 拿到什么就是什么。

---

## 卡片模板

```md
### 任务 <编号> · <一句话目标>      日期：____

**前提**（上电前确认）
- [ ] ...
- [ ] ...

**跑这一条**
```bash
bash tools/run_xxx.sh --参数
```

**跑的时候看这两个信号**
- 现象 A：____ → 正常的话应该看到 ____
- 现象 B：____ → 正常的话应该看到 ____

**什么情况立刻停**
- ____
- ____

**带回来**
- [ ] /tmp/xxx/xxx.png
- [ ] /tmp/xxx/xxx.log
- [ ] /tmp/diag_*.tar.gz
- [ ] 一句话：____
```

---

## 任务 A0 · 真机建图体检（2026-11-11 发出）

**目标**：确认 `slam_toolbox` 能在真机上建图，并且和 Nav2 共存。

### 前提（上电后、跑之前确认）

- [ ] 雷达上电，终端里 `ros2 topic hz /scan` 有频率输出（约 10 Hz）
- [ ] micro-ROS agent 起来了，`ros2 topic echo /odom --once` 有数据
- [ ] **没有**别的终端在跑 nav2 / amcl / map_server（否则会抢 `map->odom`）
- [ ] 场地里人走开，车周围留 1 米

### 跑这一条

```bash
cd ~/fishbot_full_ws          # 你的工作区
bash tools/run_a0_real.sh --duration 300
```

> 如果它报 `找不到 odom -> base_link 的 TF`，先试试底盘帧的真实名字：
> `ros2 run tf2_tools view_frames` 看一眼树，然后加参数
> `--base-frame base_footprint`（或你实际的帧名）。

### 跑的时候看这两个信号

| 看什么 | 正常 | 不正常 |
|---|---|---|
| **RViz 里的地图** | 灰色区域（未知）随着车走**变大**，墙体线条逐渐清晰 | 一直是全灰、或者一片白没有任何墙 |
| **终端里的 slam 日志** | 偶尔有 `Update`，没有连续报错 | 刷屏 `Failed to compute odom pose` |

> 脚本跑起来之后，可以手动遥控车慢慢走一圈（约 3×4 米），或者用键盘遥控原地转两圈。
> **别开快车**，开快了地图会糊。

### 什么情况立刻停

- 终端开始刷 `Failed to compute odom pose`（帧名不对，跑下去没意义）
- RViz 里 TF 树报红（`map` 或 `odom` 帧不存在）
- 车自己动起来了（说明有别的节点在发 `/cmd_vel`，危险）

停下来之后**照样跑一次诊断包**：

```bash
bash tools/collect_diag.sh /tmp
```

### 带回来

- [ ] `/tmp/a0real/map.png` —— 地图快照
- [ ] `/tmp/a0real/slam.log` —— slam 日志
- [ ] `/tmp/a0real/diag_*.tar.gz` —— 诊断包（脚本自动生成）
- [ ] 一句话：**地图长起来了吗？有没有报错？**

### 判读规则（留给你现场自己判断，不用等 AI）

| 你看到 | 结论 | 下一步 |
|---|---|---|
| 地图长起来了、没报错 | ✅ A0 通过 | 可以开始 A1 |
| 地图长但边缘糊/重影 | ⚠️ 参数问题 | 大概率 `minimum_time_interval` 太小或开太快，回来调 |
| 一直报 `odom pose` | ❌ 帧名或 TF 断 | 加 `--base-frame` 重跑，还不行就带诊断包回来 |
| 全灰没变化 | ❌ 雷达数据没进 slam | 看 `/scan` 的 `frame_id` 和 TF 树 |

---

## 两个工具的说明

### `tools/collect_diag.sh`

一条命令收齐现场状态，输出一个几十 KB 的 `diag_*.tar.gz`。**只读，不起进程，不改任何东西。**

```bash
bash tools/collect_diag.sh            # 打包到 /tmp
bash tools/collect_diag.sh /home/my   # 打包到指定目录
```

收的东西：环境变量与网卡 / 包版本 / 进程与端口 / 节点与话题 / TF 树与 `map→odom` 广播者 /
关键话题频率 / `/map` 统计 / 最近三份巡检报告 / 最近日志的错误汇总。

### `tools/run_a0_real.sh`

A0 专用：体检 → 起 slam → 起 nav2 纯导航部分 → 观察 N 秒 → 存图 → 收诊断 → **按进程组关干净**。

```bash
bash tools/run_a0_real.sh --duration 300          # 完整跑（含 nav2）
bash tools/run_a0_real.sh --duration 180 --no-nav # 只测 slam
bash tools/run_a0_real.sh --base-frame base_footprint
```

退出码：`2` = 没有 `/scan`；`3` = 没有 `odom→base` 的 TF；`4` = 你选择不继续；`5` = slam 没起来。

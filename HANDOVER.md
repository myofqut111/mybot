# HANDOVER · 接手须知

写给"明天开始接这个项目的人"。目标：**半小时内能把它跑起来，一小时内知道该改哪儿。**

| 项目 | 灭火器巡检机器人 |
|---|---|
| 一句话 | 车自己按路线开、到点拍照识别灭火器在不在、跑完出一份带召回率/精确率的报告 |
| 仓库 | `https://github.com/myofqut111/mybot` |
| 演示视频 | `https://www.bilibili.com/video/BV1DJWg6kEAf/`（6 分 19 秒） |
| 当前版本 | console `0.8.0` / patrol_v2 `0.5.0` / safety `0.2.0` / vision `0.2.0` |
| 文档 | `README.md`（总览）、`技术栈.md`、本地《开发手册》《命令手册》 |

---

## 1. 三样东西分别在哪

| 东西 | 位置 | 说明 |
|---|---|---|
| **代码** | Windows `D:\deepseek\mybot_src` = GitHub `myofqut111/mybot` | 唯一真源，改完推这里 |
| **虚拟机** | `my@192.168.1.103`（内网）／`my@192.168.202.128`（能上外网时） | 跑导航、巡检、安全、控制台 |
| **板子（Jetson）** | USB `xmz@192.168.55.1`；内网 `192.168.1.104` | 只跑**视觉**（camera_publisher + yolo_detector） |
| **摄像头** | ESP32，`http://192.168.1.100:81/stream` | 单张抓拍，不走长连接 |

> ⚠️ **代码里的包名是 `mybot_*`，机器上装的是 `fishbot_*`。** 改名是为了给别人看，
> 机器上没有重装（见 §10）。在机器上敲命令请用 `fishbot_`，在仓库里读代码看到的是 `mybot_`。

---

## 2. 环境一览

| 项 | 值 |
|---|---|
| 操作系统 | 虚拟机 Ubuntu 22.04、ROS 2 Humble、Python 3.10 |
| 工作区 | 虚拟机 `~/fishbot_full_ws`、板子 `/home/xmz/fishbot_full_ws` |
| SSH 密钥 | 仓库外保存：`D:\deepseek\_ssh_keys\vm_ed25519`、`board_ed25519` |
| 地图 | `/home/my/maps/map_v2.yaml`（`main2` 等路线在这张图上） |
| 路线库 | `~/.ros/mybot_patrol_routes.json`（老名 `fishbot_...` 会自动迁移） |
| 期望值基线 | `~/.ros/mybot_patrol_expect.json` |
| 报告 | `~/patrol_reports/*.jsonl` + `.summary.json` + `.summary.txt` |
| 断点续跑 | `~/.ros/patrol_progress.json` |

---

## 3. 没有硬件时：先跑 mock（5 分钟）

不需要车、不需要摄像头、不需要板子。

```bash
cd ~/fishbot_full_ws
colcon build --packages-select fishbot_console          # 不要加 --symlink-install
source install/setup.bash
ros2 launch fishbot_console mock.launch.py
```

浏览器开 `http://localhost:8080`，能看到控制台、mock 地图、假机器人位姿。

**注意**：`mock.launch.py` 只提供地图/TF/里程计/假导航，**不提供** route_manager 服务和检测结果。
要跑通"录点 → 巡检 → 报告"完整链路，得另外起 route_manager 和视觉节点（或照抄
`.ros` 里的现成路线库直接跑）。

---

## 4. 真机启动（按顺序，四条栈）

```bash
cd ~/fishbot_full_ws && source install/setup.bash

# 一条命令起全栈（导航 + 巡检 + 控制台）
ros2 launch fishbot_console robot.launch.py \
  map_yaml_file:=/home/my/maps/map_v2.yaml \
  nav:=true patrol:=true rviz:=false http_port:=8080
```

板子上（视觉单独起）：

```bash
ssh xmz@192.168.1.104
cd ~/fishbot_full_ws && source install/setup.bash
ros2 launch fishbot_vision vision.launch.py
```

**起来之后看页面顶部四个灯**：电量、位姿、相机、错误数。

| 灯 | 正常 | 不正常怎么办 |
|---|---|---|
| 电量 | 12.x V 绿 | 11.x 以下先充电；阈值已经是 10.5/10.0（见 §10） |
| 位姿 | 有坐标 | 红 = AMCL 没初始位姿，在 RViz 用 `2D Pose Estimate` 点一下 |
| 相机 | 有画面 | 红 = 摄像头没通电或 IP 变了（`curl http://192.168.1.100:81/capture`） |
| 错误数 | 0 | 大于 0 点开日志面板看 |

### 停止

```bash
pkill -f 'console[.]launch[.]py'      # 先停 launch，否则 respawn 会把节点拉回来
pkill -f 'robot[_]console'
pkill -f 'rosbridge_[w]ebsocket'; pkill -f 'rosapi[_]node'
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*   # 有残留才清，别在进程还活着时清
```

> 括号技巧（`[c]onsole`）是为了让 `pkill` 不匹配到执行它自己那条命令行——
> 直接写 `pkill -f console.launch.py` 会把自己杀掉。

---

## 5. 一次巡检怎么跑

1. 车开到点位 → 页面「录点」填 `路线名 / 任务 / 期望值` → 点「记下当前位置」
2. 重复，凑够一条路线（至少 2 个点才能看出效果）
3. 选路线 → 「开始巡检」
4. 「报告」区实时出结果，点「看摘要」看人读版
5. 跑完自动学基线：**第一次跑只记录，第二次开始才会判定"应该有/应该没有"**

---

## 6. 参数在哪改

三张 yaml，改完 `colcon build` 才生效（装出来是实体文件，不是软链）：

| 文件 | 管什么 | 常改的 |
|---|---|---|
| `src/mybot_console/config/console.yaml` | 控制台端口、报告目录、路线库路径、电量阈值 | `http_port` `reports_dir` `battery_warn_v` `battery_crit_v` |
| `src/mybot_patrol_v2/config/patrol_v2.yaml` | 超时、失败策略、复核、基线、电量 | `waypoint_timeout_s` `failure_policy` `recheck_strategy` `baseline_policy` |
| `src/mybot_vision/config/vision_params.yaml` | 摄像头地址、抓拍频率、压缩参数 | `stream_url` `capture_rate` `compressed_rate` `jpeg_quality` |

**当前关键值**（改之前先看这里，别瞎猜）：

| 参数 | 值 | 说明 |
|---|---|---|
| `battery_warn_v` / `battery_crit_v` | **10.5 / 10.0** | 11-08 从 11.8/11.5 下调；**12V 铅酸请改回 11.8/11.5**，尚未真机复验 |
| `waypoint_timeout_s` | 180 | 单点导航超时 |
| `task_result_timeout_s` | 130 | 等检测结果超时 |
| `failure_policy` | `skip` | 单点失败继续跑下一个（`stop` / `retry`） |
| `max_consecutive_failures` | 3 | 连续 3 点失败整轮中止 |
| `resume` | `true` | 断点续跑，跳过已完成点位 |
| `recheck_strategy` / `recheck_turn_deg` | `turn` / 35 | 没看到目标时原地左右转，不往前顶墙 |
| `baseline_policy` | `fill` | 只补空缺、不覆盖（`update` 会跟随场景变化） |

---

## 7. 故障排查（全是真踩过的）

| 症状 | 原因 | 处置 |
|---|---|---|
| 页面"半启动"，但 RViz 里地图正常 | nav2 只剩孤儿 `lifecycle_manager_localization`，RViz 画的是历史**残影** | 看「启动日志」；`pgrep -af nav2` 确认；重起导航栈 |
| 车原地打转 | 起了**两套** nav2，两个 amcl 抢 `map→odom`、两个 controller 抢 `/cmd_vel` | 0.7.8 起面板会拒绝重复起栈；手动 launch 仍需自己查 `pgrep` |
| 车一顿一顿 | 遥控空闲时也在发 0 速度 | 0.5.5 已修（`teleop_step`）；若复现查 `/cmd_vel` 发布者 |
| 按「停止」说"没有巡检在运行" | 只杀了 launch，执行器无视 SIGTERM | 0.6.3 起按 launch→包目录→节点名三层兜底，TERM 无效升 KILL |
| 开始巡检立刻失败 | 路线名为空 → `Route does not exist`；或点位在图外 | 0.7.6/0.7.7 已改为明确报错，看页面提示 |
| 点位明明存在却报"结果 无点位" | 电量低于 `battery_crit_v`，在第一个点之前就中止 | 0.4.7 起报告写「未开始（原因）」；先看电量 |
| `PackageNotFoundError: fishbot-console` | egg-info 没跟着新版本刷新 | 用 `tools/deploy_console.sh`（会先删再拷 egg-info） |
| `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` | `/dev/shm` 里攒了一堆 fastrtps 段（进程被杀没释放） | 停干净后 `rm -f /dev/shm/fastrtps_*` |
| 连不上板子 | 板子在内网 `192.168.1.104`，笔记本不在同网段 | 用 USB 直连 `192.168.55.1` |
| 网页打不开 | 端口被孤儿 rosbridge 占了 | `ss -ltnp \| grep 9090`，按 PID 杀 |

---

## 8. 代码结构（13 个包）

```
src/
├── mybot_console/          控制台：HTTP 服务 + 页面 + 节点管理（最重要，改得最多）
├── mybot_patrol_v2/        巡检执行器：规划/导航/检测/复核/报告/基线
├── mybot_patrol_interfaces/  action 与 srv 定义（改接口先改这里）
├── mybot_patrol_route_manager/ 路线库：增删点位、取路线
├── mybot_patrol_task_manager/  任务派发：按轮数展开作业
├── mybot_patrol_executor/      旧版执行器（保留，v2 已替代）
├── mybot_vision/           camera_publisher + yolo_detector（跑在板子上）
├── mybot_safety/           看门狗：底盘掉线/没人指挥时刹车
├── mybot_navigation/       nav2 配置与 launch
├── mybot_bringup/          URDF → TF
├── mybot_description/      模型描述
├── ros_serial2wifi/        串口↔WiFi 桥（旧链路，现用 micro-ROS）
└── ydlidar_ros2/           雷达驱动
```

**三个"纯逻辑"模块**（不依赖 ROS，最好改、最好测）：

| 模块 | 职责 |
|---|---|
| `mybot_patrol_v2/plan.py` | 把"路线 × 轮数"展开成作业序列，处理续跑跳过 |
| `mybot_patrol_v2/report.py` | 写 jsonl / summary.json / summary.txt，算召回率精确率 |
| `mybot_console/data.py` | 把所有 JSON 变成页面能用的结构，含电量分档、地图边界、点位解析 |

---

## 9. 测试怎么跑

```bash
cd src/mybot_console    && python3 -m unittest discover -s test   # 285 例
cd src/mybot_patrol_v2  && python3 -m unittest discover -s test   # 184 例（跳 7）
cd src/mybot_safety     && python3 -m unittest discover -s test   # 15 例
```

- **一律用 `discover`**，不要 `grep -c "^test"`（会少算），也不要写两个同名测试类（后者会覆盖前者，这个坑踩过）。
- 改代码前先跑一遍，确认基线是绿的。

---

## 10. 已知限制（别当 bug 修）

1. **机器上装的还是 `fishbot_*`**：仓库改名成 `mybot_*` 只为了对外展示。哪天拿仓库代码往老工作区部署，会出现 `mybot_*` 与 `fishbot_*` 并存、同名节点互抢——**要么全重装，要么别混用**。
2. **4 处 `fishbot` 是有意保留的兼容代码**：三个 `migrate_legacy_path()`（老数据自动迁移）+ `tools/deploy_console.sh` 的 `fishbot_full_ws` 兜底。
3. **电量阈值 10.5/10.0 未经真机复验**：这是为"3S 锂电"调的；12V 铅酸必须改回 11.8/11.5，否则会提前中止。
4. **`--symlink-install` 在本机装出来是实体目录**：改了 src 不编译根本不生效，而且验证还会假装成功。
5. **GitHub 里的 mp4 不会内嵌播放**：只有 README 拖入的附件可以，所以视频放 B 站 + 仓库备份。
6. **现场是隔离内网，没有外网出口**：微信/邮件等推送在物理上发不出去（详见《后续路线方案》B2）。
7. 磁盘 `/dev/sda3` 用了 86%，长期跑要注意。

---

## 11. 接口速查

- **HTTP**：11 个 GET + 13 个 POST，完整清单见本地《命令手册》§11.1
  常用：`/api/state`、`/api/stacks`、`/api/patrol`、`/api/runs`、`/api/report`、`/api/route_names`、`/api/logs`
- **ROS action**：`/patrol/execute`（PatrolExecute）
- **ROS 服务**：`/patrol_route/{get,list,record_point,delete_point,set_active}`、`/patrol/task/{request,result}`
- **关键 topic**：`/camera/camera/image_raw(+compressed)`、`/yolo_detector/image_annotated(+compressed)`、`/cmd_vel`、`/scan`、`/odom`

---

## 12. 交接检查单

上线/交接前逐条打勾：

- [ ] `colcon build` 通过（不加 `--symlink-install`），三套单测全绿
- [ ] 页面四个灯全绿
- [ ] 用 `main2` 跑完整一轮，报告里有召回率/精确率
- [ ] 故意藏一辆灭火器，报告能判"未发现"
- [ ] 中途点「停止」，能在 5 秒内停下并留记录
- [ ] 拔掉摄像头网线，页面相机灯变红且报告写清原因
- [ ] 版本号四处一致（`setup.py` / `package.xml` / 页面 env 行 / `pip show`）
- [ ] 收工：进程 0 个、端口 8080/9090 空闲、`/dev/shm` 无 fastrtps 残留

# fishbot_explore · 自主探索

扔进一个没去过的房间，让车自己走一圈把地图建完。**这里实现的是第一步：只“看得见”，不控制车。**

## 为什么先做这一步

前沿检测是纯算法：`/map` 进来，候选点出去。它**不发导航指令**，所以推理错了最多是 RViz 里红点画歪，
不会撞东西。先把算法调到在你自己的地图上找得准，再谈让车动。

## 名词

| 词 | 意思 |
|---|---|
| 未知 | `OccupancyGrid.data < 0`，没看过的地方 |
| 空闲 | `0 <= data < 50`，走过且确认能过 |
| 障碍 | `data >= 50` |
| **前沿** | 空闲格子中，四邻里至少有一个未知的那些格子——"看过了，但再往外没看"的那条边 |

> 地图**边界之外不算未知**。否则任何地图的四边永远全是前沿点，噪声压不住。

## 跑起来

```bash
# 终端 1：起你的导航栈（要有 /map 话题）
ros2 launch fishbot_console robot.launch.py map_yaml_file:=/home/my/maps/map_v2.yaml rviz:=true

# 终端 2：只跑前沿检测
ros2 launch fishbot_explore explore.launch.py
```

RViz 里加 **MarkerArray**，话题 `/explore/frontiers`：红球是候选点，越大格子越多，文字是 `#序号 分数`。
终端每秒会打一行 `前沿 N 个，最佳 (x, y) 分数 …`。

> 用**静态地图**跑的时候，未知区域是 0（图上画满），所以大概率只在地图外侧找到少量点。
> 真正要验证的是换成 `slam_toolbox` 实时建图后（地图边建边长），前沿点才会跟着扩展的方向冒出来。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_topic` | `/map` | 订阅哪张地图 |
| `marker_topic` | `/explore/frontiers` | RViz 看的红点 |
| `candidates_topic` | `/explore/candidates` | 候选点 PoseArray（下一步给 Nav2 用） |
| `min_size` | 4 | 一个前沿块至少多少格才算数（滤掉噪声） |
| `unknown_radius_m` | 1.0 | 算"去了能看到多少"的半径 |
| `max_candidates` | 20 | 最多发多少个 |
| `w_distance` / `w_gain` | 0.6 / 0.4 | 距离近 vs 未知多，谁更重要 |
| `period_s` | 1.0 | 多久算一次 |

## 代码结构

| 文件 | 干什么 |
|---|---|
| `fishbot_explore/frontier.py` | **纯算法**：`Grid` 包装、`frontier_cells`、`cluster`、`find_frontiers`。不 import ROS，能单测 |
| `fishbot_explore/node.py` | 唯一碰 ROS 的文件：订阅 `/map`、每秒算一次、发 marker + PoseArray |
| `test/test_frontier.py` | 17 例：边界、聚类、坐标换算、质心落墙里、两种权重、上限 |

```bash
cd src/fishbot_explore && PYTHONPATH=. python3 -m unittest discover -s test
```

## 下一步（还没做）

1. **让车自己走过去**：取 `find_frontiers` 的第一个候选，发 `NavigateToPose`，到了就停（先不做循环）
2. **闭环**：走到 → 重新算 → 再去；一个前沿都没有了 → 回起点
3. **防卡死**：连续两次导航失败就拉黑那个点；总时长超限就收工
4. **对比**：和 `m-explore-ros2`（explore_lite）比路径长度和覆盖率

## 已知限制

- 只认四邻接的前沿，斜角上的未知区会漏（够用，先不加）
- 没有可达性检查：候选点可能在墙里或者被家具围住——第 1 步之后再补
- 质心可能落在障碍上，现在是就近挪到空闲格（`nearest_free`），挪不过去就丢弃

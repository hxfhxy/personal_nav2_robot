# personal_nav2_robot

ROS 2 Humble 下的差速小车仿真导航。仿真用 Gazebo Classic 11。默认场景是 `playground`，第二张图是 TurtleBot3 的房子。全局路径是自己写的栅格 A*，路径平滑是自己写的 QP，速度指令是自己写的 DWA。恢复行为、速度平滑和行为树仍用 Nav2 官方插件。

仿真里地图和场地已经对齐，不启动 AMCL。`map → odom` 是固定的单位变换，`odom → base_footprint` 用 Gazebo 真值位姿。

## 环境

- Ubuntu 22.04 / ROS 2 Humble
- Gazebo Classic 11（`gazebo_ros`），不是 Jazzy 的 `gz-sim`

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs
```

## 目录

| 包 | 内容 |
| --- | --- |
| `my_nav2_robot` | 车体 URDF、Gazebo 世界、地图、launch、Nav2 参数、行为树 |
| `my_nav2_planner` | 全局规划插件 `my_nav2_planner/MyAStarPlanner` |
| `my_nav2_controller` | 局部控制插件 `my_nav2_controller/MyDWAController` |
| `my_nav2_smoother` | 路径平滑插件 `my_nav2_smoother/MyQpSmoother` |

行为树里的插件名仍是 `GridBased` 和 `FollowPath`，对应关系写在 `config/nav2_params.yaml` 的 `plugin:` 上。

## 如何启动和使用

每个新终端都要先加载环境。改过 C++ 插件后要重新编译；只改了 yaml、launch、urdf 或 rviz，因为用了 `--symlink-install`，重新 launch 即可。

```bash
cd /home/hzy/个人项目/personal_nav2_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 导航

同一时间只能有一个 `gzserver`。旧进程还占着端口时，新的 Gazebo 会马上退出，仿真和 RViz 里都不会出现车。先关掉上一轮的 Gazebo 和 RViz，再启动：

```bash
ros2 launch my_nav2_robot full_navigation.launch.py
```

这次会打开 Gazebo 和 RViz。默认场景是 `playground`，车生成在场地中心 `(0, 0)`。Nav2 在 launch 约 5 秒后才起来，等 RViz 里地图、车体和激光都出现再发目标。RViz 的 Fixed Frame 是 `map`。俯视图里 +X 朝上，+Y 朝左。

`playground` 里有四个 0.5 m 的方块，位置大约是 `(2, 0)`、`(2, 2)`、`(-2, 2)`、`(-2, -2)`，以及 `y = 3` 附近的一堵墙。目标点要落在空白格子上。

第二张图是 TurtleBot3 的房子，墙和走廊都比空场地多。地图由房子碰撞体栅格化得到，和 Gazebo 里的墙对齐。需要已安装 `ros-humble-turtlebot3-gazebo`。车生成在 `(-2.0, -0.8)`，不要再放回原点，那里贴着墙。

```bash
ros2 launch my_nav2_robot full_navigation.launch.py scene:=house
```

在 RViz 顶部工具栏选 **Nav2 Goal**，在地图上单击落点，按住拖出车头朝向后松开。行为树会调用 A* 规划，红线是全局路径 `/plan`，车在 Gazebo 里按 DWA 开过去。再点一个新目标会替换当前任务。停下当前任务用左侧 **Navigation 2** 面板里的 Cancel。

也可以在另一个已经 `source` 过的终端发目标，例如开到 `(1, -1)`：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

常用 launch 参数：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `scene` | `playground` | `playground` 四个方块，`house` TurtleBot3 房子 |
| `gui` | `true` | 是否打开 Gazebo 窗口 |
| `rviz` | `true` | 是否打开 RViz |
| `use_sim_time` | `true` | 全栈使用仿真时钟 |
| `world` | 空 | 非空时替换该场景的 Gazebo 世界 |
| `map` | 空 | 非空时替换该场景的地图 YAML |
| `params_file` | `config/nav2_params.yaml` | Nav2 参数 |
| `slam` | `False` | `True` 时改为建图，见下一节 |

工作区路径含中文时，launch 会把地图拷到 `/tmp/my_nav2_robot_map`，避免 Nav2 的 `RewrittenYaml` 把路径写坏。

无界面跑导航：

```bash
ros2 launch my_nav2_robot full_navigation.launch.py gui:=false rviz:=false
```

### 只看车或只开仿真

不加载地图和 Nav2，只在 RViz 里看模型：

```bash
ros2 launch my_nav2_robot display.launch.py
```

只开 Gazebo，并把模型 RViz 也打开：

```bash
ros2 launch my_nav2_robot gazebo_sim.launch.py use_rviz:=true
```

### 建图

`slam:=True` 时启动 slam_toolbox，不发布固定的 `map → odom`，也不开 map_server。在 RViz 里用键盘或速度指令把车开起来，地图才会扫出来。

```bash
ros2 launch my_nav2_robot full_navigation.launch.py slam:=True
```

## 坐标系

```
map → odom → base_footprint → base_link → laser_link
```

| 变换 | 谁在发 | 说明 |
| --- | --- | --- |
| `map → odom` | `static_transform_publisher` | 单位变换。导航模式下不启 AMCL |
| `odom → base_footprint` | `ground_truth_tf.py` | 订阅 Gazebo `libgazebo_ros_p3d` 的 `/ground_truth`，原样发成 TF |
| `/odom` | 差速轮插件 | 只提供速度，`publish_odom_tf` 为 false |
| `base_footprint` 以下 | `robot_state_publisher` | 固定关节，雷达在 `laser_link` |

轮式里程计会打滑。地图又钉在 `odom` 上时，打滑会把激光点拖离墙面，代价地图就在空白处留下障碍。仿真阶段因此改用真值位姿。

## 车和雷达

差速两轮加后万向轮，车体约 0.40 m × 0.30 m × 0.15 m，轮半径 0.06 m。二维雷达水平安装，话题 `/scan`，消息类型 `sensor_msgs/LaserScan`，量程 0.12–8.0 m。

底盘中心在轮轴后方 0.08 m，离地间隙 0.04 m。重心若压在轮轴上，车头会压到地面，雷达跟着低头，2.5 m 以内的回波打在地面和车身上。

## 现在用的模块

| 模块 | 插件 |
| --- | --- |
| 全局规划 `GridBased` | `my_nav2_planner/MyAStarPlanner` |
| 局部控制 `FollowPath` | `my_nav2_controller/MyDWAController` |
| 路径平滑 `qp_smoother` | `my_nav2_smoother/MyQpSmoother` |
| 恢复 | `Spin`、`BackUp`、`Wait` 等官方行为 |
| 地图 | `map_server`，不启 AMCL |
| 建图 | `slam_toolbox`，仅 `slam:=True` 时启动 |

`nav2_params.yaml` 里还留着 AMCL 参数，launch 不会加载这个节点。

## 自己写的算法

A* 在全局代价地图上做八邻域搜索。代价大于等于 253（贴身膨胀和致命障碍）的格子不可走，未知格子先当可走。斜向移动时，两侧直线邻居有墙就剪掉，避免穿墙角。启发函数是 octile 距离，步代价里加了一点格子代价值，让路径少贴着膨胀走。输出是栅格中心连成的折线，中间点朝向还是单位四元数，终点朝向用目标朝向。

平滑在 A* 和 DWA 之间。未知数就是折线上的路径点，x 和 y 各解一次。代价是两项二次项：路径点贴近原来的 A* 点，以及相邻三点的二阶差分 `x_{i-1} - 2 x_i + x_{i+1}`。起点和终点固定，所以这个 QP 变成线性方程组。解出的点再当作三次 B 样条的控制点，夹紧结点让曲线从起点收到终点。曲线扫进代价大于等于 253 的格子，就把控制点往原路径退；仍然进墙就退回 A* 折线。输出按 0.1 m 取样，并给每个点补上沿路径的朝向。

DWA 按当前速度和加速度开动态窗口，采样 `(vx, wz)`，向前滚约 0.8 s。轨迹上任一点的代价到 253 就丢掉这条轨迹。碰撞只查车体中心。打分看终点离前方路径有多远、朝向差多少，再加一点线速度奖励，并稍微偏向更直的角速度。还没有离障碍远近和足迹碰撞。

代价地图两边都是静态层、激光障碍层和膨胀层。膨胀半径 0.55 m。激光标记距离 2.5 m，清除距离 3.0 m。A* 只把 253 及以上当成墙，膨胀层里大部分格子仍然可走。局部代价地图是 `odom` 下 4 m × 4 m 的滚动窗口。

## 下一步

QP 的未知数仍是路径点，B 样条只负责把这些点画成曲线，还没有曲率上限，障碍也还不是不等式约束。后面再慢慢收：给二次代价加上曲率或离障碍的距离，或者直接把未知数换成控制点。DWA 还没有离障碍远近和足迹碰撞。离开 Gazebo 真值时，再开 AMCL 发布 `map → odom`。

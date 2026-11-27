# personal_nav2_robot

用于学习 Nav2 的差速小车仿真，环境是 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。

当前先打通 **三维雷达 → GICP 里程计 → AMCL 定位 → Nav2 → 底盘**，再逐项替换规划、平滑和控制算法。默认跑自己写的 A*、QP 和 DWA；官方 Navfn、SimpleSmoother、DWB 保留下来做对照，可以单独切回去。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
cd /home/hzy/个人项目/personal_nav2_robot
colcon build --symlink-install
source install/setup.bash
ros2 launch my_nav2_robot full_navigation.launch.py
```

依赖安装：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-gazebo
```

GICP 的 small_gicp v1.0.0 在第一次编译时由 CMake FetchContent 下载。前端默认采用 Release 优化构建，让配准跟上 10 Hz 雷达。改 C++ 后重新编译；通过 symlink 安装的 YAML、launch、URDF 和 RViz 配置修改后重新启动即可。

同一时间只运行一套 Gazebo。启动后等地图、激光和 Nav2 就绪，在 RViz 顶部使用 **Nav2 Goal** 指定位置和朝向。需要修正定位初值时使用 **2D Pose Estimate**。默认初始位姿按场景出生点设置，只在初始化时使用，导航过程不使用真值纠正 GICP。

仿真默认使用 local_only:=true，只接收本机 ROS 数据，避免局域网其他仿真的 /clock、/map、TF 和模型描述串入。新开终端发送目标或读取话题时也先执行 export ROS_LOCALHOST_ONLY=1。有意连接远程 ROS 节点时才使用 local_only:=false。同机运行其他 ROS 项目时，可在相关终端统一设置独立的 ROS_DOMAIN_ID。

无窗口且无 GPU / 显示服务时，改用 CPU 光线雷达：

```bash
ros2 launch my_nav2_robot full_navigation.launch.py gui:=false rviz:=false lidar_type:=ray
```

命令行下发目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.0}, orientation: {w: 1.0}}}}"
```

## 当前链路

- Gazebo 的同一颗雷达同时输出三维点云 /points 和中心线二维 /scan。/points 用于 GICP 和代价地图，/scan 用于 AMCL 或 slam_toolbox。
- GICP 从 URDF 的静态 TF 读取雷达到底盘的外参，发布底盘 /odom 和 odom → base_footprint；包含平面位姿和底盘坐标系下的速度。/odom 的原点是第一帧的底盘位置。
- AMCL 结合已有地图、/scan 和里程计，发布 map → odom。正常导航不发布固定的 map → odom。
- 行为树依次调用全局规划、路径平滑和路径跟踪，并执行重规划、清图、旋转、等待、后退等恢复行为。
- 控制器输出 /cmd_vel_nav，官方 velocity_smoother 输出 /cmd_vel，Gazebo 差速插件执行。

导航使用的 TF 链是 map → odom → base_footprint → base_link → laser_link。map → odom 由 AMCL 提供；odom → base_footprint 由选定的里程计提供，两者职责不同。

三维雷达为 16 线、水平 720 点、10 Hz、垂直 ±15°、量程 0.30–12 m。Gazebo 的 LaserScan 输出取中心垂直线；16 线均匀采样时中心线接近水平。车体约 0.40 × 0.30 × 0.15 m，轮半径 0.06 m。代价地图通过高度阈值排除地面点。车体中心在轮轴后方 0.08 m，车尾达到 x=-0.28 m；导航足迹向后覆盖到 -0.30 m，以清除车尾自身的雷达回波。

## 一次替换一个模块

行为树中的实例名始终是 GridBased、SmoothPath、FollowPath，启动参数只替换它们背后的插件类。

不加参数时三个模块都是自写的：全局规划 `my_nav2_planner/MyAStarPlanner`、局部控制 `my_nav2_controller/MyDWAController`、路径平滑 `my_nav2_smoother/MyQpSmoother`。

```bash
ros2 launch my_nav2_robot full_navigation.launch.py
```

要拿官方插件做对照，单独换回一个：

```bash
ros2 launch my_nav2_robot full_navigation.launch.py planner:=navfn
ros2 launch my_nav2_robot full_navigation.launch.py smoother:=simple
ros2 launch my_nav2_robot full_navigation.launch.py controller:=dwb
```

三个可以任意组合：planner:=astar|navfn、smoother:=qp|simple、controller:=dwa|dwb。自写 DWA 每周期把 map 下的路径变换到局部代价地图的 odom 坐标系，再进行路径匹配。

## 真值对照模式

/ground_truth 始终保留用于评估，GICP 不从它读取运动初值或纠正估计。/wheel_odom 是差速插件的原始输出，也不直接驱动默认导航。

需要复现已知地图和理想位姿下的导航时：

```bash
ros2 launch my_nav2_robot full_navigation.launch.py \
  odom_source:=ground_truth localization:=ground_truth
```

这个模式由 ground_truth_tf.py 同时发布 /odom 和 odom → base_footprint，map → odom 为单位变换，AMCL 不启动。GICP 节点也不启动，避免同一 TF 有两个发布者。可以追加 planner、controller、smoother 参数作算法对照。

odom_source:=ground_truth 单独使用时，仍通过 AMCL 定位；localization:=ground_truth 必须与真值里程计配对。

## 场景和地图

只保留 TurtleBot3 房子一个场景（依赖 `ros-humble-turtlebot3-gazebo`）。车出生在 (-2.0, -0.8)，原点贴着墙，不要放回 (0, 0)；AMCL 初始位姿同步使用该值，GICP 的 odom 仍从局部零点起算。

```bash
ros2 launch my_nav2_robot full_navigation.launch.py
```

可以通过 world:=... 和 map:=... 覆盖资源。自定义地图的坐标原点若不同，使用 RViz 的 2D Pose Estimate 设置正确初始位置。

地图、BT 和运行参数会复制到独立的 /tmp/my_nav2_robot_* 目录，避免 Humble 的参数重写处理中文路径时出错。

## 建图和单独仿真

```bash
ros2 launch my_nav2_robot full_navigation.launch.py slam:=True
```

建图时 map → odom 由 slam_toolbox 发布，不启动 AMCL 或静态 map_server；里程计仍由 GICP 提供。/scan 已由雷达输出。这里接通建图入口，建图质量和覆盖率还需要单独验证。

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/my_nav2_map
```

退出建图后，用 map:=/tmp/my_nav2_map.yaml 启动定位导航，再根据保存地图的坐标系设置初始位姿。

只看模型：ros2 launch my_nav2_robot display.launch.py。

只开仿真：ros2 launch my_nav2_robot gazebo_sim.launch.py use_rviz:=true。单独仿真默认使用真值里程计，也可追加 odom_source:=gicp。

## 包与实现

- my_nav2_robot：URDF、Gazebo 世界、地图、launch、Nav2 参数、行为树、RViz。
- my_nav2_frontend：先在底盘坐标系中过滤车身包围盒内的回波和低于 0.05 m 的地面点，再做 PCL VoxelGrid（0.15 m）和 small_gicp 相邻帧配准。这是平地仿真的预处理，相关参数为 min_registration_height、self_filter_min、self_filter_max。订阅队列只保留最新一帧，避免计算繁忙时积压旧位姿。/lidar_odom 保留雷达原始累积位姿用于调试；/odom 是经过外参补偿和平面投影的底盘里程计。失败帧不更新参考帧、不发布新的导航位姿，下一次配准仍相对于最后成功帧。
- my_nav2_planner：八邻域 A*，octile 启发函数，禁止斜穿墙角；代价 ≥253 的已知格子不可走，未知区暂时允许通过。
- my_nav2_smoother：以原路径点为变量，固定首尾，对参考偏差和二阶差分建立二次代价；x、y 分别用高斯消元求解，然后作为三次 B 样条控制点。曲线碰撞时回退，输出间距 0.1 m。
- my_nav2_controller：基础 DWA。采样 (vx, wz)、预测 0.8 s 轨迹、撞到致命格的丢掉，剩下的按「离路径多远、沿路径剩余长度、朝向差多少、速度大小」四项加权打分，权重在 `FollowPath` 下面。剩余长度按路径弧长算，不用到终点的直线距离——房子里的路径要绕墙，拐角处直着走反而离终点更远，用直线距离会让车停在拐角。位置进入 `goal_tolerance` 之后停止平移、只原地转到目标朝向，避免在终点附近来回蹭。碰撞检查从轨迹第 1 个点开始（第 0 点是当前位置，算进去会死锁）；车心已在致命格或所有候选都被判撞时进入脱困模式，允许以 `escape_reverse_speed` 倒车并按末端代价下降选方向。碰撞检查中心点用 253、车体轮廓用 254：253 是膨胀带，轮廓点落进去只说明离得近；254 才是真压到障碍。轮廓按 5 cm 在边上插点，避免细桌腿从两个角之间漏过去，`check_footprint: false` 可退回只查中心。代价地图的 `min_obstacle_height` 取 0.25 m（高于车壳顶 0.19 m，滤掉自车回波，桌腿和墙仍在），`obstacle_min_range` 取 0.10 m（否则贴到 0.3 m 内的桌腿会被丢掉）。

GICP 仍是帧间里程计，没有子地图、回环或 IMU 融合；全局漂移由 AMCL 校正。里程计中的固定协方差只是仿真初值，不是完整的不确定度估计。当前重点是先验证定位导航链路，然后再分别改进这些算法。

## 验证

合成点云回归检查（不需要 Gazebo）：

```bash
python3 src/my_nav2_frontend/test/test_gicp_odometry.py
```

这个检查向实际 GICP 节点输入已知运动的点云，检查外参补偿、平面位姿、速度、TF，以及配准失败后的恢复，不发布 /ground_truth。

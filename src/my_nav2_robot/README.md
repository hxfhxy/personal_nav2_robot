# my_nav2_robot

Humble 最小可跑栈：Gazebo Classic 11 + RViz2 + 官方 Nav2 插件。
结构对齐 `Personal_Nav2_Robot` 的核心包 `my_nav2_robot`，插件先全部用官方最基础的，之后再按同样方式换成自己的。

## 环境

- Ubuntu 22.04 / ROS 2 Humble
- Gazebo Classic 11（`gazebo_ros`），不是 Jazzy 的 `gz-sim` / `ros_gz`
- `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs`

## 启动

```bash
cd /home/hzy/个人项目/personal_nav2_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select my_nav2_robot
source install/setup.bash
```

只看模型：

```bash
ros2 launch my_nav2_robot display.launch.py
```

只开仿真：

```bash
ros2 launch my_nav2_robot gazebo_sim.launch.py use_rviz:=true
```

已知地图导航（`playground`）：

```bash
ros2 launch my_nav2_robot full_navigation.launch.py
```

SLAM 建图：

```bash
ros2 launch my_nav2_robot full_navigation.launch.py slam:=True
```

RViz2 里用 Nav2 面板或 GoalTool 发目标。TF 为 `map -> odom -> base_footprint -> base_link`。

## 官方插件（当前）

| 模块 | 名称 | 插件 |
| --- | --- | --- |
| 全局规划 | `GridBased` | `nav2_navfn_planner/NavfnPlanner` |
| 局部控制 | `FollowPath` | `dwb_core::DWBLocalPlanner` |
| 平滑 | `simple_smoother` | `nav2_smoother::SimpleSmoother` |
| 行为树 | 包内 XML | Humble 默认 `navigate_to_pose_w_replanning_and_recovery.xml` |
| 定位 | AMCL | `nav2_amcl::DifferentialMotionModel` |
| 建图 | slam_toolbox | `solver_plugins::CeresSolver` |

换自己的插件时改两处，和参考项目一样：

1. `config/nav2_params.yaml` 里的 `plugin:` 字符串
2. `behavior_trees/*.xml` 里的 `planner_id` / `controller_id`

以后新增包可以沿用参考项目命名：`my_nav2_planner`、`my_nav2_controller`、`my_nav2_smoother`、`my_nav2_bt_nodes`。

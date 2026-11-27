"""启动 Gazebo Classic、加载 URDF，并选择 GICP 或真值里程计。

不含地图和 Nav2，可以单独用来验证模型、雷达和 TF 是否正常。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_robot')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    odom_source = LaunchConfiguration('odom_source')
    lidar_type = LaunchConfiguration('lidar_type')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' lidar_type:=', lidar_type]), value_type=str)

    # 同一时间只能有一个 gzserver，旧进程占着端口时新的会马上退出
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'pause': 'false',
            'verbose': 'false',
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    # 二选一，避免多个节点竞争 odom -> base_footprint。
    ground_truth_tf = Node(
        package='my_nav2_robot',
        executable='ground_truth_tf.py',
        name='ground_truth_tf',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", odom_source, "' == 'ground_truth'"])),
    )
    gicp_frontend = Node(
        package='my_nav2_frontend', executable='gicp_frontend', name='gicp_frontend',
        parameters=[{'use_sim_time': use_sim_time, 'publish_navigation_odom': True}],
        output='screen',
        condition=IfCondition(PythonExpression(["'", odom_source, "' == 'gicp'"])),
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_nav2_robot',
            '-x', x_pose, '-y', y_pose, '-z', z_pose,
            '-timeout', '120',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'model.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'playground.sdf'),
            description='Gazebo Classic world SDF'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('local_only', default_value='true', choices=['true', 'false']),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', PythonExpression([
            "'1' if '", LaunchConfiguration('local_only'), "' == 'true' else '0'"])),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('odom_source', default_value='ground_truth',
                              choices=['gicp', 'ground_truth']),
        DeclareLaunchArgument('lidar_type', default_value='gpu_ray', choices=['gpu_ray', 'ray']),
        # ROS 环境未必 source 过 Gazebo setup.sh；house 的 model://ground_plane 必须可解析。
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models'),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        gzserver,
        gzclient,
        robot_state_publisher,
        ground_truth_tf,
        gicp_frontend,
        spawn_entity,
        rviz_node,
    ])

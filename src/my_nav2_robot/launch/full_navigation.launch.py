"""完整导航入口。

串起 Gazebo、robot_state_publisher、真值 TF、GICP 雷达前端、Nav2 和 RViz。
`slam:=True` 时用 slam_toolbox 取代 map_server 和固定的 map -> odom。
"""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _cache_map(map_yaml):
    """Copy a map next to its image under /tmp.

    nav2 RewrittenYaml corrupts non-ASCII paths, and the workspace path is Chinese.
    """
    map_yaml = os.path.abspath(map_yaml)
    cache_dir = os.path.join(tempfile.gettempdir(), 'my_nav2_robot_map')
    os.makedirs(cache_dir, exist_ok=True)
    image_name = os.path.basename(map_yaml).replace('.yaml', '.pgm')
    with open(map_yaml, 'r', encoding='utf-8') as source:
        lines = source.readlines()
    for line in lines:
        if line.startswith('image:'):
            image_name = line.split(':', 1)[1].strip()
            break
    image_path = image_name if os.path.isabs(image_name) else os.path.join(
        os.path.dirname(map_yaml), image_name)
    cached_image = os.path.basename(image_path)
    cached_yaml = os.path.join(cache_dir, os.path.basename(map_yaml))
    shutil.copy2(image_path, os.path.join(cache_dir, cached_image))
    with open(cached_yaml, 'w', encoding='utf-8') as target:
        for line in lines:
            if line.startswith('image:'):
                target.write(f'image: {cached_image}\n')
            else:
                target.write(line)
    return cached_yaml


def _scene_paths(context, pkg_share):
    """Pick the Gazebo world, map, and spawn for playground or house."""
    scene = LaunchConfiguration('scene').perform(context).strip()
    world_override = LaunchConfiguration('world').perform(context).strip()
    map_override = LaunchConfiguration('map').perform(context).strip()
    if scene == 'house':
        world = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'worlds', 'turtlebot3_house.world')
        map_yaml = os.path.join(pkg_share, 'maps', 'house.yaml')
        x_pose, y_pose = '-2.0', '-0.8'
    else:
        world = os.path.join(pkg_share, 'worlds', 'playground.sdf')
        map_yaml = os.path.join(pkg_share, 'maps', 'playground.yaml')
        x_pose, y_pose = '0.0', '0.0'
    if world_override:
        world = world_override
    if map_override:
        map_yaml = map_override
    return world, _cache_map(map_yaml), x_pose, y_pose


def _launch_setup(context):
    pkg_share = get_package_share_directory('my_nav2_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    world, map_yaml, x_pose, y_pose = _scene_paths(context, pkg_share)

    slam = LaunchConfiguration('slam')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    nav2_launch_dir = os.path.join(nav2_bringup, 'launch')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'use_rviz': 'false',
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items(),
    )

    # 只开 map_server。AMCL 不启动，避免它用激光改 map -> odom。
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        condition=UnlessCondition(slam),
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        condition=UnlessCondition(slam),
    )
    # Nav2 的 planner / controller / smoother 都自己起，但行为树里的插件名还是官方的，
    # 实际调用哪个类由 config/nav2_params.yaml 的 plugin: 决定。
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'slam_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    # 导航时把 map 钉在 odom 上。没有 AMCL 时这条就是唯一的 map -> odom。
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(slam),
    )

    # 相邻两帧 /points 做 GICP。结果在 lidar_odom，不改 map -> odom。
    gicp_frontend = Node(
        package='my_nav2_frontend',
        executable='gicp_frontend',
        name='gicp_frontend',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'points_topic': '/points',
            'ground_truth_topic': '/ground_truth',
            'voxel_leaf': 0.15,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return [
        gazebo_sim,
        gicp_frontend,
        static_map_to_odom,
        TimerAction(period=5.0, actions=[map_server, map_lifecycle, navigation, slam_cmd]),
        rviz_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_robot')
    return LaunchDescription([
        DeclareLaunchArgument(
            'scene',
            default_value='playground',
            description='playground: four boxes. house: turtlebot3 house.'),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='True: slam_toolbox mapping. False: map_server, no AMCL.'),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Override the scene world. Empty uses scene.'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Override the scene map YAML. Empty uses scene.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=_launch_setup),
    ])

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _cache_map_files(pkg_share):
    """Copy map files to /tmp so nav2 RewrittenYaml does not corrupt non-ASCII paths."""
    cache_dir = os.path.join(tempfile.gettempdir(), 'my_nav2_robot_map')
    os.makedirs(cache_dir, exist_ok=True)
    for map_file in ('playground.yaml', 'playground.pgm'):
        source = os.path.join(pkg_share, 'maps', map_file)
        if os.path.exists(source):
            shutil.copy2(source, os.path.join(cache_dir, map_file))
    return os.path.join(cache_dir, 'playground.yaml')


def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    default_map = _cache_map_files(pkg_share)

    slam = LaunchConfiguration('slam')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='True: slam_toolbox mapping. False: map_server, no AMCL.'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'playground.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Map YAML. Copied to /tmp to avoid non-ASCII RewrittenYaml bugs.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),
        gazebo_sim,
        static_map_to_odom,
        TimerAction(period=5.0, actions=[map_server, map_lifecycle, navigation, slam_cmd]),
        rviz_node,
    ])

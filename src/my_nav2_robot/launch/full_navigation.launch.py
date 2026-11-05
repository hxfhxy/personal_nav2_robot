import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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

    slam_for_bringup = PythonExpression([
        "'True' if '", slam, "'.lower() in ('true', '1') else 'False'"
    ])

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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': slam_for_bringup,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
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
            description='True: slam_toolbox mapping. False: AMCL + map_server.'),
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
        TimerAction(period=5.0, actions=[nav2]),
        rviz_node,
    ])

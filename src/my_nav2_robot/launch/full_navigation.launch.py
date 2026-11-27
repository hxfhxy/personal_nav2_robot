"""GICP odometry + AMCL localization. Planner/controller/smoother default to my own plugins."""
import os
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _cache_map(map_yaml, cache_dir):
    """Cache resources at ASCII paths for Humble parameter rewriting."""
    with open(map_yaml, encoding='utf-8') as source:
        data = yaml.safe_load(source)
    image = os.path.expanduser(data['image'])
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), image)
    data['image'] = 'map_image' + os.path.splitext(image)[1]
    shutil.copy2(image, os.path.join(cache_dir, data['image']))
    cached_yaml = os.path.join(cache_dir, 'map.yaml')
    with open(cached_yaml, 'w', encoding='utf-8') as target:
        yaml.safe_dump(data, target)
    return cached_yaml


def _scene_paths(context, pkg_share):
    scene = LaunchConfiguration('scene').perform(context)
    if scene == 'house':
        world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                             'worlds', 'turtlebot3_house.world')
        x_pose, y_pose = '-2.0', '-0.8'
    else:
        world = os.path.join(pkg_share, 'worlds', 'playground.sdf')
        x_pose, y_pose = '0.0', '0.0'
    world = LaunchConfiguration('world').perform(context).strip() or world
    map_yaml = LaunchConfiguration('map').perform(context).strip() or os.path.join(
        pkg_share, 'maps', scene + '.yaml')
    return world, map_yaml, x_pose, y_pose


def _prepare_params(context, pkg_share, cache_dir, map_yaml, x_pose, y_pose):
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as source:
        params = yaml.safe_load(source)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    def set_clock(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'ros__parameters':
                    child['use_sim_time'] = use_sim_time
                set_clock(child)
    set_clock(params)
    choices = {
        'planner': ('planner_server', 'GridBased', {
            'navfn': 'nav2_navfn_planner/NavfnPlanner',
            'astar': 'my_nav2_planner/MyAStarPlanner'}),
        'controller': ('controller_server', 'FollowPath', {
            'dwb': 'dwb_core::DWBLocalPlanner',
            'dwa': 'my_nav2_controller/MyDWAController'}),
        'smoother': ('smoother_server', 'SmoothPath', {
            'simple': 'nav2_smoother::SimpleSmoother',
            'qp': 'my_nav2_smoother/MyQpSmoother'}),
    }
    for argument, (server, instance, plugins) in choices.items():
        params[server]['ros__parameters'][instance]['plugin'] = plugins[
            LaunchConfiguration(argument).perform(context)]
    if map_yaml:
        params['map_server']['ros__parameters']['yaml_filename'] = _cache_map(map_yaml, cache_dir)
    amcl = params['amcl']['ros__parameters']
    amcl['tf_broadcast'] = True
    amcl['initial_pose']['x'] = float(x_pose)
    amcl['initial_pose']['y'] = float(y_pose)
    bt = params['bt_navigator']['ros__parameters']
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        path = bt[key].replace('$(find-pkg-share my_nav2_robot)', pkg_share)
        cached_bt = os.path.join(cache_dir, os.path.basename(path))
        shutil.copy2(path, cached_bt)
        bt[key] = cached_bt
    cached_params = os.path.join(cache_dir, 'nav2_params.yaml')
    with open(cached_params, 'w', encoding='utf-8') as target:
        yaml.safe_dump(params, target, allow_unicode=True)
    return cached_params


def _launch_setup(context):
    pkg_share = get_package_share_directory('my_nav2_robot')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    world, map_yaml, x_pose, y_pose = _scene_paths(context, pkg_share)
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    localization = LaunchConfiguration('localization').perform(context)
    odom_source = LaunchConfiguration('odom_source').perform(context)
    if not slam and localization == 'ground_truth' and odom_source != 'ground_truth':
        raise RuntimeError('Fixed map -> odom requires odom_source:=ground_truth; use AMCL with GICP.')
    cache_dir = tempfile.mkdtemp(prefix='my_nav2_robot_')
    params_file = _prepare_params(context, pkg_share, cache_dir,
                                  None if slam else map_yaml, x_pose, y_pose)
    use_sim_time = LaunchConfiguration('use_sim_time')
    selected = '/'.join(LaunchConfiguration(key).perform(context)
                        for key in ('planner', 'controller', 'smoother'))
    communication = ('localhost' if LaunchConfiguration('local_only').perform(context) == 'true'
                     else 'network')
    nodes = [
        LogInfo(msg=f'Odometry: {odom_source}; localization: {"slam" if slam else localization}; '
                    f'planner/controller/smoother: {selected}; ROS communication: {communication}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')),
            launch_arguments={
                'world': world, 'gui': LaunchConfiguration('gui'), 'use_rviz': 'false',
                'use_sim_time': use_sim_time, 'x_pose': x_pose, 'y_pose': y_pose,
                'odom_source': odom_source, 'lidar_type': LaunchConfiguration('lidar_type'),
                'local_only': LaunchConfiguration('local_only'),
            }.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav2.rviz')],
             parameters=[{'use_sim_time': use_sim_time}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    delayed = []
    if slam:
        delayed.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'slam_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file,
                              'autostart': 'true'}.items()))
    else:
        delayed.append(Node(package='nav2_map_server', executable='map_server', name='map_server',
                            output='screen', parameters=[params_file]))
        lifecycle_nodes = ['map_server']
        if localization == 'amcl':
            delayed.append(Node(package='nav2_amcl', executable='amcl', name='amcl',
                                output='screen', parameters=[params_file]))
            lifecycle_nodes.append('amcl')
        else:
            nodes.append(Node(
                package='tf2_ros', executable='static_transform_publisher', name='static_map_to_odom',
                arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
                parameters=[{'use_sim_time': use_sim_time}]))
        delayed.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': lifecycle_nodes}]))
    delayed.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file,
                          'autostart': 'true', 'use_composition': 'False'}.items()))
    nodes.append(TimerAction(period=5.0, actions=delayed))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_robot')
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='playground', choices=['playground', 'house']),
        DeclareLaunchArgument('slam', default_value='False', choices=['True', 'False', 'true', 'false']),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('local_only', default_value='true', choices=['true', 'false'],
                              description='Limit this simulation to localhost to avoid foreign maps, TF and clocks.'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', PythonExpression([
            "'1' if '", LaunchConfiguration('local_only'), "' == 'true' else '0'"])),
        DeclareLaunchArgument('odom_source', default_value='gicp', choices=['gicp', 'ground_truth']),
        DeclareLaunchArgument('localization', default_value='amcl', choices=['amcl', 'ground_truth']),
        DeclareLaunchArgument('lidar_type', default_value='gpu_ray', choices=['gpu_ray', 'ray']),
        DeclareLaunchArgument('planner', default_value='astar', choices=['astar', 'navfn'],
                              description='astar: my_nav2_planner/MyAStarPlanner, navfn: 官方 NavfnPlanner'),
        DeclareLaunchArgument('controller', default_value='dwa', choices=['dwa', 'dwb'],
                              description='dwa: my_nav2_controller/MyDWAController, dwb: 官方 DWB'),
        DeclareLaunchArgument('smoother', default_value='qp', choices=['qp', 'simple'],
                              description='qp: my_nav2_smoother/MyQpSmoother, simple: 官方 SimpleSmoother'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=_launch_setup),
    ])

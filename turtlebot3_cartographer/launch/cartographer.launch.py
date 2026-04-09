# cartographer.launch.py — namespace opzionale, senza remap vuoti

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    pkg = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir', default=os.path.join(pkg, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='turtlebot3_lds_2d.lua'
    )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    rviz_config = os.path.join(pkg, 'rviz', 'tb3_cartographer.rviz')
    occ_grid_launch = os.path.join(pkg, 'launch', 'occupancy_grid.launch.py')

    decl = [
        DeclareLaunchArgument('namespace', default_value=namespace, description='ROS namespace'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz),
    ]

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
    )

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(occ_grid_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    group = GroupAction([
        PushRosNamespace(namespace),
        cartographer_node,
        occupancy_grid,
        rviz
    ])

    return LaunchDescription(decl + [group])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('rovpang_navigation')
    map_file = os.path.join(pkg_dir, 'maps', 'rovpang_map.yaml')
    waypoints_file = os.path.join(pkg_dir, 'config', 'waypoints.yaml')

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigation'), 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'map': map_file}.items(),
    )

    goto_node = Node(
        package='rovpang_navigation',
        executable='goto_server',
        name='rovpang_go_to_server',
        output='screen',
        parameters=[waypoints_file],
    )

    return LaunchDescription([nav_launch, goto_node])

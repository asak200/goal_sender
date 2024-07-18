from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            output='screen',
            arguments=['-f', 'the_map']
        )
    ])
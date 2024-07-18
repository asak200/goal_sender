from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    position_publisher = Node(
        package='goal_sender',
        executable='get_pose'
    )
    
    raw_map_saver = Node(
        package='goal_sender',
        executable='image_saver_node',
    )

    map_updater = Node(
        package='goal_sender',
        executable='map_updater'
    )

    return LaunchDescription([
        raw_map_saver,
        position_publisher,
        map_updater,
    ])

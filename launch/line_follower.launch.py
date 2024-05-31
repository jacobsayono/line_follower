from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node definitions
    detect_node = Node(
        package='line_follower',
        executable='detect_line',
        name='detect_line',
        remappings=[('/image_in', 'camera/image_raw')]
    )

    follow_node = Node(
        package='line_follower',
        executable='follow_line',
        name='follow_line',
        remappings=[('/cmd_vel', 'robot/cmd_vel')]
    )

    # Launch Description
    return LaunchDescription([
        detect_node,
        follow_node
    ])

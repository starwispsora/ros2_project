from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='your_package_name',
            executable='line_follower_node',
            name='line_follower_node'
        )
    ])
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_namespace = LaunchConfiguration('robot_namespace', default='tb3_0')

    # launch file for TurtleBot3
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch_file = os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')

    # Declare the launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='tb3_0',
            description='Namespace for the robot (use different namespaces for multiple robots)'),

        # Include the Gazebo launch file if you want to run in simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Start the line following node
        Node(
            package='line_following_robot',
            executable='line_following_robot',
            namespace=robot_namespace,
            name='line_follower',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])

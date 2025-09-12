from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='assignment1_pkg',
            executable='laserscan_subscriber',
            output='screen')
    ])
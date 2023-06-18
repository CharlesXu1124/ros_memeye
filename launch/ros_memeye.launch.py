from launch import LaunchDescription
from launch_ros.actions import Node


# launch script for launching ros2 nodes
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_memeye',
            executable='ros_memeye_node.py',
            output='screen'
        ),
    ])
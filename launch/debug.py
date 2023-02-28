from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seahawk_deck',
            executable='rviz_markers',
            name='rviz_markers'
        ),
    ])

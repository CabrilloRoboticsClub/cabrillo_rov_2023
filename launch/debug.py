from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seahawk_deck',
            executable='rviz_markers',
            name='rviz_markers'
        ),

       Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'launch/debug_kinematics.rviz']
        )

    ])

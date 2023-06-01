from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='republish_front_camera',
            output='screen',
            arguments=['h264', 'raw', '--ros-args', '--log-level', 'fatal'],
            remappings=[
                ('/in/h264', 'camera/front/h264'),
                ('/out', 'camera/front/image'),
            ]
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='republish_claw_camera',
            output='screen',
            arguments=['h264', 'raw', '--ros-args', '--log-level', 'fatal'],
            remappings=[
                ('/in/h264', 'camera/claw/h264'),
                ('/out', 'camera/claw/image'),
            ]
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='republish_top_camera',
            output='screen',
            arguments=['h264', 'raw', '--ros-args', '--log-level', 'fatal'],
            remappings=[
                ('/in/h264', 'camera/top/h264'),
                ('/out', 'camera/top/image'),
            ]
        ),
        Node(
            package='seahawk_deck',
            executable='thrust',
            name='thrust',
            output='screen'
        ),
        Node(
            package='seahawk_deck',
            executable='input_xbox_one',
            name='input',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        )
    ])

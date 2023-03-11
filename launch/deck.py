from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='republish_front_camera',
            output='screen',
            arguments=['h264', 'raw', '--ros-args', '--log-level fatal'],
            remappings=[
                ('in/h264', 'front_camera/h264 '),
                ('out', 'front_camera/image')
            ]
        ),
        Node(
            package='seahawk_deck',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
            parameters=[
                {"linear_x_scale": 1},
                {"linear_y_scale": 1},
                {"linear_z_scale": 1},
                {"angular_x_scale": 0},
                {"angular_y_scale": 0.5},
                {"angular_z_scale": 0.5}
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='h264_image_transport',
            executable='h264_cam_node',
            name='front_camera',
            output='screen',
            parameters=[{
                'input_fn': '/dev/video0',
                'fps': 20,
                'size': '3280x2464',
                'frame_id': 'rov_frame',
            }],
            remappings=[
                ('image_raw/h264', 'front_camera/h264'),
            ]
        ),
        # Node(
        #     package='seahawk_rov',
        #     executable='i2c_proxy',
        #     name='i2c_proxy',
        #     output='screen'
        # )
    ])

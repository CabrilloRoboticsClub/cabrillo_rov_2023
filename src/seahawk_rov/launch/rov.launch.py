from launch import LaunchDescription
from launch_ros.actions import Node

# from math import pi

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='h264_image_transport',
            executable='h264_cam_node',
            name='front_camera',
            output='screen',
            parameters=[{
                'input_fn': '/dev/video0',
                'fps': 30,
                'size': '1280x960',
                'frame_id': 'front_camera',
            }],
            remappings=[
                ('image_raw/h264', 'front_camera/h264'),
            ]
        ),
        Node(
            package='seahawk_rov',
            executable='seahawk_rov',
            name='seahawk_rov',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_transform',
            output='screen',
            arguments=[
                # Offset from parent frame in meters
                "--x",      "0.17",
                "--y",      "0.05",
                "--z",      "0.07",
                # # Quaternion rotation. Alternately use --{roll,pitch,yaw} but those probably aren't ideal in the end
                # "--qx",     "0",
                # "--qy",     "0",
                # "--qz",     "0",
                # "--qw",     "0",
                # # RPY rotation in radians. This is not ideal, it would be better to use Quaternion rotation
                # "--roll",   f"{pi}",
                # "--pitch",  f"0",
                # "--yaw",    f"{-pi*0.5}",
                # Parent frame to which the offset is provided
                "--frame-id", "base_link",
                # Created frame which is offset from parent frame
                "--child-frame-id", "logic_tube_bno085",
            ]
        )
    ])

from os import readlink
from sys import exit
from launch import LaunchDescription
from launch_ros.actions import Node

# from math import pi


# this is so cursed but it works so
# basically the reason it works is that the usb cameras always mount 4 /dev/video devices in sequence, and then
# the csi port camera only has one <10 device it mounts, so depending on the mounting order the csi port camera
# can be either /dev/video0, /dev/video4, or /dev/video8
# 0: 3 + 7 = 10
# 4: 2 + 7 = 9
# 8: 2 + 6 = 8
claw_camera_path = readlink('/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index2')
top_camera_path = readlink('/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index2')
match int(claw_camera_path[-1]) + int(top_camera_path[-1]):
    case 10:
        front_camera_path = '/dev/video0'
    case 9:
        front_camera_path = '/dev/video4'
    case 8:
        front_camera_path = '/dev/video8'
    case _:
        exit(1)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='h264_image_transport',
            executable='h264_cam_node',
            name='front_camera',
            output='screen',
            parameters=[{
                'input_fn': front_camera_path,
                'fps': 30,
                'size': '1280x960',
                'frame_id': 'front_camera',
            }],
            remappings=[
                ('image_raw/h264', 'camera/front/h264'),
            ]
        ),
        Node(
            package='h264_image_transport',
            executable='h264_cam_node',
            name='claw_camera',
            output='screen',
            parameters=[{
                'input_fn': claw_camera_path,
                'fps': 30,
                'size': '1280x960',
                'frame_id': 'claw_camera',
            }],
            remappings=[
                ('image_raw/h264', 'camera/top/h264'),
            ]
        ),
        Node(
            package='h264_image_transport',
            executable='h264_cam_node',
            name='top_camera',
            output='screen',
            parameters=[{
                'input_fn': top_camera_path,
                'fps': 30,
                'size': '1280x960',
                'frame_id': 'top_camera',
            }],
            remappings=[
                ('image_raw/h264', 'camera/top/h264'),
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

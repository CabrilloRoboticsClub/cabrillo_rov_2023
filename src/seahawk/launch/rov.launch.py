import re
import pathlib
import subprocess

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.event_handlers import OnShutdown

microros_serial_device = '/dev/ttyS0'

claw_camera_path = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index2'
top_camera_path = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index2'

v4l2_devices = subprocess.run("v4l2-ctl --list-devices", shell=True, stdout=subprocess.PIPE, check=True, encoding='utf-8').stdout
if m := re.search(r"mmal.*\n\s*(/dev/video\d)", v4l2_devices) is not None:
    front_camera_path = m.group(1)
    subprocess.run(f'v4l2-ctl --set-ctrl h264_i_frame_period=90 -d {front_camera_path}', shell=True, check=True)
    subprocess.run(f'v4l2-ctl --set-ctrl repeat_sequence_header=1 -d {front_camera_path}', shell=True, check=True)
else:
    print("Front camera not found!")
    front_camera_path = None


def generate_launch_description():
    nodes = [
        Node(
            package='seahawk',
            executable='seahawk_rov',
            name='seahawk_rov',
            output='both'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_transform',
            output='both',
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
    ]

    if front_camera_path is not None and pathlib.Path(front_camera_path).exists():
        nodes.append(
            Node(
                package='h264_image_transport',
                executable='h264_cam_node',
                name='front_camera',
                output='both',
                respawn=True,
                respawn_delay=0,
                parameters=[{
                    'input_fn': front_camera_path,
                    'fps': 30,
                    'size': '1280x960',
                    'frame_id': 'front_camera',
                    'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                    'qos_overrides./parameter_events.publisher.history': 'keep_last',
                    'qos_overrides./parameters_events.publisher.durability': 'volatile',
                    'qos_overrides./parameter_events.publisher.depth': 1,
                }],
                remappings=[
                    ('image_raw/h264', 'camera/front/h264'),
                ],
            ),
        )

    if pathlib.Path(claw_camera_path).exists():
        nodes.append(
            Node(
                package='h264_image_transport',
                executable='h264_cam_node',
                name='claw_camera',
                output='both',
                respawn=True,
                respawn_delay=0,
                parameters=[{
                    'input_fn': str(pathlib.Path(claw_camera_path).resolve()),
                    'fps': 30,
                    'size': '1280x960',
                    'frame_id': 'claw_camera',
                    'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                    'qos_overrides./parameter_events.publisher.history': 'keep_last',
                    'qos_overrides./parameters_events.publisher.durability': 'volatile',
                    'qos_overrides./parameter_events.publisher.depth': 1,
                }],
                remappings=[
                    ('image_raw/h264', 'camera/claw/h264'),
                ],
            ))
    else:
        print("Claw camera not found!")

    if pathlib.Path(top_camera_path).exists():
        nodes.append(
            Node(
                package='h264_image_transport',
                executable='h264_cam_node',
                name='top_camera',
                output='both',
                respawn=True,
                respawn_delay=0,
                parameters=[{
                    'input_fn': str(pathlib.Path(top_camera_path).resolve()),
                    'fps': 30,
                    'size': '1280x960',
                    'frame_id': 'top_camera',
                    'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                    'qos_overrides./parameter_events.publisher.history': 'keep_last',
                    'qos_overrides./parameters_events.publisher.durability': 'volatile',
                    'qos_overrides./parameter_events.publisher.depth': 1,
                }],
                remappings=[
                    ('image_raw/h264', 'camera/top/h264'),
                ]
            ))
    else:
        print("Top camera not found!")

    if microros_serial_device is not None and pathlib.Path(microros_serial_device).exists():
        nodes.append(
            ExecuteProcess(
                cmd=[[
                    FindExecutable(name='docker'),
                    " run " ,
                    " --rm ",
                    " --name micro-ros-agent ",
                    f" --device {microros_serial_device} ",
                    " --network host ",
                    " microros/micro-ros-agent:humble ",
                    f" serial --dev {microros_serial_device} baudrate=115200 ",
                ]],
                shell=True,
                name="micro-ros-agent",
                output='both',
            ),
        )

        nodes.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=lambda event, ctx: subprocess.run("docker kill micro-ros-agent", shell=True)
                )
            ),
        )
    else:
        print("No micro-ros serial device.")

    return LaunchDescription(nodes)

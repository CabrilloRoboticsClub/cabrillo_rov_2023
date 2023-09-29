from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

import pathlib
import os

def find_resource(package_name, relpath):
    """This is probably done somewhere in launch but the documentation is garbage."""
    relpath = pathlib.Path(relpath)
    for searchpath in os.environ['AMENT_PREFIX_PATH'].split(':'):
        search = pathlib.Path(searchpath) / 'share' / package_name / relpath
        if search.exists():
            print("Found: ", search)
            return str(search)
    print("WARNING: Not Found:", relpath)
    return str(relpath)
        
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
            package='seahawk',
            executable='thrust',
            name='thrust',
            output='screen'
        ),
        Node(
            package='seahawk',
            executable='input_xbox_one',
            name='input',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='viewer_node',
            output='screen',
            arguments=['--perspective-file', find_resource('seahawk', 'resources/deck.perspective'),]
        ),
    ])

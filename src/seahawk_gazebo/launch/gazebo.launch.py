import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, SetParameter


PKG_NAME = 'seahawk_description'
PKG_PATH = get_package_share_path(PKG_NAME)
MODEL_PATH = os.path.join(PKG_PATH, f'urdf/{PKG_NAME}.urdf')

ARGUMENTS = [
    DeclareLaunchArgument(
        name='model_path',
        default_value=MODEL_PATH,
        description=f'The robot model math, the default is {MODEL_PATH}'),
]


def generate_launch_description():

    return LaunchDescription([
        *ARGUMENTS,

        SetParameter(name='use_sim_time', value=True),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('seahawk_gazebo'),
                    'launch/gazebo_server.launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('seahawk_gazebo'),
                    'launch/gazebo_gui.launch.py'))
        ),

        Node(
            name='tf_footprint_base',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_footprint',
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0'
            ]
        ),

        # TODO: add joint state bridge if the robot model gets joints added
        Node(
            name="gz_bridge",
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[
                {
                    'config_file': os.path.join(
                        get_package_share_directory('seahawk_gazebo'),
                        'config/ros_gz_bridge.yaml')
                }
            ]
        ),

        # TODO: Make sure the world argument matches the world you're using above
        # TODO: You might be able to load the xml from the robot_description topic:
        #   https://github.com/gazebosim/ros_gz/blob/27f20ffbb2331a5c87685c62053d7eb20544e09a/ros_gz_sim_demos/launch/joint_states.launch.py#L68
        Node(
            name='spawn_model',
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', MODEL_PATH,
                '-z', '0.1',
            ]
        ),

        # TODO: suggest the drive input topic changes to /drive/cmd_vel or
        #  remove the remappings here in the Nodes below
        Node(
            package='seahawk_deck',
            executable='input_xbox_one',
            name='input',
            output='log',
            remappings=[
                ('/drive/twist', '/drive/cmd_vel')
            ]
        ),

        Node(
            package='seahawk_deck',
            executable='thrust',
            name='thrust',
            output='screen',
            remappings=[
                ('/drive/twist', '/drive/cmd_vel')
            ]
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
    ])


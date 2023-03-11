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

from launch_ros.actions import Node


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
    world_path = LaunchConfiguration('world_path')

    return LaunchDescription([
        *ARGUMENTS,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('seahawk_gazebo'),
                    'launch/gazebo_server.launch.py')),
            launch_arguments={
                'world_path': world_path
            }.items()
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
    ])


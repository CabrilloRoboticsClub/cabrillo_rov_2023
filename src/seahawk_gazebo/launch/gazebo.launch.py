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
    SetEnvironmentVariable, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext

from launch_ros.actions import Node


PKG_NAME = 'seahawk_description'
PKG_PATH = get_package_share_path(PKG_NAME)
MODEL_PATH = os.path.join(PKG_PATH, f'urdf/{PKG_NAME}.urdf')

ARGUMENTS = [
    DeclareLaunchArgument(
        'world_path',
        default_value='empty.sdf',
        description='The world path, the default is empty.world'),
    DeclareLaunchArgument(
        name='model_path',
        default_value=MODEL_PATH,
        description=f'The robot model math, the default is {MODEL_PATH}'),
]


def launch_setup(context: LaunchContext):
    """Function that is executed with the given the Launch Context.

    This is the only way I've found you can get a launch argument's string
    value.
    """
    world_path = LaunchConfiguration('world_path')
    model_path = LaunchConfiguration('model_path')

    # TODO: See if there's any way to avoid setting this env variable. I think
    #   Gazebo should be able to read mesh relative paths out of a URDF.
    gazebo_env = SetEnvironmentVariable(
        name='IGN_FILE_PATH',
        value=str(Path(PKG_PATH).parent.resolve())
    )

    gazebo_server_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch/gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-s {world_path.perform(context)}'
        }.items()
    )

    gazebo_gui_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch/gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-g {world_path.perform(context)}'
        }.items()
    )

    tf_footprint_base_node = Node(
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
    )

    # TODO: Make sure the world argument matches the world you're using above
    # TODO: You might be able to load the xml from the robot_description topic:
    #   https://github.com/gazebosim/ros_gz/blob/27f20ffbb2331a5c87685c62053d7eb20544e09a/ros_gz_sim_demos/launch/joint_states.launch.py#L68
    spawn_model_node = Node(
        name='spawn_model',
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', model_path,
            '-z', '0.1',
        ]
    )

    return [
        gazebo_env,
        gazebo_server_launch_include,
        gazebo_gui_launch_include,
        tf_footprint_base_node,
        spawn_model_node
    ]


def generate_launch_description():
    return LaunchDescription([
        *ARGUMENTS,
        OpaqueFunction(function=launch_setup)
    ])

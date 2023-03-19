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


PKG_NAME = 'seahawk_description'
PKG_PATH = get_package_share_path(PKG_NAME)

ARGUMENTS = [
    DeclareLaunchArgument(
        'world_path',
        default_value='empty.sdf',
        description='The world path, the default is empty.sdf'),
]


def launch_setup(context: LaunchContext):
    """Function that is executed with the given the Launch Context.

    This is the only way I've found you can get a launch argument's string
    value.
    """
    world_path = LaunchConfiguration('world_path')

    # TODO: See if there's any way to avoid setting this env variable. I think
    #   Gazebo should be able to read mesh relative paths out of a URDF.
    return [
        SetEnvironmentVariable(
            name='IGN_FILE_PATH',
            value=str(Path(PKG_PATH).parent.resolve())
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch/gz_sim.launch.py')),
            launch_arguments={
                'gz_args': f'-s {world_path.perform(context)}'
            }.items()
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        *ARGUMENTS,
        OpaqueFunction(function=launch_setup)
    ])

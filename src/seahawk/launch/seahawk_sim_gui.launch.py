import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


PKG_NAME = 'seahawk_description'
PKG_PATH = get_package_share_path(PKG_NAME)


def generate_launch_description():
    return LaunchDescription([
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
                'gz_args': f'-g'
            }.items()
        )
    ])
import os

from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    PKG_NAME = 'seahawk_description'
    PKG_PATH = get_package_share_path(PKG_NAME)
    MODEL_PATH = os.path.join(PKG_PATH, f'urdf/{PKG_NAME}.urdf')
    # TODO: Add an argument for the world file
    WORLD_NAME = 'empty'

    # TODO: See if there's any way to avoid setting this env variable. I think
    #   Gazebo should be able to read mesh relative paths out of a URDF.
    gazebo_env = SetEnvironmentVariable(
        name='IGN_FILE_PATH',
        value=os.path.join(PKG_PATH, '..')
    )

    robot_model = DeclareLaunchArgument(
        name='model',
        default_value=MODEL_PATH
    )

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{WORLD_NAME}.sdf'}.items()
    )

    tf_footprint_base_node = Node(
        name='tf_footprint_base',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40'
        ]
    )

    # TODO: Make sure the world argument matches the world you're using above
    spawn_model_node = Node(
        name='spawn_model',
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-file', MODEL_PATH,
        ]
    )

    return LaunchDescription([
        gazebo_env,
        robot_model,
        gazebo_launch_include,
        tf_footprint_base_node,
        spawn_model_node
    ])

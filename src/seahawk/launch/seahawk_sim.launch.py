from os import path, environ

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # If VS Code was installed with snap, the 'GTK_PATH' variable must be unset.
    if "GTK_PATH" in environ and "snap" in environ["GTK_PATH"]:
        environ.pop("GTK_PATH")

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    PKG_NAME = 'seahawk_sim'
    pkg_path = get_package_share_path(PKG_NAME)
    model_path = path.join(pkg_path, f'urdf/{PKG_NAME}.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            name='model',
            default_value=model_path
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'robot_description':  ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/seahawk_description", "-entity", "seahawk_II"]
        )

    ])

# Referenced: https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
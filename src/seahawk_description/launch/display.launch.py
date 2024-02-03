# Source: https://www.youtube.com/watch?v=jDsb8xEdbKM
import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # TODO: use sim time if Gazebo is running
    # https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html

    PKG_NAME = 'seahawk_description'
    pkg_path = get_package_share_path(PKG_NAME)
    model_path = os.path.join(pkg_path, f'urdf/{PKG_NAME}.urdf')

    enable_joint_state_pub_gui = DeclareLaunchArgument(
        name='enable_joint_state_pub_gui',
        default_value='true',
        choices=['true', 'false']
    )
    robot_model = DeclareLaunchArgument(
        name='model',
        default_value=model_path
    )
    rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=os.path.join(pkg_path, 'rviz/robot.rviz')
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # https://github.com/ros/robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # https://github.com/ros/joint_state_publisher
    # http://wiki.ros.org/joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('enable_joint_state_pub_gui'))
    )

    # TODO: Is this node useful for anything? They include it in the exported
    #   URDF package's launch file by default
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('enable_joint_state_pub_gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    )

    return LaunchDescription([
        enable_joint_state_pub_gui,
        robot_model,
        rviz_config_file,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])

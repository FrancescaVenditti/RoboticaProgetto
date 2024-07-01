import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    # Define paths to xacro files
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'), 'robots', 'panda_arm.urdf.xacro')
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_moveit_config'), 'srdf', 'panda_arm.srdf.xacro')

    # Define the robot description using xacro command
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=false']
    )

    # Define the robot semantic description using xacro command
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=false']
    )

    # Define parameters for the nodes
    robot_description = {'robot_description': robot_description_config}
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    return LaunchDescription([
        Node(
            package='franka_moveit_planning_demo',
            executable='franka_moveit_planning_node',
            output='screen',
            parameters=[robot_description, robot_description_semantic],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])

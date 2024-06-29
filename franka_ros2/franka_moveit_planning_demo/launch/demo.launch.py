from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='franka_moveit_planning_demo',
            executable='franka_moveit_planning_node'
        )
    ])
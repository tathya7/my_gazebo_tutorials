from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Node for the walker
    walker_node = Node(
        package='walker',
        executable='walker',
        name='walker_node',
    )
    # Launch all the nodes and actions
    return LaunchDescription([
        walker_node,
    ])
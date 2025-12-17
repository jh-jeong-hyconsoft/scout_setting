import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
  

    return LaunchDescription([
        Node(
          package='sonar_ros',
          executable='relay_node.py',
          name='relay_ros',
          output='screen')
    ])

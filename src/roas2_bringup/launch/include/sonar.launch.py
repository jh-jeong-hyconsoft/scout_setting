import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
  
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='TF frame prefix to prepend to sonar frames'
    )

    return LaunchDescription([
        Node(
          package='sonar_ros',
          executable='sonar_sensor.py',
          name='sonar_ros',
          output='screen',
          parameters=[{
              'frame_prefix': LaunchConfiguration('frame_prefix'),
          }]),
        frame_prefix_arg,
    ])

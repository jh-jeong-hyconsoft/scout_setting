import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description() :
    ld = LaunchDescription()

    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='TF frame prefix, e.g., robot1/'
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'frame_prefix': LaunchConfiguration('frame_prefix'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('roas2_bringup'),
                        'urdf/roas2.urdf.xacro',
                    ]),
                ]),
        }]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui_node)

    ld.add_action(frame_prefix_arg)

    return ld

# import os
# import launch
# import launch_ros

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import FindExecutable, PathJoinSubstitution
# from launch.substitutions import LaunchConfiguration, Command
# from launch_ros.actions import Node


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

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('roas2_bringup'),
                        'urdf/include/tracer_v1.xacro',
                    ]),
                ]),
        }]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_config_path = os.path.join(get_package_share_directory('roas2_bringup'), 'rviz', 'view_robot.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"],[rviz_config_path]],
    )
    ld.add_action(rviz2_node)

    # ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui_node)


    return ld
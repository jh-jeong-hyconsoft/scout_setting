import os
import signal
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.event_handlers import OnProcessStart
from launch.events.process import ShutdownProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    # rover.launch.py와 동일한 구조 + namespace 지원
    
    # namespace 인자 (bringup에서 전달받음)
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace, e.g., robot1'
    )
    
    default_file_name = 'gnss_params.yaml'
    name_arg_file_name = "gnss_file_name"
    arg_file_name = DeclareLaunchArgument(
        name_arg_file_name,
        default_value=TextSubstitution(text=str(default_file_name))
    )

    name_arg_file_path = 'gnss_config_path'
    arg_file_path = DeclareLaunchArgument(
        name_arg_file_path,
        default_value=[
            get_package_share_directory('roas2_bringup'),
            '/config/',
            LaunchConfiguration(name_arg_file_name)
        ]
    )

    # namespace: robot_namespace/septentrio_gnss_driver 형태
    gnss_namespace = [LaunchConfiguration('robot_namespace'), '/septentrio_gnss_driver']

    composable_node = ComposableNode(
        name='septentrio_gnss_driver',
        namespace=gnss_namespace,
        package='septentrio_gnss_driver',
        plugin='rosaic_node::ROSaicNode',
        parameters=[LaunchConfiguration(name_arg_file_path)]
    )

    # 타임아웃을 짧게 설정 - SIGINT 5초 후 SIGTERM, SIGTERM 5초 후 SIGKILL
    container = ComposableNodeContainer(
        name='septentrio_gnss_driver_container',
        namespace=gnss_namespace,
        package='rclcpp_components',
        executable='component_container_isolated',
        emulate_tty=True,
        sigterm_timeout='5',
        sigkill_timeout='5',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return LaunchDescription([
        robot_namespace_arg,
        arg_file_name,
        arg_file_path,
        container
    ])

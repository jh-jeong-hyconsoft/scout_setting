import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace

from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 패키지 share 디렉토리 (config/ekf.yaml 등 파일 경로용)
    # launch_ros.substitutions.FindPackageShare 는 Substitution 이라
    # 바로 .find() 해서 쓸 수 없으므로, ament_index 의 헬퍼를 사용한다.
    pkg_share = get_package_share_directory('roas2_bringup')

    # Namespace / TF prefix arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='ROS namespace for this robot, e.g., robot1'
    )
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='robot1/',
        description='TF frame prefix, e.g., robot1/'
    )
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='indoor',
        description='Operating environment: outdoor | indoor'
    )
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='robot1/map',
        description='Map frame id'
    )
    ouster_hostname_arg = DeclareLaunchArgument(
        'ouster_hostname',
        default_value='192.168.50.20',
        description='Ouster sensor hostname or IP'
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='robot1/odom',
        description='Odometry frame id (can be prefixed outside)'
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='robot1/base_link',
        description='Base link frame id (can be prefixed outside)'
    )

    # Scout 2.0
    tracer_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('roas2_bringup'), '/launch/include/scout.launch.py']
            ),
        launch_arguments={
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
        }.items(),
    )
            
    # Sonar
    sonar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('roas2_bringup'), '/launch/include/sonar.launch.py']
            ),
        launch_arguments={
            'frame_prefix': LaunchConfiguration('frame_prefix')
        }.items(),
    )
            
    # Relay
    relay_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('roas2_bringup'), '/launch/include/relay.launch.py']
            ),
    )

    # Ouster Lidar
    ouster_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('roas2_bringup'), '/launch/include/ouster.launch.py']
            ),
        launch_arguments={
            'frame_prefix': LaunchConfiguration('frame_prefix'),
            'sensor_hostname': LaunchConfiguration('ouster_hostname'),
        }.items(),
    )

    # RViz
    rviz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('roas2_bringup'), '/launch/view_robot.launch.py']
            ),
        launch_arguments={
            'frame_prefix': LaunchConfiguration('frame_prefix')
        }.items(),
    )

    # Environment conditions
    is_outdoor = IfCondition(PythonExpression(["'", LaunchConfiguration('environment'), "' == 'outdoor'"]))
    is_indoor = IfCondition(PythonExpression(["'", LaunchConfiguration('environment'), "' == 'indoor'"]))

    # Group everything under the robot namespace
    ns_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_namespace')),
        tracer_bringup,
        rviz_bringup,
        sonar_bringup,
        relay_bringup,
        ouster_bringup,
        IncludeLaunchDescription(  # GNSS only in outdoor
            PythonLaunchDescriptionSource([
                get_package_share_directory('roas2_bringup'), '/launch/include/gnss.launch.py'
            ]),
            launch_arguments={
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
            }.items(),
            condition=is_outdoor,
        ),
    ])

    return LaunchDescription([
        robot_namespace_arg,
        frame_prefix_arg,
        environment_arg,
        map_frame_arg,
        ouster_hostname_arg,
        odom_frame_arg,
        base_frame_arg,
        ns_group,
    ])

    


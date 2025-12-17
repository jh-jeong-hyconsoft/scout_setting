import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, TimerAction
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
    # robot_namespace 하나만 설정하면 나머지 프레임 인자들이 자동으로 파생됨
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='ROS namespace for this robot, e.g., robot1'
    )
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value=[LaunchConfiguration('robot_namespace'), '/'],
        description='TF frame prefix, derived from robot_namespace'
    )
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='indoor',
        description='Operating environment: outdoor | indoor'
    )
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value=[LaunchConfiguration('robot_namespace'), '/map'],
        description='Map frame id, derived from robot_namespace'
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value=[LaunchConfiguration('robot_namespace'), '/odom'],
        description='Odometry frame id, derived from robot_namespace'
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value=[LaunchConfiguration('robot_namespace'), '/base_link'],
        description='Base link frame id, derived from robot_namespace'
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
            'frame_prefix': LaunchConfiguration('frame_prefix')
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

    # GNSS는 PushRosNamespace 밖에서 실행 (파라미터 로딩 문제 방지)
    # Ouster lifecycle 전환이 완료된 후 시작하도록 3초 지연
    gnss_bringup = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('roas2_bringup'), '/launch/include/gnss.launch.py'
                ]),
                launch_arguments={
                    'robot_namespace': LaunchConfiguration('robot_namespace'),
                }.items(),
            ),
        ],
        condition=is_outdoor,
    )

    # Group everything under the robot namespace
    ns_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_namespace')),
        tracer_bringup,
        rviz_bringup,
        sonar_bringup,
        relay_bringup,
        ouster_bringup,
        # IncludeLaunchDescription(  # Localization (navsat+ekf_global) in outdoor
        #     PythonLaunchDescriptionSource([
        #         get_package_share_directory('roas2_bringup'), '/launch/localization.launch.py'
        #     ]),
        #     launch_arguments={
        #         'map_frame': LaunchConfiguration('map_frame'),
        #         'odom_frame': LaunchConfiguration('odom_frame'),
        #         'base_frame': LaunchConfiguration('base_frame'),
        #     }.items(),
        #     condition=is_outdoor,
        # ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'navsat_transform.yaml'),
                {
                    'map_frame': LaunchConfiguration('map_frame'),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'base_link_frame': LaunchConfiguration('base_frame'),
                    'world_frame': LaunchConfiguration('map_frame'),
                }
            ],
            remappings=[
                ('gps/fix', 'septentrio_gnss_driver/navsatfix'),
                ('imu', 'gnss/imu/data'),
                ('odometry/gps', 'odometry/gps'),
            ],
            condition=is_outdoor,
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'ekf_global.yaml'),
                {
                    'map_frame': LaunchConfiguration('map_frame'),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'base_link_frame': LaunchConfiguration('base_frame'),
                    'world_frame': LaunchConfiguration('map_frame'),
                }
            ],
            condition=is_outdoor,
        ),
        Node(  # Local EKF only in indoor
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'ekf.yaml'),
                {
                    'map_frame': LaunchConfiguration('map_frame'),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'base_link_frame': LaunchConfiguration('base_frame'),
                    'world_frame': LaunchConfiguration('map_frame'),
                }
            ],
            condition=is_indoor,
        ),
    ])

    return LaunchDescription([
        robot_namespace_arg,
        frame_prefix_arg,
        environment_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        ns_group,
        gnss_bringup,  # Ouster lifecycle 전환 후 3초 지연 시작
    ])

    


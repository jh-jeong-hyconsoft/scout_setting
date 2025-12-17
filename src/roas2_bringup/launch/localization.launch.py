import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = FindPackageShare(package='roas2_bringup').find('roas2_bringup')

    # Namespace / frames / topics
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    gps_fix_topic = LaunchConfiguration('gps_fix_topic') # NavSatFix
    imu_topic = LaunchConfiguration('imu_topic') # GNSS IMU (sensor_msgs/Imu)

    args = [
        DeclareLaunchArgument('robot_namespace', default_value='', description='ROS namespace, e.g., robot1'),
        DeclareLaunchArgument('frame_prefix', default_value='', description='TF frame prefix, e.g., robot1/'),
        DeclareLaunchArgument('map_frame', default_value='map', description='Map frame id'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame id'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Base link frame id'),
        # Use relative topics by default so namespace applies
        DeclareLaunchArgument('gps_fix_topic', default_value='septentrio_gnss_driver/navsatfix'),
        DeclareLaunchArgument('imu_topic', default_value='gnss/imu/data'),
    ]

    # NavSat Transform
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'navsat_transform.yaml'),
            {
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_link_frame': base_frame,
                'world_frame': map_frame,
            }
        ],
        remappings=[
            ('gps/fix', gps_fix_topic),  # 상대 경로로 변경
            ('imu', imu_topic),           # navsat_transform이 'imu' 구독
            ('odometry/gps', 'odometry/gps'),
        ],
    )

    # Robot Localization EKF
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf_global.yaml'),
            {
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_link_frame': base_frame,
                'world_frame': map_frame,
                # Override topic parameters with relative names so namespace applies
                'odom1': 'odometry/gps',
                'imu0': 'gnss/imu/data',
                'pose0': 'pcl_pose',
                'odom0': 'odom',
            }
        ]
    )

    return LaunchDescription(args + [navsat, ekf_global])
import launch
import lifecycle_msgs.msg
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    # 인자
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='TF frame prefix, e.g., robot1/'
    )
    file_name_arg = DeclareLaunchArgument(
        'ouster_file_name',
        default_value=TextSubstitution(text='driver_params.yaml'),
        description='Ouster driver parameter file name'
    )
    path_to_config_arg = DeclareLaunchArgument(
        'ouster_config_path',
        default_value=[get_package_share_directory('ouster_ros'), '/config/', LaunchConfiguration('ouster_file_name')],
        description='Full path to Ouster driver parameter YAML'
    )

    # 프리픽스 + 기본 프레임명 결합
    sensor_frame = PythonExpression(["'", LaunchConfiguration('frame_prefix'), "'", " + 'os_sensor'"])
    lidar_frame = PythonExpression(["'", LaunchConfiguration('frame_prefix'), "'", " + 'os_lidar'"])
    imu_frame = PythonExpression(["'", LaunchConfiguration('frame_prefix'), "'", " + 'os_imu'"])
    point_cloud_frame = lidar_frame

    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace='ouster',
        parameters=[
            # 1) 기본 Ouster driver YAML (driver_params.yaml 등)
            LaunchConfiguration('ouster_config_path'),
            # 2) launch 인자로 덮어쓸 값들 (특히 hostname / frame prefix)
            {
                'sensor_frame': sensor_frame,
                'lidar_frame': lidar_frame,
                'imu_frame': imu_frame,
                'point_cloud_frame': point_cloud_frame,
                'pub_static_tf': True,
            }
        ],
        output='screen',
    )

    sensor_configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(os_driver),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    ))
    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='inactive',
            entities=[
                LogInfo(msg="ouster os_driver activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    return LaunchDescription([
        frame_prefix_arg,
        file_name_arg,
        path_to_config_arg,
        os_driver,
        sensor_configure_event,
        sensor_activate_event,
    ])

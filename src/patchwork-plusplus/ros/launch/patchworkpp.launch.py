import os
import time
import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# This configuration parameters are not exposed thorugh the launch system, meaning you can't modify
# those throw the ros launch CLI. If you need to change these values, you could write your own
# launch file and modify the 'parameters=' block from the Node class.
class config:
    # TBU. Examples are as follows:
    max_range: float = 80.0
    # deskew: bool = False


def wait_for_lidar_and_launch_patchworkpp(context, *args, **kwargs):
    rclpy.init(args=None)
    node = rclpy.create_node('patchworkpp_topic_waiter')
    future = rclpy.task.Future()

    def listener_callback(_msg):
        if not future.done():
            future.set_result(True)

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )

    input_topic = LaunchConfiguration('cloud_topic').perform(context)
    node.create_subscription(PointCloud2, input_topic, listener_callback, qos_profile)

    while not future.done() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    # Allow a brief stabilization window
    time.sleep(2.0)

    params_file = LaunchConfiguration('params_file').perform(context)
    ground_topic = LaunchConfiguration('ground_topic').perform(context)
    nonground_topic = LaunchConfiguration('nonground_topic').perform(context)

    patchworkpp_cmd = Node(
        package='patchworkpp',
        executable='patchworkpp_node',
        name='patchworkpp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', input_topic),
            ('/patchworkpp/nonground', nonground_topic),
            ('/patchworkpp/ground', ground_topic),
        ],
        parameters=[params_file],
    )

    node.destroy_node()
    rclpy.shutdown()
    return [patchworkpp_cmd]


def generate_launch_description():
    # ROS configuration
    pointcloud_topic = LaunchConfiguration("cloud_topic", default="/ouster/points")
    visualize = LaunchConfiguration("visualize", default="true")

    # Optional ros bag play
    bagfile = LaunchConfiguration("bagfile", default="")

    # Parameters file
    params_file = LaunchConfiguration("params_file")
    pkg_dir = get_package_share_directory('patchworkpp')
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, 'config', 'param.yaml'),
        description="Patchwork++ parameter file",
    )

    # Cloud topic argument
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/ouster/points",
        description="PointCloud2 topic for Patchwork++ input",
    )

    # Output topics
    ground_topic_arg = DeclareLaunchArgument(
        "ground_topic",
        default_value="/ouster/ground_points",
        description="Ground points output topic",
    )
    nonground_topic_arg = DeclareLaunchArgument(
        "nonground_topic",
        default_value="/ouster/segmented_points",
        description="Non-ground points output topic",
    )

    # Defer Patchwork++ start until LiDAR is publishing
    delayed_patchworkpp = TimerAction(
        period=1.0,
        actions=[OpaqueFunction(function=wait_for_lidar_and_launch_patchworkpp)],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("patchworkpp"), "rviz", "patchworkpp.rviz"]
            ),
        ],
        condition=IfCondition(visualize),
    )

    # bagfile_play = ExecuteProcess(
    #     cmd=["ros2", "bag", "play", bagfile],
    #     output="screen",
    #     condition=IfCondition(PythonExpression(["'", bagfile, "' != ''"])),
    # )
    return LaunchDescription(
        [
            params_file_arg,
            cloud_topic_arg,
            ground_topic_arg,
            nonground_topic_arg,
            delayed_patchworkpp,
            rviz_node,
            #bagfile_play,
        ]
    )

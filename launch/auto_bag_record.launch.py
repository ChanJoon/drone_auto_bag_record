"""Launch arming-triggered rosbag2 recording for QAV250 flight tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("drone_auto_bag_record"), "config", "auto_bag_record.yaml"]
    )

    args = [
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("output_dir", default_value="/home/cj/bags/predictnav"),
        DeclareLaunchArgument("bag_prefix", default_value="qav250_position_vins"),
        DeclareLaunchArgument("record_on_armed", default_value="true"),
        DeclareLaunchArgument("stop_on_disarm", default_value="true"),
        DeclareLaunchArgument("start_immediately", default_value="false"),
        DeclareLaunchArgument("compression_mode", default_value="none"),
        DeclareLaunchArgument("max_bag_duration", default_value="0"),
    ]

    recorder = Node(
        package="drone_auto_bag_record",
        executable="auto_bag_recorder",
        name="drone_auto_bag_recorder",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "output_dir": LaunchConfiguration("output_dir"),
                "bag_prefix": LaunchConfiguration("bag_prefix"),
                "record_on_armed": LaunchConfiguration("record_on_armed"),
                "stop_on_disarm": LaunchConfiguration("stop_on_disarm"),
                "start_immediately": LaunchConfiguration("start_immediately"),
                "compression_mode": LaunchConfiguration("compression_mode"),
                "max_bag_duration": LaunchConfiguration("max_bag_duration"),
            },
        ],
    )

    return LaunchDescription(args + [recorder])

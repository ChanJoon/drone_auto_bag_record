#!/usr/bin/env python3
"""Start and stop rosbag2 recording from MAVROS armed state."""

from __future__ import annotations

import os
import signal
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from mavros_msgs.msg import State
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


DEFAULT_TOPICS = [
    "/rosout",
    "/parameter_events",
    "/diagnostics",
    "/tf",
    "/tf_static",
    "/mavros/state",
    "/mavros/extended_state",
    "/mavros/sys_status",
    "/mavros/estimator_status",
    "/mavros/status_event",
    "/mavros/statustext/recv",
    "/mavros/timesync_status",
    "/mavros/time_reference",
    "/mavros/battery",
    "/mavros/vfr_hud",
    "/mavros/rc/in",
    "/mavros/rc/out",
    "/mavros/imu/data",
    "/mavros/imu/data_raw",
    "/mavros/imu/mag",
    "/mavros/altitude",
    "/mavros/local_position/pose",
    "/mavros/local_position/pose_cov",
    "/mavros/local_position/odom",
    "/mavros/local_position/velocity_body",
    "/mavros/local_position/velocity_body_cov",
    "/mavros/local_position/velocity_local",
    "/mavros/odometry/out",
    "/mavros/vision_pose/pose",
    "/mavros/setpoint_position/local",
    "/mavros/setpoint_raw/attitude",
    "/mavros/setpoint_raw/target_attitude",
    "/mavros/setpoint_raw/local",
    "/mavros/setpoint_raw/target_local",
    "/vins_estimator/odometry",
    "/vins_estimator/path",
    "/vins_estimator/imu_propagate",
    "/vins_estimator/camera_pose",
    "/vins_estimator/camera_pose_visual",
    "/vins_estimator/extrinsic",
    "/vins_estimator/image_track",
    "/vins_estimator/key_poses",
    "/vins_estimator/key_poses_array",
    "/vins_estimator/keyframe_pose",
    "/vins_estimator/keyframe_point",
    "/vins_estimator/point_cloud",
    "/vins_estimator/margin_cloud",
    "/feature_tracker/feature",
    "/vins_restart",
    "/vins_imu_switch",
    "/vins_cam_switch",
    "/camera/depth/image_rect_raw",
    "/camera/depth/camera_info",
    "/camera/depth/metadata",
    "/camera/extrinsics/depth_to_infra1",
    "/camera/extrinsics/depth_to_infra2",
    "/camera/infra1/image_rect_raw",
    "/camera/infra1/camera_info",
    "/camera/infra1/metadata",
    "/camera/infra2/image_rect_raw",
    "/camera/infra2/camera_info",
    "/camera/infra2/metadata",
    "/camera/color/image_raw",
    "/camera/color/image_rect_raw",
    "/camera/color/camera_info",
    "/predictnav/depth/image_64x64_norm",
    "/predictnav/goal",
    "/predictnav/start",
    "/predictnav/land",
    "/move_base_simple/goal",
    "/clicked_point",
    "/initialpose",
    "/predictnav_flight_manager/raw_action",
    "/predictnav_flight_manager/ctbr_command",
    "/predictnav_flight_manager/status",
]


def _bool_param(value: Any) -> bool:
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _string_list_param(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item) for item in list(value) if str(item)]


class AutoBagRecorder(Node):
    """Arm-state triggered wrapper around ``ros2 bag record``."""

    def __init__(self) -> None:
        super().__init__("drone_auto_bag_recorder")
        self._declare_parameters()
        self._load_parameters()

        self.process: subprocess.Popen | None = None
        self.recording_output: Path | None = None
        self.last_armed = False

        # MAVROS state is a low-bandwidth status topic; Reliable/Volatile matches the common publisher.
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state_sub = self.create_subscription(
            State,
            self.state_topic,
            self._on_state,
            state_qos,
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.timer = self.create_timer(1.0, self._on_timer)

        if self.start_immediately:
            self._start_recording(reason="start_immediately")

        self.get_logger().info(
            f"Auto bag recorder ready: output_dir={self.output_dir} "
            f"record_on_armed={self.record_on_armed} topics={len(self.topics)}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("output_dir", "~/bags/predictnav")
        self.declare_parameter("bag_prefix", "qav250")
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("topics", DEFAULT_TOPICS)

        self.declare_parameter("record_on_armed", True)
        self.declare_parameter("stop_on_disarm", True)
        self.declare_parameter("start_immediately", False)
        self.declare_parameter("include_state_topic", True)

        self.declare_parameter("record_all", False)
        self.declare_parameter("regex", "")
        self.declare_parameter("exclude_regex", "")
        self.declare_parameter("include_hidden_topics", False)
        self.declare_parameter("include_unpublished_topics", False)
        self.declare_parameter("no_discovery", False)

        self.declare_parameter("storage_id", "sqlite3")
        self.declare_parameter("compression_mode", "none")
        self.declare_parameter("compression_format", "zstd")
        self.declare_parameter("max_cache_size", 268435456)
        self.declare_parameter("max_bag_size", 0)
        self.declare_parameter("max_bag_duration", 0)
        self.declare_parameter("log_level", "info")
        self.declare_parameter("stop_timeout_s", 8.0)

    def _load_parameters(self) -> None:
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.bag_prefix = str(self.get_parameter("bag_prefix").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.topics = _string_list_param(self.get_parameter("topics").value)

        self.record_on_armed = _bool_param(self.get_parameter("record_on_armed").value)
        self.stop_on_disarm = _bool_param(self.get_parameter("stop_on_disarm").value)
        self.start_immediately = _bool_param(self.get_parameter("start_immediately").value)
        self.include_state_topic = _bool_param(self.get_parameter("include_state_topic").value)

        self.record_all = _bool_param(self.get_parameter("record_all").value)
        self.regex = str(self.get_parameter("regex").value)
        self.exclude_regex = str(self.get_parameter("exclude_regex").value)
        self.include_hidden_topics = _bool_param(
            self.get_parameter("include_hidden_topics").value
        )
        self.include_unpublished_topics = _bool_param(
            self.get_parameter("include_unpublished_topics").value
        )
        self.no_discovery = _bool_param(self.get_parameter("no_discovery").value)

        self.storage_id = str(self.get_parameter("storage_id").value)
        self.compression_mode = str(self.get_parameter("compression_mode").value)
        self.compression_format = str(self.get_parameter("compression_format").value)
        self.max_cache_size = int(self.get_parameter("max_cache_size").value)
        self.max_bag_size = int(self.get_parameter("max_bag_size").value)
        self.max_bag_duration = int(self.get_parameter("max_bag_duration").value)
        self.log_level = str(self.get_parameter("log_level").value)
        self.stop_timeout_s = max(1.0, float(self.get_parameter("stop_timeout_s").value))

        if self.include_state_topic and self.state_topic not in self.topics:
            self.topics.insert(0, self.state_topic)

    def _on_state(self, msg: State) -> None:
        armed = bool(msg.armed)
        if self.record_on_armed and armed and not self.last_armed:
            self._start_recording(reason="mavros_armed")
        if self.stop_on_disarm and not armed and self.last_armed:
            self._stop_recording(reason="mavros_disarmed")
        self.last_armed = armed

    def _on_timer(self) -> None:
        running = self._process_is_running()
        output = str(self.recording_output) if self.recording_output is not None else ""
        self.status_pub.publish(String(data=f"running={running}, output={output}"))
        if self.process is not None and not running:
            self.get_logger().warn("ros2 bag process exited unexpectedly")
            self.process = None
            self.recording_output = None

    def _start_recording(self, *, reason: str) -> None:
        if self._process_is_running():
            self.get_logger().info("Record request ignored because ros2 bag is already running")
            return

        output = self._make_output_path()
        output.parent.mkdir(parents=True, exist_ok=True)
        command = self._build_record_command(output)
        if command is None:
            return

        self.get_logger().warn(f"Starting ros2 bag record: reason={reason} output={output}")
        self.get_logger().info(" ".join(command))
        self.process = subprocess.Popen(
            command,
            preexec_fn=os.setsid,
        )
        self.recording_output = output

    def _stop_recording(self, *, reason: str) -> None:
        if not self._process_is_running():
            self.process = None
            self.recording_output = None
            return

        assert self.process is not None
        self.get_logger().warn(f"Stopping ros2 bag record: reason={reason}")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=self.stop_timeout_s)
        except subprocess.TimeoutExpired:
            self.get_logger().error("ros2 bag did not stop after SIGINT; sending SIGTERM")
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=2.0)
        finally:
            self.process = None
            self.recording_output = None

    def _process_is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def _make_output_path(self) -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"{self.bag_prefix}_{stamp}"
        return Path(self.output_dir).expanduser() / name

    def _build_record_command(self, output: Path) -> list[str] | None:
        command = [
            "ros2",
            "bag",
            "record",
            "-o",
            str(output),
            "-s",
            self.storage_id,
            "--max-cache-size",
            str(self.max_cache_size),
            "--log-level",
            self.log_level,
        ]
        if self.max_bag_size > 0:
            command.extend(["--max-bag-size", str(self.max_bag_size)])
        if self.max_bag_duration > 0:
            command.extend(["--max-bag-duration", str(self.max_bag_duration)])
        if self.compression_mode != "none":
            command.extend(["--compression-mode", self.compression_mode])
            command.extend(["--compression-format", self.compression_format])
        if self.include_hidden_topics:
            command.append("--include-hidden-topics")
        if self.include_unpublished_topics:
            command.append("--include-unpublished-topics")
        if self.no_discovery:
            command.append("--no-discovery")
        if self.record_all:
            command.append("--all")
        if self.regex:
            command.extend(["--regex", self.regex])
        if self.exclude_regex:
            command.extend(["--exclude", self.exclude_regex])
        if not self.record_all and not self.regex:
            if not self.topics:
                self.get_logger().error("No topics configured and record_all/regex are disabled")
                return None
            command.extend(self.topics)
        return command

    def destroy_node(self) -> bool:
        self._stop_recording(reason="node_shutdown")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoBagRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

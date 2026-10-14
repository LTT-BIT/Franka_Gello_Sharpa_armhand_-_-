"""Publish Manus hand tracking frames as standard ROS PoseArray messages."""

from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from litchi_hardware.core.types import HandSide, HandTrackingFrame
from litchi_hardware.hardware.manus import (
    ManusBackend,
    ManusNativeConfig,
    ManusZmqConfig,
    create_tracker,
)


def frame_to_pose_array(
    frame: HandTrackingFrame, *, stamp: RosTime | None = None
) -> PoseArray:
    message = PoseArray()
    if stamp is None:
        seconds = int(frame.timestamp_s)
        message.header.stamp.sec = seconds
        message.header.stamp.nanosec = int((frame.timestamp_s - seconds) * 1e9)
    else:
        message.header.stamp = stamp
    message.header.frame_id = frame.frame_id
    for keypoint in frame.keypoints:
        pose = RosPose()
        pose.position.x, pose.position.y, pose.position.z = keypoint.position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = keypoint.orientation
        message.poses.append(pose)
    return message


class ManusDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("manus_driver")
        self.declare_parameter("use_fake_hardware", False)
        self.declare_parameter("backend", "native")
        self.declare_parameter("endpoint", "tcp://127.0.0.1:2044")
        self.declare_parameter("frame_timeout_s", 0.02)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("publish_left", True)
        self.declare_parameter("publish_right", True)
        self.declare_parameter("manage_client", True)
        self.declare_parameter("client_executable", "")
        self.declare_parameter("client_working_directory", "")
        self.declare_parameter("client_arguments", [])
        self.declare_parameter("native_library", "")
        self.declare_parameter("native_host_discovery_s", 1)
        self.declare_parameter("native_loopback_only", False)
        self.declare_parameter("calibration_directory", "")
        self.declare_parameter("frame_id", "manus_hand")

        use_fake_hardware = self.get_parameter("use_fake_hardware").value
        backend = ManusBackend(self.get_parameter("backend").value)
        client_executable = self.get_parameter("client_executable").value
        common_config = {
            "name": self.get_name(),
            "frame_timeout_s": self.get_parameter("frame_timeout_s").value,
            "frame_id": self.get_parameter("frame_id").value,
        }
        if backend is ManusBackend.NATIVE:
            config = ManusNativeConfig(
                **common_config,
                native_library=self.get_parameter("native_library").value or None,
                native_host_discovery_s=self.get_parameter(
                    "native_host_discovery_s"
                ).value,
                native_loopback_only=self.get_parameter(
                    "native_loopback_only"
                ).value,
                calibration_directory=(
                    self.get_parameter("calibration_directory").value or None
                ),
            )
        else:
            config = ManusZmqConfig(
                **common_config,
                endpoint=self.get_parameter("endpoint").value,
                manage_client=self.get_parameter("manage_client").value,
                client_executable=client_executable or None,
                client_working_directory=(
                    self.get_parameter("client_working_directory").value or None
                ),
                client_arguments=tuple(
                    self.get_parameter("client_arguments").value
                ),
            )
        self._tracker = create_tracker(
            config, use_fake_hardware=use_fake_hardware
        )
        self._tracker.connect()

        self._side_publishers = {}
        if self.get_parameter("publish_left").value:
            self._side_publishers[HandSide.LEFT] = self.create_publisher(
                PoseArray, "/manus/left/keypoints", qos_profile_sensor_data
            )
        if self.get_parameter("publish_right").value:
            self._side_publishers[HandSide.RIGHT] = self.create_publisher(
                PoseArray, "/manus/right/keypoints", qos_profile_sensor_data
            )
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_frames)
        self._last_warning_ns = 0
        self.get_logger().info(
            f"Manus driver connected via {self._tracker.info.metadata['transport']}; "
            f"fake={use_fake_hardware}"
        )

    def _publish_frames(self) -> None:
        for side, publisher in self._side_publishers.items():
            try:
                frame = self._tracker.read_frame(side)
                # MANUS publishTime uses an SDK/device clock domain rather than
                # the ROS system clock. Stamp at receipt so downstream age
                # checks compare values from the same clock domain.
                publisher.publish(
                    frame_to_pose_array(
                        frame, stamp=self.get_clock().now().to_msg()
                    )
                )
            except TimeoutError as exc:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_warning_ns > 2_000_000_000:
                    self.get_logger().warning(str(exc))
                    self._last_warning_ns = now_ns
            except Exception as exc:
                self.get_logger().error(
                    f"failed to publish {side.value} Manus frame: {exc}"
                )

    def destroy_node(self) -> bool:
        self._tracker.disconnect()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ManusDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

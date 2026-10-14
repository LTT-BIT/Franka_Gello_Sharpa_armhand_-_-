"""Translate Manus PoseArray streams into Sharpa JointState commands."""

from __future__ import annotations

from functools import partial

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState as RosJointState

from litchi_hardware.core.types import HandSide, HandTrackingFrame, Pose
from litchi_hardware.hardware.manus import MANUS_SHARPA_25_SCHEMA

from .analytic_retargeter import AnalyticRetargeter, RetargetingConfig
from .v4_retargeter import SharpaV4Retargeter, V4RetargetingConfig


def pose_array_to_frame(message: PoseArray, side: HandSide) -> HandTrackingFrame:
    timestamp_s = float(message.header.stamp.sec) + (
        float(message.header.stamp.nanosec) * 1e-9
    )
    keypoints = tuple(
        Pose(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        for pose in message.poses
    )
    return HandTrackingFrame(
        side=side,
        keypoints=keypoints,
        timestamp_s=timestamp_s,
        schema=MANUS_SHARPA_25_SCHEMA,
        frame_id=message.header.frame_id or "manus_hand",
    )


class RetargetNode(Node):
    def __init__(self) -> None:
        super().__init__("manus_sharpa_retarget")
        self.declare_parameter("enable_left", True)
        self.declare_parameter("enable_right", True)
        self.declare_parameter("left_input_topic", "/manus/left/keypoints")
        self.declare_parameter("right_input_topic", "/manus/right/keypoints")
        self.declare_parameter("left_command_topic", "/sharpa/left/command")
        self.declare_parameter("right_command_topic", "/sharpa/right/command")
        self.declare_parameter("backend", "v4")
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("flexion_scale", 1.0)
        self.declare_parameter("abduction_scale", 1.0)
        self.declare_parameter("v4_filter_alpha", 1.0)
        self.declare_parameter("v4_worker_python", "")
        self.declare_parameter("max_input_age_s", 0.25)

        backend = str(self.get_parameter("backend").value).lower()
        if backend == "v4":
            worker_python = str(self.get_parameter("v4_worker_python").value)
            self._retargeter = SharpaV4Retargeter(
                V4RetargetingConfig(
                    filter_alpha=float(
                        self.get_parameter("v4_filter_alpha").value
                    ),
                    worker_python=worker_python or None,
                )
            )
        elif backend == "analytic":
            config = RetargetingConfig(
                smoothing_alpha=self.get_parameter("smoothing_alpha").value,
                flexion_scale=self.get_parameter("flexion_scale").value,
                abduction_scale=self.get_parameter("abduction_scale").value,
            )
            self._retargeter = AnalyticRetargeter(config)
        else:
            raise ValueError("backend must be 'v4' or 'analytic'")
        self._max_input_age_s = float(self.get_parameter("max_input_age_s").value)
        self._side_publishers = {}
        self._side_subscriptions = []

        for side in HandSide:
            enabled = self.get_parameter(f"enable_{side.value}").value
            if not enabled:
                continue
            input_topic = self.get_parameter(f"{side.value}_input_topic").value
            command_topic = self.get_parameter(
                f"{side.value}_command_topic"
            ).value
            self._side_publishers[side] = self.create_publisher(
                RosJointState, command_topic, 10
            )
            self._side_subscriptions.append(
                self.create_subscription(
                    PoseArray,
                    input_topic,
                    partial(self._on_keypoints, side),
                    qos_profile_sensor_data,
                )
            )
        if not self._side_publishers:
            raise ValueError("at least one of enable_left or enable_right must be true")
        self._last_warning_ns = 0
        self.get_logger().info(
            f"Manus-to-Sharpa {backend} retargeting active for "
            + ", ".join(side.value for side in self._side_publishers)
        )

    def _on_keypoints(self, side: HandSide, message: PoseArray) -> None:
        try:
            frame = pose_array_to_frame(message, side)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if (
                self._max_input_age_s > 0
                and frame.timestamp_s > 0
                and now_s - frame.timestamp_s > self._max_input_age_s
            ):
                self._warn_throttled(
                    f"dropped stale {side.value} Manus frame "
                    f"({now_s - frame.timestamp_s:.3f}s old)"
                )
                return
            action = self._retargeter.retarget(frame)
            command = RosJointState()
            command.header = message.header
            command.name = list(action.features["joint_names"])
            command.position = list(action.features["joint_position"])
            self._side_publishers[side].publish(command)
        except Exception as exc:
            self._warn_throttled(f"failed to retarget {side.value} hand: {exc}")

    def _warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warning_ns > 1_000_000_000:
            self.get_logger().warning(message)
            self._last_warning_ns = now_ns

    def destroy_node(self) -> bool:
        close = getattr(self._retargeter, "close", None)
        if close is not None:
            close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RetargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""ROS adapter for continuity monitoring and idle RGB readiness; no hardware output."""
from __future__ import annotations

from pathlib import Path
import threading
import time

from .continuity import ContinuityMonitor
from .image_metadata import read_image_metadata
from .camera_stream_health import CameraReadiness


class LiveContinuity:
    def __init__(self, node, *, recording_active=None):
        import yaml
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        required = [topic for topic in node.config.topics if topic.required]
        self.monitor = ContinuityMonitor([topic.topic for topic in required])
        self._recording_active = recording_active or (lambda: self.monitor.active)
        self.readiness = CameraReadiness([topic.topic for topic in required],
            rgb_enabled=getattr(node.config, 'rgb_preflight_enabled', False),
            rgb_threshold=getattr(node.config, 'rgb_brightness_threshold', 25.),
            rgb_stable_sec=getattr(node.config, 'rgb_preflight_stable_sec', 3.))
        self._subscriptions = []
        self._stop = threading.Event()
        self._thread = None
        overrides = {}
        args = list(node.rosbag_record_args)
        for index, arg in enumerate(args):
            if arg == "--qos-profile-overrides-path" and index + 1 < len(args):
                overrides = yaml.safe_load(Path(args[index + 1]).read_text()) or {}
            elif arg.startswith("--qos-profile-overrides-path="):
                overrides = yaml.safe_load(Path(arg.split("=", 1)[1]).read_text()) or {}
        for topic in required:
            if not getattr(node.config, 'continuity_enabled', True) and not self.readiness.observes(topic.topic):
                continue
            profile = overrides.get(topic.topic, node.rosbag_record_default_qos)
            qos = QoSProfile(
                depth=int(profile.get("depth", 10)),
                history=HistoryPolicy.KEEP_ALL if profile.get("history") == "keep_all" else HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT if profile.get("reliability") == "best_effort" else ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if profile.get("durability") == "transient_local" else DurabilityPolicy.VOLATILE,
            )
            message_type = get_message(topic.type_name)

            def callback(serialized, *, item=topic, cls=message_type):
                camera = self.readiness.observes(item.topic)
                if not self.monitor.active and not camera:
                    return
                now = time.monotonic()
                recording = self._recording_active()
                if camera:
                    self.readiness.set_brightness_paused(recording)
                try:
                    if item.type_name == "sensor_msgs/msg/Image":
                        message = read_image_metadata(serialized)
                        if message is None:
                            raise ValueError("unsupported Image CDR format for metadata monitoring")
                        header_ns = message.header_time_ns
                    else:
                        message = deserialize_message(serialized, cls)
                        stamp = message.header.stamp
                        header_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    if camera:
                        brightness = brightness_error = None
                        if not recording and self.readiness.checks_brightness(item.topic):
                            from .rgb_brightness import mean_rgb
                            try:
                                brightness = mean_rgb(message)
                            except ValueError as error:
                                brightness_error = str(error)
                        self.readiness.observe(item.topic, header_ns, now,
                            brightness=brightness, brightness_error=brightness_error)
                    if self.monitor.active:
                        self.monitor.observe_message(item.topic, message, header_ns, now)
                except Exception as error:
                    if camera:
                        self.readiness.observe(item.topic, None, now)
                    if self.monitor.active:
                        self.monitor.invalid(item.topic, f"消息元数据无效: {error}", now)

            self._subscriptions.append(node.create_subscription(message_type, topic.topic, callback, qos, raw=True))

    def start(self, node, controller):
        from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        self._executor = executor

        def run():
            next_check = 0.0
            while not self._stop.is_set():
                try:
                    executor.spin_once(timeout_sec=0.02)
                except ExternalShutdownException:
                    return
                now = time.monotonic()
                # High-rate arm streams must not cause per-message report copies.
                if now >= next_check:
                    self.monitor.watchdog(now)
                    controller.check_continuity()
                    next_check = now + 0.02

        self._thread = threading.Thread(target=run, name="collection-continuity", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._executor.shutdown()

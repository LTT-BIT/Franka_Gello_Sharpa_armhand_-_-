"""Future real-output entry: UDP targets -> existing Sharpa ROS driver topics.

No vendor SDK/arm connection. Output is disabled unless --enable-output is passed.
Requires fresh /sharpa/<side>/joint_states. Run only in the existing ROS environment.
"""
import argparse
import json
import socket
import time

from .retarget import JOINT_NAMES
from .transport import TargetGate, normalize_feedback


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5572)
    parser.add_argument("--enable-output", action="store_true")
    args = parser.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    rclpy.init(args=[])
    node = Node("litchibot_sharpa_relay")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.setblocking(False)
    gate = TargetGate()
    feedback = {}
    publishers = {side: node.create_publisher(JointState, f"/sharpa/{side}/command", 10)
                  for side in ("left", "right")} if args.enable_output else {}

    def read_feedback(side, msg):
        try:
            values = normalize_feedback(side, msg.name, msg.position)
        except ValueError:
            return
        # Arrival alone cannot refresh stale feedback; retain its ROS stamp age.
        stamp_ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age_ns = node.get_clock().now().nanoseconds-stamp_ns
        if not 0 <= age_ns <= 250_000_000:
            return
        feedback[side] = (values, time.monotonic_ns()-age_ns)

    subscriptions = [node.create_subscription(JointState, f"/sharpa/{side}/joint_states",
        lambda msg, side=side: read_feedback(side, msg), qos_profile_sensor_data)
        for side in ("left", "right")]
    warning_at = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            try:
                data, address = sock.recvfrom(16384)
            except BlockingIOError:
                continue
            try:
                packet = json.loads(data)
                if not isinstance(packet, dict):
                    raise ValueError("Packet must be an object")
                if packet.get("schema") == "litchibot.sharpa_stop.v1":
                    side = packet.get("side")
                    if side in ("left", "right"):
                        gate.clear(side)
                    continue
                if not args.enable_output:
                    continue
                q = gate.accept(packet, feedback=feedback.get(packet.get("side")))
                msg = JointState()
                age_ns = time.monotonic_ns()-packet["source_received_monotonic_ns"]
                stamp_ns = node.get_clock().now().nanoseconds-age_ns
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                msg.header.frame_id = f"litchibot:{packet['session_id']}:{packet['sequence']}"
                msg.name = list(JOINT_NAMES)
                msg.position = q.tolist()
                # The only ROS command publication site in the new path.
                publishers[packet["side"]].publish(msg)
            except Exception as error:
                if time.monotonic()-warning_at > 1:
                    node.get_logger().warning(f"Target rejected: {error}")
                    warning_at = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Launch one Orbbec camera with the patched vendor driver."""

import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    serial = os.environ.get("ORBBEC_TEST_SERIAL", "")
    name = os.environ.get("ORBBEC_TEST_NAME", "camera")
    profile_name = "camera_secondary_params.yaml" if serial else "camera_primary_params.yaml"
    profile_path = (
        Path(__file__).resolve().parents[2]
        / "ros_ws/src/teleop_camera_bringup/config"
        / profile_name
    )
    parameters = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    parameters.update({
        # SDK 2.8.6 fails CCP control acquisition when network enumeration is disabled.
        "enumerate_net_device": not bool(serial),
        "net_device_ip": "" if serial else "192.168.1.123",
        "net_device_port": 0 if serial else 8090,
        "serial_number": serial or "CP4N651006F",
        "device_access_mode": "default" if serial else os.environ.get("ORBBEC_TEST_ACCESS_MODE", "ca"),
        # On head-camera firmware 1.3.02, 1 Hz temperature reads coincided with
        # repeated control-transfer disconnects. Keep this off until validated.
        "diagnostic_period": 0.0 if not serial else 1.0,
        "enable_point_cloud": False,
        "enable_colored_point_cloud": False,
    })
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="camera_container",
                namespace=name,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="orbbec_camera",
                        plugin="orbbec_camera::OBCameraNodeDriver",
                        name="camera",
                        namespace=name,
                        parameters=[parameters],
                    )
                ],
                output="screen",
            )
        ]
    )

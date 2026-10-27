from __future__ import annotations

import os
import ipaddress
import uuid
from datetime import datetime, timezone
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


LICENSE_ENVIRONMENT_VARIABLE = "ORBBEC_SDK_LICENSE_ACCEPTED"


def _connection_arguments(name: str, entry: dict) -> dict[str, str]:
    address = str(entry.get("net_device_ip", "")).strip()
    if not address:
        if "net_device_port" in entry:
            raise ValueError(f"{name}: net_device_port requires net_device_ip")
        return {"enumerate_net_device": "true" if name == "cam0" else "false"}
    if name != "cam0":
        raise ValueError("Only cam0 supports network connection in this deployment")
    ipaddress.IPv4Address(address)
    port = int(entry.get("net_device_port", 8090))
    if not 1 <= port <= 65535:
        raise ValueError(f"{name}: invalid network device port")
    return {"enumerate_net_device": "false", "net_device_ip": address, "net_device_port": str(port)}


def _prepare_camera_parameters(parameter_file: Path, camera_name: str, diagnostic_dir: Path) -> Path:
    """Persist a unique diagnostic profile so CSV files survive collector exit."""
    parameters = yaml.safe_load(parameter_file.read_text(encoding="utf-8")) or {}
    if not parameters.get("enable_frame_timestamp_csv", False):
        return parameter_file
    diagnostic_dir.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:12]
    stem = f"{camera_name}_{run_id}"
    parameters["frame_timestamp_csv_file"] = str(diagnostic_dir / f"{stem}.csv")
    runtime_file = diagnostic_dir / f"{stem}_params.yaml"
    with runtime_file.open("x", encoding="utf-8") as stream:
        yaml.safe_dump(parameters, stream, sort_keys=False)
    return runtime_file


def _camera_actions(context):
    if os.environ.get(LICENSE_ENVIRONMENT_VARIABLE) != "YES":
        raise RuntimeError(
            "Orbbec SDK runtime is license-gated. Review "
            "ros_ws/src/orbbec_camera/SDK/End User License Agreement.txt, "
            f"then set {LICENSE_ENVIRONMENT_VARIABLE}=YES only if accepted."
        )
    deployment_path = Path(
        LaunchConfiguration("deployment_config").perform(context)
    ).expanduser()
    config = yaml.safe_load(deployment_path.read_text(encoding="utf-8")) or {}
    cameras = (config.get("camera_bringup") or {})
    expected = {"cam0", "cam1", "cam2"}
    if set(cameras).intersection(expected) != expected:
        raise ValueError("camera deployment config must define cam0, cam1, cam2")
    serials = []
    semantics = []
    for name in sorted(expected):
        entry = cameras[name]
        serial = str(entry.get("serial_number", "")).strip()
        semantic = str(entry.get("semantic", "")).strip()
        if not serial or serial.startswith("REPLACE_"):
            raise ValueError(f"{name} serial_number is not deployed")
        if not semantic or semantic.startswith("REPLACE_"):
            raise ValueError(f"{name} physical semantic is not deployed")
        serials.append(serial)
        semantics.append(semantic)
    if len(set(serials)) != 3 or len(set(semantics)) != 3:
        raise ValueError("camera serials and physical semantics must be unique")

    driver_share = Path(get_package_share_directory("orbbec_camera"))
    driver_launch = driver_share / "launch/gemini435_le.launch.py"
    if not driver_launch.is_file():
        raise RuntimeError(
            "Orbbec runtime was not built. Rebuild after explicit SDK EULA "
            "acceptance with LIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME=ON."
        )
    bringup_share = Path(get_package_share_directory("teleop_camera_bringup"))
    actions = []
    for name in ("cam0", "cam1", "cam2"):
        entry = cameras[name]
        parameter_file = bringup_share / "config" / str(entry["config_file"])
        parameter_file = _prepare_camera_parameters(
            parameter_file, name,
            Path(cameras.get("timestamp_diagnostics_dir", "/collection_data/diagnostics/camera_timestamps")),
        )
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver_launch)),
            launch_arguments={
                "camera_name": name,
                "serial_number": str(entry["serial_number"]),
                **_connection_arguments(name, entry),
                "sync_mode": "standalone",
                "config_file_path": str(parameter_file),
                "log_level": str(cameras.get("log_level", "none")),
                "log_file_name": f"{name}.log",
            }.items(),
        )
        actions.append(
            TimerAction(
                period=float(entry.get("start_delay_sec", 0.0)),
                actions=[GroupAction([include])],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "deployment_config",
                description=(
                    "YAML containing unique camera serials and physical semantics"
                ),
            ),
            OpaqueFunction(function=_camera_actions),
        ]
    )

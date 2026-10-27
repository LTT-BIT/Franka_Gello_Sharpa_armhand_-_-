from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[4]


def test_deployment_config_declares_three_serial_identity_and_semantic_slots():
    path = REPO_ROOT / "data_collection/config/cameras.yaml"
    cameras = yaml.safe_load(path.read_text(encoding="utf-8"))["camera_bringup"]
    assert {"cam0", "cam1", "cam2"} <= set(cameras)
    for name in ("cam0", "cam1", "cam2"):
        assert set(cameras[name]) >= {
            "serial_number",
            "semantic",
            "config_file",
            "start_delay_sec",
        }


def test_build_and_runtime_each_require_explicit_sdk_license_acceptance():
    cmake = (REPO_ROOT / "ros_ws/src/orbbec_camera/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    launch = (
        REPO_ROOT
        / "ros_ws/src/teleop_camera_bringup/launch/triple_camera.launch.py"
    ).read_text(encoding="utf-8")
    assert "LIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME" in cmake
    assert "ORBBEC_SDK_LICENSE_ACCEPTED" in launch
    assert "!= \"YES\"" in launch


def test_camera_direct_ip_and_legacy_discovery_arguments():
    import importlib.util
    import pytest
    path = REPO_ROOT / "ros_ws/src/teleop_camera_bringup/launch/triple_camera.launch.py"
    spec = importlib.util.spec_from_file_location("camera_connections", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    connect = module._connection_arguments
    assert connect("cam0", {}) == {"enumerate_net_device": "true"}
    assert connect("cam1", {}) == {"enumerate_net_device": "false"}
    assert connect("cam0", {"net_device_ip": "192.168.1.123"}) == {
        "enumerate_net_device": "false", "net_device_ip": "192.168.1.123", "net_device_port": "8090"}
    for entry in [{"net_device_ip": "bad"}, {"net_device_ip": "192.168.1.123", "net_device_port": 0},
                  {"net_device_port": 8090}]:
        with pytest.raises(ValueError):
            connect("cam0", entry)
    with pytest.raises(ValueError):
        connect("cam1", {"net_device_ip": "192.168.1.123"})


def test_original_license_notice_and_sdk_eula_are_retained():
    required = [
        "ros_ws/src/orbbec_camera/LICENSE",
        "ros_ws/src/orbbec_camera/NOTICE",
        "ros_ws/src/orbbec_camera/SDK/LICENSE.txt",
        "ros_ws/src/orbbec_camera/SDK/End User License Agreement.txt",
        "ros_ws/src/orbbec_camera_msgs/LICENSE",
        "ros_ws/src/orbbec_camera_msgs/NOTICE",
    ]
    assert all((REPO_ROOT / path).is_file() for path in required)


def test_head_camera_uses_640x400_20hz_rgb_and_raw_depth_preset():
    path = REPO_ROOT / "ros_ws/src/teleop_camera_bringup/config/camera_primary_params.yaml"
    params = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert (params["color_width"], params["color_height"], params["color_fps"]) == (
        640,
        400,
        20,
    )
    assert params["enable_depth"] is True
    assert (params["depth_width"], params["depth_height"], params["depth_fps"]) == (
        640,
        400,
        20,
    )
    assert params["preset_resolution_config"] == "640, 400, 1, 1"
    assert params["depth_format"] == "Y16"
    assert params["depth_registration"] is False
    assert params["enable_depth_undistortion"] is False
    assert params["enable_point_cloud"] is False
    assert params["enable_colored_point_cloud"] is False


def test_wrist_cameras_use_640x480_30hz_rgb_only():
    path = (
        REPO_ROOT
        / "ros_ws/src/teleop_camera_bringup/config/camera_secondary_params.yaml"
    )
    params = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert (params["color_width"], params["color_height"], params["color_fps"]) == (
        640,
        480,
        30,
    )
    assert params["enable_depth"] is False


def test_cam1_global_repair_is_isolated_from_other_camera_profiles():
    root = REPO_ROOT / "ros_ws/src/teleop_camera_bringup/config"
    original = yaml.safe_load((root / "camera_secondary_params.yaml").read_text())
    repaired = yaml.safe_load((root / "camera_cam1_global_params.yaml").read_text())
    assert repaired["time_domain"] == "global"
    assert repaired["enable_sync_host_time"] is True
    assert repaired["enable_frame_timestamp_csv"] is True
    assert repaired["color_format"] == "YUYV"
    assert original["color_format"] == "ANY"
    assert original["enable_sync_host_time"] is True
    assert {k: v for k, v in repaired.items() if k not in {"enable_sync_host_time", "enable_frame_timestamp_csv", "color_format"}} == {
        k: v for k, v in original.items() if k not in {"enable_sync_host_time", "color_format"}}
    for name in ("cameras.yaml", "cameras.yaml.example"):
        cameras = yaml.safe_load((REPO_ROOT / "data_collection/config" / name).read_text())["camera_bringup"]
        assert cameras["cam1"]["config_file"] == "camera_cam1_global_params.yaml"
        assert cameras["cam2"]["config_file"] == "camera_secondary_params.yaml"
        assert cameras["cam0"]["config_file"] == "camera_primary_params.yaml"


def test_timestamp_diagnostics_are_unique_persistent_and_do_not_modify_source(tmp_path):
    import importlib.util
    path = REPO_ROOT / "ros_ws/src/teleop_camera_bringup/launch/triple_camera.launch.py"
    spec = importlib.util.spec_from_file_location("camera_launch_diagnostics", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = REPO_ROOT / "ros_ws/src/teleop_camera_bringup/config/camera_cam1_global_params.yaml"
    before = source.read_text()
    first = module._prepare_camera_parameters(source, "cam1", tmp_path)
    second = module._prepare_camera_parameters(source, "cam1", tmp_path)
    assert first != second
    params = yaml.safe_load(first.read_text())
    assert params["enable_sync_host_time"] == yaml.safe_load(before)["enable_sync_host_time"]
    assert params["time_domain"] == "global"
    assert str(tmp_path) in params["frame_timestamp_csv_file"]
    assert params["frame_timestamp_csv_file"] != yaml.safe_load(second.read_text())["frame_timestamp_csv_file"]
    assert source.read_text() == before
    cam2 = source.parent / "camera_secondary_params.yaml"
    assert module._prepare_camera_parameters(cam2, "cam2", tmp_path) == cam2

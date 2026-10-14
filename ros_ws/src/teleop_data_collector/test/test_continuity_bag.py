"""Real serialized ROS bag checks, without cameras, robots or LeRobot conversion."""
import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from teleop_data_collector.bag_validation import inspect_bag
from teleop_data_collector.collector_contract import TopicContract


def write_camera_bag(path, hz, missing, *, congestion=False, separate_missing=False):
    topic = "/cam0/color/image_raw" if hz == 20 else "/cam1/color/image_raw"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type="sensor_msgs/msg/Image", serialization_format="cdr"))
    image = Image()
    image.height = 2
    image.width = 2
    image.step = 6
    image.encoding = "rgb8"
    image.data = bytes(12)
    skipped = {4 + 10 * i for i in range(missing)} if separate_missing else set(range(4, 4 + missing))
    for i in range(hz * 4):
        # Inject within the first second, which must not hide internal gaps.
        if i in skipped:
            continue
        stamp = 1_000_000_000 + round(i * 1_000_000_000 / hz)
        image.header.stamp.sec = stamp // 1_000_000_000
        image.header.stamp.nanosec = stamp % 1_000_000_000
        receive = stamp + (200_000_000 if congestion and i >= 4 else 0)
        writer.write(topic, serialize_message(image), receive)
    del writer
    return TopicContract("camera", topic, "sensor_msgs/msg/Image", min_frequency_hz=1)


@pytest.mark.parametrize("hz", [20, 30])
@pytest.mark.parametrize("missing", [0, 1, 2, 3])
def test_real_bag_uses_shared_policy_including_untrimmed_internal_gaps(tmp_path, hz, missing):
    path = tmp_path / "bag"
    contract = write_camera_bag(path, hz, missing)
    failures, report = inspect_bag(path, [contract], trim_start_sec=1, trim_end_sec=.2, continuity_enabled=True)
    assert bool(failures) == (missing >= 3)
    assert bool(report["source_warnings"]) == (0 < missing < 3)
    stats = report["continuity"]["streams"][contract.topic]
    assert stats["estimated_missing_frames_max_run"] == missing
    assert stats["source_gap_failure_count"] == int(missing == 3)


def test_transport_only_and_original_mode_compatibility(tmp_path):
    path = tmp_path / "congested"
    contract = write_camera_bag(path, 30, 0, congestion=True)
    failures, report = inspect_bag(path, [contract], continuity_enabled=True)
    assert not failures
    assert not report["source_warnings"]
    assert report["continuity"]["transport_warnings"]
    path = tmp_path / "legacy"
    contract = write_camera_bag(path, 30, 3)
    failures, report = inspect_bag(path, [contract])
    assert not failures  # 133 ms remains within the original 150 ms policy.
    assert report["continuity"] == {}


@pytest.mark.parametrize("hz", [20, 30])
def test_real_bag_three_separate_missing_frames_are_only_warnings(tmp_path, hz):
    path = tmp_path / "separate"
    contract = write_camera_bag(path, hz, 3, separate_missing=True)
    failures, report = inspect_bag(path, [contract], continuity_enabled=True)
    assert not failures
    stats = report["continuity"]["streams"][contract.topic]
    assert stats["estimated_missing_frames_total"] == 3
    assert stats["estimated_missing_frames_max_run"] == 1
    assert stats["source_gap_failure_count"] == 0
    assert stats["source_gap_warning_count"] == 3

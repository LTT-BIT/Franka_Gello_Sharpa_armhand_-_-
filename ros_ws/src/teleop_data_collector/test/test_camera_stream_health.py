from teleop_data_collector.camera_stream_health import StreamWindow


def test_stream_window_requires_full_stable_period_and_frequency():
    window = StreamWindow("/camera", 10.0)
    for index in range(51):
        window.observe(index * 0.1, max_gap_sec=0.15)

    assert window.ready(5.0, stable_sec=5.0, max_gap_sec=0.15)
    assert window.frequency_hz == 10.0


def test_stream_window_resets_after_large_gap():
    window = StreamWindow("/camera", 10.0)
    for index in range(21):
        window.observe(index * 0.1, max_gap_sec=0.15)
    window.observe(2.3, max_gap_sec=0.15)

    assert window.reset_count == 1
    assert window.sample_count == 1
    assert not window.ready(2.3, stable_sec=2.0, max_gap_sec=0.15)


def test_stream_window_becomes_unready_when_frames_stop():
    window = StreamWindow("/camera", 10.0)
    for index in range(21):
        window.observe(index * 0.1, max_gap_sec=0.15)

    assert window.ready(2.0, stable_sec=2.0, max_gap_sec=0.15)
    assert not window.ready(2.2, stable_sec=2.0, max_gap_sec=0.15)
    assert "STALE" in window.describe(2.2, max_gap_sec=0.15)


def test_source_readiness_rejects_clock_regression_despite_regular_receive_rate():
    window = StreamWindow("/cam1/color/image_raw", 25, require_source_timestamps=True)
    for i in range(160):
        window.observe(i / 30, max_gap_sec=.15, header_ns=1_000_000_000 + round(i * 1e9 / 30))
    assert window.ready(159 / 30, stable_sec=5, max_gap_sec=.15)
    window.observe(160 / 30, max_gap_sec=.15, header_ns=500_000_000)
    assert window.source_reset_count == 1
    assert not window.ready(160 / 30, stable_sec=5, max_gap_sec=.15)
    for i in range(1, 161):
        window.observe((160 + i) / 30, max_gap_sec=.15, header_ns=500_000_000 + round(i * 1e9 / 30))
    assert window.ready(320 / 30, stable_sec=5, max_gap_sec=.15)


def test_source_readiness_rejects_invalid_and_frozen_timestamps():
    window = StreamWindow("/cam1/color/image_raw", 25, require_source_timestamps=True)
    for i in range(160):
        window.observe(i / 30, max_gap_sec=.15, header_ns=1_000_000_000)
    assert not window.ready(159 / 30, stable_sec=5, max_gap_sec=.15)
    window.observe(6, max_gap_sec=.15, header_ns=None)
    assert not window.ready(6, stable_sec=5, max_gap_sec=.15)


def test_source_readiness_checks_source_frequency_independently():
    window = StreamWindow("/cam1/color/image_raw", 25, require_source_timestamps=True)
    for i in range(160):
        window.observe(i / 30, max_gap_sec=.15, header_ns=1_000_000_000 + i * 100_000_000)
    assert window.frequency_hz > 25
    assert window.source_frequency_hz == 10
    assert not window.ready(159 / 30, stable_sec=5, max_gap_sec=.15)


def test_pre_episode_camera_gate_revokes_ready_and_waits_after_reconnect():
    from teleop_data_collector.camera_stream_health import CameraReadiness
    topics = ['/cam0/color/image_raw', '/cam0/depth/image_raw']
    gate = CameraReadiness(topics)
    assert not gate.snapshot(0)['ready']
    for i in range(102):
        for topic in topics:
            gate.observe(topic, 1_000_000_000 + i * 50_000_000, i * .05)
    assert gate.snapshot(5.05)['ready']
    assert not gate.snapshot(5.3)['ready']
    # Only color returning cannot conceal a missing depth stream.
    for i in range(102):
        gate.observe(topics[0], 10_000_000_000 + i * 50_000_000, 10 + i * .05)
    assert gate.snapshot(15.05)['waiting_topics'] == [topics[1]]
    for i in range(103):
        for topic in topics:
            gate.observe(topic, 16_000_000_000 + i * 50_000_000, 16 + i * .05)
    assert gate.snapshot(21.1)['ready']
    # Invalid metadata must immediately remove readiness, even while idle.
    gate.observe(topics[0], None, 21.11)
    assert not gate.snapshot(21.11)['ready']


def test_pre_episode_gate_skips_disabled_cameras_and_camera_info():
    from teleop_data_collector.camera_stream_health import CameraReadiness
    gate = CameraReadiness(['/cam0/color/camera_info', '/sharpa/left/joint_states'])
    assert gate.snapshot(0)['ready']
    assert not gate.observes('/cam0/color/camera_info')


def test_episode_camera_gate_requires_three_seconds_not_five():
    from teleop_data_collector.camera_stream_health import CameraReadiness
    gate = CameraReadiness(['/cam0/color/image_raw'])
    for i in range(60):
        gate.observe('/cam0/color/image_raw', 1_000_000_000 + i * 50_000_000, i * .05)
    assert not gate.snapshot(2.95)['ready']
    gate.observe('/cam0/color/image_raw', 4_000_000_000, 3.0)
    status = gate.snapshot(3.0)
    assert status['ready']
    assert status['stable_sec'] == 3

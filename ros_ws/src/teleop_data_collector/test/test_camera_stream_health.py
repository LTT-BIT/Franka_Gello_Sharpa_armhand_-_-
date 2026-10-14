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

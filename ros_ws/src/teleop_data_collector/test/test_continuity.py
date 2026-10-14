from types import SimpleNamespace

import pytest

from teleop_core.contract import COMMAND_STATUS_TOPIC, VALIDATED_COMMAND_TOPIC, LEFT_COMMAND_JOINT_NAMES
from teleop_data_collector.continuity import ContinuityMonitor, ContinuityPolicy, policy_for_topic


@pytest.mark.parametrize("hz", [20, 30])
@pytest.mark.parametrize("missing", [0, 1, 2, 3])
def test_periodic_policy(hz, missing):
    policy = ContinuityPolicy.periodic(hz, "missing_frames")
    gap = (missing + 1) * 1000 / hz
    assert policy.missing(gap) == missing
    assert policy.severity(gap) == (None if missing == 0 else "warning" if missing < 3 else "failure")


@pytest.mark.parametrize("hz", [20, 30])
def test_jitter_boundary(hz):
    policy = ContinuityPolicy.periodic(hz, "missing_frames")
    assert policy.severity(3.5 * 1000 / hz + 4.999) == "warning"
    assert policy.severity(3.5 * 1000 / hz + 5.001) == "failure"


def test_transport_warning_and_silence_are_distinct():
    topic = "/cam0/color/image_raw"
    monitor = ContinuityMonitor([topic])
    monitor.begin("episode0", 0)
    monitor.observe(topic, 1_000_000_000, 0)
    monitor.observe(topic, 1_050_000_000, .2)
    assert monitor.snapshot()["auto_stop_reason"] is None
    assert monitor.snapshot()["alerts"][0]["category"] == "transport"
    monitor.watchdog(1.2)
    assert monitor.snapshot()["auto_stop_reason"] is None
    monitor.watchdog(1.201)
    assert monitor.snapshot()["auto_stop_reason"]["category"] == "silence"


@pytest.mark.parametrize("stamp", [None, 0, 1_000_000_000, 999_999_999])
def test_invalid_source_time(stamp):
    topic = "/sharpa/left/command"
    monitor = ContinuityMonitor([topic])
    monitor.begin("episode0", 0)
    monitor.observe(topic, 1_000_000_000, 0)
    monitor.observe(topic, stamp, .03)
    assert monitor.snapshot()["auto_stop_reason"]["category"] == "timestamp"


def test_first_frame_watchdog_and_episode_reset():
    topic = "/sharpa/left/joint_states"
    monitor = ContinuityMonitor([topic])
    monitor.begin("episode0", 0)
    monitor.watchdog(1.01)
    assert monitor.end()["auto_stop_reason"]
    monitor.observe(topic, 123, 10)
    monitor.begin("episode1", 20)
    monitor.observe(topic, 40_000_000_000, 20)
    assert monitor.snapshot()["streams"][topic]["sample_count"] == 1
    assert monitor.snapshot()["auto_stop_reason"] is None


def test_arm_hold_and_independent_sides():
    monitor = ContinuityMonitor([COMMAND_STATUS_TOPIC, VALIDATED_COMMAND_TOPIC])
    monitor.begin("episode0", 0)
    # No action required when both sides are held.
    for i in range(60):
        monitor.observe_message(COMMAND_STATUS_TOPIC, SimpleNamespace(accepted_sides=[]), 1_000_000_000 + i * 33_333_333, i / 30)
        monitor.watchdog(i / 30)
    assert monitor.snapshot()["auto_stop_reason"] is None
    monitor.observe_message(COMMAND_STATUS_TOPIC, SimpleNamespace(accepted_sides=["left"]), 2_999_999_980, 2)
    monitor.observe_message(VALIDATED_COMMAND_TOPIC, SimpleNamespace(name=LEFT_COMMAND_JOINT_NAMES), 3_001_000_000, 2.001)
    monitor.observe_message(VALIDATED_COMMAND_TOPIC, SimpleNamespace(name=LEFT_COMMAND_JOINT_NAMES), 3_134_333_333, 2.134)
    assert monitor.snapshot()["auto_stop_reason"]["topic"] == f"{VALIDATED_COMMAND_TOPIC}#left"
    assert not monitor.snapshot()["streams"][f"{VALIDATED_COMMAND_TOPIC}#right"]["enabled"]
    # A HOLD interval cannot become a source gap on re-engagement.
    monitor.begin("episode1", 5)
    for accepted, stamp, now in [(["left"], 5_000_000_000, 5), ([], 5_040_000_000, 5.04), (["left"], 9_000_000_000, 9)]:
        monitor.observe_message(COMMAND_STATUS_TOPIC, SimpleNamespace(accepted_sides=accepted), stamp, now)
        monitor.observe_message(VALIDATED_COMMAND_TOPIC, SimpleNamespace(name=LEFT_COMMAND_JOINT_NAMES), stamp + 1_000_000, now + .001)
    assert monitor.snapshot()["streams"][f"{VALIDATED_COMMAND_TOPIC}#left"]["source_gap_failure_count"] == 0


def test_first_action_delay_since_engagement_is_detected():
    monitor = ContinuityMonitor([VALIDATED_COMMAND_TOPIC, COMMAND_STATUS_TOPIC])
    monitor.begin("episode0", 0)
    monitor.observe_message(COMMAND_STATUS_TOPIC, SimpleNamespace(accepted_sides=["left"]), 1_000_000_000, 0)
    monitor.observe_message(VALIDATED_COMMAND_TOPIC, SimpleNamespace(name=LEFT_COMMAND_JOINT_NAMES), 1_200_000_000, .2)
    assert monitor.snapshot()["auto_stop_reason"]["category"] == "source"


@pytest.mark.parametrize("gap, severity", [(250, None), (250.01, "warning"), (500, "warning"), (500.01, "failure")])
def test_tactile_baseline(gap, severity):
    topic = "/sharpa/left/tactile/thumb/deformation"
    policy = policy_for_topic(topic)
    assert policy.kind == "publication_gap"
    assert policy.frequency_hz is None
    assert policy.severity(gap) == severity


def test_tactile_streams_independent_and_bounded():
    topics = [f"/sharpa/{side}/tactile/{finger}/{kind}" for side in ("left", "right") for finger in ("thumb", "index", "middle", "ring", "pinky") for kind in ("wrench", "deformation")]
    monitor = ContinuityMonitor(topics)
    monitor.begin("episode0", 0)
    for topic in topics:
        monitor.observe(topic, 1_000_000_000, 0)
        monitor.observe(topic, 1_050_000_000, .05)
    bad_topic = topics[-1]
    monitor.observe(bad_topic, 1_600_000_000, .6)
    assert monitor.snapshot()["auto_stop_reason"]["topic"] == bad_topic
    assert sum(s["source_gap_failure_count"] for s in monitor.snapshot()["streams"].values()) == 1
    for i in range(2000):
        monitor.observe(bad_topic, 1_650_000_000 + i * 300_000_000, .65 + i * .3)
    report = monitor.end()
    assert len(report["alerts"]) <= len(topics) * 4
    # A subsequent warning cannot erase an earlier failure.
    assert report["failures"] and report["auto_stop_reason"]
    assert report["streams"][bad_topic]["source_gap_warning_count"] >= 1999


def test_clock_regression_diagnostic_preserves_both_timestamps():
    topic = "/cam1/color/image_raw"
    monitor = ContinuityMonitor([topic])
    monitor.begin("episode0", 0)
    monitor.observe(topic, 30_000_000_000, 0)
    monitor.observe(topic, 2_753_502_000, .0667)
    reason = monitor.snapshot()["auto_stop_reason"]
    assert reason["previous_source_ns"] == 30_000_000_000
    assert reason["current_source_ns"] == 2_753_502_000
    assert "倒退 27246.498 ms" in reason["message"]


@pytest.mark.parametrize("hz,topic", [(20, "/cam0/color/image_raw"), (30, "/cam1/color/image_raw")])
@pytest.mark.parametrize("missing_per_gap", [1, 2])
def test_separate_small_gaps_never_fail_based_on_episode_total(hz, topic, missing_per_gap):
    monitor = ContinuityMonitor([topic])
    monitor.begin("episode0", 0)
    stamp = 1_000_000_000
    period = round(1e9 / hz)
    monitor.observe(topic, stamp, 0)
    for _ in range(5):
        stamp += (missing_per_gap + 1) * period
        monitor.observe(topic, stamp, (stamp - 1e9) / 1e9)
        stamp += period  # normal frame separates each loss interval
        monitor.observe(topic, stamp, (stamp - 1e9) / 1e9)
    report = monitor.snapshot()
    assert report["auto_stop_reason"] is None
    stats = report["streams"][topic]
    assert stats["estimated_missing_frames_total"] == 5 * missing_per_gap
    assert stats["estimated_missing_frames_max_run"] == missing_per_gap
    assert stats["source_gap_failure_count"] == 0
    # Only one subsequent interval missing three frames triggers failure.
    stamp += 4 * period
    monitor.observe(topic, stamp, (stamp - 1e9) / 1e9)
    assert monitor.snapshot()["auto_stop_reason"]["estimated_missing_frames"] == 3

from types import SimpleNamespace
import numpy as np
import pytest

from teleop_data_collector.rgb_brightness import mean_rgb, RgbReadiness


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_mean_excludes_row_padding_and_does_not_overflow(encoding):
    data=np.full((480,1928),255,dtype=np.uint8)
    data[:,:1920]=24
    image=SimpleNamespace(height=480,width=640,step=1928,encoding=encoding,data=memoryview(data))
    assert mean_rgb(image)==24


def feed(gate, value=25, start=0., count=91):
    for i in range(count):
        now = start+i/30
        for topic in gate.topics:
            gate.observe(topic, 10**9+round(now*10**9), now, value)
    return now


def test_three_cameras_threshold_recovery_and_stale_frames():
    gate = RgbReadiness(['cam0', 'cam1', 'cam2'])
    assert not gate.snapshot(0)['ready']
    now = feed(gate, 25)
    assert gate.snapshot(now)['ready']
    gate.observe('cam1', 10**9+round((now+.03)*10**9), now+.03, 24)
    result = gate.snapshot(now+.03)
    assert not result['ready'] and 'cam1' in result['message'] and '< 25/255' in result['message']
    now = feed(gate, 100, start=now+.06)
    assert gate.snapshot(now)['ready']
    assert not gate.snapshot(now+.2)['ready']


def test_missing_repeated_and_invalid_frames_reset_brightness_readiness():
    gate = RgbReadiness(['cam0'])
    now = feed(gate)
    for header, mean, error in [(None, 100, None), (1, None, 'bad image'), (1, float('nan'), None)]:
        gate.observe('cam0', header, now+.03, mean, error)
        assert not gate.snapshot(now+.03)['ready']
        now = feed(gate, start=now+.06)
        assert gate.snapshot(now)['ready']
    gate.observe('cam0', 10**9+round(now*10**9), now+.03, 100)
    assert not gate.snapshot(now+.03)['ready']


def test_recording_pauses_check_and_next_episode_requires_new_frames():
    gate = RgbReadiness(['cam0'])
    now = feed(gate)
    assert gate.snapshot(now)['ready']
    gate.set_paused(True)
    feed(gate, 0, start=now+.03)
    assert gate.snapshot(now)['streams']['cam0']['mean'] is None
    gate.set_paused(False)
    assert not gate.snapshot(now)['ready']
    now = feed(gate, start=now+4)
    assert gate.snapshot(now)['ready']
    assert RgbReadiness([]).snapshot(0)['ready']


@pytest.mark.parametrize('patch', [dict(encoding='mono8'),dict(step=1),dict(data=b''),dict(height=0)])
def test_invalid_rgb_is_not_silently_accepted(patch):
    values=dict(height=2,width=2,step=6,encoding='rgb8',data=bytes(12));values.update(patch)
    with pytest.raises(ValueError):mean_rgb(SimpleNamespace(**values))


@pytest.mark.parametrize('dark_camera',[None,0,1,2])
def test_postflight_does_not_reject_dark_cameras(tmp_path,dark_camera):
    rosbag2_py=pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image
    from teleop_data_collector.bag_validation import inspect_bag
    from teleop_data_collector.collector_contract import TopicContract
    from teleop_core.contract import CAMERA_COLOR_TOPIC
    bag=tmp_path/'bag'
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    contracts=[]
    for camera in range(3):
        topic=CAMERA_COLOR_TOPIC.format(index=camera)
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type='sensor_msgs/msg/Image',serialization_format='cdr'))
        contracts.append(TopicContract(f'cam{camera}',topic,'sensor_msgs/msg/Image',min_frequency_hz=1))
    for i in range(121):
        for camera,contract in enumerate(contracts):
            message=Image();message.width=2;message.height=2;message.step=6;message.encoding='rgb8'
            stamp=10_000_000_000+round(i*1e9/30)
            message.header.stamp.sec=stamp//10**9;message.header.stamp.nanosec=stamp%10**9
            message.data=bytes([7 if camera==dark_camera else 120]*12)
            writer.write(contract.topic,serialize_message(message),stamp)
    del writer
    failures, report = inspect_bag(bag, contracts)
    assert 'rgb_brightness' not in report
    assert not any('RGB亮度' in reason for reason in failures)

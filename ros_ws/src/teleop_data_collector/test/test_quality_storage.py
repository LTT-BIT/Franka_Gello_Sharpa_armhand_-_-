from pathlib import Path

import pytest

from teleop_data_collector.quality_storage import allocate_episode, archive_episode, prepare_parent


def test_grades_rename_without_copy_and_reclassify(tmp_path):
    parent = prepare_parent(str(tmp_path / 'saved'))
    stage = allocate_episode(tmp_path / 'original', 'episode', parent)
    stage.mkdir()
    data = stage / 'data.mcap'
    data.write_bytes(b'original bag')
    inode = data.stat().st_ino
    for quality, folder in [('优等', 'A'), ('一般', 'B'), ('报错', 'C'), ('放弃', 'D')]:
        stage = archive_episode(stage, parent, quality)
        assert stage.parent.name == folder
        assert (stage / 'data.mcap').stat().st_ino == inode
        assert archive_episode(stage, parent, quality) == stage
    assert (stage / 'data.mcap').read_bytes() == b'original bag'


def test_collision_never_overwrites_even_empty_destination(tmp_path):
    parent = prepare_parent(str(tmp_path))
    source = parent / 'pending' / 'episode0'
    source.mkdir()
    (source / 'data.mcap').write_bytes(b'bag')
    target = parent / 'A' / source.name
    target.mkdir()
    with pytest.raises(FileExistsError):
        archive_episode(source, parent, '优等')
    assert (source / 'data.mcap').read_bytes() == b'bag'
    assert not list(target.iterdir())


def test_sequence_survives_moves_restart_and_parent_changes(tmp_path):
    base = tmp_path / 'original'
    base.mkdir()
    (base / 'episode51').mkdir()
    parent = prepare_parent(str(tmp_path / 'saved'))
    first = allocate_episode(base, 'episode', parent)
    assert first.name == 'episode52'
    first.mkdir()
    archive_episode(first, parent, '优等')
    other = prepare_parent(str(tmp_path / 'another'))
    assert allocate_episode(base, 'episode', other).name == 'episode53'
    assert allocate_episode(base, 'episode', None).name == 'episode54'
    assert allocate_episode(base, 'episode', parent).name == 'episode55'


def test_invalid_paths_symlinks_and_unmounted_roots_rejected(tmp_path, monkeypatch):
    with pytest.raises(ValueError, match='绝对路径'):
        prepare_parent('relative')
    target = tmp_path / 'external'
    target.mkdir()
    (tmp_path / 'A').symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match='符号链接'):
        prepare_parent(str(tmp_path))
    monkeypatch.setenv('COLLECTION_STORAGE_ROOTS', '["/media"]')
    with pytest.raises(ValueError, match='可写的主机路径范围'):
        prepare_parent(str(tmp_path))


def test_real_rosbag_can_be_read_after_directory_archive(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message, deserialize_message
    from std_msgs.msg import String
    parent = prepare_parent(str(tmp_path / 'saved'))
    source = allocate_episode(tmp_path / 'original', 'episode', parent)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/offline_probe', type='std_msgs/msg/String',
                                               serialization_format='cdr'))
    writer.write('/offline_probe', serialize_message(String(data='offline archive check')), 1_000_000_000)
    del writer
    destination = archive_episode(source, parent, '优等')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(destination), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic, data, stamp = reader.read_next()
    assert topic == '/offline_probe' and stamp == 1_000_000_000
    assert deserialize_message(data, String).data == 'offline archive check'
    assert not reader.has_next()


def test_visible_staging_preserves_legacy_hidden_data_and_numbering(tmp_path):
    parent=prepare_parent(str(tmp_path/'saved'))
    assert (parent/'pending').is_dir() and not (parent/'.pending').exists()
    legacy=parent/'.pending'/'episode91'
    legacy.mkdir(parents=True)
    (legacy/'bag.mcap').write_bytes(b'old interrupted recording')
    stage=allocate_episode(tmp_path/'original','episode',parent)
    assert stage==parent/'pending'/'episode92'
    assert (legacy/'bag.mcap').read_bytes()==b'old interrupted recording'

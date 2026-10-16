"""No ROS or devices: verify the host preflight fails before runtime startup."""
from unittest.mock import Mock
import pytest

from teleop_runtime.dds_buffers import check_receive_buffer, MIN_RECEIVE_BUFFER_BYTES


@pytest.mark.parametrize('value', [MIN_RECEIVE_BUFFER_BYTES, 2 * MIN_RECEIVE_BUFFER_BYTES])
def test_accept_deployed_receive_limit(tmp_path, value):
    path = tmp_path / 'rmem_max'
    path.write_text(f'{value}\n')
    assert check_receive_buffer(path) == value


@pytest.mark.parametrize('value', ['212992', '0', 'not-a-number'])
def test_refuse_small_or_invalid_receive_limit(tmp_path, value):
    path = tmp_path / 'rmem_max'
    path.write_text(value)
    with pytest.raises(RuntimeError):
        check_receive_buffer(path)


def test_unreadable_receive_limit(tmp_path):
    with pytest.raises(RuntimeError, match='Cannot read'):
        check_receive_buffer(tmp_path / 'missing')


def test_unified_preflight_refuses_before_ports_or_docker(monkeypatch):
    from teleop_runtime import dds_buffers, full_launcher
    check = Mock(side_effect=RuntimeError('DDS limit too small'))
    run = Mock()
    socket = Mock()
    monkeypatch.setattr(dds_buffers, 'check_receive_buffer', check)
    monkeypatch.setattr(full_launcher.subprocess, 'run', run)
    monkeypatch.setattr(full_launcher.socket, 'socket', socket)
    with pytest.raises(RuntimeError, match='DDS limit'):
        full_launcher.preflight(5593)
    run.assert_not_called()
    socket.assert_not_called()

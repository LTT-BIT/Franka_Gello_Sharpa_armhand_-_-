import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PySide6.QtWidgets import QApplication
from teleop_runtime.transfer_settings import read_settings,atomic_json,state_root
from apps.operator_gui.transfer_panel import TransferPanel


def test_transfer_controls_persist_without_starting_any_service(tmp_path):
    app=QApplication.instance() or QApplication([])
    panel=TransferPanel()
    try:
        assert not panel.upload_checkbox.isChecked() and not panel.delete_checkbox.isChecked()
        remote=tmp_path/'not-mounted'
        panel.remote_edit.setText(str(remote));panel.save_remote()
        panel.upload_checkbox.setChecked(True)
        panel.delete_checkbox.setChecked(True)
        panel.upload_d_checkbox.setChecked(True)
        config=read_settings()
        assert config['remote_parent']==str(remote)
        assert config['upload_enabled'] and config['delete_enabled'] and config['upload_d_enabled']
        assert not remote.exists() and not state_root().exists()
        atomic_json(state_root()/'sessions/session/status.json',dict(pid=os.getpid(),state='running',
            updated=1,queued=3,message='episode1 · 上传中 A',progress='读回远端校验',bytes=1048576))
        panel.refresh()
        assert '队列：3 段' in panel.status_label.text()
        assert '读回远端校验' in panel.status_label.text()
    finally:
        panel.timer.stop();panel.close();panel.deleteLater();app.processEvents()


def test_preferences_restore_in_new_panel_and_keep_unfinished_edit(tmp_path):
    app=QApplication.instance() or QApplication([])
    first=TransferPanel()
    remote=str(tmp_path/'remote')
    first.remote_edit.setText(remote)
    first.remote_edit.setModified(True)
    first.refresh()
    assert first.remote_edit.text()==remote
    first.upload_checkbox.setChecked(True)
    first.upload_d_checkbox.setChecked(True)
    first.delete_checkbox.setChecked(True)
    first.timer.stop();first.close()
    second=TransferPanel()
    try:
        assert second.remote_edit.text()==remote
        assert all(widget.isChecked() for widget in (second.upload_checkbox,second.upload_d_checkbox,second.delete_checkbox))
        assert not (tmp_path/'remote').exists()
    finally:
        second.timer.stop();second.close()
        first.deleteLater();second.deleteLater();app.processEvents()


def test_mount_status_updates_without_remote_io(tmp_path,monkeypatch):
    app=QApplication.instance() or QApplication([])
    mounted={'value':False}
    def detect(path):
        if not mounted['value']:
            raise RuntimeError('尚未挂载')
        return ('42',str(tmp_path/'remote'),'fuse.sshfs','offline-test')
    monkeypatch.setattr('apps.operator_gui.transfer_panel.mount_for',detect)
    panel=TransferPanel()
    try:
        panel.remote_edit.setText(str(tmp_path/'remote'/'subfolder'))
        assert '未挂载' in panel.remote_label.text()
        assert 'color: black;' in panel.remote_edit.styleSheet()
        mounted['value']=True
        panel.refresh()
        assert '已挂载' in panel.remote_label.text()
        assert 'color: black;' in panel.remote_edit.styleSheet()
        mounted['value']=False
        panel.refresh()
        assert '未挂载' in panel.remote_label.text()
        assert '#c22525' in panel.remote_label.styleSheet()
        assert not (tmp_path/'remote').exists()
    finally:
        panel.timer.stop();panel.close();panel.deleteLater();app.processEvents()


def test_readback_switch_defaults_on_and_persists_off():
    app=QApplication.instance() or QApplication([])
    first=TransferPanel()
    assert first.verify_checkbox.isChecked()
    first.verify_checkbox.setChecked(False)
    assert read_settings()['verify_upload_enabled'] is False
    first.timer.stop();first.close()
    second=TransferPanel()
    try:
        assert not second.verify_checkbox.isChecked()
    finally:
        second.timer.stop();second.close()
        first.deleteLater();second.deleteLater();app.processEvents()

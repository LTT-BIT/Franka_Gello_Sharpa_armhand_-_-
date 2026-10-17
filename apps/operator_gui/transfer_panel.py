"""Shared transfer controls; standalone mode starts neither teleop nor transfers."""
from __future__ import annotations
import sys
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,
                              QLineEdit,QPushButton,QCheckBox,QFileDialog)
from teleop_runtime.transfer_settings import read_settings, update_settings, read_status
from teleop_runtime.transfer_worker import mount_for


class TransferPanel(QWidget):
    local_parent_changed=Signal(str)

    def __init__(self,parent=None):
        super().__init__(parent)
        column=QVBoxLayout(self)
        column.setContentsMargins(0,0,0,0)
        self.upload_heading=QLabel('自动上传设置')
        heading_font=self.upload_heading.font()
        heading_font.setBold(True)
        self.upload_heading.setFont(heading_font)
        column.addWidget(self.upload_heading)
        column.addSpacing(self.fontMetrics().lineSpacing())
        self.remote_label=QLabel('远端父目录（检测中）')
        column.addWidget(self.remote_label)
        row=QHBoxLayout()
        self.remote_edit=QLineEdit()
        self.remote_edit.textChanged.connect(self.refresh_mount_status)
        self.remote_edit.setFocusPolicy(Qt.ClickFocus)
        self.remote_edit.editingFinished.connect(self.save_remote)
        self.remote_edit.returnPressed.connect(self.finish_remote)
        row.addWidget(self.remote_edit,stretch=1)
        browse=QPushButton('浏览…')
        browse.setFocusPolicy(Qt.NoFocus)
        browse.clicked.connect(self.browse)
        row.addWidget(browse)
        column.addLayout(row)
        self.remote_preview=QLabel()
        self.remote_preview.setWordWrap(True)
        self.remote_preview.setTextFormat(Qt.PlainText)
        self.remote_preview.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.remote_preview.setStyleSheet('color: #285d49; background: white; border: 1px solid #b8b8b8; padding: 8px;')
        column.addWidget(self.remote_preview)
        self.remote_edit.textChanged.connect(self.update_remote_preview)
        self.update_remote_preview()
        self.upload_checkbox=QCheckBox('自动上传（A、B）')
        self.upload_d_checkbox=QCheckBox('同时上传 D 类')
        self.upload_d_checkbox.setFocusPolicy(Qt.NoFocus)
        self.upload_d_checkbox.setToolTip('需同时开启自动上传。开启后，D 类只有上传成功才允许自动删除（是否读回校验由下方开关决定）；上传关闭或失败时保留本地。')
        self.verify_checkbox=QCheckBox('上传后读回校验')
        self.verify_checkbox.setFocusPolicy(Qt.NoFocus)
        self.verify_checkbox.setToolTip('开启：读回远端文件并核对 SHA-256；关闭：传输完成后直接归档，省去读回。关闭后若开启自动删除，传输成功即删除本地。当前已开始的任务按开始时的设置收尾。')
        self.verify_checkbox.toggled.connect(lambda value:self.save(verify_upload_enabled=value))
        self.delete_checkbox=QCheckBox('自动删除')
        self.upload_checkbox.setFocusPolicy(Qt.NoFocus)
        self.delete_checkbox.setFocusPolicy(Qt.NoFocus)
        self.delete_checkbox.setToolTip('A/B 及选择上传的 D：远端上传成功后删除本地 bag；开启读回校验时须先校验通过；C 和不上传的 D：本段文件操作完成后直接删除。不处理启动前已有的数据。')
        self.upload_checkbox.toggled.connect(lambda value:self.save(upload_enabled=value))
        self.upload_d_checkbox.toggled.connect(lambda value:self.save(upload_d_enabled=value))
        self.delete_checkbox.toggled.connect(lambda value:self.save(delete_enabled=value))
        column.addWidget(self.upload_checkbox)
        column.addWidget(self.upload_d_checkbox)
        column.addWidget(self.verify_checkbox)
        column.addWidget(self.delete_checkbox)
        column.addSpacing(self.fontMetrics().lineSpacing())
        column.addWidget(QLabel('上传／删除状态'))
        self.status_label=QLabel('等待采集器启动')
        self.status_label.setWordWrap(True)
        self.status_label.setTextFormat(Qt.PlainText)
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        column.addWidget(self.status_label)
        self.remote_edit.textEdited.connect(self.save_remote)
        self.local_parent=''
        self.refresh()
        self.timer=QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def save(self,**changes):
        try:
            if self.remote_edit.isModified():
                value=self.remote_edit.text().strip()
                changes.setdefault('remote_parent',str(Path(value).expanduser()) if value else '')
            update_settings(**changes)
            self.remote_edit.setModified(False)
        except (OSError,ValueError) as error:
            self.status_label.setText(f'设置保存失败：{error}')
            return
        self.refresh()

    def save_remote(self):
        value=self.remote_edit.text().strip()
        if value:
            value=str(Path(value).expanduser())
        self.save(remote_parent=value)

    def finish_remote(self):
        self.save_remote()
        self.remote_edit.clearFocus()
        self.window().setFocus(Qt.OtherFocusReason)

    def browse(self):
        chosen=QFileDialog.getExistingDirectory(self,'选择挂载后的远端父目录',self.remote_edit.text())
        if chosen:
            self.remote_edit.setText(chosen)
            self.finish_remote()

    def update_remote_preview(self):
        value=self.remote_edit.text().strip()
        parent=Path(value).expanduser() if value else None
        self.remote_preview.setText(f"A  优等  →  {parent / 'A' if parent else '待选择'}\nB  一般  →  ../B\nD  放弃  →  ../D")

    def refresh_mount_status(self):
        value=self.remote_edit.text().strip()
        mounted=False
        reason='请选择远端父目录'
        if value:
            path=Path(value).expanduser()
            if not path.is_absolute():
                reason='远端父目录必须是绝对路径'
            else:
                try:
                    info=mount_for(path)
                    mounted=info[2]=='fuse.sshfs'
                    reason=f'挂载点：{info[1]}' if mounted else '所选目录未挂载为 SSHFS'
                except (OSError,ValueError,RuntimeError) as error:
                    reason=str(error)
        self.remote_label.setText('远端父目录（SSHFS 已挂载）' if mounted else '远端父目录（SSHFS 未挂载）')
        color='#285d49' if mounted else '#c22525'
        self.remote_label.setStyleSheet(f'color: {color};')
        self.remote_edit.setStyleSheet('color: black;')
        tooltip=reason+'\n每秒检测本机挂载信息；挂载状态不代表服务器网络读写一定正常。'
        self.remote_label.setToolTip(tooltip)
        self.remote_edit.setToolTip(tooltip)

    def refresh(self):
        self.refresh_mount_status()
        try:
            config=read_settings()
            status=read_status()
        except (OSError,ValueError) as error:
            self.status_label.setText(f'后台状态读取失败：{error}')
            return
        if not self.remote_edit.hasFocus() and not self.remote_edit.isModified():
            self.remote_edit.setText(config['remote_parent'])
            self.remote_edit.setCursorPosition(0)
        for widget,key in ((self.upload_checkbox,'upload_enabled'),(self.upload_d_checkbox,'upload_d_enabled'),(self.verify_checkbox,'verify_upload_enabled'),(self.delete_checkbox,'delete_enabled')):
            widget.blockSignals(True);widget.setChecked(config[key]);widget.blockSignals(False)
        if config['local_parent']!=self.local_parent:
            self.local_parent=config['local_parent']
            self.local_parent_changed.emit(self.local_parent)
        lines=[str(status.get('message','等待采集器启动'))]
        if status.get('queued'):
            lines.append(f"队列：{status['queued']} 段")
        if status.get('progress'):
            lines.append(f"{status['progress']} · {status.get('bytes',0)/1024**2:.1f} MiB")
        if status.get('last_result') and status['last_result']!=lines[0]:
            lines.append(str(status['last_result']))
        self.status_label.setText('\n'.join(lines))
        self.status_label.setMinimumHeight(self.status_label.fontMetrics().lineSpacing()*len(lines)+4)
        self.updateGeometry()
        self.status_label.setToolTip(str(status.get('current','')))
        self.status_label.setStyleSheet('color: #c22525;' if status.get('error') else 'color: #285d49;')


def main():
    app=QApplication(sys.argv)
    window=QWidget()
    window.setWindowTitle('采集上传设置（不启动遥操或录制）')
    layout=QVBoxLayout(window)
    layout.addWidget(QLabel('本机保存父目录'))
    row=QHBoxLayout()
    local=QLineEdit(read_settings()['local_parent'])
    row.addWidget(local)
    browse=QPushButton('浏览…');row.addWidget(browse)
    layout.addLayout(row)
    def save_local():
        value=local.text().strip()
        update_settings(local_parent=str(Path(value).expanduser()) if value else '')
    def choose_local():
        value=QFileDialog.getExistingDirectory(window,'选择本机保存父目录',local.text())
        if value:local.setText(value);save_local()
    browse.clicked.connect(choose_local)
    local.editingFinished.connect(save_local)
    local.textEdited.connect(save_local)
    panel=TransferPanel()
    layout.addWidget(panel)
    panel.local_parent_changed.connect(lambda value:local.setText(value) if not local.hasFocus() else None)
    note=QLabel('设置会保留。上传后台随采集器启动；首次启动只建立基线，不传输或删除旧数据。')
    note.setWordWrap(True);layout.addWidget(note)
    window.resize(460,380);window.show()
    return app.exec()

if __name__=='__main__':raise SystemExit(main())

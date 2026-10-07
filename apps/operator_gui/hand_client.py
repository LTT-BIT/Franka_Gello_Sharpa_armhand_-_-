"""Operator transport only; the remote MotionLatch decides motion state."""
import json
import time
from PySide6.QtCore import QObject,QTimer,Signal
from PySide6.QtNetwork import QTcpSocket,QAbstractSocket


class HandClient(QObject):
    status=Signal(dict)
    event=Signal(str)
    def __init__(self,port,token,parent=None,*,normal_forwarding=False):
        super().__init__(parent)
        self.port=port;self.token=token;self.held=False;self.sequence=0;self.id=0
        self.pending={};self.buffer=b'';self.last_response=None;self.last_status={}
        self.last_delay_warning=-float('inf')
        self.normal_forwarding=normal_forwarding
        self.socket=QTcpSocket(self)
        self.socket.connected.connect(self.connected)
        self.socket.readyRead.connect(self.read)
        self.socket.disconnected.connect(self.offline)
        self.socket.errorOccurred.connect(lambda unused:self.offline())
        self.timer=QTimer(self);self.timer.setInterval(50);self.timer.timeout.connect(self.tick);self.timer.start()

    def connected(self):
        self.socket.setSocketOption(QAbstractSocket.LowDelayOption,1)
        self.last_response=time.monotonic()

    def request(self,command,arguments=None):
        if self.socket.state()!=QAbstractSocket.ConnectedState:
            self.event.emit('Hand supervisor offline');return
        self.id+=1;self.pending[self.id]=command
        row={'id':self.id,'command':command,'arguments':arguments or {},'token':self.token}
        if command=='heartbeat':
            self.sequence+=1;row.update(sequence=self.sequence,monotonic_ns=time.monotonic_ns())
        self.socket.write((json.dumps(row)+'\n').encode())
        self.socket.flush()

    def tick(self):
        if self.socket.state()==QAbstractSocket.UnconnectedState:
            self.socket.connectToHost('127.0.0.1',self.port);return
        if self.last_response is not None and time.monotonic()-self.last_response>.20:
            if self.normal_forwarding or self.last_status.get('normal_forwarding'):
                # Delayed heartbeat replies are diagnostics; only actual TCP loss
                # takes the normal forwarding switch offline.
                if time.monotonic()-self.last_delay_warning>=2:
                    self.last_delay_warning=time.monotonic()
                    self.event.emit('Hand supervisor heartbeat reply delayed (diagnostic only)')
                return
            self.socket.abort();self.offline();return
        if 'heartbeat' not in self.pending.values():self.request('heartbeat',{'hold':self.held})

    def read(self):
        self.buffer+=bytes(self.socket.readAll())
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            try:row=json.loads(line)
            except ValueError:continue
            command=self.pending.pop(row.get('id'),'?');self.last_response=time.monotonic()
            if row.get('ok'):
                self.last_status=row['result'];self.status.emit(row['result'])
            else:self.event.emit(f'[{command}] rejected: '+str(row.get('error')))

    def offline(self):
        self.held=False;self.pending.clear();self.buffer=b'';self.last_response=None
        self.status.emit({'hands':{s:{'state':'OFFLINE','permit':bool(self.last_status.get('hands',{}).get(s,{}).get('permit'))} for s in ('left','right')},'reason':'Hand supervisor disconnected'})

    def close(self):
        self.request('disengage_all');self.socket.flush();self.timer.stop();self.socket.abort()

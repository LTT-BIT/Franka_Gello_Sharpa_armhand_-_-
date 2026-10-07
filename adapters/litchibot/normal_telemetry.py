"""Bounded, coalesced diagnostics. No SDK calls and no authority mutations."""
from collections import OrderedDict
import threading


class DeferredTelemetry:
    def __init__(self):
        self.pending=OrderedDict();self.condition=threading.Condition()
        self.stopping=False;self.errors=0;self.dropped=0
        self.thread=threading.Thread(target=self.run,name='normal-telemetry',daemon=True)
        self.thread.start()

    def submit(self,key,job):
        with self.condition:
            if self.stopping:return
            if key not in self.pending and len(self.pending)>=32:
                victim=next((k for k in self.pending if not self.hard_key(k)),None)
                if victim is None:self.dropped+=1;return
                self.pending.pop(victim);self.dropped+=1
            self.pending[key]=job
            if self.hard_key(key):self.pending.move_to_end(key,last=False)
            self.condition.notify()

    @staticmethod
    def hard_key(key):
        return isinstance(key,tuple) and len(key)>1 and key[0]=='event' and key[1][0]=='HARD_FAULT'

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.pending or self.stopping)
                if not self.pending:return
                _,job=self.pending.popitem(last=False)
            try:job()
            except Exception:self.errors+=1

    def close(self):
        with self.condition:
            self.stopping=True;self.condition.notify_all()
        self.thread.join(timeout=1)

"""One persistent native-enable worker per side; never stores/sends targets."""
import threading


class NativeReenable:
    def __init__(self,operations):
        self.condition=threading.Condition();self.stopping=False
        self.operations=operations;self.states={s:'IDLE' for s in operations};self.errors={s:'' for s in operations}
        self.threads={s:threading.Thread(target=self.run,args=(s,),name='native-enable-'+s,daemon=True) for s in operations}
        for t in self.threads.values():t.start()

    def state(self,side):
        with self.condition:return self.states[side],self.errors[side]

    def start(self,side):
        with self.condition:
            if self.stopping or self.states[side]!='IDLE':return False
            self.states[side]='RUNNING';self.condition.notify_all();return True

    def retry(self,side):
        with self.condition:
            if self.states[side]=='FAILED':self.states[side]='IDLE';self.errors[side]=''

    def consumed(self,side):
        with self.condition:
            if self.states[side]=='READY':self.states[side]='IDLE'

    def run(self,side):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.stopping or self.states[side]=='RUNNING')
                if self.stopping:return
            try:self.operations[side]();result,error='READY',''
            except Exception as e:result,error='FAILED',str(e)
            with self.condition:
                self.states[side]=result;self.errors[side]=error

    def close(self):
        with self.condition:self.stopping=True;self.condition.notify_all()
        # Teardown must wait before original stop/disconnect: a running native
        # call cannot be safely cancelled or allowed to enable after disconnect.
        for t in self.threads.values():t.join()

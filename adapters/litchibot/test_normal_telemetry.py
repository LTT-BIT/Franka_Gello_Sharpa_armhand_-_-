import threading
from adapters.litchibot.normal_telemetry import DeferredTelemetry


def test_slow_diagnostics_coalesce_and_preserve_first_hard_root():
    worker=DeferredTelemetry();entered=threading.Event();release=threading.Event();done=[]
    def slow():entered.set();release.wait(1)
    try:
        worker.submit('slow',slow);assert entered.wait(1)
        worker.submit(('event',('HARD_FAULT','left','root')),lambda:done.append('root'))
        for i in range(100):worker.submit(('diagnostic',i),lambda:None)
        for i in range(100):worker.submit('status',lambda i=i:done.append(i))
        assert len(worker.pending)==32
        release.set();worker.close()
        assert done==['root',99] and worker.dropped>0 and worker.errors==0
    finally:release.set();worker.close()

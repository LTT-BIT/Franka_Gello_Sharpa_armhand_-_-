"""Local operator hold-to-run panel. Does not connect to hardware or publish joints."""
import argparse
import json
from pathlib import Path
import socket
import sys
import time
import tkinter as tk

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from adapters.litchibot.validation_safety import load_permit,control_paths


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--permit',required=True)
    permit=load_permit(parser.parse_args().permit)
    socket_path,status_path=control_paths(permit)
    transport=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    window=tk.Tk();window.title('Sharpa supervised hardware validation — hold to run')
    held=False;sequence=0
    text=tk.StringVar(value='Waiting for guarded driver readiness')
    tk.Label(window,textvariable=text,wraplength=500).pack(padx=20,pady=12)
    def send(hold,reason=''):
        nonlocal sequence
        sequence+=1
        transport.sendto(json.dumps({'token':permit['session_token'],'hold':hold,
            'sequence':sequence,'monotonic_ns':time.monotonic_ns(),'reason':reason}).encode(),str(socket_path))
    def release(event=None):
        nonlocal held
        if held:
            held=False
            try:send(False,'Operator released hold / lost focus')
            except OSError:pass
    def press(event):
        nonlocal held
        held=True
    button=tk.Button(window,text='按住：允许小范围验收（松开即锁定停机）',state='disabled')
    button.pack(padx=20,pady=12)
    button.bind('<ButtonPress-1>',press)
    window.bind_all('<ButtonRelease-1>',release)
    window.bind('<FocusOut>',release)
    def tick():
        nonlocal held
        try:
            status=json.loads(status_path.read_text())
            fresh=0<=time.time()-status['unix_s']<=0.20
            ready=fresh and status['phase'] in ('READY','ARMED')
            button.configure(state='normal' if ready else 'disabled')
            text.set(f"{status['phase']}: {status['reason']} | hardware command calls={status['hardware_command_calls']}")
            if held and ready:
                send(True)
            elif held:
                release()
        except (OSError,ValueError,KeyError) as error:
            release();button.configure(state='disabled');text.set('Guard unavailable: '+str(error))
        window.after(50,tick)
    def close():
        release();transport.close();window.destroy()
    window.protocol('WM_DELETE_WINDOW',close)
    tick();window.mainloop()


if __name__=='__main__':main()

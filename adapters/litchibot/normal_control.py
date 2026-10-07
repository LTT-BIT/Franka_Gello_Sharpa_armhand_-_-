"""Normal LitchiBot forwarding authority; deliberately NOT a validation latch."""
import numpy as np
from .retarget import JOINT_NAMES, JOINT_LIMITS
from .normal_continuity import DeltaDistribution
from .validation_safety import TIMEOUT_S


class RecoverableCondition(ValueError):
    """Reject/pause, without latching or changing the operator permit."""


class UnrecoverableCommand(RecoverableCondition):
    """Legacy name: this invalid packet must be dropped, not latched."""


class BasicNormalTarget:
    """Final-target integrity + per-packet freshness. No motion/validity-count gate."""
    threshold=None
    def __init__(self):
        self.previous={};self.context={};self.distribution=DeltaDistribution()

    def clear(self,side):
        pass  # Stop forwarding does not restart retarget or reset observation history.

    def observe(self,p,now_ns=None):
        self.accept(p,now_ns)
        return True

    def accept(self,p,now_ns=None):
        import time
        now_ns=time.monotonic_ns() if now_ns is None else now_ns
        if not isinstance(p,dict) or p.get('schema')!='litchibot.sharpa_target.v1':
            raise UnrecoverableCommand('Malformed final target envelope')
        side=p.get('side')
        if side not in ('left','right'):raise UnrecoverableCommand('Wrong final command hand side')
        if tuple(p.get('joint_names',()))!=JOINT_NAMES:raise UnrecoverableCommand('Malformed final 22D joint order')
        try:q=np.asarray(p.get('positions_rad'),dtype=float)
        except (ValueError,TypeError) as e:raise UnrecoverableCommand('Malformed final command values') from e
        self.context[side]=dict(joint=None,source_joint=None,source_value=None,target_value=None,delta=None,dt=None)
        if q.shape==(22,):
            bad=np.flatnonzero(~np.isfinite(q) | (q<JOINT_LIMITS[:,0]) | (q>JOINT_LIMITS[:,1]))
            if len(bad):self.context[side].update(joint=JOINT_NAMES[int(bad[0])],target_value=q[int(bad[0])])
        if q.shape!=(22,) or not np.isfinite(q).all():raise UnrecoverableCommand('Final command requires 22 finite radians; NaN/Inf blocked')
        if np.any(q<JOINT_LIMITS[:,0]) or np.any(q>JOINT_LIMITS[:,1]):raise UnrecoverableCommand('Final physical joint-limit violation')
        stamp=p.get('source_received_monotonic_ns')
        age=(now_ns-stamp)/1e9 if type(stamp) is int else None
        self.context[side]['dt']=age
        if type(stamp) is not int or not 0<=(now_ns-stamp)/1e9<=TIMEOUT_S:
            raise RecoverableCondition('Stale/future individual source packet rejected')
        seq=p.get('sequence');session=p.get('session_id')
        if type(seq) is not int or seq<0 or not isinstance(session,str) or not session:
            raise RecoverableCondition('Source identity metadata unavailable')
        old=self.previous.get(side)
        if old and session==old[2] and (seq<=old[3] or stamp<=old[4]):
            raise RecoverableCondition('Repeated/out-of-order source packet rejected')
        adjacent=old and session==old[2] and 0<(stamp-old[4])/1e9<=TIMEOUT_S
        delta=np.abs(q-old[0]) if adjacent else None
        try:a=np.asarray(p.get('glove_joint_angles_rad',[]),dtype=float)
        except (ValueError,TypeError):a=np.array([])
        if delta is not None:
            self.distribution.add(side,'target',delta)
            if a.shape==(20,) and old[1].shape==(20,) and np.isfinite(a).all() and np.isfinite(old[1]).all():
                with np.errstate(over='ignore',invalid='ignore'):source_delta=np.abs(a-old[1])
                if np.isfinite(source_delta).all():self.distribution.add(side,'source',source_delta)
        j=int(np.argmax(delta)) if delta is not None else 0
        self.context[side]=dict(joint=JOINT_NAMES[j],source_joint=None,source_value=None,
                                target_value=q[j],delta=delta[j] if delta is not None else None,
                                dt=(stamp-old[4])/1e9 if adjacent else None)
        self.previous[side]=(q.copy(),a.copy(),session,seq,stamp)
        return q.copy()  # Existing vendor fallback is already in q; no rewrite/clamp.


class NormalHandAuthority:
    """Only per-side forwarding permits and recoverable data/control availability."""
    normal_path=True
    def __init__(self,configuration,now,**unused):
        self.configuration=configuration;self.started=now;self.phase='READY';self.reason=''
        self.requested={s:False for s in ('left','right')}
        self.samples={s:0 for s in self.requested};self.side_reasons={s:'' for s in self.requested}
        self.source_seen={};self.valid_seen={};self.data_ready={s:True for s in self.requested}
        self.control_ready=True;self.control_seen=None;self.connection=None;self.sequence=None

    def request_hand(self,side,enabled,now):
        if side not in self.requested:raise RecoverableCondition('Unknown hand permit side')
        if enabled and self.phase=='FAULT':raise UnrecoverableCommand(self.reason)
        self.requested[side]=bool(enabled)

    def note_source(self,side,now):
        self.source_seen[side]=now

    def valid_data(self,side,now):
        self.valid_seen[side]=now;self.samples[side]+=1;self.data_ready[side]=True
        self.side_reasons[side]=''

    def pause_data(self,side,reason):
        self.data_ready[side]=False;self.side_reasons[side]=str(reason)
        self.requested[side]=False

    def pause_control(self,reason):
        self.control_ready=False;self.sequence=None;self.reason=str(reason)
        self.requested={s:False for s in self.requested}

    def control_connected(self):
        self.control_ready=True
        if self.phase!='FAULT':self.reason=''

    def gui_packet(self,p,now):
        stamp=p.get('monotonic_ns');seq=p.get('sequence');generation=p.get('_connection_generation',0)
        if generation!=self.connection:
            self.connection=generation;self.sequence=None;self.control_connected()
        if type(stamp) is not int or not 0<=now-stamp/1e9<=TIMEOUT_S or type(seq) is not int or (self.sequence is not None and seq<=self.sequence):
            raise RecoverableCondition('GUI stale/replayed heartbeat (diagnostic only)')
        self.sequence=seq;self.control_seen=stamp/1e9;self.control_ready=True
        if self.phase!='FAULT':self.reason=''

    def side_state(self,side,now):
        if self.phase=='FAULT':return 'OFFLINE'
        if not self.control_ready or not self.data_ready[side]:return 'OFFLINE'
        if not self.requested[side]:return 'READY'
        return 'ACTIVE'

    def watch(self,now,unused=None):
        if self.phase=='FAULT':return
        for side in self.requested:
            if now-self.valid_seen.get(side,self.started)>TIMEOUT_S:
                self.pause_data(side,'Waiting for fresh usable source packet')

    def trip(self,reason):
        if self.phase=='FAULT':return
        self.phase='FAULT';self.reason=str(reason)
        self.requested={s:False for s in self.requested}

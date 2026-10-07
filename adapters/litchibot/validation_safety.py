"""Fail-closed supervised acceptance policy. No hardware or ROS imports."""
import json
import os
from pathlib import Path
import time

import numpy as np

from .retarget import JOINT_NAMES, JOINT_LIMITS
from .transport import validate_packet

TIMEOUT_S = 0.20
JUMP_RAD = 0.10
DIAGNOSTIC_JUMP_RAD = 0.18
EXCURSION_RAD = 0.15
SPEED_RAD_S = 0.5
WARMUP_FRAMES = 30
SOURCE_NAMES = ('thumb_cmc_yaw','thumb_cmc_swing','thumb_mcp_flex','thumb_pip_flex',
                'index_mcp_swing','index_mcp_flex','index_pip_flex','index_dip_flex',
                'middle_mcp_swing','middle_mcp_flex','middle_pip_flex','middle_dip_flex',
                'ring_mcp_swing','ring_mcp_flex','ring_pip_flex','ring_dip_flex',
                'pinky_mcp_swing','pinky_mcp_flex','pinky_pip_flex','pinky_dip_flex')


def load_permit(path, now=None):
    p = Path(path)
    st = p.stat()
    if not p.is_file() or st.st_uid != os.getuid() or st.st_mode & 0o077:
        raise ValueError('Validation permit must be owned by operator account, mode 0600')
    data = json.loads(p.read_text())
    required = ('adult_qualified_operator','sop_reviewed','physical_estop_checked',
                'workspace_clear','known_pinky_risk_acknowledged','connect_auto_enable_acknowledged')
    if data.get('mode') != 'supervised_hardware_validation' or any(data.get(k) is not True for k in required):
        raise ValueError('Explicit qualified-operator/SOP/risk attestations required')
    if not str(data.get('operator','')).strip() or not str(data.get('sop_reference','')).strip():
        raise ValueError('Named operator and actual SOP reference required')
    now = time.time() if now is None else now
    expiry = data.get('expires_unix_s')
    if not isinstance(expiry,(int,float)) or not np.isfinite(expiry) or not 0 < expiry-now <= 900:
        raise ValueError('Permit must expire within 15 minutes')
    serials = data.get('serials',{})
    if set(serials) != {'left','right'} or any(not isinstance(v,str) or not v.strip() for v in serials.values()) or serials['left']==serials['right']:
        raise ValueError('Distinct explicit left/right device serials required')
    token = data.get('session_token','')
    if not isinstance(token,str) or len(token)!=64 or any(c not in '0123456789abcdef' for c in token):
        raise ValueError('Fresh 32-byte hex session token required')
    if not isinstance(data.get('profile'),str) or not data['profile']:
        raise ValueError('Explicit glove profile required')
    return data


def control_paths(permit):
    directory = Path('/tmp') / f'litchibot-validation-{os.getuid()}'
    directory.mkdir(mode=0o700,exist_ok=True)
    st = directory.stat()
    if directory.is_symlink() or st.st_uid != os.getuid() or st.st_mode & 0o077:
        raise ValueError('Private validation control directory required')
    stem = permit['session_token'][:16]
    return directory / (stem+'.sock'), directory / (stem+'.json')


class RawContinuityGate:
    """Check EVERY worker frame before latest-frame coalescing or slew limiting."""
    def __init__(self, *, diagnostic=False):
        self.previous = {}
        self.threshold = DIAGNOSTIC_JUMP_RAD if diagnostic else JUMP_RAD

    def observe(self, packet, now_ns=None):
        q = validate_packet(packet,now_ns=now_ns,max_age_s=TIMEOUT_S,allow_dry_run=True)
        if tuple(packet.get('glove_joint_names',()))!=SOURCE_NAMES:
            raise ValueError('Source joint names/order mismatch')
        masks = (packet.get('valid_target_mask'),packet.get('valid_glove_joint_mask'))
        if any(not isinstance(m,list) or len(m)!=n or any(v is not True for v in m)
               for m,n in zip(masks,(22,20))):
            raise ValueError('All source and target joints must be valid for hardware validation')
        if packet.get('glove_status')!='solved' or packet.get('valid_glove_joint_count')!=20 or packet.get('held_joints') or packet.get('initial_fallback_joints') or packet.get('clipped_joints'):
            raise ValueError('Solved, no hold/fallback/schema clipping required')
        angles = np.asarray(packet.get('glove_joint_angles_rad'),dtype=float)
        if angles.shape!=(20,) or not np.isfinite(angles).all():
            raise ValueError('20 finite source joint angles required')
        side = packet['side']; previous = self.previous.get(side)
        if previous:
            old,old_angles,session,sequence,stamp = previous
            if packet['session_id']!=session or packet['sequence']<=sequence or packet['source_received_monotonic_ns']<=stamp:
                raise ValueError('Source session/order changed')
            if (packet['source_received_monotonic_ns']-stamp)/1e9>TIMEOUT_S:
                raise ValueError('Source gap watchdog expired')
            delta = np.abs(q-old)
            accepted = self.check_delta(side, delta, np.abs(angles-old_angles), packet['source_received_monotonic_ns'])
        self.previous[side] = (q.copy(),angles.copy(),packet['session_id'],packet['sequence'],packet['source_received_monotonic_ns'])
        return accepted if previous else True

    def check_delta(self, side, delta, source_delta, stamp_ns):
        if delta.max()>=self.threshold or (source_delta is not None and source_delta.max()>=self.threshold):
            name = JOINT_NAMES[int(np.argmax(delta))]
            raise ValueError(f'Raw discontinuity: {side} {name}, target delta={delta.max():.6f} rad')
        return True

    def observe_ros_target(self, side, names, positions, stamp_ns, ros_now_ns, sequence, session, now_ns):
        """Manus V4 target boundary. Caller must verify matching 25-keypoint input."""
        packet={'schema':'litchibot.sharpa_target.v1','dry_run':True,'engaged':True,
            'side':side,'joint_names':names,'positions_rad':positions,'valid_glove_joint_count':25,
            'source_received_monotonic_ns':now_ns-(ros_now_ns-stamp_ns),
            'sequence':sequence,'session_id':session}
        q=validate_packet(packet,now_ns=now_ns,max_age_s=TIMEOUT_S,allow_dry_run=True)
        old=self.previous.get(side)
        if old:
            if sequence<=old[3] or session!=old[2] or stamp_ns<=old[4]:
                raise ValueError('Manus source order/session mismatch')
            if (stamp_ns-old[4])/1e9>TIMEOUT_S:
                raise ValueError('Manus raw discontinuity/source timeout')
            accepted = self.check_delta(side, np.abs(q-old[0]), None, stamp_ns)
        self.previous[side]=(q.copy(),None,session,sequence,stamp_ns)
        return accepted if old else True


class MotionLatch:
    """Both-hand latched stop; restart required after any trip or deadman release."""
    def __init__(self, permit, now, *, gui_controlled=False, source='litchibot',validation_only=True):
        self.permit = permit
        self.phase = 'WARMUP'
        self.reason = ''
        self.started = now
        self.armed_at = None
        self.last_hold = None
        self.control_sequence = -1
        self.last = {}
        self.baseline = {}
        self.measured = {}
        self.commanded = {}
        self.samples = {'left':0,'right':0}
        self.gui_controlled=gui_controlled
        self.source=source
        self.validation_only=validation_only
        self.requested={'left':False,'right':False}
        self.last_gui=None
        self.recovery_started=None

    def gui_heartbeat(self, now):
        self.last_gui=now

    def request_hand(self, side, enabled, now):
        if not self.gui_controlled or side not in self.requested:
            raise ValueError('Managed hand side required')
        if enabled:
            if self.phase in ('FAULT','RECOVERING'):raise ValueError(self.reason)
            if self.requested[side]:return
            if self.last_gui is None or now-self.last_gui>TIMEOUT_S:
                raise ValueError('Live GUI lease required')
            if self.samples[side]<WARMUP_FRAMES or side not in self.measured:
                raise ValueError('Hand acquisition not ready')
            if not self.validation_only or self.phase!='ARMED':
                self.baseline[side]=self.measured[side].copy()
        self.requested[side]=bool(enabled)
        if self.gui_controlled and not self.validation_only and not any(self.requested.values()) and self.phase=='ARMED':
            self.phase='READY';self.last_hold=None;self.armed_at=None
        self.commanded.pop(side,None)
        if side in self.last:
            _,identity,stamp=self.last[side]
            self.last[side]=(self.measured[side].copy(),identity,stamp)

    def side_state(self, side, now):
        if self.phase in ('FAULT','RECOVERING'):return self.phase
        if self.phase=='READY' and self.reason.startswith('Recovered'):return 'READY'
        if not self.gui_controlled:return self.phase
        if not self.requested[side]:return 'DISABLED'
        if self.phase=='ARMED' and self.last_gui is not None and now-self.last_gui<=TIMEOUT_S:
            return 'ACTIVE'
        return 'READY'

    def pause(self, reason):
        if self.validation_only or not self.gui_controlled:
            raise ValueError('Soft recovery only belongs to managed normal mode')
        if self.phase=='FAULT':return
        self.phase='RECOVERING';self.reason=str(reason)
        self.recovery_started=time.monotonic()
        self.requested={'left':False,'right':False}
        self.samples={'left':0,'right':0}
        self.commanded.clear();self.last_hold=None;self.armed_at=None

    def trip(self, reason):
        if self.phase=='FAULT':return
        self.phase = 'FAULT'; self.reason = str(reason)
        if self.gui_controlled:self.requested={'left':False,'right':False}

    def heartbeat(self, message, now):
        if self.phase=='FAULT':
            return
        if message.get('token')!=self.permit['session_token']:
            self.trip('Invalid supervisor session'); return
        stamp = message.get('monotonic_ns')
        seq = message.get('sequence')
        if not isinstance(stamp,int) or not 0 <= now-stamp/1e9 <= TIMEOUT_S or not isinstance(seq,int) or seq<=self.control_sequence:
            self.trip('Stale/replayed supervisor heartbeat'); return
        self.control_sequence = seq
        if message.get('hold') is not True:
            self.trip(message.get('reason','Supervisor released hold-to-run')); return
        if self.phase not in ('READY','ARMED'):
            self.trip('Supervisor attempted arm before readiness'); return
        self.last_hold = now
        if self.phase=='READY':
            if any(now-self.last[s][2]>TIMEOUT_S for s in ('left','right')):
                self.trip('Stale acquisition at arm'); return
            self.baseline = {s:self.measured[s].copy() for s in self.last}
            self.phase = 'ARMED'; self.armed_at = now

    def watch(self, now, wall_now):
        if self.gui_controlled and any(self.requested.values()) and (self.last_gui is None or now-self.last_gui>TIMEOUT_S):
            self.trip('GUI enable lease expired')
        if self.validation_only and wall_now >= self.permit['expires_unix_s']:
            self.trip('Permit expired')
        if self.phase=='ARMED':
            if now-self.last_hold>TIMEOUT_S:
                self.trip('Deadman watchdog expired')
            elif self.validation_only and now-self.armed_at>=60:
                self.trip('60-second acceptance window completed')
        if self.phase in ('READY','ARMED','RECOVERING') and any(
                now-(self.last[s][2] if s in self.last else self.recovery_started)>TIMEOUT_S
                for s in ('left','right')):
            self.trip('Command watchdog expired')
        if self.phase=='WARMUP' and now-self.started>30:
            self.trip('Acquisition timeout')

    def command(self, side, names, positions, stamp_ns, ros_now_ns, frame_id, now, feedback):
        if self.phase=='FAULT':
            raise ValueError(self.reason)
        if side not in ('left','right') or tuple(names)!=JOINT_NAMES:
            raise ValueError('Left/right joint names/order mismatch')
        fields = frame_id.split(':')
        if len(fields)!=4 or fields[:2]!=[self.source,side] or not fields[2] or not fields[3].isdigit():
            raise ValueError('Missing side/session/sequence identity')
        if not 0 <= (ros_now_ns-stamp_ns)/1e9 <= TIMEOUT_S:
            raise ValueError('Stale/future ROS command')
        q = np.asarray(positions,dtype=float); measured = np.asarray(feedback,dtype=float)
        if any(v.shape!=(22,) or not np.isfinite(v).all() for v in (q,measured)):
            raise ValueError('Expected finite 22D command and feedback')
        if any(np.any(v<JOINT_LIMITS[:,0]) or np.any(v>JOINT_LIMITS[:,1]) for v in (q,measured)):
            raise ValueError('Command/feedback joint limit violation')
        previous = self.last.get(side)
        if previous:
            old,identity,last_time = previous
            if fields[2]!=identity[0] or int(fields[3])<=identity[1]:
                raise ValueError('ROS session/order changed')
            if now-last_time>TIMEOUT_S:
                raise ValueError('ROS command gap watchdog expired')
            if self.validation_only and np.abs(q-old).max()>=JUMP_RAD:
                raise ValueError('ROS single-frame discontinuity')
        forwarding=self.phase=='ARMED' and (not self.gui_controlled or self.requested[side])
        if forwarding:
            if self.validation_only and np.abs(q-self.baseline[side]).max()>EXCURSION_RAD:
                raise ValueError('Acceptance excursion exceeds 0.15 rad')
            actual = self.commanded.get(side)
            base,dt = (actual[0],min(0.05,now-actual[1])) if actual else (measured,1/30)
            if np.abs(q-base).max()>SPEED_RAD_S*dt+1e-8:
                raise ValueError('Hardware command slew/acquisition exceeded')
        elif (not self.gui_controlled or self.requested[side]) and np.abs(q-measured).max()>EXCURSION_RAD:
            raise ValueError('Acquisition target is too far from measured pose')
        self.last[side] = (q.copy(),(fields[2],int(fields[3])),now)
        self.measured[side] = measured.copy()
        self.samples[side] += 1
        if self.phase in ('WARMUP','RECOVERING') and min(self.samples.values())>=WARMUP_FRAMES:
            recovered=self.phase=='RECOVERING'
            self.phase = 'READY'
            if recovered:self.reason='Recovered stable input; GUI Start hand required again'
        self.watch(now,time.time())
        if forwarding and self.phase=='ARMED':
            self.commanded[side]=(q.copy(),now)
        return forwarding and self.phase=='ARMED'

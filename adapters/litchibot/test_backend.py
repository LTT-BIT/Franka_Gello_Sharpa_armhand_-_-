import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from adapters.litchibot.backend import LitchiBotHandPipeline
from adapters.litchibot.retarget import JOINT_LIMITS, JOINT_NAMES, SharpaRetargeter
from adapters.litchibot.transport import TargetGate, validate_packet, normalize_feedback

ROOT = Path(__file__).resolve().parents[2]
CANONICAL = ('thumb_cmc_yaw', 'thumb_cmc_swing', 'thumb_mcp_flex', 'thumb_pip_flex') + tuple(
    f'{finger}_{channel}' for finger in ('index','middle','ring','pinky')
    for channel in ('mcp_swing','mcp_flex','pip_flex','dip_flex'))


def pose(side='left', value=0.1):
    return SimpleNamespace(side=SimpleNamespace(value=side), status=SimpleNamespace(value='solved'),
        joint_angles_rad=np.full(20, value), valid_joint_mask=np.ones(20, dtype=bool),
        kinematic_hand_pose=dict.fromkeys(CANONICAL, value),
        thumb_cmc_orientation_palm_xyzw=np.array([0.,0.,0.,1.]), node_positions_root_m=np.zeros((20,3)))


class Session:
    def __init__(self):
        self.started = self.closed = False
    def subscribe_solved(self, callback):
        self.callback = callback
    def subscribe_errors(self, callback):
        self.error_callback = callback
    def start(self):
        self.started = True
    def close(self):
        self.closed = True


def pipeline(**kwargs):
    session = Session()
    p = LitchiBotHandPipeline(session=session, sdk=SimpleNamespace(CANONICAL_JOINT_NAMES=CANONICAL), **kwargs)
    return p, session


def frame(poses, timestamp=10**9, seq=1):
    return SimpleNamespace(hands=poses, sequence=seq, source_raw_sequence=seq,
        source_id='receiver:test', source_received_monotonic_ns=timestamp,
        solved_monotonic_ns=timestamp+10**6, source_device_timestamp_ns=123,
        profile_id='LYG226360006')


def test_dry_run_never_constructs_sender_even_when_engaged(monkeypatch, tmp_path):
    monkeypatch.setattr('adapters.litchibot.backend.time.monotonic_ns', lambda: 1_010_000_000)
    factory = Mock(side_effect=AssertionError('dry-run reached transport'))
    p, session = pipeline(sender_factory=factory, debug_log=tmp_path/'run.jsonl')
    session.callback(frame([pose('left',0.1), pose('right',0.7)]))
    p.tick(active={'left':True,'right':True})
    assert p.last_command['left']['positions_rad'] != p.last_command['right']['positions_rad']
    for side in ('left','right'):
        row = p.last_command[side]
        assert row['side']==side and row['source_received_monotonic_ns']==10**9
        assert row['source_device_timestamp_ns']==123
        assert row['joint_names']==list(JOINT_NAMES) and len(row['positions_rad'])==22
        assert row['sent'] is False
        json.dumps(row, allow_nan=False)
    p.close()
    factory.assert_not_called()
    assert session.closed
    records=[json.loads(l) for l in (tmp_path/'run.jsonl').read_text().splitlines()]
    assert [r['type'] for r in records]==['metadata','target','target','summary']
    assert records[-1]['sent']==0


def test_partial_joint_holds_only_its_dependent_target():
    mapper=SharpaRetargeter('left')
    good=pose(value=0.3)
    previous=mapper.map(good,CANONICAL,10**9)
    partial=pose(value=0.7)
    partial.valid_joint_mask[2:4]=False
    partial.joint_angles_rad[2:4]=np.nan
    partial.kinematic_hand_pose['thumb_mcp_flex']=float('nan')
    partial.kinematic_hand_pose['thumb_pip_flex']=float('nan')
    result=mapper.map(partial,CANONICAL,2*10**9)
    assert result['valid_glove_joint_count']==18
    assert set(result['held_joints'])=={'thumb_MCP_FE','thumb_IP'}
    for name in result['held_joints']:
        i=JOINT_NAMES.index(name)
        assert result['positions_rad'][i]==previous['positions_rad'][i]
    assert result['positions_rad'][5]>previous['positions_rad'][5]
    assert np.isfinite(result['positions_rad']).all()
    with pytest.raises(ValueError,match='Cross-side'):
        mapper.map(pose('right'),CANONICAL,3*10**9)


def test_startup_invalid_is_explicit_fallback_not_valid_zero():
    partial=pose();partial.valid_joint_mask[2:4]=False
    result=SharpaRetargeter('left').map(partial,CANONICAL,10**9)
    assert result['initial_fallback_joints']==['thumb_MCP_FE','thumb_IP']
    assert not result['valid_target_mask'][2] and not result['valid_target_mask'][4]


def test_stale_disconnected_malformed_and_sdk_error_do_not_crash(monkeypatch):
    clock=[1_010_000_000]
    monkeypatch.setattr('adapters.litchibot.backend.time.monotonic_ns',lambda:clock[0])
    p,s=pipeline()
    p.tick(active={})
    assert 'waiting' in p.status.sides['left'].fault
    s.callback(frame([pose()]))
    p.tick(active={});assert p.statistics['left']['count']==1
    p.tick(active={});assert p.statistics['left']['count']==1  # no stale-frame repetition
    clock[0]=2*10**9
    p.tick(active={});assert 'stale' in p.status.sides['left'].fault
    bad=pose();bad.joint_angles_rad=np.zeros(19)
    s.callback(frame([bad,pose('right')],timestamp=clock[0],seq=2))
    p.tick(active={})
    assert p.status.sides['left'].fault and p.last_command['right'] is not None
    s.error_callback(RuntimeError('device disconnected'))
    assert 'device disconnected' in p.status.last_error
    p.tick(active={})
    p.close()


def packet():
    return dict(schema='litchibot.sharpa_target.v1', dry_run=False, side='left',
        source_received_monotonic_ns=1_000_000_000, joint_names=list(JOINT_NAMES),
        positions_rad=[0.5]*22, engaged=True, valid_glove_joint_count=20,
        valid_target_mask=[True]*22, session_id='test',sequence=1)


def test_real_send_requires_engagement_and_dry_run_false(monkeypatch):
    monkeypatch.setattr('adapters.litchibot.backend.time.monotonic_ns',lambda:1_010_000_000)
    sender=Mock();factory=Mock(return_value=sender)
    p,s=pipeline(dry_run=False,sender_factory=factory)
    s.callback(frame([pose()]))
    p.tick(active={});factory.assert_not_called()
    s.callback(frame([pose()],seq=2))
    p.tick(active={'left':True});sender.send.assert_called_once()
    p.tick(active={'left':False});sender.disengage.assert_called_once_with('left')
    p.close()


def test_relay_rejects_bad_shape_order_freshness_and_limits():
    q=packet();q['positions_rad']=[0.1]*22
    validate_packet(q,now_ns=1_010_000_000)
    for patch in ({'joint_names':list(reversed(JOINT_NAMES))}, {'positions_rad':[0]*21},
                  {'positions_rad':[float('nan')]*22}, {'dry_run':True}, {'engaged':False},
                  {'positions_rad':[10]*22}):
        with pytest.raises(ValueError):validate_packet({**q,**patch},now_ns=1_010_000_000)
    with pytest.raises(ValueError):validate_packet(q,now_ns=2_000_000_000)


def test_relay_requires_feedback_and_limits_acquisition_slew():
    p=packet();p['positions_rad']=[0.1]*22;p['positions_rad'][5]=1.4
    gate=TargetGate()
    with pytest.raises(ValueError,match='feedback'):gate.accept(p,feedback=None,now_ns=1_010_000_000)
    p['valid_target_mask'][2]=False
    feedback=(np.zeros(22),1_000_000_000)
    q=gate.accept(p,feedback=feedback,now_ns=1_010_000_000)
    assert q[5]<=2/30+1e-9 and q[2]==0
    with pytest.raises(ValueError,match='order'):gate.accept(p,feedback=feedback,now_ns=1_020_000_000)
    p['sequence']=2
    with pytest.raises(ValueError,match='feedback'):gate.accept(p,feedback=(np.zeros(22),0),now_ns=1_020_000_000)


def test_existing_sharpa_ros_feedback_prefixes_are_routed_and_reordered():
    names=['left_'+n for n in reversed(JOINT_NAMES)]
    values=list(reversed(range(22)))
    assert normalize_feedback('left',names,values)==list(range(22))
    with pytest.raises(ValueError):normalize_feedback('right',names,values)


def test_manus_files_and_calibration_unchanged():
    baseline=json.loads((ROOT/'docs/litchibot/manus_baseline.json').read_text())
    for rel,expected in baseline.items():
        if rel=='ops/run/run_sharpa_hands_cyclonedds.sh':
            continue  # Authorized Terminal 2 source switch; separately regression-tested.
        assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==expected, rel


def test_manufacturer_core_functions_are_unchanged():
    provenance=json.loads((ROOT/'adapters/litchibot/vendor/provenance.json').read_text())
    for entry in provenance:
        if not entry['destination'].endswith('.py'):
            continue
        source=Path(entry['source'])
        if not source.exists():
            pytest.skip('Source comparison requires manufacturer GUI checkout')
        original=ast.parse(source.read_text())
        staged=ast.parse((ROOT/entry['destination']).read_text())
        def methods(tree):
            return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(tree)
                    if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name!='map_hand_frame'}
        assert methods(original)==methods(staged)


def test_mapping_units_limits_and_flex_direction():
    for side in ('left','right'):
        mapper=SharpaRetargeter(side)
        a=mapper.map(pose(side,0),CANONICAL,10**9)
        b=mapper.map(pose(side,0.5),CANONICAL,2*10**9)
        c=mapper.map(pose(side,0),CANONICAL,3*10**9)
        assert b['positions_rad'][5] > a['positions_rad'][5]
        np.testing.assert_allclose(a['positions_rad'],c['positions_rad'])
        assert np.all(np.array(b['positions_rad'])>=JOINT_LIMITS[:,0])
        assert np.all(np.array(b['positions_rad'])<=JOINT_LIMITS[:,1])


def test_value_preserving_target_gate_has_no_shared_speed_cap_but_keeps_validation():
    p=packet();p['positions_rad']=[.1]*22;p['positions_rad'][5]=1.4
    gate=TargetGate(max_speed_rad_s=None)
    feedback=(np.zeros(22),1_010_000_000)
    q=gate.accept(p,feedback=feedback,now_ns=1_010_000_000)
    np.testing.assert_array_equal(q,p['positions_rad'])  # initial acquisition is not clamped
    p['sequence']=2;p['positions_rad'][5]=0
    q=gate.accept(p,feedback=feedback,now_ns=1_010_000_001)
    np.testing.assert_array_equal(q,p['positions_rad'])  # even a 1ns adjacent interval
    with pytest.raises(ValueError,match='order'):gate.accept(p,feedback=feedback,now_ns=1_010_000_001)
    for patch in ({'positions_rad':[float('nan')]*22},{'positions_rad':[10]*22},
                  {'joint_names':list(reversed(JOINT_NAMES))},{'positions_rad':[0]*21}):
        with pytest.raises(ValueError):TargetGate(max_speed_rad_s=None).accept({**p,**patch},feedback=feedback,now_ns=1_010_000_001)
    with pytest.raises(ValueError,match='feedback'):TargetGate(max_speed_rad_s=None).accept(p,feedback=None,now_ns=1_010_000_001)
    with pytest.raises(ValueError,match='feedback'):TargetGate(max_speed_rad_s=None).accept(p,feedback=(np.zeros(22),0),now_ns=1_010_000_001)

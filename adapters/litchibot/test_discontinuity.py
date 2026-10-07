import json
from types import SimpleNamespace

import numpy as np

from adapters.litchibot.diagnostics import DiagnosticLog
from adapters.litchibot.analyze_diagnostic_layers import rotation_delta
from adapters.litchibot.analyze_discontinuity import analyze
from adapters.litchibot.retarget import SharpaRetargeter
from adapters.litchibot.test_backend import CANONICAL,pose


def test_diagnostics_do_not_change_retarget_or_hold_values():
    for side in ('left','right'):
        plain,observed=SharpaRetargeter(side),SharpaRetargeter(side)
        for i,angle in enumerate((0.0,0.2,1.65,0.0)):
            p=pose(side,angle)
            p.valid_joint_mask[2]=False
            a=plain.map(p,CANONICAL,(i+1)*10**9)
            b=observed.map(p,CANONICAL,(i+1)*10**9,diagnostics=True)
            stages=b.pop('diagnostic_stages')
            assert a==b
            assert stages['final_target_rad']==a['positions_rad']
            assert 'index_MCP_FE' in stages['mapping_rules']


def test_diagnostic_logger_keeps_full_frames_and_handles_nonfinite(tmp_path):
    logger=DiagnosticLog(tmp_path/'diagnostics.jsonl')
    logger.raw(SimpleNamespace(to_dict=lambda:{'orientation_wxyz':np.array([1,0,0,0])}))
    logger.solved(SimpleNamespace(to_dict=lambda:{'residual':float('nan')}))
    logger.close();logger.close()
    rows=[json.loads(l) for l in (tmp_path/'diagnostics.jsonl').read_text().splitlines()]
    assert rows[1]['data']['orientation_wxyz']==[1,0,0,0]
    assert rows[2]['data']['residual'] is None
    assert rotation_delta([1,0,0,0],[-1,0,0,0])==0


def test_offline_analysis_uses_adjacent_same_side_frames(tmp_path):
    path=tmp_path/'targets.jsonl'
    rows=[]
    for i in range(12):
        for side in ('left','right'):
            q=[0.0]*22
            q[18]=1.5708 if side=='left' and i<6 else 0.0
            rows.append({'type':'target','side':side,'positions_rad':q,'joint_names':[str(k) for k in range(22)],
                         'sequence':i,'source_received_monotonic_ns':10**9+i*33_333_333})
    path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    stats=analyze(path)['sides']
    assert stats['left']['peak_event']['joint_index_0based']==18
    assert stats['left']['peak_event']['frame_index_0based']==6
    assert stats['left']['peak_event']['abs_delta_rad']==1.5708
    assert stats['left']['per_joint'][18]['anomaly_count']==1
    assert stats['right']['max_abs_delta_rad']==0

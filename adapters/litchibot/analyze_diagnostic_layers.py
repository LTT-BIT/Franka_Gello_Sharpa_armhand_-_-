"""Join opt-in raw/solved/target diagnostics by SDK identity, not nearest time."""
import argparse
import json
from pathlib import Path

import numpy as np

from adapters.litchibot.analyze_discontinuity import analyze


def rotation_delta(a,b):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    if not np.isfinite(a).all() or not np.isfinite(b).all() or min(np.linalg.norm(a),np.linalg.norm(b))<1e-10:
        return None
    dot=abs(float(a @ b)/(np.linalg.norm(a)*np.linalg.norm(b)))
    return float(2*np.arccos(np.clip(dot,0,1)))


def inspect_layers(diagnostic_path, target_path):
    targets=analyze(target_path)
    raw,solved={},{}
    with Path(diagnostic_path).open() as stream:
        for line in stream:
            record=json.loads(line);kind,data=record['type'],record['data']
            if kind=='raw':raw[(data['source_id'],data['sequence'])]=data
            elif kind=='solved':solved[(data['source_id'],data['sequence'])]=data
    result={}
    for side,stats in targets['sides'].items():
        context=stats['peak_context'];event=stats['peak_event'];index=event['frame_index_0based']
        before=next(r for r in context if r['frame_index_0based']==index-1)
        after=next(r for r in context if r['frame_index_0based']==index)
        hands=[];raw_frames=[]
        for row in (before,after):
            frame=solved.get((row['source_id'],row['sequence']))
            hands.append(next((h for h in frame['hands'] if h['side']==side),None) if frame else None)
            raw_frames.append(raw.get((row['source_id'],row['source_raw_sequence'])))
        raw_deltas={}
        if all(raw_frames):
            a={s['sensor_id']:s for s in raw_frames[0]['samples'] if s['side']==side}
            b={s['sensor_id']:s for s in raw_frames[1]['samples'] if s['side']==side}
            raw_deltas={str(k):{'rotation_delta_rad':rotation_delta(a[k]['orientation_wxyz'],b[k]['orientation_wxyz']),
                               'before':a[k],'after':b[k]} for k in a.keys() & b.keys()}
        result[side]={'peak_event':event,'target_before':before,'target_after':after,
                      'solved_hand_before':hands[0],'solved_hand_after':hands[1],
                      'raw_before':raw_frames[0],'raw_after':raw_frames[1],
                      'raw_same_side_sensor_rotations':raw_deltas}
        if all(hands):
            result[side]['solved_node_rotation_delta_rad']=[rotation_delta(a,b) for a,b in
                zip(hands[0]['node_orientations_root_xyzw'],hands[1]['node_orientations_root_xyzw'])]
            q=[h.get('thumb_cmc_orientation_palm_xyzw') for h in hands]
            result[side]['thumb_cmc_rotation_delta_rad']=rotation_delta(*q) if all(v is not None for v in q) else None
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('diagnostics');parser.add_argument('targets');parser.add_argument('--output',required=True)
    args=parser.parse_args()
    Path(args.output).write_text(json.dumps(inspect_layers(args.diagnostics,args.targets),indent=2)+'\n')

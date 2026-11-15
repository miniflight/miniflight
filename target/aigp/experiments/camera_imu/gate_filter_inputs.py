"""Keep accepted public pixel tracks; discard the previous navigator's states."""
import argparse,gzip,json
from pathlib import Path
import numpy as np


def export(source,destination):
    with gzip.open(source/'control.jsonl.gz','rt') as stream:rows=list(map(json.loads,stream))
    with np.load(source/'public-inputs.npz',allow_pickle=False) as data:
        sent=data['motor_sent_host_time'];motors=data['motor_outputs']
    if len(rows)!=len(sent) or not np.array_equal([r['motor_commands'] for r in rows],motors):
        raise ValueError('Cannot pair controller decisions with own sends exactly')
    selected=[];seen=set()
    for row,host in zip(rows,sent):
        gate=row.get('gate_number',1)
        for kind,field,corners in [('aperture','camera_observation','corners'),('outer_top','partial_gate','top_corners')]:
            o=row.get(field)
            if o is None or o.get('gate_number',gate)!=gate:continue
            identity=(o['image_id'],gate,kind)
            if identity in seen:continue
            seen.add(identity)
            quality=dict(reprojection_px=o['reprojection_rmse_px']) if kind=='aperture' else dict(edge_residual_px=o['edge_residual_px'],template=o['template'])
            selected.append(dict(host_time=float(host),event='visual_measurement',image_id=o['image_id'],capture_time=o['capture_time'],
                kind=kind,points=o[corners],context_gate=gate,quality=quality))
    with gzip.open(destination,'wt') as stream:
        for row in selected:stream.write(json.dumps(row,separators=(',',':'))+'\n')
    return dict(observations=len(selected),full=sum(r['kind']=='aperture' for r in selected),partial=sum(r['kind']=='outer_top' for r in selected),
        timing='Available by this send-return; paired commands match exactly.',states_exported=0)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('destination',type=Path);args=parser.parse_args()
    print(json.dumps(export(args.source,args.destination),indent=2))

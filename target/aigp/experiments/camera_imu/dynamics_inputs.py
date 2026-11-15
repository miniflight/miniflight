"""Extract a calibration input without estimator translation or native reports."""
import argparse,gzip,json
from pathlib import Path
import numpy as np


def export(source,destination):
    rows=[json.loads(line) for line in gzip.open(source,'rt')]
    control={r['imu_time']:r for r in rows if r.get('camera_orientation') is not None and r.get('motor_commands') is not None}
    control=list(control.values());seen=set();images=[]
    for row in rows:
        image=row.get('camera_observation')
        if image and image['image_id'] not in seen:
            seen.add(image['image_id']);images.append((image['capture_time'],row['gate_number'],image['corners']))
    np.savez_compressed(destination,time=np.array([r['imu_time'] for r in control]),
        camera_orientation=np.array([r['camera_orientation'] for r in control]),motors=np.array([r['motor_commands'] for r in control]),
        image_time=np.array([r[0] for r in images]),image_gate=np.array([r[1] for r in images]),corners=np.array([r[2] for r in images]))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('destination',type=Path)
    args=parser.parse_args();export(args.source,args.destination)

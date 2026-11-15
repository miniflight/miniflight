"""End-to-end measurements. Public inputs are exported before scoring references."""
import json,gzip
from pathlib import Path
import numpy as np


def statistics(values):
    values=np.asarray(values,dtype=float);values=values[np.isfinite(values)]
    if not len(values):return dict(count=0,median=None,p95=None,maximum=None,mean=None)
    return dict(count=len(values),median=float(np.median(values)),p95=float(np.quantile(values,.95)),maximum=float(max(values)),mean=float(np.mean(values)))


def collect(path):
    imu=[];camera=[];sends=[];profiles=[];updates=[];arms=[];observations=[]
    with path.open() as stream:
        for row in map(json.loads,stream):
            event=row.get('event')
            if event=='public_imu':
                imu.extend([[s['time'],s['received_at'],*s['acceleration'],*s['gyro']] for s in row['samples']])
            elif event=='public_camera':camera.append(row)
            elif event=='profile':profiles.append(row)
            elif event=='visual_measurement':observations.append(row)
            elif event=='update':updates.append(row)
            elif event=='call':
                if row['name']=='send' and row['arguments'].get('kind')=='MotorControls':sends.append(row)
                elif row['name']=='arm':arms.append(row)
    values=np.asarray(imu,dtype=float).reshape(-1,8)
    arrays=dict(imu_time=values[:,0],imu_received_at=values[:,1],acceleration=values[:,2:5],gyro=values[:,5:8],
        frame_id=np.array([r['image_id'] for r in camera],dtype=np.int64),
        frame_header_time_ns=np.array([r['header_time_ns'] for r in camera],dtype=np.int64),
        frame_time_estimated_ns=np.array([-1 if r['estimated_time_ns'] is None else r['estimated_time_ns'] for r in camera],dtype=np.int64),
        frame_received_at=np.array([r['received_at'] for r in camera]),
        frame_shape_hw=np.array([r['shape'][:2] for r in camera],dtype=np.int32).reshape(-1,2),
        motor_sent_host_time=np.array([r['host_time'] for r in sends]),
        motor_outputs=np.array([r['arguments']['outputs'] for r in sends]).reshape(-1,4))
    return arrays,dict(profiles=profiles,updates=updates,sends=sends,arms=arms,observations=observations)

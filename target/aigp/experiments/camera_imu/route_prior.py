"""A prior route learned from public pixels; used for planning, never filter state."""
import hashlib,json
import numpy as np
from scipy.spatial.transform import Rotation


def load(path):
    raw=path.read_bytes();data=json.loads(raw)
    if set(data)!={'frame','source','gates'}:raise ValueError('Expected a public estimated-map export')
    allowed={'id','center','yaw','covariance','last_seen'};ids=[]
    for gate in data['gates']:
        if set(gate)!=allowed:raise ValueError('Unexpected route fields')
        if np.asarray(gate['center']).shape!=(3,) or not np.isfinite(gate['center']).all():raise ValueError('Invalid route centre')
        ids.append(gate['id'])
    if sorted(ids)!=list(range(1,len(ids)+1)):raise ValueError('Route IDs must be consecutive public mission slots')
    return dict(sha256=hashlib.sha256(raw).hexdigest(),source=str(path),points={str(g['id']):g['center'] for g in data['gates']},
        use='Planning only. Public learned relative route; no navigation-state initialization or native reference.')


def projected(prior,current,key,before,center):
    points=prior['points'];origin=points.get(str(current));target=points.get(str(key))
    if origin is None or target is None:return None
    incoming=np.asarray(origin)-np.asarray(points.get(str(current-1),[0.,0.,0.]));actual=np.asarray(center)-before
    if min(np.linalg.norm(incoming[:2]),np.linalg.norm(actual[:2]))<1e-6:return None
    yaw=np.arctan2(actual[1],actual[0])-np.arctan2(incoming[1],incoming[0])
    return np.asarray(center)+Rotation.from_euler('z',yaw).apply(np.asarray(target)-origin)

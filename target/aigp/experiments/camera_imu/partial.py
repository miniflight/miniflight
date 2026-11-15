"""Two visible frame corners constrain an upright gate's relative position."""
import numpy as np
from .vision import K_INV


def relative_from_top(top,camera,width,center_to_top):
    rays=np.c_[top,np.ones(2)] @ K_INV.T @ camera.T
    vertical=rays[:,2]
    if np.min(abs(vertical)/np.linalg.norm(rays,axis=1))<.04 or np.prod(vertical)<=0:return None
    edge=vertical[0]*rays[1]-vertical[1]*rays[0]
    length=np.linalg.norm(edge)
    if length<1e-9:return None
    depths=width/length*abs(vertical[::-1])
    center=np.mean(rays*depths[:,None],axis=0)+[0.,0.,center_to_top]
    return center if np.all(depths>0) and np.linalg.norm(center)<50. else None

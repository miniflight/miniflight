"""Local down direction from the image edges of upright gates."""
import numpy as np
from scipy.spatial.transform import Rotation
from .vision import K_INV


def vertical_planes(corners,camera):
    rays=np.c_[corners,np.ones(4)] @ K_INV.T
    normals=np.array([np.cross(rays[a],rays[b]) for a,b in [(0,3),(1,2)]])
    lengths=np.linalg.norm(normals,axis=1)
    if np.min(lengths)<1e-9:return None
    return normals/lengths[:,None] @ camera.T


def fit_down(planes):
    _,singular,v=np.linalg.svd(planes,full_matrices=False)
    down=v[-1]*(1. if v[-1,2]>=0 else -1.)
    return down,float(singular[-1]/np.sqrt(len(planes))),float(singular[1]/max(singular[0],1e-9))


class GateGravity:
    def __init__(self):
        self.rows=[]
        self.gate=None

    def observe(self,stamp,corners,camera,reprojection,gate):
        if gate!=self.gate:self.rows=[];self.gate=gate
        side=float(np.min(np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1)))
        if side<80. or reprojection>.5:return None
        planes=vertical_planes(corners,camera)
        if planes is None:return None
        self.rows=[r for r in self.rows if stamp-r[0]<=.5]
        self.rows.append((stamp,planes*(side/100.)))
        if len(self.rows)<5 or stamp-self.rows[0][0]<.1:return None
        down,residual,observability=fit_down(np.vstack([r[1] for r in self.rows]))
        angle=float(np.arccos(np.clip(down[2],-1.,1.)))
        if residual>.01 or observability<.1 or angle>np.radians(10.) or angle<1e-8:return None
        axis=np.cross(down,[0.,0.,1.]);axis/=np.linalg.norm(axis)
        correction=Rotation.from_rotvec(axis*min(.15*angle,np.radians(.15)))
        self.rows=[(t,n @ correction.as_matrix().T) for t,n in self.rows]
        diagnostics=dict(start=self.rows[0][0],samples=len(self.rows),window_s=stamp-self.rows[0][0],
            minimum_side_px=side,innovation_deg=float(np.degrees(angle)),
            correction_deg=float(np.degrees(correction.magnitude())),plane_residual=residual,observability=observability)
        return correction,diagnostics

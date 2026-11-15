"""Local down direction from the image edges of upright gates."""
import numpy as np
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

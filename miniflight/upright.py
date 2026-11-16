"""Upright image edges constrain gravity without knowing gate position or yaw."""
import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def edge_gravity(pixels,rotation,camera,intrinsics,omega,time_offset):
    inverse=np.linalg.inv(intrinsics);rays=np.c_[pixels,np.ones(4)] @ inverse.T
    down=np.asarray(rotation).T @ [0.,0.,1.];step=Rotation.from_rotvec(-np.asarray(omega)*time_offset).as_matrix()
    image_down=camera @ step @ down;values=[];noise=[];attitude=[];time=[]
    for first,last in [(0,3),(1,2)]:
        normal=np.cross(rays[first],rays[last]);length=np.linalg.norm(normal)
        if length<1e-10:raise ValueError('Degenerate upright edge')
        normal/=length;derivative=(np.eye(3)-np.outer(normal,normal))/length
        row=np.zeros(8)
        for axis in range(2):
            row[2*first+axis]=image_down @ derivative @ np.cross(inverse[:,axis],rays[last])
            row[2*last+axis]=image_down @ derivative @ np.cross(rays[first],inverse[:,axis])
        values.append(normal @ image_down);noise.append(row)
        attitude.append(normal @ camera @ step @ skew(down))
        time.append(normal @ camera @ skew(step @ down) @ omega)
    return np.array(values),np.array(noise),np.array(attitude),np.array(time)


def split_upright(pixels,rotation,camera,intrinsics,noise,omega,time_offset=0.):
    """Separate gravity information from the remaining pixel-noise directions."""
    residual,G,H,time=edge_gravity(pixels,rotation,camera,intrinsics,omega,time_offset)
    L=np.linalg.cholesky(noise);U,s,V=np.linalg.svd(G @ L,full_matrices=True)
    rank=int(np.sum(s>max(s)*1e-10))
    if rank<1:raise ValueError('Unobservable upright edge noise')
    W=U[:,:rank].T/s[:rank,None]
    return dict(residual=-W @ residual,attitude=W @ H,time=W @ time,
        remaining=V[rank:] @ np.linalg.inv(L),noise_projection=-W @ G)

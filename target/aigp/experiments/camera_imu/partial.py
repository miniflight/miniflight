"""Two visible frame corners constrain an upright gate's relative position."""
import numpy as np
import cv2
from .vision import K_INV
from target.aigp.experiments.gate_pose import fit_outline


def same_red_component(image,aperture,top):
    """Associate two feature types in pixels before comparing their noisy depth."""
    blue,green,red=cv2.split(image.astype(np.float32)/255)
    mask=np.uint8(red-(blue+green)/2>.08)*255
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    _,labels=cv2.connectedComponents(mask)
    def identify(points):
        points=np.round(points).astype(int);valid=(points[:,0]>=0)&(points[:,0]<image.shape[1])&(points[:,1]>=0)&(points[:,1]<image.shape[0])
        ids=labels[points[valid,1],points[valid,0]];ids=ids[ids>0]
        return None if not len(ids) else int(np.bincount(ids).argmax())
    fractions=np.array([.25,.5,.75])[:,None]
    edge=top[1]-top[0];normal=np.array([-edge[1],edge[0]])/np.linalg.norm(edge)
    outer=identify(top[0]+fractions*edge+2*normal)
    samples=[]
    for start,end in zip(aperture,np.roll(aperture,-1,axis=0)):
        edge=end-start;normal=np.array([-edge[1],edge[0]])/max(np.linalg.norm(edge),1e-9)
        samples.extend(start+fractions*edge-2*normal)
    return outer is not None and outer==identify(np.array(samples))


def outer_top(image,opening=None):
    blue,green,red=cv2.split(image.astype(np.float32)/255)
    mask=np.uint8(red-(blue+green)/2>.08)*255
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    height,width=image.shape[:2];candidates=[]
    for contour in contours:
        x,y,w,h=cv2.boundingRect(contour)
        if x<1 or x+w>=width or y<1 or w<30 or h<8:continue
        if opening is not None:
            if cv2.pointPolygonTest(cv2.convexHull(contour),tuple(map(float,np.mean(opening,axis=0))),False)<0:continue
            if y+h>=height:continue
        elif y+h<height:continue
        # Discard the cropped contour's bottom edge; retain measured top/side intersections.
        fit=fit_outline(contour,(height+2,width,3))
        if fit is None:continue
        residual=float(max(np.array(fit[1])[[0,1,3]]))
        if residual>.8:continue
        candidates.append((cv2.contourArea(cv2.convexHull(contour)),fit[0][:2],residual))
    if not candidates:return None
    _,top,residual=max(candidates,key=lambda c:c[0])
    return top,residual


def measure_rim(top,pose):
    xyz,translation=pose[1],pose[2]
    right=(xyz[1]-xyz[0])/1.5;up=(xyz[0]-xyz[3])/1.5
    normal=np.cross(right,up);rays=np.c_[top,np.ones(2)] @ K_INV.T
    denominator=rays @ normal
    if np.min(abs(denominator))<.05:return None
    points=rays*((normal @ translation)/denominator)[:,None]
    return np.array([np.linalg.norm(points[1]-points[0]),np.mean((points-translation) @ up)])


class RimTemplate:
    def __init__(self):
        self.samples=[];self.frames=set();self.frozen=None

    def observe(self,image,corners,pose,frame_id):
        side=np.min(np.linalg.norm(corners-np.roll(corners,1,axis=0),axis=1))
        if self.frozen is not None or frame_id in self.frames or side<40 or pose[0]>.5:return
        top=outer_top(image,corners)
        if top is None:return
        measured=measure_rim(top[0],pose)
        if measured is None or not (1.5<measured[0]<6. and .75<measured[1]<4.5):return
        self.frames.add(frame_id);self.samples.append(measured)
        self.samples=self.samples[-40:]

    def model(self):
        if self.frozen is None and len(self.samples)>=5:
            median=np.median(self.samples,axis=0);bounds=np.quantile(self.samples,[.1,.9],axis=0)
            spread=float(np.max((bounds[1]-bounds[0])/median))
            if spread<=.1:self.frozen=dict(width_m=float(median[0]),center_to_top_m=float(median[1]),samples=len(self.samples),fractional_spread=spread)
        return self.frozen


def top_points(top,camera,width):
    rays=np.c_[top,np.ones(2)] @ K_INV.T @ camera.T
    vertical=rays[:,2]
    if np.min(abs(vertical)/np.linalg.norm(rays,axis=1))<.04 or np.prod(vertical)<=0:return None
    edge=vertical[0]*rays[1]-vertical[1]*rays[0]
    length=np.linalg.norm(edge)
    if length<1e-9:return None
    depths=width/length*abs(vertical[::-1])
    return rays*depths[:,None] if np.all(depths>0) else None


def relative_from_top(top,camera,width,center_to_top):
    points=top_points(top,camera,width)
    if points is None:return None
    center=np.mean(points,axis=0)+[0.,0.,center_to_top]
    return center if np.linalg.norm(center)<50. else None


def predicted_aperture(top,camera,width,center_to_top):
    """Predict missing aperture vertices; none are admitted as measured corners."""
    points=top_points(top,camera,width)
    if points is None:return None
    center=points.mean(axis=0)+[0.,0.,center_to_top];right=(points[1]-points[0])/width
    xyz=(center+.75*np.array([-right-[0,0,1],right-[0,0,1],right+[0,0,1],-right+[0,0,1]])) @ camera
    if np.min(xyz[:,2])<=.05:return None
    from .vision import K
    pixels=xyz @ K.T
    return pixels[:,:2]/pixels[:,2,None],(None,xyz,center @ camera)

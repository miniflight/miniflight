"""Deterministic camera outages; the IMU and controller clocks keep running."""
import math


class CameraGaps:
    def __init__(self,duration,record=None,period=3.):
        if not math.isfinite(duration) or not 0<=duration<period:raise ValueError('Invalid camera gap duration')
        self.duration,self.period,self.record=duration,period,record;self.epoch=None

    def interval(self,stamp):
        if not self.duration or self.epoch is None:return None
        index=math.floor((stamp-self.epoch)/self.period);start=self.epoch+index*self.period
        return (start,start+self.duration) if index>=0 and start<=stamp<start+self.duration else None

    def select(self,frames,now,locked):
        if not self.duration:return frames
        if self.epoch is None and locked:self.epoch=now+1.
        if self.epoch is None:return frames
        retained=[]
        for frame in frames:
            stamp=frame.time_ns*1e-9;interval=self.interval(stamp)
            if interval is not None:
                if self.record is not None:self.record(event='camera_gap',image_id=frame.id,capture_time=stamp,start=interval[0],end=interval[1])
            else:retained.append(frame)
        return tuple(retained)

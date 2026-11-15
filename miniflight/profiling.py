"""Wall and thread CPU timing; results never enter controller observations."""
from functools import wraps
from time import perf_counter_ns,thread_time_ns


def timed(stage):
    def decorate(function):
        @wraps(function)
        def measured(self,*args,**kwargs):
            timing=getattr(self,'timing',None)
            if timing is None:return function(self,*args,**kwargs)
            wall,cpu=perf_counter_ns(),thread_time_ns()
            try:return function(self,*args,**kwargs)
            finally:
                previous=timing.get(stage,(0,0,0))
                timing[stage]=(previous[0]+perf_counter_ns()-wall,previous[1]+thread_time_ns()-cpu,previous[2]+1)
        return measured
    return decorate


def timing_data(timing):
    return {stage:dict(wall_ns=value[0],thread_cpu_ns=value[1],calls=value[2]) for stage,value in timing.items()}

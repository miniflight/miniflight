"""Isolated numerical worker. Input is only a public map and pixel/IMU window."""
import json,os,sys,time


def main():
    try:os.nice(10)
    except PermissionError:pass
    from miniflight.relocalization import locate
    data=json.load(sys.stdin);wall=time.perf_counter_ns();cpu=time.thread_time_ns()
    result=locate(data['prior'],data['views'],data['seed'],data['camera'],data['intrinsics'])
    json.dump(dict(result=result,profile=dict(wall_ns=time.perf_counter_ns()-wall,thread_cpu_ns=time.thread_time_ns()-cpu,calls=1)),sys.stdout,allow_nan=False)


if __name__=='__main__':main()

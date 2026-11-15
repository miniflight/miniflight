"""Client receipt-to-send timing on one host clock; exposure time stays unknown."""
import csv,gzip,json
from pathlib import Path
import numpy as np
from .pipeline import statistics


def measure(data,observations,start=None,end=None):
    commands=np.sort(data['motor_sent_host_time'])
    commands=commands[(commands>=(np.inf if start is None else start))&(commands<=(np.inf if end is None else end))]
    receipts=dict(zip(data['frame_id'],data['frame_received_at']));ready={}
    for row in observations:
        stamp=row['host_time']
        if start is None or stamp<start or end is not None and stamp>end:continue
        ready[row['image_id']]=max(stamp,ready.get(row['image_id'],-np.inf))
    rows=[];missing=0;invalid=0;unsent=0
    for identity,stamp in sorted(ready.items(),key=lambda item:item[1]):
        receipt=receipts.get(identity)
        if receipt is None:missing+=1;continue
        if stamp<receipt:invalid+=1;continue
        index=np.searchsorted(commands,stamp)
        if index==len(commands):unsent+=1;continue
        sent=commands[index]
        rows.append(dict(image_id=int(identity),received_host_s=float(receipt),candidate_ready_host_s=stamp,
            send_return_host_s=float(sent),receipt_to_candidate_ms=float((stamp-receipt)*1000),
            candidate_to_send_return_ms=float((sent-stamp)*1000),receipt_to_send_return_ms=float((sent-receipt)*1000)))
    imu=np.sort(data['imu_received_at']);indices=np.searchsorted(imu,commands,side='right')-1
    age=(commands[indices>=0]-imu[indices[indices>=0]])*1000
    report={key:statistics([r[key] for r in rows]) for key in
            ['receipt_to_candidate_ms','candidate_to_send_return_ms','receipt_to_send_return_ms']}
    report.update(latest_imu_receipt_to_send_return_ms=statistics(age),candidate_images=len(ready),
        missing_frame_receipts=missing,invalid_host_order=invalid,candidates_without_subsequent_command=unsent,
        exposure_to_actuation_ms=None,clock='host monotonic seconds',
        note='Receipt is the final image packet receipt, before decode. Candidate-ready is the last raw constraint logged for that image. '
             'Send-return is after the client send call; actuator application and exposure time are unknown. '
             'Candidates include rejected observations; these timings do not assert that each candidate affected the command.')
    return report,rows


def update(root):
    """Also works after the temporary flight trace has been removed."""
    report=json.loads((root/'pipeline.json').read_text())
    window=report.get('timing',{}).get('host_window')
    if window is None:
        score=json.loads((root/'score.json').read_text())
        with gzip.open(root/'pipeline-timing.csv.gz','rt') as stream:
            controller=[float(r['host_time']) for r in csv.DictReader(stream) if r['scope']=='controller']
        window=dict(start=score.get('armed_host_time'),end=max(controller,default=None))
    with np.load(root/'public-inputs.npz',allow_pickle=False) as data,gzip.open(root/'public-observations.jsonl.gz','rt') as stream:
        latency,rows=measure(data,list(map(json.loads,stream)),**window)
    report['timing'].update(host_window=window,client_latency=latency)
    (root/'pipeline.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    with gzip.open(root/'client-latency.csv.gz','wt') as stream:
        writer=csv.DictWriter(stream,fieldnames=['image_id','received_host_s','candidate_ready_host_s','send_return_host_s',
            'receipt_to_candidate_ms','candidate_to_send_return_ms','receipt_to_send_return_ms'])
        writer.writeheader();writer.writerows(rows)
    return latency


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    print(json.dumps(update(parser.parse_args().root),indent=2))

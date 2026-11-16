import gzip,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from miniflight.position import PositionConfig
from miniflight.state import ImuIncrement
from target.aigp.experiments.camera_imu import gate_filter_live,sensors,vision
from target.aigp.experiments.camera_imu.gate_filter_events import export,replay,state


class EstimatorEventTest(unittest.TestCase):
    def trace(self,root):
        rows=[]
        def record(**row):rows.append(dict(host_time=0.,**row))
        c=gate_filter_live.Inertial(PositionConfig(.27,60.,thrust_curve=(9.81/.27,0.),body_resistance_per_m=(.047,)*3),record)
        with patch.dict(sensors.SETTINGS,gyro_attitude=True,ground_support=False):
            c.update([ImuIncrement(float(t),.01,(0.,0.,-9.81),(0.,0.,0.)) for t in np.arange(0.,.511,.01)])
        c.command(.52,[.27]*4);c.update([ImuIncrement(.55,.04,(0.,0.,-9.81),(0.,0.,0.))])
        points=.75*np.array([[0,-1,-1],[0,1,-1],[0,1,1],[0,-1,1]])+[10.,0.,0.]
        h=(points @ vision.CAMERA_FROM_BODY.T) @ vision.K.T;pixels=h[:,:2]/h[:,2,None]
        c.pixels(1,.53,10,pixels)
        record(event='estimator_state',sequence=c.input_count,state=state(c.history))
        raw=root/'flight.jsonl';raw.write_text(''.join(json.dumps(r)+'\n' for r in rows));export(raw,root)

    def test_live_delayed_pixel_trace_replays_all_means_and_covariance_exactly(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.trace(root);result=replay(root,root/'replayed')
            self.assertTrue(result['exact_mean_and_covariance']);self.assertEqual(result['matched_live_states'],1)
            self.assertGreater(result['inputs'],4)

    def test_recorded_outputs_cannot_seed_the_estimator(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.trace(root)
            with gzip.open(root/'estimator-states.jsonl.gz','rt') as h:rows=[json.loads(l) for l in h]
            rows[0]['state']['position']=[1234.,0.,0.]
            with gzip.open(root/'estimator-states.jsonl.gz','wt') as h:
                for r in rows:h.write(json.dumps(r)+'\n')
            with self.assertRaisesRegex(AssertionError,'differs'):replay(root,root/'replayed')

    def test_native_fields_are_rejected_at_the_input_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);self.trace(root)
            with gzip.open(root/'estimator-inputs.jsonl.gz','rt') as h:rows=[json.loads(l) for l in h]
            rows[0]['native_position']=[1234.,0.,0.]
            with gzip.open(root/'estimator-inputs.jsonl.gz','wt') as h:
                for r in rows:h.write(json.dumps(r)+'\n')
            with self.assertRaisesRegex(ValueError,'Unexpected'):replay(root,root/'replayed')

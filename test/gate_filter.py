import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from miniflight.gate_filter import GateFilter,GateHistory
from target.aigp.experiments.camera_imu.vision import CAMERA_FROM_BODY,K

POINTS=.75*np.array([[0,-1,-1],[0,1,-1],[0,1,1],[0,-1,1]])


def filter():
    f=GateFilter(Rotation.from_euler('xyz',[.05,-.1,.1]).as_matrix(),[.001,-.002,.003],
        [7.7911544524,106.6595510654],[.0476857765]*3,CAMERA_FROM_BODY,K)
    f.p=np.array([.2,-.3,.1]);f.v=np.array([3.,.4,-.2])
    f.add_gate(1,np.array([.2,-1.,15.]),np.array([1.,0.,0.]))
    return f


class JointGateTest(unittest.TestCase):
    def test_accelerometer_factor_updates_force_bias_without_interval_assumption(self):
        f=filter();u=np.full(4,.27);expected,H=f.acceleration_measurement(u);eps=1e-6;numeric=[]
        for column in range(len(f.P)):
            a=f.copy();b=f.copy();step=np.eye(len(f.P))[column]*eps;a.inject(step);b.inject(-step)
            numeric.append((a.acceleration_measurement(u)[0]-b.acceleration_measurement(u)[0])/(2*eps))
        np.testing.assert_allclose(H,np.array(numeric).T,atol=2e-7)
        before=np.linalg.norm(expected-(expected+[0.,0.,-.3]))
        f.observe_acceleration(expected+[0.,0.,-.3],u,.5)
        self.assertLess(np.linalg.norm(f.acceleration_measurement(u)[0]-(expected+[0.,0.,-.3])),before)

    def test_live_origin_shift_preserves_joint_relative_geometry_and_covariance(self):
        from miniflight.position import PositionConfig
        from target.aigp.experiments.camera_imu.gate_filter_live import Inertial
        f=filter();observer=Inertial(PositionConfig(.27,60.,thrust_curve=(8.,106.),body_resistance_per_m=(.05,)*3))
        observer.history=GateHistory(f,1.,[0.,0.,0.]);observer.times=[1.];observer.positions=[f.p.copy()]
        observer.velocities=[f.v.copy()];observer.rotations=[f.R @ CAMERA_FROM_BODY.T]
        before=f.relative(1)[0];cov=f.P.copy();pixels=f.measurement(1,POINTS)[0]
        observer.reset_origin(np.array([2.,-.5,1.]))
        np.testing.assert_allclose(observer.history.filter.relative(1)[0],before)
        np.testing.assert_array_equal(observer.history.filter.P,cov)
        np.testing.assert_allclose(observer.history.filter.measurement(1,POINTS)[0],pixels)

    def test_nearer_binding_needs_three_distinct_images_and_keeps_old_landmark(self):
        f=filter();f.gates[1]=(f.gates[1][0],f.p+np.array([40.,0.,0.]),0.)
        camera=f.C @ f.R.T;seed=(camera @ [12.,0.,0.],camera @ [0.,1.,0.]);before=f.p.copy()
        for _ in range(3):self.assertFalse(f.consider_nearer(1,7,1.,seed))
        self.assertFalse(f.consider_nearer(1,8,1.03,seed));self.assertTrue(f.consider_nearer(1,9,1.06,seed))
        np.testing.assert_allclose(f.gates[1][1]-f.p,[12.,0.,0.]);np.testing.assert_array_equal(f.p,before)
        self.assertEqual(len(f.gates),2);self.assertEqual(f.retargets,1)

    def test_track_export_is_independent_of_previous_motion_estimates(self):
        import gzip,json,tempfile
        from pathlib import Path
        from target.aigp.experiments.camera_imu.gate_filter_inputs import export
        row=dict(gate_number=1,motor_commands=[.27]*4,position=[100]*3,velocity=[100]*3,camera_orientation=np.eye(3).tolist(),
            camera_observation=dict(image_id=7,capture_time=1.,corners=[[1,2],[3,2],[3,4],[1,4]],reprojection_rmse_px=.2))
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);np.savez(root/'public-inputs.npz',motor_sent_host_time=[1001.],motor_outputs=[[.27]*4])
            def write():
                with gzip.open(root/'control.jsonl.gz','wt') as stream:stream.write(json.dumps(row)+'\n')
            write();export(root,root/'first.gz');row.update(position=[-999]*3,velocity=[-999]*3,camera_orientation=(-np.eye(3)).tolist())
            write();export(root,root/'second.gz')
            with gzip.open(root/'first.gz','rt') as a,gzip.open(root/'second.gz','rt') as b:self.assertEqual(a.read(),b.read())

    def test_partial_pixels_jointly_estimate_uncertain_rim_dimensions(self):
        f=filter();f.add_rim(2.84,1.4,.1);expected,H=f.measurement(1,None,rim=True);eps=1e-6
        numeric=[]
        for column in range(len(f.P)):
            a=f.copy();b=f.copy();step=np.eye(len(f.P))[column]*eps;a.inject(step);b.inject(-step)
            numeric.append((a.measurement(1,None,True)[0]-b.measurement(1,None,True)[0]).ravel()/(2*eps))
        np.testing.assert_allclose(H,np.array(numeric).T,atol=2e-7)
        before=f.P.copy();result=f.observe(1,None,expected+[.1,-.2],rim=True)
        self.assertTrue(result['accepted']);self.assertLess(np.trace(f.P),np.trace(before))
        index=f.gates[1][0];rim=f.rim[0]
        self.assertGreater(np.linalg.norm(f.P[index:index+4,rim:rim+2]),1e-8)

    def test_raw_pixel_jacobian_includes_attitude_and_gate_depth(self):
        f=filter();pixels,H=f.measurement(1,POINTS);numeric=[];eps=1e-6
        for column in range(len(f.P)):
            a=f.copy();b=f.copy();step=np.eye(len(f.P))[column]*eps;a.inject(step);b.inject(-step)
            numeric.append((a.measurement(1,POINTS)[0]-b.measurement(1,POINTS)[0]).ravel()/(2*eps))
        np.testing.assert_allclose(H,np.array(numeric).T,atol=2e-7)

    def test_process_jacobian_repropagates_corrected_attitude_into_motion(self):
        before=filter();nominal=before.copy();gyro=np.array([.2,-.3,.1]);u=np.array([.26,.28,.27,.25])
        F=nominal.predict(.013,gyro,u);numeric=[];eps=1e-6
        def difference(a,b):
            tail=np.concatenate([np.r_[a.gates[k][1]-b.gates[k][1],a.gates[k][2]-b.gates[k][2]] for k in a.gates])
            return np.r_[a.p-b.p,a.v-b.v,Rotation.from_matrix(b.R.T @ a.R).as_rotvec(),a.bg-b.bg,a.ba-b.ba,a.size-b.size,tail]
        for column in range(len(F)):
            a=before.copy();b=before.copy();step=np.eye(len(F))[column]*eps;a.inject(step);b.inject(-step)
            a.predict(.013,gyro,u);b.predict(.013,gyro,u)
            numeric.append((difference(a,nominal)-difference(b,nominal))/(2*eps))
        np.testing.assert_allclose(F,np.array(numeric).T,atol=2e-7)

    def test_late_pixels_equal_chronological_joint_update_after_braking(self):
        f=filter();a=GateHistory(f.copy(),0.,[0.,0.,0.]);b=GateHistory(f.copy(),0.,[0.,0.,0.])
        u=np.full(4,.2689415);a.command(.001,u);b.command(.001,u)
        rows=[]
        for stamp in [.2,.4,.6]:
            truth=f.copy();truth.p+=truth.v*stamp;truth.R=truth.R @ Rotation.from_euler('y',-.1*stamp).as_matrix()
            rows.append(dict(id=len(rows),time=stamp,key=1,points=POINTS,pixels=truth.measurement(1,POINTS)[0]))
        for i in range(1,101):
            stamp=i*.01;gyro=[0.,-.1 if stamp<.5 else .1,0.]
            a.imu(stamp,gyro,[0.,0.,-9.81]);b.imu(stamp,gyro,[0.,0.,-9.81])
            for row in rows:
                if abs(stamp-row['time'])<1e-9:a.observe(row)
                if abs(stamp-row['time']-.13)<1e-9:b.observe(row)
        np.testing.assert_allclose(a.filter.p,b.filter.p,atol=1e-10)
        np.testing.assert_allclose(a.filter.v,b.filter.v,atol=1e-10)
        np.testing.assert_allclose(a.filter.R,b.filter.R,atol=1e-10)
        np.testing.assert_allclose(a.filter.P,b.filter.P,atol=1e-10)
        self.assertEqual(a.rejected_late,0);self.assertEqual(b.rejected_late,0)
        self.assertGreater(np.linalg.eigvalsh(a.filter.P).min(),-1e-10)
        self.assertGreater(np.linalg.norm(a.filter.P[:6,6:9]),1e-5)


if __name__=='__main__':unittest.main()

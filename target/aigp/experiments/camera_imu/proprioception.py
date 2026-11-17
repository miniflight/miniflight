"""Frozen command/body response tests. Fitting reads public inputs only."""
import argparse,gzip,hashlib,json
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation,Slerp
from target.aigp import MOTOR_MIX
from .thrust_response import PUBLIC,command_times


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    x=np.asarray(values);x=x[np.isfinite(x)]
    return dict(count=int(x.size),mae=float(np.mean(abs(x))),rms=float(np.sqrt(np.mean(x*x))),p95=float(np.quantile(abs(x),.95)),maximum=float(np.max(abs(x)))) if x.size else dict(count=0)


def load(source):
    with np.load(source/'public-inputs.npz',allow_pickle=False) as f:
        if set(f.files)!=PUBLIC:raise ValueError('Unexpected public sensor fields')
        d={k:f[k] for k in f.files}
    keep=np.r_[True,np.diff(d['imu_time'])>0]
    d['time']=d['imu_time'][keep];d['force']=d['acceleration'][keep];d['omega']=d['gyro'][keep]
    d['commands']=command_times(d['motor_sent_host_time'],d['imu_received_at'],d['imu_time'])
    return d


def efforts(outputs,curve):
    a,b=curve;force=a*outputs+b*outputs**2
    return -force.mean(axis=-1),force @ np.array(MOTOR_MIX)[:,1:]/4.


def identification(d,curve,coefficients=None):
    # Integrated20ms increments avoid differentiating the cached public gyro.
    grid=np.arange(d['commands'][0]+4.,min(d['time'][-1],d['commands'][-1])-.1,.005)
    omega=np.column_stack([np.interp(grid,d['time'],d['omega'][:,j]) for j in range(3)])
    force=np.interp(grid,d['time'],d['force'][:,2]);k=4
    integral=cumulative_trapezoid(omega,grid,axis=0,initial=0.)
    dw=omega[k:]-omega[:-k];iw=integral[k:]-integral[:-k];span=grid[k:]-grid[:-k]
    profiles=[];solutions=[]
    for lag in np.arange(0.,.0801,.005):
        index=np.searchsorted(d['commands'],grid-lag,side='right')-1
        axial,torque=efforts(d['motor_outputs'][index],curve)
        it=cumulative_trapezoid(torque,grid,axis=0,initial=0.);it=it[k:]-it[:-k]
        fit=[];errors=[]
        for j in range(3):
            X=np.c_[it[:,j],-iw[:,j],span]
            value=lsq_linear(X,dw[:,j],bounds=([0.,0.,-5.],[1e5,100.,5.]),tol=1e-10).x if coefficients is None else np.array(coefficients[j])
            fit.append(value.tolist());errors.append((X @ value-dw[:,j])/span)
        e=np.column_stack(errors);scale=np.maximum(.02,np.std(dw/span[:,None],axis=0))
        profiles.append(dict(lag_s=float(lag),axial_force=stats(axial-force),rotation_acceleration_mae_rad_s2=np.mean(abs(e),axis=0).tolist(),rotation_loss=float(np.mean((e/scale)**2))))
        solutions.append(fit)
    force_index=int(np.argmin([p['axial_force']['rms'] for p in profiles]));rotation_index=int(np.argmin([p['rotation_loss'] for p in profiles]))
    return profiles,solutions[rotation_index],force_index,rotation_index


def fit(source,calibration,output):
    config=json.loads(calibration.read_text())['position_config'];d=load(source)
    profiles,rotation,fi,ri=identification(d,config['thrust_curve'])
    model=dict(schema=1,training_source=str(source),training_public_sha256=digest(source/'public-inputs.npz'),calibration_sha256=digest(calibration),
        position_config=config,rotation_coefficients=rotation,force_lag_s=profiles[fi]['lag_s'],rotation_lag_s=profiles[ri]['lag_s'],training_delay_profiles=profiles,
        equation='omega_dot[j]=gain[j]*mixed_command_force[j]-damping[j]*omega[j]+offset[j]',
        force_units='Specific force m/s2, not newtons. Axial lag fit holds the prior thrust curve fixed; vertical drag is omitted in that lag diagnostic.',
        timing='Effective lag on the arrival-aligned public clock; physical actuation and IMU publication delays are not separately identified.',
        role='Diagnostic only. This fitted rotational model does not replace live gyro propagation or change controller commands.')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(model,indent=2,allow_nan=False)+'\n');return model


def predict(d,states,model,horizons=(.02,.05,.1,.2,.4)):
    config=model['position_config'];curve=config['thrust_curve'];drag=np.array(config['body_resistance_per_m']);coeff=np.array(model['rotation_coefficients'])
    records=[]
    for state in states[::5]:
        origin=state['time'];index=np.searchsorted(d['time'],origin,side='right')-1
        if index<0:continue
        R0=np.array(state['rotation']);v0=np.array(state['velocity']);w0=d['omega'][index]-state['gyro_bias'];last=np.searchsorted(d['commands'],origin,side='right')-1
        if last<0:continue
        for horizon in horizons:
            if origin+horizon>d['time'][-1]:continue
            R=R0.copy();delta=np.eye(3);v=v0.copy();w=w0.copy();elapsed=0.
            while elapsed<horizon-1e-9:
                dt=min(.005,horizon-elapsed);stamp=origin+elapsed
                indices=[min(last,max(0,np.searchsorted(d['commands'],stamp-model[k],side='right')-1)) for k in ['force_lag_s','rotation_lag_s']]
                axial,_=efforts(d['motor_outputs'][indices[0]],curve);_,torque=efforts(d['motor_outputs'][indices[1]],curve)
                force=np.array([0.,0.,axial])-drag*abs(R.T @ v)*(R.T @ v)+R.T @ state['force_bias']
                dw=coeff[:,0]*torque-coeff[:,1]*w+coeff[:,2];step=Rotation.from_rotvec((w+.5*dw*dt)*dt).as_matrix()
                v+=(R @ force+[0.,0.,9.81])*dt;R=R @ step;delta=delta @ step;w+=dw*dt;elapsed+=dt
            # Predict the endpoint sensor value with only commands known at origin.
            command=min(last,max(0,np.searchsorted(d['commands'],origin+horizon-model['force_lag_s'],side='right')-1))
            axial,_=efforts(d['motor_outputs'][command],curve);local=R.T @ v
            force=np.array([0.,0.,axial])-drag*abs(local)*local+R.T @ state['force_bias']
            records.append(dict(origin=origin,horizon=horizon,force=force,delta=delta,omega=w,constant_force=d['force'][index],constant_delta=Rotation.from_rotvec(w0*horizon).as_matrix()))
    return {key:np.array([r[key] for r in records]) for key in records[0]}


def evaluate(source,estimates,frozen,output):
    model=json.loads(frozen.read_text());d=load(source)
    if digest(source/'public-inputs.npz')==model['training_public_sha256']:raise ValueError('Response validation needs a separate flight')
    with gzip.open(estimates/'estimates.jsonl.gz','rt') as h:states=[json.loads(l) for l in h]
    predictions=predict(d,states,model);output.mkdir(parents=True,exist_ok=True)
    path=output/'predictions.npz';np.savez_compressed(path,**predictions);frozen_hash=digest(path)
    # All predictions are now frozen. References below are scoring only.
    t=predictions['origin'];end=t+predictions['horizon']
    measured=np.column_stack([np.interp(end,d['time'],d['force'][:,j]) for j in range(3)])
    with np.load(source/'native-reference.npz',allow_pickle=False) as f:a=f['attitude'];rates=f['body_rates']
    a=a[np.r_[True,np.diff(a[:,0])>0]];orientation=Slerp(a[:,0],Rotation.from_euler('xyz',a[:,1:]))
    valid=(t>=a[0,0])&(end<=a[-1,0]);truth=np.full((len(t),3,3),np.nan)
    truth[valid]=np.swapaxes(orientation(t[valid]).as_matrix(),1,2) @ orientation(end[valid]).as_matrix()
    rotation=np.full(len(t),np.nan);baseline=np.full(len(t),np.nan)
    rotation[valid]=np.degrees(Rotation.from_matrix(np.swapaxes(predictions['delta'][valid],1,2) @ truth[valid]).magnitude())
    baseline[valid]=np.degrees(Rotation.from_matrix(np.swapaxes(predictions['constant_delta'][valid],1,2) @ truth[valid]).magnitude())
    angular=np.column_stack([np.interp(end,rates[:,0],rates[:,i]) for i in range(1,4)])
    force_error=predictions['force']-measured;rate_error=predictions['omega']-angular
    metrics={}
    for horizon in np.unique(predictions['horizon']):
        m=predictions['horizon']==horizon
        metrics[str(horizon)]=dict(force=stats(np.linalg.norm(force_error[m],axis=1)),force_axes_mae=np.mean(abs(force_error[m]),axis=0).tolist(),
            constant_force=stats(np.linalg.norm(predictions['constant_force'][m]-measured[m],axis=1)),rotation_deg=stats(rotation[m]),constant_rate_rotation_deg=stats(baseline[m]),body_rate=stats(np.linalg.norm(rate_error[m],axis=1)))
    profiles,_,fi,ri=identification(d,model['position_config']['thrust_curve'],model['rotation_coefficients'])
    report=dict(model_sha256=digest(frozen),public_inputs_sha256=digest(source/'public-inputs.npz'),estimates_sha256=digest(estimates/'estimates.jsonl.gz'),predictions_sha256=frozen_hash,refit=False,
        forecast='At each origin, initialize from the causal public estimator and latest gyro. Hold the latest sent motor commands after the origin. No future commands, gyro, force or camera state enter predictions.',
        scoring='Force against public accelerometer; relative rotation and body rate against private telemetry after prediction freeze. Global state errors are separately scored without test-specific alignment.',
        horizons=metrics,diagnostic_delay_profiles=profiles,diagnostic_force_lag_s=profiles[fi]['lag_s'],diagnostic_rotation_lag_s=profiles[ri]['lag_s'],
        timing=model['timing'],prediction_count=len(t))
    (output/'response.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    assert digest(path)==frozen_hash
    np.savez_compressed(output/'prediction-errors.npz',origin=t,horizon=predictions['horizon'],force=force_error,rotation=rotation,constant_rotation=baseline,body_rate=rate_error)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['fit','evaluate']);p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--calibration',type=Path);p.add_argument('--model',type=Path);p.add_argument('--estimates',type=Path);a=p.parse_args()
    report=fit(a.source,a.calibration,a.output) if a.mode=='fit' else evaluate(a.source,a.estimates,a.model,a.output)
    print(json.dumps({k:v for k,v in report.items() if k not in ['training_delay_profiles','diagnostic_delay_profiles','position_config']},indent=2))

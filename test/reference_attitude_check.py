"""X1 independent vehicle attitude checks; never use CSV Euler-angle columns.

Pitch/roll come from measured specific force minus velocity-derived vehicle
acceleration; yaw comes from horizontal velocity. Climb angle is also reported
but is not body pitch: mounting offset and suspension pitch differ from road
grade, especially on routes with little grade excitation.
The 2 s time-domain average suppresses suspension/vibration in the roll
regressor; sampling rates are read from timestamps, never assumed.
"""
import numpy as np


def time_average(values, times, width=2.):
    lo=np.searchsorted(times,times-width/2)
    hi=np.searchsorted(times,times+width/2,side='right')
    sums=np.r_[0.,np.cumsum(values)]
    return (sums[hi]-sums[lo])/(hi-lo)


def regression(actual, expected):
    if len(expected)<30 or np.std(expected)<1e-5:
        raise AssertionError('insufficient attitude excitation for physical regression')
    slope,bias=np.linalg.lstsq(np.c_[expected,np.ones(len(expected))],actual,rcond=None)[0]
    return dict(samples=len(expected),slope=float(slope),bias_deg=float(np.rad2deg(bias)),
                correlation=float(np.corrcoef(actual,expected)[0,1]))


def check_attitude(times, velocities, rpy, imu_times, accelerations):
    t=np.asarray(times);v=np.asarray(velocities);rpy=np.asarray(rpy)
    assert np.all(np.diff(t)>0), 'reference timestamps must advance'
    speed=np.linalg.norm(v[:,:2],axis=1)
    course=np.unwrap(np.arctan2(v[:,1],v[:,0]))
    moving=speed>5.
    pitch_climb=regression(rpy[moving,1],-np.arctan2(v[moving,2],speed[moving]))
    yaw=regression(np.unwrap(rpy[moving,2]),np.unwrap(np.arctan2(v[moving,1],v[moving,0])))
    lateral=time_average(speed*np.gradient(course,t),t)
    acc=np.stack([np.interp(t,imu_times,time_average(accelerations[:,j],imu_times)) for j in range(3)],axis=1)
    # Specific force in FLU: fy ~= v*yaw_rate + g*sin(roll), fz ~= g*cos(roll).
    roll_expected=np.arctan2(acc[:,1]-lateral,acc[:,2])
    steady=moving & (np.abs(lateral)>.3)
    roll=regression(time_average(rpy[:,0],t)[steady],roll_expected[steady])
    # Small-attitude vehicle approximation: subtract longitudinal, lateral and
    # vertical kinematic accelerations from FLU specific force. The residual
    # points opposite gravity; +middle Euler angle is nose down. All smoothing
    # uses timestamps. No CSV Euler angle is used to construct this regressor.
    gravity=acc.copy()
    gravity[:,0]-=time_average(np.gradient(speed,t),t)
    gravity[:,1]-=lateral
    gravity[:,2]-=time_average(np.gradient(v[:,2],t),t)
    pitch_expected=-np.arctan2(gravity[:,0],np.linalg.norm(gravity[:,1:],axis=1))
    pitch=regression(time_average(rpy[:,1],t)[moving],pitch_expected[moving])
    result=dict(pitch=pitch,roll=roll,yaw=yaw)
    for axis,metrics in result.items():
        assert abs(metrics['slope']-1)<.2 and metrics['correlation']>.9, (axis,metrics)
        assert abs(metrics['bias_deg'])<8, (axis,metrics)
    result['pitch_climb_diagnostic']=pitch_climb
    return result

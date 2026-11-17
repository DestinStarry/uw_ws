"""Host receive timestamps, finite sample rates, latency and estimated noise.

Only IMU and pressure-depth observations are exposed to control. No servo,
position or velocity truth is used as an observation.
"""
from collections import deque
import math
import numpy as np


def multiply(a,b):
    w,x,y,z=a;v,i,j,k=b
    return np.array([w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v])


class SensorModel:
    def __init__(self, imu_hz=100., depth_hz=2., imu_delay=.02, depth_delay=.05,
                 zero_time=2., angle_noise_deg=.15, gyro_noise=.002,
                 gyro_bias=(.005,-.003,.004), depth_noise=.005, seed=1):
        values=(imu_hz,depth_hz,imu_delay,depth_delay,zero_time,angle_noise_deg,gyro_noise,depth_noise)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Sensor parameters must be finite')
        if not 0<imu_hz<=500 or not 0<depth_hz<=100 or min(values[2:])<0 or max(imu_delay,depth_delay)>2:
            raise ValueError('Invalid sensor parameters')
        self.bias=np.asarray(gyro_bias,dtype=float)
        if self.bias.shape!=(3,) or not np.isfinite(self.bias).all():
            raise ValueError('Invalid gyro bias')
        self.imu_period,self.depth_period=1/imu_hz,1/depth_hz
        self.imu_delay,self.depth_delay=imu_delay,depth_delay
        self.zero_time=zero_time
        self.angle_noise=math.radians(angle_noise_deg);self.gyro_noise=gyro_noise
        self.depth_noise=depth_noise
        self.rng=np.random.default_rng(seed)
        self.next_imu=self.next_depth=-float('inf')
        self.start=None;self.reference=None;self.quaternions=[];self.gyros=[]
        self.queues={'imu':deque(),'depth':deque()}
        self.last_depth=None;self.last_depth_time=None;self.depth_rate=0.

    def poll(self, now, stamp_ns, state):
        if self.start is None:
            self.start=now
        if now>=self.next_imu:
            self.next_imu=now+self.imu_period
            error=self.rng.normal(0,self.angle_noise,3)
            dq=np.r_[1.,error*.5];dq/=np.linalg.norm(dq)
            q=multiply(state['quaternion'],dq);q/=np.linalg.norm(q)
            gyro=np.asarray(state['gyro'])+self.bias+self.rng.normal(0,self.gyro_noise,3)
            if self.reference is None:
                if self.quaternions and np.dot(q,self.quaternions[0])<0:
                    q=-q
                self.quaternions.append(q.copy());self.gyros.append(gyro.copy())
                if now-self.start>=self.zero_time and len(self.quaternions)>=10:
                    self.reference=np.mean(self.quaternions,axis=0)
                    self.reference/=np.linalg.norm(self.reference)
                    self.bias_estimate=np.mean(self.gyros,axis=0)
                    self.quaternions.clear();self.gyros.clear()
            if self.reference is not None:
                conjugate=self.reference*np.array([1,-1,-1,-1])
                sample=dict(quaternion=multiply(conjugate,q),gyro=gyro-self.bias_estimate,
                            acceleration=np.asarray(state['acceleration']).copy(),stamp_ns=stamp_ns)
                self.queues['imu'].append((now+self.imu_delay,sample))
        if now>=self.next_depth:
            self.next_depth=now+self.depth_period
            depth=float(state['depth'])+self.rng.normal(0,self.depth_noise)
            if self.last_depth is not None:
                dt=now-self.last_depth_time
                self.depth_rate+=.2*((depth-self.last_depth)/dt-self.depth_rate)
            self.last_depth,self.last_depth_time=depth,now
            self.queues['depth'].append((now+self.depth_delay,
                dict(depth=depth,depth_rate=self.depth_rate,stamp_ns=stamp_ns,received_at=now)))
        ready={}
        for kind,queue in self.queues.items():
            while queue and queue[0][0]<=now:
                _,ready[kind]=queue.popleft()
        return ready

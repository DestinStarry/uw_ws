"""Firmware quantization plus configurable, unmeasured actuator dynamics.

No actual joint angles are read here or fed to allocation/control.
Array order is internal [1,3,2,4]; inversion and wiring cancel after calibration.
"""
from collections import deque
import math
import numpy as np


class ActuatorModel:
    def __init__(self, delay=.03, pwm_hz=50., motor_tau=.12, motor_deadzone=0.,
                 motor_exponent=1., servo_rate=math.pi, servo_tau=.05,
                 servo_backlash=math.radians(.3)):
        values=(delay,pwm_hz,motor_tau,motor_deadzone,motor_exponent,servo_rate,servo_tau,servo_backlash)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Actuator parameters must be finite')
        if (not 0<=delay<=2 or not 1<=pwm_hz<=1000 or motor_tau<0 or servo_tau<0
                or not 0<=motor_deadzone<1 or motor_exponent<=0 or servo_rate<=0
                or not 0<=servo_backlash<math.pi/2):
            raise ValueError('Invalid actuator dynamics')
        self.delay,self.period=delay,1/pwm_hz
        self.motor_tau,self.deadzone,self.exponent=motor_tau,motor_deadzone,motor_exponent
        self.servo_rate,self.servo_tau,self.backlash=servo_rate,servo_tau,servo_backlash
        self.queue=deque()
        self.desired_forces=np.zeros(4);self.desired_angles=np.zeros(2)
        self.forces=np.zeros(4);self.angles=np.zeros(2)
        self.last_pwm=-float('inf')

    def command(self, now, forces, angles):
        # Latest command overwrites commands scheduled for the same physics instant.
        item=(now+self.delay,np.array(forces,dtype=float),np.array(angles,dtype=float))
        if self.queue and self.queue[-1][0]==item[0]:
            self.queue[-1]=item
        else:
            self.queue.append(item)
        # Finite history even during excessive command traffic.
        while len(self.queue)>1024:
            self.queue.popleft()

    def stop(self):
        self.queue.clear()
        self.desired_forces[:]=0
        self.desired_angles[:]=0
        # Rotor inertia remains; zero PWM does not teleport thrust to zero.

    def _board_targets(self, forces, angles):
        limits=np.where(forces>=0,33.,24.5)
        normalized=np.round(np.clip(forces/limits,-1,1),4)
        # Motor.cpp: all channels inverted; calibrated physical wiring cancels inversion.
        pwm=1500+np.trunc(-normalized*400).astype(int)
        effective=-(pwm-1500)/400.
        magnitude=np.maximum(0,(np.abs(effective)-self.deadzone)/(1-self.deadzone))
        self.desired_forces=np.sign(effective)*magnitude**self.exponent*limits
        servo=np.round(np.clip(angles/(math.pi/2),-1,1),4)
        servo_pwm=1500+np.trunc(servo*500).astype(int)
        target=(servo_pwm-1500)/500.*(math.pi/2)
        self.desired_angles=np.where(np.abs(target-self.desired_angles)>self.backlash,
                                     target,self.desired_angles)

    def step(self, now, dt):
        if now-self.last_pwm>=self.period-1e-9:
            self.last_pwm=now
            latest=None
            while self.queue and self.queue[0][0]<=now:
                latest=self.queue.popleft()
            if latest:
                self._board_targets(latest[1],latest[2])
        alpha=1 if self.motor_tau==0 else -math.expm1(-dt/self.motor_tau)
        self.forces+=alpha*(self.desired_forces-self.forces)
        alpha=1 if self.servo_tau==0 else -math.expm1(-dt/self.servo_tau)
        delta=alpha*(self.desired_angles-self.angles)
        self.angles+=np.clip(delta,-self.servo_rate*dt,self.servo_rate*dt)
        return self.forces.copy(),self.angles.copy()

"""Observable hardware behavior without privileged plant feedback."""
import math
import numpy as np
import pytest
from uw_robot_mujoco.actuator_model import ActuatorModel
from uw_robot_mujoco.sensor_model import SensorModel


def state():
    return dict(quaternion=np.array([1.,0,0,0]),gyro=np.zeros(3),
                acceleration=np.array([0.,0,9.80665]),depth=0.,
                depth_rate=99999.,position=object())


def test_firmware_pwm_quantization_and_order():
    model=ActuatorModel(delay=0,motor_tau=0,motor_deadzone=0,motor_exponent=1,
                        servo_tau=0,servo_rate=1000,servo_backlash=0)
    forces=np.array([33.,-24.5,16.6,-12.3])
    angles=np.array([.37,-.63])
    model.command(0,forces,angles)
    actual,servo=model.step(0,.02)
    u=np.round(forces/np.where(forces>=0,33,24.5),4)
    pwm=1500+np.trunc(-u*400).astype(int)
    assert np.allclose(actual,-(pwm-1500)/400*np.where(forces>=0,33,24.5))
    expected=np.trunc(np.round(angles/(math.pi/2),4)*500)/500*(math.pi/2)
    assert np.allclose(servo,expected)


def test_delay_rotor_inertia_servo_rate_and_stop_flush():
    model=ActuatorModel(delay=.03)
    model.command(0,[20]*4,[1,-1])
    assert np.all(model.step(.02,.002)[0]==0)
    forces,angles=model.step(.04,.002)
    assert 0<forces[0]<20
    assert np.max(np.abs(angles))<=math.pi*.002
    model.command(.04,[33]*4,[1,-1])
    model.stop()
    before=model.forces.copy()
    forces,_=model.step(.06,.002)
    assert np.all(forces<before) and np.all(forces>0)
    assert not model.queue
    for i in range(1000):
        forces,angles=model.step(.062+i*.002,.002)
    assert np.max(np.abs(forces))<1e-6
    assert np.max(np.abs(angles))<1e-6


def test_nonlinear_deadzone_and_reverse_response():
    model=ActuatorModel(delay=0,motor_tau=0,motor_deadzone=.05,motor_exponent=2.)
    model.command(0,[1.,-1.,33.,-24.5],[0,0])
    forces,_=model.step(0,.02)
    assert forces[0]==forces[1]==0
    assert forces[2]==pytest.approx(33)
    assert forces[3]==pytest.approx(-24.5)


def test_sensor_zeroing_latency_and_no_truth_velocity():
    model=SensorModel(zero_time=.2,angle_noise_deg=0,gyro_noise=0,depth_noise=0,
                      imu_hz=100,depth_hz=2,imu_delay=.02)
    sample=state()
    imus=[];depths=[]
    for i in range(200):
        now=i*.005
        ready=model.poll(now,int(now*1e9),sample)
        if 'imu' in ready:
            assert now-ready['imu']['stamp_ns']/1e9>=.02-1e-8
            imus.append(ready['imu'])
        if 'depth' in ready:
            depths.append(ready['depth'])
    assert imus and depths
    assert imus[0]['stamp_ns']>=200000000
    assert np.allclose(imus[-1]['gyro'],0,atol=1e-12)
    assert depths[-1]['depth_rate']==0, 'Do not expose simulator velocity as pressure-derived rate'
    assert len(depths)==2


def test_seeded_noise_reproducible():
    a,b=SensorModel(zero_time=0),SensorModel(zero_time=0)
    for i in range(100):
        x=a.poll(i*.01,i*10000000,state())
        y=b.poll(i*.01,i*10000000,state())
        assert x.keys()==y.keys()
        for key in x:
            for field in x[key]:
                assert np.allclose(x[key][field],y[key][field])


@pytest.mark.parametrize('kwargs',[dict(delay=-1),dict(motor_tau=float('nan')),
    dict(servo_rate=0),dict(motor_deadzone=1),dict(motor_exponent=0)])
def test_invalid_actuator_parameters(kwargs):
    with pytest.raises(ValueError):
        ActuatorModel(**kwargs)


def test_default_mapping_does_not_suppress_allocated_small_forces():
    model=ActuatorModel(delay=0,motor_tau=0)
    model.command(0,[1.,-1.,2.,-2.],[0,0])
    forces,_=model.step(0,.02)
    assert np.allclose(forces,[1,-1,2,-2],atol=.09)
    assert np.all(np.abs(forces)>.9)

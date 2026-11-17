"""Passive underwater physics: no controller and zero actuator commands."""
from pathlib import Path
import math
import mujoco
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from uw_robot_mujoco.sim_engine import Simulation


def plant(**kwargs):
    return Simulation(Path(get_package_share_directory('uw_robot_mujoco'))/'assets/scene.xml',
                      [15,40,40],[.2,.7,.7],**kwargs)


def tilt(sim, axis, degrees):
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.eye(3)[axis], math.radians(degrees))
    sim.data.qpos[3:7] = q
    mujoco.mj_forward(sim.model, sim.data)


def inclination(sim):
    return math.degrees(math.acos(np.clip(sim.data.xmat[sim.body].reshape(3,3)[2,2],-1,1)))


def test_neutral_buoyancy_balances_full_gravity():
    sim=plant()
    mass=sim.model.body_subtreemass[sim.body]
    assert sim.model.opt.gravity[2] == pytest.approx(-9.80665)
    assert sim.buoyancy_force[2] == pytest.approx(mass*9.80665)
    start=sim.state()['position'].copy()
    sim.stop()
    sim.step(2000)
    assert abs(sim.state()['position'][2]-start[2])<.005
    assert inclination(sim)<.2
    assert np.all(sim.data.ctrl==0)


@pytest.mark.parametrize('axis,angle',[(0,20),(0,-20),(1,25),(1,-25)])
def test_zero_thrust_passively_returns_to_level(axis,angle):
    sim=plant()
    tilt(sim,axis,angle)
    assert sim.water_wrench()[3+axis]*angle < 0
    # The hydrostatic restoring torque never imposes a yaw heading.
    assert abs(sim.water_wrench()[5])<1e-10
    sim.stop()
    sim.step(8000)
    assert inclination(sim)<1.0
    assert np.all(sim.data.ctrl==0), 'Restoring force must not come from a hidden controller'
    assert np.isfinite(sim.data.qpos).all()


def test_buoyancy_mismatch_and_current_have_physical_effects():
    neutral=plant()
    volume=neutral.displaced_volume
    for factor,sign in [(1.03,1),(.97,-1)]:
        sim=plant(displaced_volume=volume*factor)
        start=sim.state()['position'][2]
        sim.step(1000)
        assert (sim.state()['position'][2]-start)*sign>0.001
    sim=plant(water_current=(.2,0,0))
    assert sim.water_wrench()[0]>0
    sim.step(1000)
    assert sim.state()['position'][0]>0.001


@pytest.mark.parametrize('kwargs',[
    dict(gravity=0),dict(water_density=-1),dict(water_viscosity=float('nan')),
    dict(displaced_volume=-1),dict(buoyancy_offset=(0,0)),dict(water_current=(float('nan'),0,0)),
])
def test_invalid_environment_parameters(kwargs):
    with pytest.raises(ValueError):
        plant(**kwargs)

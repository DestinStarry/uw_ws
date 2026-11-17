from pathlib import Path
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from uw_robot_mujoco.sim_engine import Simulation


def plant():
    return Simulation(Path(get_package_share_directory('uw_robot_mujoco'))/'assets/scene.xml',
                      [4,10,10],[.2,.7,.7])


def test_mapping_and_stop():
    sim = plant()
    assert sim.command([3,6,9,12],[.1,-.2])
    assert np.allclose(sim.data.ctrl[sim.thrusters], [.1,.2,.3,.4])
    assert np.allclose(sim.data.ctrl[sim.servos],[.1,-.2])
    assert not sim.command([float('nan'),0,0,0],[0,0])
    assert np.all(sim.data.ctrl==0)
    sim.command([1,1,1,1],[0,0],False)
    assert np.all(sim.data.ctrl==0)


def test_forward_motion_and_sensors():
    sim = plant()
    before=sim.state()['position'][0]
    sim.command([5,5,5,5],[0,0])
    sim.step(200)
    state=sim.state()
    assert state['position'][0]>before
    assert np.isfinite(state['gyro']).all()
    assert abs(np.linalg.norm(state['quaternion'])-1)<1e-6
    assert np.isfinite(state['depth_rate'])

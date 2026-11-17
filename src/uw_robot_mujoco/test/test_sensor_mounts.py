"""Physical sensor mounting, stereo geometry and actual offscreen rendering."""
import math
from pathlib import Path
import time

import cv2
import mujoco
import numpy as np
import pytest

from uw_robot_mujoco.sim_engine import Simulation
from uw_robot_mujoco.stereo_camera import StereoCamera, configure_stereo, pinhole_parameters

ASSETS = Path(__file__).resolve().parents[1]/'assets'


def plant():
    return Simulation(ASSETS/'scene.xml', [15., 40., 40.], [.2, .7, .7])


def test_imu_mount_and_off_center_acceleration():
    sim = plant()
    sid = sim.model.site('imu_site').id
    np.testing.assert_allclose(sim.model.site_pos[sid], [.06, 0., .04])
    np.testing.assert_allclose(sim.model.site_quat[sid], [1., 0., 0., 0.])
    np.testing.assert_allclose(sim.data.site_xpos[sid]-sim.data.xpos[sim.body], [.06, 0., .04])
    # 移动 IMU 后姿态/角速度仍测艇体；匀速偏航时应包含偏心向心加速度。
    sim.model.opt.gravity[:] = 0
    sim.data.qvel[5] = 2.
    mujoco.mj_forward(sim.model, sim.data)
    sim.data.qacc[:] = 0
    mujoco.mj_inverse(sim.model, sim.data)
    np.testing.assert_allclose(sim.data.sensor('body_gyro').data, [0., 0., 2.], atol=1e-8)
    # 获取特定零加速度状态下的加速度传感器输出。
    mujoco.mj_sensorAcc(sim.model, sim.data)
    assert sim.data.sensor('body_linacc').data[0] == pytest.approx(-.24, abs=.02)


def test_stereo_forward_direction_baseline_and_projection():
    sim = plant()
    left, right = configure_stereo(sim.model)
    mujoco.mj_forward(sim.model, sim.data)
    np.testing.assert_allclose(sim.data.cam_xpos[left]-sim.data.cam_xpos[right], [0., .06, 0.])
    for cid in (left, right):
        rotation = sim.data.cam_xmat[cid].reshape(3, 3)
        np.testing.assert_allclose(-rotation[:, 2], [1., 0., 0.], atol=1e-12)
        np.testing.assert_allclose(rotation[:, 0], [0., -1., 0.], atol=1e-12)
    k, pl, pr = pinhole_parameters(640, 360, 60., .06)
    assert k[0] == pytest.approx(180/math.tan(math.pi/6))
    assert pr[3] == pytest.approx(-k[0]*.06)
    target = sim.data.xpos[sim.body] + [.285+2., 0., 0.]
    pixels = []
    for cid in (left, right):
        local = sim.data.cam_xmat[cid].reshape(3, 3).T @ (target-sim.data.cam_xpos[cid])
        pixels.append(k[0]*local[0]/(-local[2])+k[2])
    assert pixels[0]-pixels[1] == pytest.approx(k[0]*.06/2.)


def test_render_worker_does_not_change_physics_and_produces_stereo():
    sim = plant()
    configure_stereo(sim.model)
    # 已知前方目标，防止两目只有天空而无法检验视差/左右关系。
    spec = mujoco.MjSpec.from_file(str(ASSETS/'scene.xml'))
    spec.worldbody.add_geom(name='stereo_test_target', type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[1.2, 0., 1.], size=[.08, .12, .12], rgba=[1., .05, .05, 1.])
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = StereoCamera(model, width=320, height=180)
    before = data.qpos.copy()
    camera.start()
    try:
        camera.submit(data, 123456789)
        deadline = time.monotonic()+30
        while camera.latest_frame() is None and not camera.error and time.monotonic()<deadline:
            # 第一次创建 GL 上下文可能超过新鲜度阈值，持续提交最新快照。
            camera.submit(data, 123456789)
            time.sleep(.02)
        assert not camera.error, camera.error
        frame = camera.latest_frame()
        assert frame is not None, 'No rendered frame'
        assert frame.ros_stamp_ns == 123456789
        assert frame.stereo.shape == (180, 640, 3)
        np.testing.assert_array_equal(data.qpos, before)
        centers = []
        for eye in (frame.left, frame.right):
            b, g, r = eye[..., 0].astype(float), eye[..., 1].astype(float), eye[..., 2].astype(float)
            mask = (r > 80) & (r > 1.5*g) & (r > 1.5*b)
            assert mask.sum() > 20, 'Camera is blocked or not facing forward'
            centers.append(np.where(mask)[1].mean())
        assert centers[0] > centers[1], 'Physical left/right eye order is reversed'
        out = Path('/home/zy/uw_ws/validation')
        out.mkdir(exist_ok=True)
        cv2.imwrite(str(out/'stereo_mount_check.jpg'), frame.stereo)
    finally:
        camera.close()

"""双目离屏渲染：只接收状态快照，工作线程不访问物理循环的 MjData。

MuJoCo 相机沿局部 -Z 看、+Y 向上；ROS 光学坐标为 +Z 向前、+Y 向下。
相机固定在艇体上，两目由同一份 qpos 和时间戳生成。
"""
from types import SimpleNamespace
import math
import threading
import time

import cv2
import mujoco
import numpy as np

from uw_robot_sensors.camera.capture import Frame


def configure_stereo(model, position=(.285, 0., 0.), baseline=.06, fovy=60.):
    """位置为双目中点；+Y 一侧为物理左目，保持 XML 中的前向安装方向。"""
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('camera_position_m must contain three finite values')
    if not math.isfinite(baseline) or not 0 < baseline < 1:
        raise ValueError('camera_baseline_m must be in (0, 1)')
    if not math.isfinite(fovy) or not 1 < fovy < 170:
        raise ValueError('camera_fovy_deg must be in (1, 170)')
    camera_ids = []
    for name, sign in (('stereo_left', 1), ('stereo_right', -1)):
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if camera_id < 0:
            raise ValueError('Model needs camera ' + name)
        model.cam_pos[camera_id] = position + [0., sign*baseline/2, 0.]
        model.cam_fovy[camera_id] = fovy
        camera_ids.append(camera_id)
    return camera_ids


def pinhole_parameters(width, height, fovy, baseline):
    """无畸变、平行双目的理想内参；右目 P 中 Tx=-fx*baseline。"""
    focal = height / (2*math.tan(math.radians(fovy)/2))
    cx, cy = width/2., height/2.
    k = [focal, 0., cx, 0., focal, cy, 0., 0., 1.]
    left = [focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.]
    right = left.copy()
    right[3] = -focal*baseline
    return k, left, right


class StereoCamera:
    def __init__(self, model, *, width=640, height=360, quality=80,
                 on_frame=None):
        if not 16 <= width <= 1920 or not 16 <= height <= 1080:
            raise ValueError('Invalid per-eye camera resolution')
        if not 1 <= quality <= 100:
            raise ValueError('Invalid JPEG quality')
        self.model = model
        self.width, self.height = int(width), int(height)
        self.settings = SimpleNamespace(quality=int(quality))
        self.on_frame = on_frame
        self.changed = threading.Condition()
        self._pending = None
        self._frame = self._jpeg = None
        self._stop = False
        self._thread = None
        self.error = ''
        self.render_fps = 0.
        self._stale_timeout = 2.

    def start(self):
        self._thread = threading.Thread(target=self._run, name='mujoco-stereo', daemon=True)
        self._thread.start()

    def submit(self, data, stamp_ns):
        # 快照覆盖，不排队；主循环只复制少量状态，不做渲染或 JPEG 编码。
        with self.changed:
            if self._stop or self.error:
                return
            self._pending = (data.qpos.copy(), data.qvel.copy(), data.time,
                             int(stamp_ns), time.monotonic())
            self.changed.notify_all()

    def latest_frame(self):
        with self.changed:
            if self._frame and time.monotonic()-self._frame.monotonic <= self._stale_timeout:
                return self._frame
        return None

    def latest_jpeg(self):
        with self.changed:
            if self._jpeg and time.monotonic()-self._jpeg[1] <= self._stale_timeout:
                return self._jpeg
        return None

    def health(self):
        frame = self.latest_frame()
        return dict(ready=frame is not None, error=self.error, synthetic=True,
                    source='mujoco_stereo', width=2*self.width, height=self.height,
                    capture_fps=self.render_fps,
                    age_s=None if frame is None else time.monotonic()-frame.monotonic)

    def _run(self):
        renderer = None
        try:
            # GL 上下文必须在创建它的线程中使用和释放。
            data = mujoco.MjData(self.model)
            renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            started, count, sequence = time.monotonic(), 0, 0
            while True:
                with self.changed:
                    self.changed.wait_for(lambda: self._stop or self._pending is not None)
                    if self._stop:
                        break
                    qpos, qvel, sim_time, stamp_ns, captured = self._pending
                    self._pending = None
                data.qpos[:], data.qvel[:], data.time = qpos, qvel, sim_time
                mujoco.mj_forward(self.model, data)
                eyes = []
                for name in ('stereo_left', 'stereo_right'):
                    renderer.update_scene(data, camera=name)
                    # MuJoCo RGB -> ROS/OpenCV BGR；左/右渲染之间不推进物理。
                    eyes.append(np.ascontiguousarray(renderer.render()[..., ::-1]))
                stereo = np.concatenate(eyes, axis=1)
                ok, encoded = cv2.imencode('.jpg', stereo,
                    [cv2.IMWRITE_JPEG_QUALITY, self.settings.quality])
                if not ok:
                    raise RuntimeError('Stereo JPEG encoding failed')
                sequence += 1
                frame = Frame(sequence, captured, stamp_ns, stereo, eyes[0], eyes[1])
                with self.changed:
                    self._frame = frame
                    self._jpeg = (sequence, captured, encoded.tobytes())
                    self.changed.notify_all()
                if self.on_frame:
                    self.on_frame(frame)
                count += 1
                elapsed = time.monotonic()-started
                if elapsed >= 1:
                    self.render_fps = count/elapsed
                    started, count = time.monotonic(), 0
        except Exception as exc:
            with self.changed:
                self.error = str(exc)
                self.changed.notify_all()
        finally:
            if renderer:
                renderer.close()

    def close(self):
        with self.changed:
            self._stop = True
            self.changed.notify_all()
        if self._thread:
            self._thread.join(timeout=10.)

"""相机采集服务：设备唯一所有者、采集/编码线程、最新帧缓存。

设计
----
* 一个进程只允许一个 ``VideoCapture``（USB 带宽与驱动限制），所有消费者共享；
* 采集线程按 ``fps`` 抓帧，编码线程按需生成 JPEG；两者都只保留最新一帧，
  不允许历史帧排队，否则预览延迟会持续增长；
* 不依赖 ROS：``camera_node`` 从这里取帧并转换成 Image。
"""
from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class Settings:
    device: str = '/dev/video0'
    width: int = 2560
    height: int = 720
    fps: float = 30.0
    quality: int = 80
    swap_eyes: bool = True
    stale_timeout: float = 2.0
    synthetic: bool = False

    def validate(self):
        if self.width < 4 or self.width % 2 or self.height < 2:
            raise ValueError('Stereo width must be even; image dimensions must be positive')
        if not 0 < self.fps <= 240 or not 1 <= self.quality <= 100:
            raise ValueError('Invalid fps or JPEG quality')
        if self.stale_timeout <= 0:
            raise ValueError('stale_timeout must be positive')


@dataclass(frozen=True)
class Frame:
    # Images are immutable by convention. Consumers must copy before drawing.
    sequence: int
    monotonic: float
    ros_stamp_ns: int
    stereo: np.ndarray
    left: np.ndarray
    right: np.ndarray


def split_eyes(image, swap):
    """只切分视图；swap 修正物理左右目，不改变整幅拼接图的线序。"""
    middle = image.shape[1] // 2
    left, right = image[:, :middle], image[:, middle:]
    return (right, left) if swap else (left, right)


class StereoCapture:
    def __init__(self, settings, stamp_ns=time.time_ns):
        settings.validate()
        self.settings = settings
        self.stamp_ns = stamp_ns
        self._stop = threading.Event()
        self.changed = threading.Condition()
        self._frame = None
        self._jpeg = None  # (sequence, capture monotonic timestamp, bytes)
        self._threads = []
        self._cap = None
        self.error = ''
        self.capture_fps = self.encode_fps = 0.0

    def start(self):
        cfg = self.settings
        if not cfg.synthetic:
            self._open_device()
        for name, target in (('capture', self._capture_loop), ('jpeg', self._encode_loop)):
            thread = threading.Thread(target=target, name='camera-' + name, daemon=True)
            self._threads.append(thread)
            thread.start()

    def _open_device(self):
        """只由采集对象拥有 VideoCapture；先检查真实分辨率，再启动线程。"""
        cfg = self.settings
        device = int(cfg.device) if str(cfg.device).isdigit() else cfg.device
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not self._cap.isOpened():
                raise RuntimeError('Cannot open camera: ' + str(cfg.device))
            for prop, value in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                                (cv2.CAP_PROP_FRAME_WIDTH, cfg.width),
                                (cv2.CAP_PROP_FRAME_HEIGHT, cfg.height),
                                (cv2.CAP_PROP_FPS, cfg.fps), (cv2.CAP_PROP_BUFFERSIZE, 1)):
                self._cap.set(prop, value)
            ok, image = self._cap.read()
            if not ok:
                raise RuntimeError('Camera opened but returned no frame')
            if image.shape[:2] != (cfg.height, cfg.width):
                raise RuntimeError('Camera returned {}, expected {}x{}'.format(
                    image.shape, cfg.width, cfg.height))
            self._publish(image)
        except BaseException:
            self._cap.release()
            self._cap = None
            raise

    def _publish(self, image):
        """覆盖最新帧，不排队；时间戳是主机收帧时刻，不是硬件曝光时刻。"""
        left, right = split_eyes(image, self.settings.swap_eyes)
        with self.changed:
            seq = self._frame.sequence + 1 if self._frame else 1
            self._frame = Frame(seq, time.monotonic(), self.stamp_ns(), image, left, right)
            self.changed.notify_all()

    def _capture_loop(self):
        started, count, failures = time.monotonic(), 0, 0
        cfg = self.settings
        try:
            while not self._stop.is_set():
                if cfg.synthetic:
                    image = np.zeros((cfg.height, cfg.width, 3), dtype=np.uint8)
                    image[:, :cfg.width // 2] = (30, 80, 180)
                    image[:, cfg.width // 2:] = (180, 100, 30)
                    cv2.putText(image, str(count), (10, cfg.height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                    ok = True
                else:
                    ok, image = self._cap.read()
                if not ok or image is None:
                    failures += 1
                    if failures >= 30:
                        raise RuntimeError('30 consecutive camera read failures')
                    self._stop.wait(0.03)
                    continue
                failures = 0
                if image.shape[:2] != (cfg.height, cfg.width):
                    raise RuntimeError('Camera frame size changed during capture')
                self._publish(image)
                count += 1
                elapsed = time.monotonic() - started
                if elapsed >= 1:
                    self.capture_fps = count / elapsed
                    started, count = time.monotonic(), 0
                if cfg.synthetic:
                    self._stop.wait(1.0 / cfg.fps)
        except Exception as exc:
            self._fail(exc)

    def _encode_loop(self):
        last, count, started = 0, 0, time.monotonic()
        try:
            while not self._stop.is_set():
                with self.changed:
                    self.changed.wait_for(lambda: self._stop.is_set() or
                                           (self._frame and self._frame.sequence != last), 0.5)
                    frame = self._frame
                if self._stop.is_set():
                    break
                if frame is None or frame.sequence == last:
                    continue
                last = frame.sequence
                # Preserve camera wire order: current Windows UI already swaps eyes.
                ok, data = cv2.imencode('.jpg', frame.stereo,
                                        [cv2.IMWRITE_JPEG_QUALITY, self.settings.quality])
                if not ok:
                    raise RuntimeError('JPEG encoding failed')
                with self.changed:
                    self._jpeg = (last, frame.monotonic, data.tobytes())
                    self.changed.notify_all()
                count += 1
                elapsed = time.monotonic() - started
                if elapsed >= 1:
                    self.encode_fps = count / elapsed
                    started, count = time.monotonic(), 0
        except Exception as exc:
            self._fail(exc)

    def _fail(self, error):
        """采集或编码失败时同时唤醒消费者，避免等待永远不会到来的下一帧。"""
        self.error = str(error)
        self._stop.set()
        with self.changed:
            self.changed.notify_all()

    def latest_frame(self):
        with self.changed:
            frame = self._frame
        if frame and time.monotonic() - frame.monotonic <= self.settings.stale_timeout:
            return frame
        return None

    def latest_jpeg(self):
        with self.changed:
            jpeg = self._jpeg
        if jpeg and time.monotonic() - jpeg[1] <= self.settings.stale_timeout:
            return jpeg
        return None

    def health(self):
        with self.changed:
            frame = self._frame
        age = time.monotonic() - frame.monotonic if frame else None
        return dict(ok=not self.error and age is not None and age <= self.settings.stale_timeout,
                    error=self.error, age_s=age, sequence=frame.sequence if frame else 0,
                    width=self.settings.width, height=self.settings.height,
                    capture_fps=self.capture_fps, encode_fps=self.encode_fps,
                    synthetic=self.settings.synthetic)

    def stop(self):
        self._stop.set()
        with self.changed:
            self.changed.notify_all()
        for thread in self._threads:
            thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None

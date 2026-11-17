import json
import time
from urllib.error import HTTPError
from urllib.request import urlopen

import cv2
import numpy as np
import pytest

from uw_robot_sensors.camera.capture import Settings, StereoCapture, split_eyes
from uw_robot_sensors.camera.stream import PreviewServer


def test_split_and_swap_preserve_correspondence():
    image = np.zeros((4, 8, 3), np.uint8)
    image[:, :4] = 7
    image[:, 4:] = 42
    left, right = split_eyes(image, True)
    assert np.all(left == 42) and np.all(right == 7)
    left, right = split_eyes(image, False)
    assert np.all(left == 7) and np.all(right == 42)


def test_invalid_settings():
    for settings in (Settings(width=3), Settings(fps=0), Settings(quality=101)):
        with pytest.raises(ValueError):
            settings.validate()


def test_capture_http_freshness_and_cleanup():
    capture = StereoCapture(Settings(width=320, height=90, fps=20.0,
                                      synthetic=True, stale_timeout=0.2), lambda: 1234567890)
    preview = PreviewServer(capture, '127.0.0.1', 0)
    base = 'http://127.0.0.1:' + str(preview.server.server_port)
    try:
        capture.start()
        preview.start()
        deadline = time.monotonic() + 3
        while capture.latest_jpeg() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        frame = capture.latest_frame()
        assert frame is not None and frame.ros_stamp_ns == 1234567890
        assert frame.left.shape == frame.right.shape == (90, 160, 3)
        with urlopen(base + '/health', timeout=2) as response:
            health = json.load(response)
            assert health['ok'] and health['synthetic']
        for path, expected in (('/snapshot.jpg', (90, 320, 3)),
                               ('/left.jpg', (90, 160, 3)), ('/right.jpg', (90, 160, 3))):
            with urlopen(base + path, timeout=2) as response:
                image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
                assert image.shape == expected
        with urlopen(base + '/?action=stream', timeout=2) as response:
            assert response.readline() == b'--frame\r\n'
            assert b'image/jpeg' in response.readline()
        with pytest.raises(HTTPError) as error:
            urlopen(base + '/unknown', timeout=2)
        assert error.value.code == 404
        capture.stop()
        time.sleep(0.25)
        assert not capture.health()['ok']
        with pytest.raises(HTTPError) as error:
            urlopen(base + '/snapshot.jpg', timeout=2)
        assert error.value.code == 503
    finally:
        preview.stop()
        capture.stop()
    assert not preview.thread.is_alive()
    assert not any(thread.is_alive() for thread in capture._threads)

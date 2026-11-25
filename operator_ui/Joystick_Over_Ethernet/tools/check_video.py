"""Offline original-resolution MJPEG test; loopback HTTP only, no command client."""
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time
import cv2
import numpy as np
import pygame
import main as ui

pygame.init()
image = np.zeros((720, 2560, 3), dtype=np.uint8)
image[:, :1280] = (255, 0, 0)  # BGR blue
image[:, 1280:] = (0, 255, 0)
ok, encoded = cv2.imencode(".jpg", image)
assert ok
jpeg = encoded.tobytes()
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        for _ in range(2):
            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
            self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
            self.wfile.write(jpeg + b"\r\n")
        self.close_connection = True
    def log_message(self, *_): pass
server = HTTPServer(("127.0.0.1", 0), Handler)
thread = threading.Thread(target=server.serve_forever, daemon=True)
thread.start()
receiver = ui.VideoReceiver(f"http://127.0.0.1:{server.server_port}/stereo.mjpg")
try:
    receiver.start()
    receiver.join(timeout=3)
    assert not receiver.is_alive(), "closed stream must not spin indefinitely"
    surface = receiver.get_surface()
    assert surface is not None
    color = surface.get_at((100, 100))
    if ui.VIDEO_SWAP_EYES:
        assert color.g > 200 and color.b < 20, color
    else:
        assert color.b > 200 and color.g < 20, color
    receiver._last_frame_at = time.monotonic() - ui.VIDEO_STALE_TIMEOUT_S - .1
    assert receiver.get_surface() is None, "stale image must disappear"
    print("PASS: 2560x720 MJPEG, eye order, EOF exit and stale image clearing")
finally:
    receiver.stop()
    receiver.join(timeout=3)
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)
    pygame.quit()

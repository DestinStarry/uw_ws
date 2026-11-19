"""HTTP 预览服务：健康状态、截图与 MJPEG 流；不拥有相机、不依赖 ROS。

端点
----
``/health``、``/snapshot.jpg``、``/stereo.jpg``、``/left.jpg``、``/right.jpg``、
``/?action=stream``、``/stereo.mjpg``。

线程
----
``ThreadingHTTPServer``，每个请求一个线程；共享帧按只读使用，要绘制检测结果
必须先复制，避免与采集线程竞争。
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading

import cv2


class PreviewServer:
    def __init__(self, capture, host='0.0.0.0', port=8080):
        self.capture = capture
        self._stop = threading.Event()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split('?', 1)[0]
                self.connection.settimeout(2.0)
                if path == '/health':
                    return self.payload(json.dumps(capture.health()).encode(), 'application/json')
                if path in ('/snapshot.jpg', '/stereo.jpg', '/left.jpg', '/right.jpg'):
                    jpeg = self.snapshot(path)
                    if jpeg is None:
                        return self.send_error(503, 'No fresh camera frame')
                    return self.payload(jpeg, 'image/jpeg')
                if path not in ('/', '/stereo.mjpg'):
                    return self.send_error(404)
                return self.stream_mjpeg()

            def stream_mjpeg(self):
                """只发送最新 JPEG；慢客户端不形成历史视频帧队列。"""
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last = 0
                try:
                    while not owner._stop.is_set():
                        with capture.changed:
                            capture.changed.wait_for(lambda: owner._stop.is_set() or
                                (capture._jpeg and capture._jpeg[0] != last), 0.5)
                        item = capture.latest_jpeg()
                        if not item or item[0] == last:
                            if capture.error:
                                break
                            owner._stop.wait(0.05)
                            continue
                        last, _, jpeg = item
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                        self.wfile.write(('Content-Length: %d\r\n\r\n' % len(jpeg)).encode())
                        self.wfile.write(jpeg + b'\r\n')
                except OSError:
                    pass

            def snapshot(self, path):
                """整幅 JPEG 复用缓存；单目截图按需编码，尊重 swap_eyes。"""
                if path in ('/snapshot.jpg', '/stereo.jpg'):
                    item = capture.latest_jpeg()
                    jpeg = item[2] if item else None
                else:
                    frame = capture.latest_frame()
                    jpeg = None
                    if frame:
                        eye = frame.left if path == '/left.jpg' else frame.right
                        ok, data = cv2.imencode('.jpg', eye,
                            [cv2.IMWRITE_JPEG_QUALITY, capture.settings.quality])
                        if ok:
                            jpeg = data.tobytes()
                return jpeg

            def payload(self, data, kind):
                self.send_response(200)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *_):
                pass

        # Bind before camera acquisition; port collisions fail without opening USB.
        self.server = ThreadingHTTPServer((host, int(port)), Handler)
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       name='camera-http', daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        with self.capture.changed:
            self.capture.changed.notify_all()
        if self.thread is not None:
            self.server.shutdown()
            self.thread.join(timeout=3.0)
        self.server.server_close()

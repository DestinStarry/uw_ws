"""TCP控制客户端：通过运行状态回传判断机载程序是否在线。"""
import socket
import threading
import time


class CommandClient:
    def __init__(self, host, port, status_timeout=2.0):
        self.host, self.port = host, port
        self.status_timeout = status_timeout
        self.sock = None
        self.connected = False
        self.latest_data = {}
        self._received_at = {}
        self._lock = threading.RLock()
        self._data_lock = self._lock
        self._last_status_time = None
        self._connected_at = 0.0
        self._last_reconnect_time = float('-inf')
        self._reconnect_interval = 3.0
        self._connecting = False
        self._closed = False
        self._status = '未连接'
        self._recv_thread = None

    @property
    def connection_status(self):
        with self._lock:
            return self._status

    def _disconnect(self, sock, reason):
        with self._lock:
            if sock is not self.sock:
                return
            self.sock = None
            self.connected = False
            self._last_status_time = None
            self.latest_data.clear()
            self._received_at.clear()
            self._status = reason
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()

    def _recv_loop(self, sock):
        # 每个连接独享socket和缓冲，旧线程不能影响重连后的状态。
        buf = b''
        while True:
            with self._lock:
                if self.sock is not sock:
                    return
                last = self._last_status_time
                deadline = self._connected_at if last is None else last
            if time.monotonic() - deadline > self.status_timeout:
                self._disconnect(sock, '机载程序无响应')
                return
            try:
                data = sock.recv(4096)
                if not data:
                    self._disconnect(sock, '未连接')
                    return
                buf += data
                if len(buf) > 65536:
                    self._disconnect(sock, '回传数据异常')
                    return
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.decode('utf-8', errors='ignore').strip()
                    if ':' not in line:
                        continue
                    key, value = (s.strip() for s in line.split(':', 1))
                    with self._lock:
                        if self.sock is not sock:
                            return
                        self.latest_data[key] = value
                        self._received_at[key] = time.monotonic()
                        # orange每0.1秒发送ARMED；传感器数据不能冒充控制心跳。
                        if key == 'ARMED' and value in ('0', '1'):
                            self._last_status_time = time.monotonic()
                            self.connected = True
                            self._status = '已连接'
            except socket.timeout:
                continue
            except OSError:
                self._disconnect(sock, '未连接')
                return

    def _connect_worker(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(2.0)
            sock.connect((self.host, self.port))
            sock.settimeout(0.2)
            with self._lock:
                if self._closed:
                    sock.close()
                    return
                self.sock = sock
                self.connected = False
                self.latest_data.clear()
                self._received_at.clear()
                self._last_status_time = None
                self._connected_at = time.monotonic()
                self._status = '等待回传'
            self._recv_thread = threading.Thread(target=self._recv_loop, args=(sock,), daemon=True)
            self._recv_thread.start()
        except OSError:
            sock.close()
            with self._lock:
                if not self._closed:
                    self._status = '未连接'
        finally:
            with self._lock:
                self._connecting = False

    def connect(self):
        return self.try_reconnect()

    def try_reconnect(self):
        with self._lock:
            now = time.monotonic()
            if (self._closed or self.sock is not None or self._connecting
                    or now - self._last_reconnect_time < self._reconnect_interval):
                return False
            self._last_reconnect_time = now
            self._connecting = True
            self._status = '连接中'
        threading.Thread(target=self._connect_worker, daemon=True).start()
        return True

    def get(self, key, default=None):
        with self._lock:
            return self.latest_data.get(key, default) if self.connected else default

    def telemetry_snapshot(self):
        """Atomic display snapshot with per-field receive ages; heartbeat is unchanged."""
        with self._lock:
            if not self.connected:
                return {}, {}
            now = time.monotonic()
            values = dict(self.latest_data)
            ages = {key: now - stamp for key, stamp in self._received_at.items()}
            return values, ages

    def send(self, message):
        with self._lock:
            sock = self.sock
        if sock is None:
            self.try_reconnect()
            return False
        try:
            sock.sendall((message + '\n').encode('utf-8'))
            return True
        except OSError:
            self._disconnect(sock, '未连接')
            return False

    def close(self):
        with self._lock:
            self._closed = True
            sock = self.sock
            self._status = '未连接'
        if sock is not None:
            self._disconnect(sock, '未连接')

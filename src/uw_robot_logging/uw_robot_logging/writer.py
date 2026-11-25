"""Bounded, asynchronous session writer. Files are touched only by its worker."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import queue
import re
import shutil
import socket
import subprocess
import threading
import time
from datetime import datetime, timezone
import uuid

from .schema import header


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, data):
    temp = path.with_suffix(path.suffix+".tmp")
    with temp.open("w", encoding="utf-8") as stream:
        json.dump(data,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp,path)


def revision(start):
    for root in [start, *start.parents, Path.cwd(), *Path.cwd().parents]:
        if (root/".git").exists():
            try:
                head = subprocess.check_output(["git","-C",str(root),"rev-parse","HEAD"],timeout=2,stderr=subprocess.DEVNULL).decode().strip()
                dirty = bool(subprocess.check_output(["git","-C",str(root),"status","--porcelain"],timeout=2,stderr=subprocess.DEVNULL).strip())
                return {"revision":head,"dirty":dirty}
            except (OSError,subprocess.SubprocessError): break
    return {"revision":None,"dirty":None}


class RecordingLimit(Exception): pass


class SessionWriter:
    def __init__(self, directory, metadata, label="", capacity=4096, flush_interval=1.,
                 min_free_mb=512., max_session_mb=2048., fsync=True):
        if capacity < 1 or not math.isfinite(flush_interval) or flush_interval <= 0:
            raise ValueError("invalid log queue size/flush interval")
        if any(not math.isfinite(v) or v < 0 for v in (min_free_mb,max_session_mb)):
            raise ValueError("invalid disk limits")
        operator_label = label[:256]
        label = re.sub(r"[^A-Za-z0-9_-]+","_",label)[:48].strip("_")
        session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_"+uuid.uuid4().hex[:8]
        self.path = Path(directory).expanduser().resolve()/(session+("_"+label if label else ""))
        self.metadata = dict(metadata, schema_version=1, session_id=session, label=operator_label, file_label=label,
                             started_utc=utc_now(), host=socket.gethostname(), python=platform.python_version(),
                             queue_capacity=capacity, flush_interval_s=flush_interval,
                             min_free_mb=min_free_mb,max_session_mb=max_session_mb,fsync=fsync)
        self.started = time.monotonic()
        self.ended = None
        self.flush_interval = flush_interval
        self.min_free_bytes = min_free_mb*1024*1024
        self.max_bytes = max_session_mb*1024*1024
        self.fsync = fsync
        self.queue = queue.Queue(maxsize=capacity)
        self.stop = threading.Event()
        self.finished = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.stats = dict(state="starting",error="",csv_rows=0,event_rows=0,dropped_csv=0,
                          dropped_events=0,abandoned_records=0,disk_free_mb=None)
        self.parameters = {}
        self.thread = threading.Thread(target=self._run,name="flight-log-writer",daemon=True)
        self.thread.start()

    def status(self):
        with self.lock:
            result = dict(self.stats)
            ended = self.ended
        result.update(session_dir=str(self.path),queue_depth=self.queue.qsize(),elapsed_s=(ended if ended is not None else time.monotonic())-self.started)
        return result

    def _update(self, **values):
        with self.lock: self.stats.update(values)

    def _submit(self, kind, payload):
        with self.lock:
            if self.stop.is_set() or self.finished.is_set(): return False
            try:
                self.queue.put_nowait((kind,payload))
                return True
            except queue.Full:
                key = "dropped_csv" if kind == "csv" else "dropped_events"
                self.stats[key] += 1
                return False

    def sample(self, row): return self._submit("csv",dict(row))
    def event(self, event): return self._submit("event",dict(event))

    def close(self, timeout=3.):
        with self.lock:
            self.stop.set()
            if not self.finished.is_set():self.stats["state"]="draining"
        self.thread.join(timeout=timeout)
        return self.finished.is_set() and self.status()["state"] == "closed"

    def _disk_check(self, size):
        free = shutil.disk_usage(self.path).free
        self._update(disk_free_mb=free/(1024*1024))
        if free < self.min_free_bytes: raise RecordingLimit("disk_low")
        if self.max_bytes and size >= self.max_bytes: raise RecordingLimit("session_size_limit")

    def _write_record(self, kind, payload, csv_writer, event_stream):
        if kind == "csv":
            csv_writer.writerow(payload)
            with self.lock: self.stats["csv_rows"] += 1
        else:
            event_stream.write(json.dumps(payload,ensure_ascii=False,allow_nan=False)+"\n")
            with self.lock: self.stats["event_rows"] += 1
            if payload.get("kind") == "parameters":
                self.parameters[payload["node"]] = payload["values"]
                atomic_json(self.path/"parameters.json",self.parameters)
            elif payload.get("kind") == "parameter_change":
                current=self.parameters.setdefault(payload["node"],{})
                current.update(payload["values"])
                for name in payload.get("deleted",[]):current.pop(name,None)
                atomic_json(self.path/"parameters.json",self.parameters)

    def _flush(self, *streams):
        for stream in streams:
            stream.flush()
            if self.fsync: os.fsync(stream.fileno())

    def _run(self):
        streams = []
        final_state, error = "closed", ""
        try:
            self.path.mkdir(parents=True,exist_ok=False)
            config_path = Path(self.metadata.get("config_path","")).expanduser()
            config_info = {"path":str(config_path),"copied":False}
            if config_path.is_file():
                data = config_path.read_bytes()
                (self.path/"robot.yaml").write_bytes(data)
                config_info.update(copied=True,sha256=hashlib.sha256(data).hexdigest())
            self.metadata["configuration"] = config_info
            self.metadata["git"] = revision(config_path.resolve())
            atomic_json(self.path/"metadata.json",self.metadata)
            self._disk_check(0)
            data_file = (self.path/"flight.csv").open("w",newline="",encoding="utf-8")
            streams.append(data_file)
            events = (self.path/"events.jsonl").open("w",encoding="utf-8")
            streams.append(events)
            csv_writer = csv.DictWriter(data_file,fieldnames=header(),extrasaction="raise")
            csv_writer.writeheader()
            self._flush(*streams)
            self._update(state="draining" if self.stop.is_set() else "recording")
            self.ready.set()
            last_flush = time.monotonic()
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    kind,payload = self.queue.get(timeout=.1)
                    try: self._write_record(kind,payload,csv_writer,events)
                    finally: self.queue.task_done()
                except queue.Empty: pass
                # Bound a session even while the queue is continuously nonempty.
                size = data_file.tell()+events.tell()
                if self.max_bytes and size >= self.max_bytes: raise RecordingLimit("session_size_limit")
                if time.monotonic()-last_flush >= self.flush_interval:
                    self._flush(*streams)
                    self._disk_check(size)
                    last_flush = time.monotonic()
            self._flush(*streams)
        except RecordingLimit as exc:
            final_state,error = "limited",str(exc)
        except Exception as exc:
            final_state,error = "failed",type(exc).__name__+": "+str(exc)
        finally:
            with self.lock:
                self.stop.set()
                self.ended=time.monotonic()
            # Current queued records may be abandoned on disk errors or storage limits.
            self._update(state=final_state,error=error,abandoned_records=self.queue.qsize())
            for stream in streams:
                try: self._flush(stream); stream.close()
                except OSError as exc:
                    final_state,error = "failed","close: "+str(exc)
                    self._update(state=final_state,error=error)
            summary = self.status()
            summary.update(ended_utc=utc_now(),clean_close=final_state=="closed")
            try:
                if self.path.is_dir(): atomic_json(self.path/"summary.json",summary)
            except OSError as exc:
                self._update(state="failed",error="summary: "+str(exc))
            self.ready.set()
            self.finished.set()

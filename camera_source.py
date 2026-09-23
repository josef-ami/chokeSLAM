"""
OV5647 fisheye camera on the Pi 5 (checkpoint D, decision #64): Picamera2
(libcamera) in a background thread, keeping only the newest frame.

    get_latest_frame() -> (frame, t_capture, frame_no) | None
        frame      numpy uint8 H x W x 3, BGR order (Picamera2 "RGB888" is
                   stored B, G, R -- what OpenCV expects), RAW: not yet
                   rotated (color_id.correct_frame does that, once)
        t_capture  the frame's sensor timestamp on the Pi's time.monotonic()
                   clock -- the clock the tracker's pose history uses
        frame_no   increases by 1 per captured frame (to skip repeats)

TIMESTAMP: libcamera's SensorTimestamp (ns) comes from the kernel's buffer
timestamp, CLOCK_MONOTONIC, the clock time.monotonic() reads on Linux. That is
checked, not assumed: if a stamp is more than 1 s away from the arrival time,
the arrival time is used instead and counted (status "stamp_fallbacks") --
arrival is later than capture by the processing latency, which
CAMERA_TIME_OFFSET_S would then have to absorb.

Picamera2 is imported lazily (in the thread), so tests and mock mode don't
need it. The mode must be the one the lens was calibrated in
(CAMERA_WIDTH x CAMERA_HEIGHT).
"""
from __future__ import annotations

import threading
import time

import config


class Picamera2Source:
    def __init__(self, width: int | None = None, height: int | None = None):
        self.size = (width or config.CAMERA_WIDTH, height or config.CAMERA_HEIGHT)
        self._lock = threading.Lock()
        self._latest = None
        self._n = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.stamp_fallbacks = 0
        self._arrivals: list[float] = []

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="camera")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def get_latest_frame(self):
        with self._lock:
            return self._latest

    def status(self) -> dict:
        with self._lock:
            latest, arr = self._latest, list(self._arrivals)
        now = time.monotonic()
        recent = [t for t in arr if now - t <= 1.0]
        return {"source": "Picamera2", "size": list(self.size), "thread_alive": self.is_alive(),
                "error": self.error, "frames": self._n,
                "fps": float(len(recent)) if recent else 0.0,
                "last_age_s": None if latest is None else round(now - latest[1], 3),
                "stamp_fallbacks": self.stamp_fallbacks, "simulated": False}

    def _run(self):
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            cfg = cam.create_video_configuration(main={"size": self.size, "format": "RGB888"}, buffer_count=4)
            cam.configure(cfg)
            cam.start()
        except Exception as e:                       # no picamera2, no camera, busy, ...
            self.error = f"could not open the camera: {type(e).__name__}: {e}"
            print(f"[camera] {self.error}")
            return
        try:
            while not self._stop.is_set():
                req = cam.capture_request()
                try:
                    frame = req.make_array("main").copy()
                    md = req.get_metadata()
                finally:
                    req.release()
                arrival = time.monotonic()
                ts = md.get("SensorTimestamp")
                t = ts / 1e9 if ts is not None else None
                if t is None or abs(arrival - t) > 1.0:
                    self.stamp_fallbacks += 1
                    t = arrival
                with self._lock:
                    self._n += 1
                    self._latest = (frame, t, self._n)
                    self._arrivals.append(arrival)
                    if len(self._arrivals) > 120:
                        del self._arrivals[:60]
        except Exception as e:
            if not self._stop.is_set():
                self.error = f"capture failed: {type(e).__name__}: {e}"
                print(f"[camera] {self.error}")
        finally:
            try:
                cam.stop()
                cam.close()
            except Exception:
                pass

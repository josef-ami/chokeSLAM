"""
Real-hardware LIDAR source, built against the documented `rplidarc1` API
(pip install rplidarc1; only dependency is pyserial). This is written to
match that package's asyncio-based interface:

    from scanner import RPLidar
    lidar = RPLidar(port, baudrate)
    async with asyncio.TaskGroup() as tg:
        tg.create_task(lidar.simple_scan())
    # lidar.output_queue: asyncio.Queue of {"a_deg": float, "d_mm": float, "q": int}

NOTE: this file could not be tested against real hardware in this session
(no device, and this sandbox has no network access to even pip-install
rplidarc1 to check the API by hand) -- it's written directly from the
package's published docs. Please sanity-check it against your own
lidar_probe.py / rplidarc1 experience before trusting it, particularly the
exact field names on the queue items and whether `simple_scan` needs to be
re-created after `stop_event` fires versus reused.

Rather than trying to segment discrete 360-degree rotations (fiddly to get
right and not actually necessary here), this keeps a rolling "latest range
seen at each angle bucket" table, so get_latest_scan() always returns a
current best estimate of the full surroundings -- fine for a sensor spinning
much faster than the ~once-per-corner rate we actually sample it at.
"""
from __future__ import annotations

import asyncio
import threading
import time


class RPLidarC1Source:
    def __init__(self, port: str, baudrate: int, timeout: float, angle_bucket_deg: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._bucket = angle_bucket_deg
        self._table: dict[int, tuple[float, float, int]] = {}   # bucket -> (angle_deg, dist_mm, quality)
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_flag = threading.Event()
        self._lidar = None   # created inside the asyncio thread

    def start(self):
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_flag.set()
        if self._lidar is not None:
            try:
                self._lidar.stop_event.set()
            except Exception:
                pass

    def get_latest_scan(self) -> list[tuple[float, float, int]]:
        with self._lock:
            return list(self._table.values())

    # -- internals -----------------------------------------------------
    def _thread_main(self):
        asyncio.run(self._async_main())

    async def _async_main(self):
        from rplidarc1 import RPLidar  # imported lazily -- only needed in "real" mode

        self._lidar = RPLidar(self.port, self.baudrate, timeout=self.timeout)
        try:
            health = await self._as_coro_maybe(self._lidar.healthcheck)
            if health is not None:
                print(f"[lidar] healthcheck: {health}")
        except Exception as e:
            print(f"[lidar] healthcheck failed (continuing anyway): {e}")

        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._lidar.simple_scan())
            tg.create_task(self._drain())

        self._lidar.reset()

    @staticmethod
    async def _as_coro_maybe(fn):
        result = fn()
        if asyncio.iscoroutine(result):
            return await result
        return result

    async def _drain(self):
        while not self._stop_flag.is_set():
            try:
                item = await asyncio.wait_for(self._lidar.output_queue.get(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            raw_angle = item.get("a_deg")
            raw_dist = item.get("d_mm")
            if raw_angle is None or raw_dist is None:
                continue
            try:
                angle = float(raw_angle) % 360.0
                dist = float(raw_dist)
            except (TypeError, ValueError):
                continue
            quality = int(item.get("q", 0))
            bucket = int(angle / self._bucket)
            with self._lock:
                self._table[bucket] = (angle, dist, quality)
        self._lidar.stop_event.set()

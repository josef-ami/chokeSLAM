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
        # Diagnostics -- the background thread runs detached with no
        # supervisor, so if _async_main() raises (bad port, wrong
        # rplidarc1 API, etc.) the thread just dies and get_latest_scan()
        # silently returns [] forever unless something surfaces this.
        # FOUND DURING FIELD TESTING: exactly this happened -- the thread
        # was dying on startup with nothing visible about why. See status().
        self._error: str | None = None
        self._started_at: float | None = None
        self._points_received_total = 0
        self._bad_item_warned = False

    def start(self):
        self._started_at = time.monotonic()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def status(self) -> dict:
        """Diagnostic snapshot. Call this any time get_latest_scan() looks
        empty or suspicious, BEFORE guessing at a cause:
          - thread_alive=False + error set -> the background thread crashed
            on startup (bad port/permissions, or the rplidarc1 API doesn't
            match what this file assumes -- see the module docstring).
          - thread_alive=True + points_received_total=0 -> connected fine
            but no scan data has arrived at all (motor not spinning, wrong
            baudrate, or output_queue items are missing the a_deg/d_mm
            fields this code expects -- check stderr for a
            '[lidar] queue item missing expected fields' warning, which
            fires once with the item's actual keys if so).
          - thread_alive=True + points_received_total>0 but
            current_table_size small/stale -> was working, may have
            stopped (check error again -- it's set even after a mid-run
            crash, not just an immediate one)."""
        with self._lock:
            table_size = len(self._table)
        return {
            "thread_alive": self.is_alive(),
            "error": self._error,
            "points_received_total": self._points_received_total,
            "current_table_size": table_size,
            "seconds_since_start": None if self._started_at is None else round(time.monotonic() - self._started_at, 1),
        }

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
        try:
            asyncio.run(self._async_main())
        except Exception:
            import traceback
            self._error = traceback.format_exc()
            print("[lidar] background thread crashed -- get_latest_scan() will keep "
                  "returning [] from here on. Call .status() for a summary, or see the "
                  "full traceback below:")
            print(self._error)

    async def _async_main(self):
        from rplidarc1 import RPLidar  # imported lazily -- only needed in "real" mode

        print(f"[lidar] connecting on {self.port} @ {self.baudrate} baud ...")
        self._lidar = RPLidar(self.port, self.baudrate, timeout=self.timeout)
        try:
            health = await self._as_coro_maybe(self._lidar.healthcheck)
            if health is not None:
                print(f"[lidar] healthcheck: {health}")
        except Exception as e:
            print(f"[lidar] healthcheck failed (continuing anyway): {e}")

        print("[lidar] starting simple_scan() + drain loop ...")
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
                if not self._bad_item_warned:
                    self._bad_item_warned = True
                    print(f"[lidar] queue item missing expected 'a_deg'/'d_mm' fields "
                          f"(actual keys: {list(item.keys())}) -- this file's field names "
                          f"were never verified against a real device, see module docstring. "
                          f"Update the .get() calls in _drain() to match.")
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
                self._points_received_total += 1
        self._lidar.stop_event.set()

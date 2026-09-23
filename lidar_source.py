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
from collections import deque

from timing import SweepClock


class RPLidarC1Source:
    def __init__(self, port: str, baudrate: int, timeout: float, angle_bucket_deg: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._bucket = angle_bucket_deg
        # bucket -> (angle_deg, dist_mm, quality, theta, arrival): theta is the
        # angle unwrapped across revolutions (timing.SweepClock), arrival the
        # Pi's time.monotonic() when the return was taken off the queue.
        self._table: dict[int, tuple[float, float, int, float, float]] = {}
        self._clock = SweepClock()
        # every return of the last ~4 s, for recordings (measure_lidar_delay.py)
        self._recent: deque = deque(maxlen=20000)
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
        """The latest return in each angle bucket: (raw angle, dist, quality)."""
        with self._lock:
            return [(a, d, q) for a, d, q, _, _ in self._table.values()]

    def get_latest_scan_timed(self) -> list[tuple[float, float, int, float]]:
        """As get_latest_scan, plus each return's measurement time on the Pi's
        time.monotonic() clock, from the sweep (timing.SweepClock): (raw angle,
        dist, quality, t). Until the clock has about one revolution of data the
        arrival time is used instead. Used by the entry re-check's de-skew."""
        with self._lock:
            fit = self._clock.fit()
            if fit is None:
                return [(a, d, q, arr) for a, d, q, _, arr in self._table.values()]
            return [(a, d, q, SweepClock.time_of(th, fit)) for a, d, q, th, _ in self._table.values()]

    def get_points_since(self, t: float) -> list[tuple[float, float, int, float, float]]:
        """Every return that arrived after Pi time t (at most the last ~4 s):
        (raw angle, dist, quality, theta, arrival). For recordings."""
        with self._lock:
            return [p for p in self._recent if p[4] > t]

    def timing_status(self) -> dict:
        """Sweep clock health: spin rate (None until about one revolution has
        arrived), how often the raw angle stepped backwards (should stay ~0),
        and seconds since the last return arrived (None before the first)."""
        with self._lock:
            last = self._recent[-1][4] if self._recent else None
            return {"spin_hz": self._clock.spin_hz(), "backwards_steps": self._clock.backwards,
                    "last_return_age_s": None if last is None else time.monotonic() - last}

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
        # RPLidar's own constructor already connects the serial port and runs
        # a healthcheck synchronously (its _initialize()) -- confirmed by
        # reading rplidarc1's actual source (scanner.py). A second explicit
        # healthcheck() call here was redundant (that's why you saw two
        # "In waiting" lines) -- removed.
        self._lidar = RPLidar(self.port, self.baudrate, timeout=self.timeout)

        print("[lidar] starting simple_scan() + drain loop ...")
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._lidar.simple_scan())
            tg.create_task(self._drain())

        self._lidar.reset()

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
                    if "a_deg" not in item or "d_mm" not in item:
                        print(f"[lidar] queue item missing 'a_deg'/'d_mm' KEYS entirely "
                              f"(actual keys: {list(item.keys())}) -- field names were never "
                              f"verified against a real device, see module docstring. "
                              f"Update the .get() calls in _drain() to match.")
                    else:
                        print(f"[lidar] queue item has 'a_deg'/'d_mm' keys present but one is "
                              f"None (item={item!r}) -- most likely a sentinel/error/"
                              f"start-of-scan marker from rplidarc1 rather than a real point. "
                              f"Skipping it (as this does) is probably correct; only worth "
                              f"digging into further if this fires constantly rather than "
                              f"occasionally.")
                continue
            try:
                angle = float(raw_angle) % 360.0
                dist = float(raw_dist)
            except (TypeError, ValueError):
                continue
            quality = int(item.get("q", 0))
            bucket = int(angle / self._bucket)
            arrival = time.monotonic()
            with self._lock:
                theta = self._clock.add(angle, arrival)
                self._table[bucket] = (angle, dist, quality, theta, arrival)
                self._recent.append((angle, dist, quality, theta, arrival))
                self._points_received_total += 1
        self._lidar.stop_event.set()

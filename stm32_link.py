"""
STM32 -> Raspberry Pi 5 link: the BNO08x heading and the drive-motor hall
encoder, over the STM32's native USB (CDC, appears as /dev/ttyACM*; the baud
rate is ignored by CDC). Agreed format (decisions #13, #17, #18):

    One ASCII line per sample, 100 Hz, '\\n'-terminated (a trailing '\\r' is
    tolerated):

        $IMU,<seq>,<t_ms>,<enc>,<yaw>

    seq    uint32  +1 every line (the Pi counts dropped lines)
    t_ms   uint32  STM32 HAL_GetTick() when sampled
    enc    int32   cumulative hall-encoder count since power-on, forward = +,
                   never reset (a lost line loses no distance)
    yaw    float   BNO08x Game Rotation Vector yaw in degrees, as the chip
                   reports it (sign fixed on the Pi by config.IMU_YAW_SIGN)

    e.g.   $IMU,1042,10420,15873,-12.37

    Firmware log lines (decision #52): the obstacle-round firmware shares the
    port and also prints lines starting with '#' (its log) or '!' (its tuning
    replies). Those are not errors: they are counted as lines_log, the last
    LOG_KEEP are kept for display, and they are never parsed as samples.
    Anything else that isn't a valid $IMU line is still counted as bad.

    Status line (checkpoint E, decision #77): the drive firmware also sends,
    at about 20 Hz,

        $STA,<seq_ack>,<status>[,<run_state>,<run_id>,<pwm>,<speed_mmps>]

    seq_ack    the seq byte of the last valid DRIVE frame it accepted (0-255)
    status     bit flags as in stm_link.py: 0 ENABLED, 1 WATCHDOG (it cut the
               motor: no valid DRIVE frame for 250 ms), 2 BUTTON (held down
               now), 3 CLOSED_LOOP, 4 IMU_OK
    run_state  (checkpoint F) 0 READY, 1 RUNNING, 2 STOPPED, 3 FINISHED: the
               firmware owns the run (the start button, drive_protocol.h)
    run_id     runs started since the STM32 booted
    pwm        motor PWM applied, -255..255;  speed_mmps  its encoder speed
    It is kept as status()["sta"] and never counted as bad. The short form
    (the checkpoint-E firmware) reads as run_state/run_id/pwm/speed None.

    Parameter echo (checkpoint F2): $PAR,<id>,<value> -- a drive-firmware
    tuning value as the STM32 now holds it (drive_protocol.h PARAM frame). Kept
    as status()["fw_params"] {id: value} and never counted as bad.

    Writing (checkpoint E): the drive side (drive_link.py) sends its binary
    DRIVE frames through write() on the same port, which this link owns.

This module only turns bytes into validated ImuSample objects and keeps link
statistics. What the samples MEAN (distance, heading, lane position) is
lane_tracker.py's job.

Reading runs in a background thread. The serial port is opened in exactly one
place (_open_serial, pyserial, imported lazily so the rest of the codebase and
the tests don't need it). Everything after the open works on a plain byte
stream, which is how it is tested (through a pseudo-terminal) -- pyserial
itself could not be installed in the sandbox this was written in.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import config

PREFIX = "$IMU"
N_FIELDS = 5            # $IMU, seq, t_ms, enc, yaw
LOG_PREFIXES = ("#", "!")   # firmware log / tuning-reply lines (#52)
LOG_KEEP = 20               # recent log lines kept for status()
STA_PREFIX = "$STA"
PAR_PREFIX = "$PAR"


def is_log_line(line: str) -> bool:
    """A firmware log ('#') or tuning-reply ('!') line: skipped, not bad."""
    return line.strip().startswith(LOG_PREFIXES)


@dataclass(frozen=True)
class ImuSample:
    seq: int
    t_ms: int
    enc: int
    yaw_deg: float
    rx_time: float = 0.0     # time.monotonic() on the Pi when the line arrived


def parse_line(line: str) -> tuple[ImuSample | None, str]:
    """One line -> (sample, "") or (None, reason). Never raises."""
    s = line.strip()
    if not s:
        return None, "empty line"
    parts = s.split(",")
    if parts[0] != PREFIX:
        return None, f"doesn't start with {PREFIX}"
    if len(parts) != N_FIELDS:
        return None, f"{len(parts)} fields, expected {N_FIELDS}"
    try:
        seq, t_ms, enc = int(parts[1]), int(parts[2]), int(parts[3])
        yaw = float(parts[4])
    except ValueError:
        return None, "non-numeric field"
    if not math.isfinite(yaw):
        return None, "yaw is not a finite number"
    if seq < 0 or t_ms < 0:
        return None, "negative seq or t_ms"
    return ImuSample(seq, t_ms, enc, yaw), ""


@dataclass
class LinkStats:
    lines_ok: int = 0
    lines_bad: int = 0
    last_bad_reason: str = ""
    seq_gaps: int = 0            # missing lines, counted from seq jumps
    seq_resets: int = 0          # seq went backwards (STM32 restarted?)
    lines_log: int = 0           # '#' / '!' firmware lines skipped (#52)
    recent_log: deque = field(default_factory=lambda: deque(maxlen=LOG_KEEP))
    last_sample: ImuSample | None = None
    fw_params: dict = field(default_factory=dict)   # $PAR: id -> (value, rx_time)
    sta: tuple | None = None     # (seq_ack, status, rx_time, run_state, run_id, pwm, speed) of the last $STA
    rate_hz: float = 0.0         # measured over the last ~1 s of arrivals
    _arrivals: deque = field(default_factory=lambda: deque(maxlen=200))

    def note(self, sample: ImuSample):
        prev = self.last_sample
        if prev is not None:
            if sample.seq > prev.seq + 1:
                self.seq_gaps += sample.seq - prev.seq - 1
            elif sample.seq <= prev.seq:
                self.seq_resets += 1
                self.fw_params.clear()       # a restarted STM32 is back on its compiled defaults
        self.last_sample = sample
        self.lines_ok += 1
        self._arrivals.append(sample.rx_time)
        recent = [t for t in self._arrivals if sample.rx_time - t <= 1.0]
        if len(recent) >= 2 and recent[-1] > recent[0]:
            self.rate_hz = (len(recent) - 1) / (recent[-1] - recent[0])


class LineAssembler:
    """Bytes in, complete text lines out. Keeps a partial line between reads
    (a read can end mid-line), drops the very first fragment after connecting
    only if it doesn't parse, and caps the buffer so garbage without newlines
    can't grow it forever."""

    MAX_BUFFER = 4096

    def __init__(self):
        self._buf = b""

    def feed(self, data: bytes) -> list[str]:
        self._buf += data
        if len(self._buf) > self.MAX_BUFFER and b"\n" not in self._buf:
            self._buf = b""
            return ["<overflow: no newline in 4 kB>"]
        *lines, self._buf = self._buf.split(b"\n")
        return [ln.decode("ascii", errors="replace") for ln in lines]


class Stm32Link:
    """Background reader. Consumers call drain() to take all samples that
    arrived since the last call (in arrival order), and status() for
    diagnostics. Mirrors lidar_source.RPLidarC1Source's shape: start(),
    stop(), is_alive(), status()."""

    def __init__(self, port: str | None = None, baudrate: int | None = None,
                 stream=None):
        """stream: an already-open binary file-like object (tests / replay);
        if None, the serial port is opened in the background thread."""
        self.port = port or config.IMU_PORT
        self.baudrate = baudrate or config.IMU_BAUDRATE
        self._stream = stream
        self._lock = threading.Lock()
        self._pending: list[ImuSample] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: str | None = None
        self.stats = LinkStats()
        self._log_fh = None
        self._port = None                      # the open stream, once the reader thread has it
        self._wlock = threading.Lock()

    # -- public ------------------------------------------------------------
    def start(self, log_path: str | None = None):
        """log_path: also write every raw line received to this file (for
        replay / regression tests, like run_init.py --dump). The file is
        OVERWRITTEN, like the --dump scan it is replayed with: appending would
        mix runs, and a replay would then pair one run's scan with another
        run's IMU lines."""
        if log_path:
            self._log_fh = open(log_path, "w", encoding="ascii", errors="replace")
        self._thread = threading.Thread(target=self._run, daemon=True, name="stm32_link")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._log_fh:
            self._log_fh.close()
            self._log_fh = None

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def drain(self) -> list[ImuSample]:
        with self._lock:
            out, self._pending = self._pending, []
        return out

    def write(self, data: bytes) -> bool:
        """Send bytes to the STM32 on the same port (the DRIVE frames). False if
        the port is not open (yet) or the write failed."""
        port = self._port
        if port is None:
            return False
        try:
            with self._wlock:
                port.write(data)
                if hasattr(port, "flush"):
                    port.flush()
            return True
        except Exception as e:
            self._error = f"write failed: {type(e).__name__}: {e}"
            return False

    def status(self) -> dict:
        s = self.stats
        last = s.last_sample
        age = None if last is None else round(time.monotonic() - last.rx_time, 3)
        return {
            "port": self.port,
            "thread_alive": self.is_alive(),
            "error": self._error,
            "lines_ok": s.lines_ok,
            "lines_bad": s.lines_bad,
            "last_bad_reason": s.last_bad_reason,
            "seq_gaps": s.seq_gaps,
            "seq_resets": s.seq_resets,
            "lines_log": s.lines_log,
            "recent_log": list(s.recent_log),
            "rate_hz": round(s.rate_hz, 1),
            "last_age_s": age,
            "stale": age is None or age > config.IMU_STALE_S,
            "sta": None if s.sta is None else {"seq_ack": s.sta[0], "status": s.sta[1],
                                               "age_s": round(time.monotonic() - s.sta[2], 3),
                                               "run_state": s.sta[3], "run_id": s.sta[4],
                                               "pwm": s.sta[5], "speed_mm_s": s.sta[6]},
            "fw_params": {k: v for k, (v, _) in s.fw_params.items()},
            "last": None if last is None else {"seq": last.seq, "t_ms": last.t_ms,
                                                "enc": last.enc, "yaw_deg": last.yaw_deg},
        }

    # -- internals ---------------------------------------------------------
    def _open_serial(self):
        import serial   # pyserial -- only needed for a real port
        return serial.Serial(self.port, self.baudrate, timeout=0.05)

    def _read_chunk(self, stream) -> bytes:
        if hasattr(stream, "in_waiting"):                  # pyserial
            return stream.read(max(1, stream.in_waiting))
        return stream.read1(4096) if hasattr(stream, "read1") else stream.read(4096)

    def _run(self):
        asm = LineAssembler()
        try:
            stream = self._stream if self._stream is not None else self._open_serial()
        except Exception as e:                              # bad port, permissions, no pyserial
            self._error = f"could not open {self.port}: {type(e).__name__}: {e}"
            print(f"[stm32] {self._error}")
            return
        self._port = stream
        first = True
        try:
            while not self._stop.is_set():
                data = self._read_chunk(stream)
                if not data:
                    if self._stream is not None and not hasattr(stream, "in_waiting"):
                        time.sleep(0.002)
                    continue
                for line in asm.feed(data):
                    now = time.monotonic()
                    if self._log_fh:
                        self._log_fh.write(line.rstrip("\r") + "\n")
                    if line.strip().startswith(STA_PREFIX):
                        parts = line.strip().split(",")
                        try:
                            ext = [int(v) for v in parts[3:7]] if len(parts) >= 7 else [None] * 4
                            self.stats.sta = (int(parts[1]), int(parts[2]), now, *ext)
                            first = False
                            continue
                        except (IndexError, ValueError):
                            pass                     # malformed: counted as bad below
                    if line.strip().startswith(PAR_PREFIX):
                        parts = line.strip().split(",")
                        try:
                            self.stats.fw_params[int(parts[1])] = (float(parts[2]), now)
                            first = False
                            continue
                        except (IndexError, ValueError):
                            pass                     # malformed: counted as bad below
                    if is_log_line(line):
                        first = False
                        self.stats.lines_log += 1
                        self.stats.recent_log.append(line.strip()[:120])
                        continue
                    sample, why = parse_line(line)
                    if sample is None:
                        if first:            # a partial first line after connecting is expected
                            first = False
                            continue
                        self.stats.lines_bad += 1
                        self.stats.last_bad_reason = f"{why}: {line[:60]!r}"
                        continue
                    first = False
                    sample = ImuSample(sample.seq, sample.t_ms, sample.enc, sample.yaw_deg, now)
                    self.stats.note(sample)
                    with self._lock:
                        self._pending.append(sample)
        except Exception as e:
            if self._stop.is_set():
                return                      # the port closing after stop() is not an error
            self._error = f"read failed: {type(e).__name__}: {e}"
            print(f"[stm32] {self._error}")


def read_log(path: str) -> list[ImuSample]:
    """Samples from a file written with Stm32Link.start(log_path=...), in
    order, for replay. rx_time is reconstructed from t_ms."""
    out = []
    with open(path, encoding="ascii", errors="replace") as fh:
        for line in fh:
            s, _ = parse_line(line)
            if s is not None:
                out.append(ImuSample(s.seq, s.t_ms, s.enc, s.yaw_deg, s.t_ms / 1000.0))
    return out

"""
stm_link.py

Bridges path_planner's DriveCommand generator to the real Pi<->STM32 wire
protocol described in the "Pi <-> STM32 Link" spec (obstacle round only).

What this module does, matching the spec's division of labor:
    - path_planner.py decides WHERE to go (geometry: steer angle / heading,
      speed, how far). That is a decision, so it lives on the Pi -- this
      module is still Pi-side code.
    - This module's job is purely to get those decisions onto the wire in
      the exact 11-byte DRIVE frame the STM32 expects, read back the 22-byte
      TELEM frame, and decide (from ENCODER DISTANCE, not time) when a
      DriveCommand segment is complete and it's time to advance to the next
      one. That "when is this segment done" decision is also a decision, so
      it also stays on the Pi, per the spec's stated rule.
    - Steering geometry (radians -> tenths-of-a-degree) is Pi-side, per the
      spec's ownership table ("Steering geometry (angle <-> radius)": Pi).
      Servo trim, direction and degrees->microseconds stay firmware-side and
      this module never touches them.

Straight segments are sent as HEADING_HOLD (not DIRECT with steer=0), per
the spec's own rationale: a heading loop closed over a 30 Hz link would be
sluggish, so straights hand an absolute heading target to the STM32's
full-rate IMU loop instead. Arc segments are sent as DIRECT, since they are
a geometric road-wheel angle the Pi computed, exactly the case DIRECT mode
is for.

Safety model implemented here (the Pi-side half of it; the STM32 half --
its own 250ms watchdog -- runs independently in firmware regardless of
what this module does):
    - TELEM staleness check: if no valid TELEM for > 250ms, treat the link
      as down and command STOP.
    - WATCHDOG bit from TELEM: if the STM32 says it already cut the motor,
      mirror that state on the Pi side rather than fighting it.
    - Checksum: DRIVE frames are XOR-checksummed on send; TELEM frames
      failing their checksum are dropped (never acted on), matching "a
      corrupt frame is never acted on; the watchdog handles the gap".
    - Explicit all-stop frame on shutdown/exception, so the car stops on
      Ctrl-C rather than coasting until the STM32 watchdog trips.

Requires pyserial (`pip install pyserial --break-system-packages`).
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Generator, Optional

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - allows import for dry-run/testing without pyserial
    serial = None

from path_planner import DriveCommand, RobotConfig


# --------------------------------------------------------------------------
# Wire constants (from the spec -- do not change without changing firmware)
# --------------------------------------------------------------------------

DRIVE_SYNC0 = 0xAA
DRIVE_SYNC1 = 0x55
DRIVE_LEN = 11

TELEM_SYNC0 = 0x55
TELEM_SYNC1 = 0xAA
TELEM_LEN = 22

TELEM_STALE_S = 0.250       # Pi-side staleness threshold, matches STM32's 250ms watchdog
DRIVE_TICK_HZ = 30.0        # Pi control tick rate

STATUS_ENABLED = 1 << 0
STATUS_WATCHDOG = 1 << 1
STATUS_BUTTON = 1 << 2
STATUS_CLOSED_LOOP = 1 << 3
STATUS_IMU_OK = 1 << 4
STATUS_TOF_OK = 1 << 5
STATUS_COLOUR_OK = 1 << 6


class SteerMode(IntEnum):
    DIRECT = 0
    HEADING_HOLD = 1
    STOP = 2


FLAG_ENABLE = 1 << 0
FLAG_CLOSED_LOOP = 1 << 1
MODE_SHIFT = 2
MODE_MASK = 0b11 << MODE_SHIFT


# --------------------------------------------------------------------------
# Frame encode/decode
# --------------------------------------------------------------------------

def _xor8(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


@dataclass
class DriveFrame:
    seq: int
    enable: bool
    closed_loop: bool
    mode: SteerMode
    steer_ddeg: int      # int16, road-wheel angle x10, + = LEFT
    speed_mmps: int      # int16, + = FORWARD
    heading_ddeg: int    # int16, absolute target heading x10 (HEADING_HOLD only)

    def encode(self) -> bytes:
        flags = 0
        if self.enable:
            flags |= FLAG_ENABLE
        if self.closed_loop:
            flags |= FLAG_CLOSED_LOOP
        flags |= (int(self.mode) << MODE_SHIFT) & MODE_MASK

        body = struct.pack(
            "<BBhhh",
            self.seq & 0xFF,
            flags,
            _clamp_i16(self.steer_ddeg),
            _clamp_i16(self.speed_mmps),
            _clamp_i16(self.heading_ddeg),
        )
        checksum = _xor8(body)
        frame = bytes([DRIVE_SYNC0, DRIVE_SYNC1]) + body + bytes([checksum])
        assert len(frame) == DRIVE_LEN, f"DRIVE frame length mismatch: {len(frame)}"
        return frame


@dataclass
class TelemFrame:
    seq_ack: int
    status: int
    distance_mm: int
    speed_mmps: int
    heading_ddeg: int
    yaw_rate_ddps: int
    tof_front_mm: float   # inf if invalid (0xFFFF)
    tof_left_mm: float
    tof_right_mm: float
    floor_colour: int
    received_at: float    # time.monotonic() when this frame was parsed

    @property
    def enabled(self) -> bool:
        return bool(self.status & STATUS_ENABLED)

    @property
    def watchdog_tripped(self) -> bool:
        return bool(self.status & STATUS_WATCHDOG)

    @property
    def imu_ok(self) -> bool:
        return bool(self.status & STATUS_IMU_OK)

    @property
    def heading_rad(self) -> float:
        return math.radians(self.heading_ddeg / 10.0)


def _clamp_i16(v: int) -> int:
    return max(-32768, min(32767, int(v)))


def _tof_or_inf(raw: int) -> float:
    return float("inf") if raw == 0xFFFF else float(raw)


def parse_telem(buf: bytes) -> Optional[TelemFrame]:
    """Parse one 22-byte TELEM frame. Returns None if sync or checksum fail
    -- per spec, a frame failing xor8 is dropped silently, never acted on."""
    if len(buf) != TELEM_LEN:
        return None
    if buf[0] != TELEM_SYNC0 or buf[1] != TELEM_SYNC1:
        return None

    body = buf[2:21]
    checksum = buf[21]
    if _xor8(body) != checksum:
        return None

    (seq_ack, status, distance_mm, speed_mmps, heading_ddeg, yaw_rate_ddps,
     tof_f, tof_l, tof_r, floor_colour) = struct.unpack("<BBihhhHHHB", body)

    return TelemFrame(
        seq_ack=seq_ack,
        status=status,
        distance_mm=distance_mm,
        speed_mmps=speed_mmps,
        heading_ddeg=heading_ddeg,
        yaw_rate_ddps=yaw_rate_ddps,
        tof_front_mm=_tof_or_inf(tof_f),
        tof_left_mm=_tof_or_inf(tof_l),
        tof_right_mm=_tof_or_inf(tof_r),
        floor_colour=floor_colour,
        received_at=time.monotonic(),
    )


# --------------------------------------------------------------------------
# Serial transport
# --------------------------------------------------------------------------

class StmLink:
    """Owns the serial port, frame sync/parsing, and the Pi-side half of the
    safety model. Does not decide where the robot goes -- only gets Pi
    decisions onto the wire and STM32 telemetry back off it.
    """

    def __init__(self, port: str = "/dev/ttyACM0", timeout_s: float = 0.02):
        if serial is None:
            raise RuntimeError(
                "pyserial not installed. Run: pip install pyserial --break-system-packages "
                "(or use StmLink.dry_run() for testing without hardware)."
            )
        self._ser = serial.Serial(port, baudrate=115200, timeout=timeout_s)
        self._rx_buf = bytearray()
        self._seq = 0
        self.last_telem: Optional[TelemFrame] = None
        self.rx_bad = 0  # counter for the "rx_bad climbing" symptom in the debug table

    # ---- lifecycle -------------------------------------------------

    def close(self, send_stop: bool = True) -> None:
        """Per spec: send an explicit all-stop frame before closing the
        port, so the car stops on Ctrl-C rather than coasting until the
        250ms STM32 watchdog trips."""
        if send_stop:
            try:
                self.send_stop()
            except Exception:
                pass  # best-effort; we're shutting down regardless
        if self._ser is not None:
            self._ser.close()

    def __enter__(self) -> "StmLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close(send_stop=True)

    # ---- TX ----------------------------------------------------------

    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s

    def send_direct(self, steer_rad: float, speed_mmps: float, closed_loop: bool = True) -> None:
        """DIRECT mode: Pi-computed road-wheel angle. Used for arcs."""
        frame = DriveFrame(
            seq=self._next_seq(),
            enable=True,
            closed_loop=closed_loop,
            mode=SteerMode.DIRECT,
            steer_ddeg=round(math.degrees(steer_rad) * 10.0),
            speed_mmps=round(speed_mmps * 1000.0) if abs(speed_mmps) <= 10 else round(speed_mmps),
            heading_ddeg=0,
        )
        self._ser.write(frame.encode())

    def send_heading_hold(self, heading_rad: float, speed_mmps: float, closed_loop: bool = True) -> None:
        """HEADING_HOLD mode: Pi names an absolute heading, STM32 closes the
        loop on its own IMU at full rate. Used for straights."""
        frame = DriveFrame(
            seq=self._next_seq(),
            enable=True,
            closed_loop=closed_loop,
            mode=SteerMode.HEADING_HOLD,
            steer_ddeg=0,  # ignored by STM32 in this mode, per spec
            speed_mmps=round(speed_mmps * 1000.0) if abs(speed_mmps) <= 10 else round(speed_mmps),
            heading_ddeg=round(math.degrees(heading_rad) * 10.0),
        )
        self._ser.write(frame.encode())

    def send_stop(self) -> None:
        """STOP mode: motor off, steering centred, overrides other fields."""
        frame = DriveFrame(
            seq=self._next_seq(),
            enable=False,
            closed_loop=False,
            mode=SteerMode.STOP,
            steer_ddeg=0,
            speed_mmps=0,
            heading_ddeg=0,
        )
        self._ser.write(frame.encode())

    # ---- RX ------------------------------------------------------------

    def poll_telem(self) -> Optional[TelemFrame]:
        """Non-blocking-ish read: pull whatever bytes are available, scan
        for a valid TELEM frame by its (reversed vs DRIVE) sync bytes, and
        return the newest one found. Call this every Pi tick, ideally more
        often than the 50Hz TELEM rate so frames don't back up.
        """
        incoming = self._ser.read(self._ser.in_waiting or 1)
        if incoming:
            self._rx_buf.extend(incoming)

        newest: Optional[TelemFrame] = None
        # Scan for sync bytes; resync by dropping one byte at a time on mismatch.
        while len(self._rx_buf) >= TELEM_LEN:
            if self._rx_buf[0] == TELEM_SYNC0 and self._rx_buf[1] == TELEM_SYNC1:
                candidate = bytes(self._rx_buf[:TELEM_LEN])
                parsed = parse_telem(candidate)
                del self._rx_buf[:TELEM_LEN]
                if parsed is not None:
                    newest = parsed
                else:
                    self.rx_bad += 1  # checksum or structural failure -- dropped, not acted on
            else:
                del self._rx_buf[0]  # resync: slide forward one byte

        if newest is not None:
            self.last_telem = newest
        return newest

    # ---- safety --------------------------------------------------------

    def is_link_healthy(self) -> bool:
        """Pi-side staleness check, per spec: telemetry older than 250ms,
        or the STM32's own watchdog bit set, means the FSM must treat this
        as STOP. Mirrors the STM32's independent 250ms cutoff rather than
        inventing a different threshold.
        """
        if self.last_telem is None:
            return False
        age = time.monotonic() - self.last_telem.received_at
        if age > TELEM_STALE_S:
            return False
        if self.last_telem.watchdog_tripped:
            return False
        return True


# --------------------------------------------------------------------------
# DriveCommand -> wire, with odometry-based segment completion
# --------------------------------------------------------------------------

def stream_commands(
    commands: Generator[DriveCommand, None, None],
    link: StmLink,
    robot: RobotConfig,
    boot_heading_rad: float = 0.0,
    tick_hz: float = DRIVE_TICK_HZ,
) -> Generator[TelemFrame, None, None]:
    """Consume path_planner's DriveCommand generator and drive the real
    link, one segment at a time, advancing to the next command only once
    the STM32's own odometry (distance_mm delta) confirms the current
    segment's distance has actually been covered -- not a timer, matching
    the spec's own distance-based convention.

    Maintains a running absolute heading (starting from `boot_heading_rad`,
    matching the STM32's own boot-relative heading convention) so that arc
    segments' curvature can be translated into the heading the STM32 should
    be holding once the arc ends and straights resume -- this is what lets
    consecutive HEADING_HOLD straights target the correct absolute heading
    after a turn, rather than blindly holding whatever the STM32 was last
    given.

    Yields each TelemFrame as it's received, so a caller can log / feed a
    dashboard / check ToF-based reactive overrides without needing its own
    poll loop.

    Straight segments (steer_angle == 0) are sent as HEADING_HOLD. Arc
    segments (steer_angle != 0) are sent as DIRECT with the Pi-computed
    road-wheel angle, per the spec's intended use of each mode.
    """
    tick_period = 1.0 / tick_hz
    heading_rad = boot_heading_rad

    for cmd in commands:
        # Establish this segment's starting odometry reference. Wait for at
        # least one fresh TELEM frame so distance_start is real, not a
        # holdover from a previous segment (or None on the very first tick).
        distance_start = _await_fresh_distance(link, tick_period)
        if distance_start is None:
            # No telemetry arrived at all -- the link is down before we've
            # even started. Don't spin forever "waiting for distance to
            # catch up" against a start value we never got; stop and bail
            # out to the caller's FSM, same as a mid-segment link failure.
            link.send_stop()
            return

        is_arc = abs(cmd.steering_angle_rad) > 1e-9
        target_heading = heading_rad  # for straights, hold current heading
        if is_arc:
            # cmd.distance_m is the arc LENGTH; curvature = 1/radius, and
            # steering_angle_rad already encodes turn direction via its
            # sign (see path_planner.curvature_to_steering_angle). Recover
            # the signed heading change this arc will produce so the
            # NEXT straight segment can target the right absolute heading.
            radius = robot.wheelbase_m / max(abs(math.tan(cmd.steering_angle_rad)), 1e-9)
            turn_rad = cmd.distance_m / radius
            if cmd.steering_angle_rad < 0:
                turn_rad = -turn_rad
            target_heading_after = heading_rad + turn_rad
        else:
            target_heading_after = heading_rad

        traveled = 0.0
        last_progress_at = time.monotonic()
        while traveled < cmd.distance_m:
            if is_arc:
                link.send_direct(cmd.steering_angle_rad, cmd.speed_mps * 1000.0)
            else:
                link.send_heading_hold(target_heading, cmd.speed_mps * 1000.0)

            telem = link.poll_telem()
            if telem is not None:
                yield telem
                if not link.is_link_healthy():
                    # Pi-side staleness/watchdog trip: stop commanding motion
                    # and hand control back to the caller's FSM rather than
                    # silently continuing to push frames into a dead link.
                    link.send_stop()
                    return
                new_traveled = abs(telem.distance_mm - distance_start) / 1000.0  # mm -> m
                if new_traveled > traveled:
                    traveled = new_traveled
                    last_progress_at = time.monotonic()

            # Belt-and-suspenders: if odometry hasn't advanced at all for a
            # full watchdog period despite a live-looking link, something is
            # wrong upstream (stalled motor, stuck wheel, bad encoder) --
            # don't let a silently-stuck segment hold the whole lap hostage.
            if time.monotonic() - last_progress_at > TELEM_STALE_S:
                link.send_stop()
                return

            time.sleep(tick_period)

        heading_rad = target_heading_after

    # Segment list exhausted: bring the car to a controlled stop rather
    # than leaving the last command's speed live on the link.
    link.send_stop()


def _await_fresh_distance(link: StmLink, tick_period: float, timeout_s: float = 1.0) -> Optional[float]:
    """Block briefly for at least one TELEM frame so a segment has a real
    starting odometry reference, rather than measuring distance-traveled
    against None or a stale value.

    Keeps sending STOP frames while waiting (not silence) -- the STM32
    watchdog cuts the motor after 250ms without a valid DRIVE frame, so a
    silent Pi-side wait would itself trip the watchdog before the first
    real command ever goes out.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        link.send_stop()
        telem = link.poll_telem()
        if telem is not None:
            return float(telem.distance_mm)
        time.sleep(tick_period)
    return None  # link never came up in time


# --------------------------------------------------------------------------
# Dry-run harness (no hardware required)
# --------------------------------------------------------------------------

class FakeSerial:
    """Minimal drop-in for pyserial's Serial, used by --dry runs so the
    whole stream_commands() path can be exercised without an STM32
    attached. Simulates a car that reaches commanded speed instantly and
    accumulates odometry accordingly -- good enough to prove the segment-
    advance logic works, not a physics simulator.

    Emits TELEM frames on a free-running ~50Hz clock (checked on every
    read()/write() call, not just in response to a DRIVE write), matching
    the spec's "sent unconditionally" telemetry model -- real hardware
    keeps sending TELEM even if the Pi never transmits anything, and any
    code that (incorrectly) assumes TELEM only follows a DRIVE write would
    pass against a write-triggered fake but hang against the real STM32.
    """

    TELEM_PERIOD_S = 1.0 / 50.0

    def __init__(self):
        self.in_waiting = 0
        self._distance_mm = 0
        self._last_speed_mmps = 0
        self._last_heading_ddeg = 0
        self._last_sim_tick = time.monotonic()
        self._last_telem_emit = time.monotonic()
        self._pending_telem = bytearray()
        self._seq_ack = 0

    def _advance_sim(self) -> None:
        now = time.monotonic()
        dt = now - self._last_sim_tick
        self._last_sim_tick = now
        self._distance_mm += int(self._last_speed_mmps * dt)

        if now - self._last_telem_emit >= self.TELEM_PERIOD_S:
            self._last_telem_emit = now
            telem_body = struct.pack(
                "<BBihhhHHHB",
                self._seq_ack,
                STATUS_ENABLED | STATUS_IMU_OK | STATUS_TOF_OK | STATUS_COLOUR_OK,
                self._distance_mm,
                self._last_speed_mmps,
                self._last_heading_ddeg,
                0,
                0xFFFF, 0xFFFF, 0xFFFF,
                0,
            )
            telem_frame = bytes([TELEM_SYNC0, TELEM_SYNC1]) + telem_body + bytes([_xor8(telem_body)])
            self._pending_telem.extend(telem_frame)
            self.in_waiting = len(self._pending_telem)

    def write(self, data: bytes) -> int:
        if len(data) == DRIVE_LEN and data[0] == DRIVE_SYNC0 and data[1] == DRIVE_SYNC1:
            body = data[2:10]
            seq, flags, steer_ddeg, speed_mmps, heading_ddeg = struct.unpack("<BBhhh", body)
            self._seq_ack = seq
            self._last_speed_mmps = speed_mmps
            mode = (flags & MODE_MASK) >> MODE_SHIFT
            if mode == SteerMode.HEADING_HOLD:
                self._last_heading_ddeg = heading_ddeg
        self._advance_sim()
        return len(data)

    def read(self, n: int) -> bytes:
        self._advance_sim()
        n = min(n, len(self._pending_telem))
        out = bytes(self._pending_telem[:n])
        del self._pending_telem[:n]
        self.in_waiting = len(self._pending_telem)
        return out

    def close(self) -> None:
        pass


def make_dry_run_link() -> StmLink:
    """Build an StmLink backed by FakeSerial instead of real hardware."""
    link = StmLink.__new__(StmLink)  # bypass __init__'s pyserial requirement
    link._ser = FakeSerial()
    link._rx_buf = bytearray()
    link._seq = 0
    link.last_telem = None
    link.rx_bad = 0
    return link


# --------------------------------------------------------------------------
# Smoke test
# --------------------------------------------------------------------------

if __name__ == "__main__":
    from path_planner import RobotConfig, Obstacle, build_visibility_graph, dijkstra, \
        smooth_polyline_to_segments, segments_to_drive_commands

    robot = RobotConfig()

    obstacles = [Obstacle(cx=3.0, cy=1.0, half_size=0.5, pass_side="right", id="obs1")]
    start, goal = (0.0, 1.5), (10.0, 1.5)
    graph = build_visibility_graph(start, goal, obstacles, robot.inflation_m)
    path_indices = dijkstra(graph, 0, len(graph.nodes) - 1)
    waypoints = [graph.nodes[i] for i in path_indices]
    segments = smooth_polyline_to_segments(waypoints, robot.min_turn_radius_m)
    commands = segments_to_drive_commands(segments, robot)

    link = make_dry_run_link()
    print("Dry-run streaming commands over FakeSerial:")
    n = 0
    for telem in stream_commands(commands, link, robot):
        n += 1
        if n % 5 == 0 or n == 1:
            print(f"  telem: distance_mm={telem.distance_mm:5d}  speed_mmps={telem.speed_mmps:4d}  "
                  f"heading_ddeg={telem.heading_ddeg:5d}  status=0x{telem.status:02x}")
    print(f"Done. {n} telemetry frames processed. rx_bad={link.rx_bad}")

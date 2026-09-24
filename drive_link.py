"""
Pi -> STM32 drive commands (checkpoint E, decision #77).

The follower's DriveCmd (road-wheel angle, + = LEFT; speed in mm/s) goes out
as the owner's binary DRIVE frame (stm_link.DriveFrame: 11 bytes, sync AA 55,
xor-8 checksum) in DIRECT mode with the STM32's closed-loop speed control on,
on the port stm32_link.Stm32Link owns (Stm32Link.write). A stop is the STOP
frame. The TELEM frame of stm_link.py is not used: the STM32 -> Pi direction
stays the $IMU line (the tracker needs its timestamp, raw yaw and cumulative
encoder) plus the $STA status line.

Run control (checkpoint F): the STM32 owns the run (start button: start /
stop / restart, drive_protocol.h). Two flag bits the owner's spec leaves
unused carry the Pi's side: PI_READY (bit 4, the Pi holds a valid
initialisation, so a press may start a run) and RUN_OVER (bit 5, the Pi's run
has ended: the STM32 goes to FINISHED). run_state() / run_id() read $STA.

Safety (the Pi half; the STM32 cuts the motor by itself after 250 ms without
a valid frame):
    healthy()  $IMU fresh (IMU_STALE_S) and $STA fresh (250 ms) and the STM32's
               WATCHDOG bit clear
    send()     sends STOP instead of motion while not healthy
    close()    sends STOP
"""
from __future__ import annotations

import struct
import time

import config
from follower import DriveCmd
from stm_link import (DriveFrame, SteerMode, STATUS_BUTTON, STATUS_ENABLED, STATUS_IMU_OK,
                      STATUS_WATCHDOG, TELEM_STALE_S)


FLAG_PI_READY = 1 << 4
FLAG_RUN_OVER = 1 << 5
RUN_READY, RUN_RUNNING, RUN_STOPPED, RUN_FINISHED = 0, 1, 2, 3
RUN_STATE_NAMES = {RUN_READY: "READY", RUN_RUNNING: "RUNNING", RUN_STOPPED: "STOPPED", RUN_FINISHED: "FINISHED"}


# Drive-firmware tuning values (drive_protocol.h ParamId): id -> (config.py name, scale
# from the config value to the firmware's value). config.py is the source of truth.
FW_PARAMS = {
    1: ("SERVO_STRAIGHT_DEG", 1.0), 2: ("SERVO_LEFT_STOP_DEG", 1.0), 3: ("SERVO_RIGHT_STOP_DEG", 1.0),
    4: ("STEER_LOCK_LEFT_DEG", 1.0), 5: ("STEER_LOCK_RIGHT_DEG", 1.0),
    6: ("SPEED_KFF", 1.0), 7: ("SPEED_OFFSET_PWM", 1.0), 8: ("SPEED_KP", 1.0), 9: ("SPEED_KI", 1.0),
    10: ("ENCODER_TICKS_PER_CM", 0.1),
}
PARAM_REPORT_ALL = 0xFF
PARAM_SYNC_S = 0.5                   # how often sync_params() compares and re-sends


def encode_param(pid: int, value: float) -> bytes:
    body = bytes([pid]) + struct.pack("<f", float(value))
    x = 0
    for v in body:
        x ^= v
    return bytes([0xAA, 0x56]) + body + bytes([x])


def fw_wanted() -> dict:
    """{id: value} the firmware should hold, from config.py now."""
    return {pid: float(getattr(config, name)) * k for pid, (name, k) in FW_PARAMS.items()}


def encode(cmd: DriveCmd, seq: int, ready: bool = False, run_over: bool = False,
           closed_loop: bool = True) -> bytes:
    if cmd.stop:
        f = DriveFrame(seq, False, False, SteerMode.STOP, 0, 0, 0).encode()
    else:
        f = DriveFrame(seq, True, closed_loop, SteerMode.DIRECT, round(cmd.steer_deg * 10.0),
                       round(cmd.speed_mm_s), 0).encode()
    extra = (FLAG_PI_READY if ready else 0) | (FLAG_RUN_OVER if run_over else 0)
    if not extra:
        return f
    b = bytearray(f)
    b[3] |= extra
    x = 0
    for v in b[2:10]:
        x ^= v
    b[10] = x
    return bytes(b)


class DriveLink:
    def __init__(self, link):
        self.link = link
        self.seq = 0
        self.sent = 0
        self.refused = 0                 # motion commands replaced by STOP (link not healthy)
        self.ready = False               # PI_READY on every frame sent
        self.run_over = False            # RUN_OVER on every frame sent
        self._last_sync = -1e9
        self.params_sent = 0

    def status_bits(self):
        sta = self.link.status().get("sta")
        return None if sta is None else sta["status"]

    def healthy(self) -> bool:
        st = self.link.status()
        sta = st.get("sta")
        if st["stale"] or sta is None or sta["age_s"] > TELEM_STALE_S:
            return False
        return not (sta["status"] & STATUS_WATCHDOG)

    def button_held(self) -> bool:
        bits = self.status_bits()
        return bits is not None and bool(bits & STATUS_BUTTON)

    def run_state(self):
        """(run_state, run_id) from a fresh $STA, else (None, None)."""
        sta = self.link.status().get("sta")
        if sta is None or sta["age_s"] > TELEM_STALE_S or sta.get("run_state") is None:
            return None, None
        return sta["run_state"], sta["run_id"]

    def send(self, cmd: DriveCmd, closed_loop: bool = True) -> bool:
        if not cmd.stop and not self.healthy():
            self.refused += 1
            cmd = DriveCmd(0.0, 0.0, stop=True)
        ok = self.link.write(encode(cmd, self.seq, self.ready, self.run_over, closed_loop))
        self.seq = (self.seq + 1) & 0xFF
        self.sent += ok
        return ok

    def fw_params(self) -> dict:
        """{id: value} as the STM32 last echoed them ($PAR)."""
        return dict(self.link.status().get("fw_params") or {})

    def param_mismatch(self) -> dict:
        """{config name: (wanted, firmware)} for every value the STM32 does not hold
        (firmware None = never echoed: old firmware, or not yet synced)."""
        have, out = self.fw_params(), {}
        for pid, want in fw_wanted().items():
            got = have.get(pid)
            if got is None or abs(got - want) > 1e-3 * max(1.0, abs(want)):
                out[FW_PARAMS[pid][0]] = (want, got)
        return out

    def sync_params(self, now=None) -> int:
        """Keep the STM32's tuning values equal to config.py: every PARAM_SYNC_S,
        ask for a report if nothing was ever echoed, and re-send every value whose
        echo differs (after an edit on the dashboard, or an STM32 restart, which
        brings back the compiled defaults). Returns the frames sent."""
        now = time.monotonic() if now is None else now
        if now - self._last_sync < PARAM_SYNC_S:
            return 0
        self._last_sync = now
        have = self.fw_params()
        n = 0
        if not have:
            n += self.link.write(encode_param(PARAM_REPORT_ALL, 0.0))
        for pid, want in fw_wanted().items():
            got = have.get(pid)
            if got is None or abs(got - want) > 1e-3 * max(1.0, abs(want)):
                n += self.link.write(encode_param(pid, want))
        self.params_sent += n
        return n

    def close(self):
        for _ in range(3):
            self.link.write(encode(DriveCmd(0.0, 0.0, stop=True), self.seq, False, self.run_over))
            self.seq = (self.seq + 1) & 0xFF
            time.sleep(0.02)


__all__ = ["DriveLink", "encode", "encode_param", "fw_wanted", "FW_PARAMS", "PARAM_REPORT_ALL", "FLAG_PI_READY", "FLAG_RUN_OVER", "RUN_READY", "RUN_RUNNING", "RUN_STOPPED",
           "RUN_FINISHED", "RUN_STATE_NAMES", "STATUS_BUTTON", "STATUS_ENABLED", "STATUS_IMU_OK", "STATUS_WATCHDOG"]

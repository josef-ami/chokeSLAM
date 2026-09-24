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

import time

import config
from follower import DriveCmd
from stm_link import (DriveFrame, SteerMode, STATUS_BUTTON, STATUS_ENABLED, STATUS_IMU_OK,
                      STATUS_WATCHDOG, TELEM_STALE_S)


FLAG_PI_READY = 1 << 4
FLAG_RUN_OVER = 1 << 5
RUN_READY, RUN_RUNNING, RUN_STOPPED, RUN_FINISHED = 0, 1, 2, 3
RUN_STATE_NAMES = {RUN_READY: "READY", RUN_RUNNING: "RUNNING", RUN_STOPPED: "STOPPED", RUN_FINISHED: "FINISHED"}


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

    def close(self):
        for _ in range(3):
            self.link.write(encode(DriveCmd(0.0, 0.0, stop=True), self.seq, False, self.run_over))
            self.seq = (self.seq + 1) & 0xFF
            time.sleep(0.02)


__all__ = ["DriveLink", "encode", "FLAG_PI_READY", "FLAG_RUN_OVER", "RUN_READY", "RUN_RUNNING", "RUN_STOPPED",
           "RUN_FINISHED", "RUN_STATE_NAMES", "STATUS_BUTTON", "STATUS_ENABLED", "STATUS_IMU_OK", "STATUS_WATCHDOG"]

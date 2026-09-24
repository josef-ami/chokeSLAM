"""
Pi -> STM32 drive commands (checkpoint E, decision #77).

The follower's DriveCmd (road-wheel angle, + = LEFT; speed in mm/s) goes out
as the owner's binary DRIVE frame (stm_link.DriveFrame: 11 bytes, sync AA 55,
xor-8 checksum) in DIRECT mode with the STM32's closed-loop speed control on,
on the port stm32_link.Stm32Link owns (Stm32Link.write). A stop is the STOP
frame. The TELEM frame of stm_link.py is not used: the STM32 -> Pi direction
stays the $IMU line (the tracker needs its timestamp, raw yaw and cumulative
encoder) plus the $STA status line.

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


def encode(cmd: DriveCmd, seq: int) -> bytes:
    if cmd.stop:
        return DriveFrame(seq, False, False, SteerMode.STOP, 0, 0, 0).encode()
    return DriveFrame(seq, True, True, SteerMode.DIRECT, round(cmd.steer_deg * 10.0),
                      round(cmd.speed_mm_s), 0).encode()


class DriveLink:
    def __init__(self, link):
        self.link = link
        self.seq = 0
        self.sent = 0
        self.refused = 0                 # motion commands replaced by STOP (link not healthy)

    def status_bits(self):
        sta = self.link.status().get("sta")
        return None if sta is None else sta["status"]

    def healthy(self) -> bool:
        st = self.link.status()
        sta = st.get("sta")
        if st["stale"] or sta is None or sta["age_s"] > TELEM_STALE_S:
            return False
        return not (sta["status"] & STATUS_WATCHDOG)

    def button_pressed(self) -> bool:
        bits = self.status_bits()
        return bits is not None and bool(bits & STATUS_BUTTON)

    def send(self, cmd: DriveCmd) -> bool:
        if not cmd.stop and not self.healthy():
            self.refused += 1
            cmd = DriveCmd(0.0, 0.0, stop=True)
        ok = self.link.write(encode(cmd, self.seq))
        self.seq = (self.seq + 1) & 0xFF
        self.sent += ok
        return ok

    def close(self):
        for _ in range(3):
            self.link.write(encode(DriveCmd(0.0, 0.0, stop=True), self.seq))
            self.seq = (self.seq + 1) & 0xFF
            time.sleep(0.02)


__all__ = ["DriveLink", "encode", "STATUS_BUTTON", "STATUS_ENABLED", "STATUS_IMU_OK", "STATUS_WATCHDOG"]

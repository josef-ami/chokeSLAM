"""
The obstacle round on the robot (checkpoint E).

    python3 run_mission.py [--log imu.log] [--dump scan.json] [--no-camera] [--no-button]

Sequence (rules 9.6-9.14):
    1. Power on: the STM32 link opens (the drive firmware streams $IMU + $STA and
       holds the motor off, servo straight, until valid DRIVE frames arrive).
    2. The LIDAR start scan is taken with the car standing still in its start
       zone, and initialisation runs (lane_init). On failure the reason is
       printed and nothing moves.
    3. Waiting state: STOP frames are sent until the start button (PB12 on the
       STM32, reported in $STA) is pressed -- the one start button (9.11).
       --no-button starts at once (bench only).
    4. The mission (mission.py) runs: tracker fed from the STM32 feed, LIDAR
       frames while the tracker asks, camera frames while it asks, DRIVE frames
       at DRIVE_HZ. A link that is not healthy (stale $IMU / $STA, watchdog)
       makes every frame a STOP.
    5. Ctrl-C, an exception or the end of the mission: STOP frames.
"""
from __future__ import annotations

import argparse
import sys
import time

import config
import lane_init as li
from drive_link import DriveLink
from follower import DriveCmd
from lane_tracker import LaneTracker
from mission import Mission
from run_init import report as init_report
from run_track import _open_link, _take_latest
from scan_processing import clean_and_project, clean_and_project_timed

STOP = DriveCmd(0.0, 0.0, stop=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log")
    ap.add_argument("--dump")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-button", action="store_true")
    a = ap.parse_args()
    from lidar_source import RPLidarC1Source
    link = _open_link(a)
    drive = DriveLink(link)
    lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
    camera = None
    try:
        lidar.start()
        if config.CAMERA_ENABLED and not a.no_camera:
            from camera_source import Picamera2Source
            camera = Picamera2Source()
            camera.start()
        raw, t0 = [], time.monotonic()
        while time.monotonic() - t0 < 5.0 and len(raw) < 300:
            drive.send(STOP)
            time.sleep(0.1)
            raw = lidar.get_latest_scan()
        first = _take_latest(link)
        if a.dump:
            import json
            with open(a.dump, "w") as fh:
                json.dump({"raw": raw, "angle_sign": config.LIDAR_ANGLE_SIGN,
                           "angle_zero_offset_deg": config.LIDAR_ANGLE_ZERO_OFFSET_DEG,
                           "imu_seq_at_scan": first.seq}, fh)
        init = li.initialise(clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
        print(init_report(init))
        if not init.ok:
            return 1
        trk = LaneTracker(init, first)
        mis = Mission(trk, log=print)
        print("waiting for the start button ..." if not a.no_button else "starting (no button)")
        while not a.no_button and not drive.button_pressed():
            for s in link.drain():
                trk.on_imu(s)
            drive.send(STOP)
            time.sleep(0.02)
        t_start = time.monotonic()
        period = 1.0 / config.DRIVE_HZ
        last_cam = None
        while True:
            tick = time.monotonic()
            raw4 = lidar.get_latest_scan_timed() if trk.wants_lidar else None
            samples = link.drain()
            for s in samples:
                trk.on_imu(s)
            if raw4 and trk.wants_lidar:
                pts, times = clean_and_project_timed(raw4, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
                trk.on_lidar_frame(pts, times)
            if camera is not None and trk.wants_camera:
                fr = camera.get_latest_frame()
                if fr is not None and fr[2] != last_cam:
                    last_cam = fr[2]
                    trk.on_camera_frame(fr[0], fr[1])
            v = 0.0
            if len(samples) >= 2:
                dt = (samples[-1].t_ms - samples[0].t_ms) / 1000.0
                if dt > 0:
                    v = (samples[-1].enc - samples[0].enc) * 10.0 / config.ENCODER_TICKS_PER_CM / dt
            cmd = mis.update(time.monotonic() - t_start, v)
            drive.send(cmd)
            if mis.phase in ("DONE", "FAILED"):
                print(f"mission {mis.phase} after {time.monotonic() - t_start:.1f} s")
                for _ in range(10):
                    drive.send(STOP)
                    time.sleep(0.02)
                return 0 if mis.phase == "DONE" else 1
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print("\nstopped by Ctrl-C")
        return 1
    finally:
        drive.close()
        lidar.stop()
        if camera is not None:
            camera.stop()
        link.stop()


if __name__ == "__main__":
    sys.exit(main())

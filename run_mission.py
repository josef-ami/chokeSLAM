"""
The obstacle round on the robot (checkpoint E; run control checkpoint F).

    python3 run_mission.py [--log imu.log] [--dump scan.json] [--no-camera]

Sequence (rules 9.6-9.14):
    1. Power on: the STM32 link opens (the drive firmware streams $IMU + $STA and
       holds the motor off, servo straight, until a run starts).
    2. Waiting: whenever the car has stood still for a second, the LIDAR returns
       taken since it came to rest are used to initialise (lane_init). While a
       valid initialisation is held, every frame carries PI_READY and the STM32's
       status LED is solid: the start button will start a run. On failure the
       reason is printed, the LED blinks every 250 ms, and it is retried.
    3. The start button (PB12 on the STM32, the one start button, 9.11) starts
       the run; the mission (mission.py) runs at DRIVE_HZ. A link that is not
       healthy (stale $IMU / $STA, watchdog) makes every frame a STOP.
    4. A press during the run stops it (the STM32 cuts the motor itself). When
       the mission ends by itself (finished or failed) the STM32 is told
       (RUN_OVER). Either way the program goes back to step 2: place the car in
       a start zone and press again to restart; each run starts from nothing.
    5. Ctrl-C or an exception: STOP frames, and the program exits.
--dump writes each run's start scan to <stem>_run<id><ext>.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import config
from drive_link import DriveLink
from mission import Mission
from run_control import RunSupervisor
from run_init import report as init_report
from run_track import _open_link


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log")
    ap.add_argument("--dump")
    ap.add_argument("--no-camera", action="store_true")
    a = ap.parse_args()
    from lidar_source import RPLidarC1Source
    link = _open_link(a)
    drive = DriveLink(link)
    lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
    camera = None

    def dump(run_id, raw, init):
        print(init_report(init))
        if a.dump:
            stem, ext = os.path.splitext(a.dump)
            with open(f"{stem}_run{run_id}{ext or '.json'}", "w") as fh:
                json.dump({"raw": raw, "angle_sign": config.LIDAR_ANGLE_SIGN,
                           "angle_zero_offset_deg": config.LIDAR_ANGLE_ZERO_OFFSET_DEG}, fh)

    try:
        lidar.start()
        if config.CAMERA_ENABLED and not a.no_camera:
            from camera_source import Picamera2Source
            camera = Picamera2Source()
            camera.start()
        sup = RunSupervisor(link, drive, lidar, camera, make_mission=lambda trk: Mission(trk, log=print),
                            log=print, dump=dump)
        print("waiting: stand the car still in a start zone; LED solid = press the button to start")
        period = 1.0 / config.DRIVE_HZ
        while True:
            tick = time.monotonic()
            sup.step(tick)
            if not link.is_alive():
                print(f"STM32 link lost: {link.status()['error']}")
                return 1
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

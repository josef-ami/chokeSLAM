"""
Speed-loop calibration for the drive firmware (checkpoint F).

    python3 drive_calibrate.py [--speeds 150,250,400,600,800] [--hold 1.5] [--closed 300,500,800,-200]

The firmware's feed-forward (SpeedPI kff, offset in drive_protocol.h) are
placeholders: no speed / PWM pair of this motor had been measured. This tool
measures them. Put the car on a stand (wheels free) for a first fit, then
repeat on the floor with a clear 3 m straight for the values to use.

    1. Start it; the status LED goes solid (this tool sends PI_READY).
    2. Press the start button: the run starts.
    3. OPEN LOOP: for each requested speed the firmware applies its feed-
       forward PWM (reported in $STA); the true speed comes from the encoder
       ($IMU). A straight line  speed = a * (pwm - offset)  is fitted, and the
       kff (= 1 / a) and offset to put in drive_protocol.h are printed.
    4. CLOSED LOOP (with --closed): each speed with the PI on; the settled
       speed and the time to reach 90 % are printed.
    5. The run ends by itself (RUN_OVER); a press stops it at any time.
The steering is held straight throughout.
"""
from __future__ import annotations

import argparse
import sys
import time

import config
from drive_link import DriveLink, RUN_RUNNING
from follower import DriveCmd
from run_track import _open_link

STOP = DriveCmd(0.0, 0.0, stop=True)


def hold(link, drive, speed, secs, closed):
    """Drive at `speed` for `secs`; return [(t, enc, pwm)] and whether the run was stopped."""
    out = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        if drive.run_state()[0] != RUN_RUNNING:
            return out, True
        drive.send(DriveCmd(0.0, speed), closed_loop=closed)
        sta = link.status()["sta"]
        for s in link.drain():
            out.append((s.t_ms / 1000.0, s.enc, sta["pwm"] if sta else None))
        time.sleep(0.02)
    return out, False


def speeds_of(rows, skip_s):
    """(mean speed mm/s after skip_s, mean pwm, [(t, v)])"""
    if len(rows) < 10:
        return None, None, []
    t0 = rows[0][0]
    tv = []
    for a, b in zip(rows[::5], rows[5::5]):
        if b[0] > a[0]:
            tv.append((a[0] - t0, (b[1] - a[1]) * 10.0 / config.ENCODER_TICKS_PER_CM / (b[0] - a[0])))
    late = [r for r in rows if r[0] - t0 >= skip_s]
    if len(late) < 5 or late[-1][0] <= late[0][0]:
        return None, None, tv
    v = (late[-1][1] - late[0][1]) * 10.0 / config.ENCODER_TICKS_PER_CM / (late[-1][0] - late[0][0])
    pw = [r[2] for r in late if r[2] is not None]
    return v, (sum(pw) / len(pw) if pw else None), tv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", default="150,250,400,600,800")
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--closed", default="")
    ap.add_argument("--log")
    a = ap.parse_args()
    link = _open_link(a)
    drive = DriveLink(link)
    try:
        drive.ready = True
        print("press the start button to begin (Ctrl-C to quit)")
        while drive.run_state()[0] != RUN_RUNNING:
            if drive.run_state()[0] is None:
                print("no run state in $STA: is the checkpoint-F drive_bridge firmware flashed?")
            link.drain()
            drive.send(STOP)
            time.sleep(0.05)
        drive.ready = False
        pts = []
        print("\nOPEN LOOP (feed-forward only)\n  asked mm/s   pwm   measured mm/s")
        for v in [float(x) for x in a.speeds.split(",") if x]:
            rows, stopped = hold(link, drive, v, a.hold, closed=False)
            if stopped:
                print("stopped by the button")
                return 1
            meas, pwm, _ = speeds_of(rows, a.hold * 0.5)
            print(f"  {v:10.0f} {pwm if pwm is None else round(pwm):5}   {meas if meas is None else round(meas)}")
            if meas is not None and pwm is not None and meas > 20.0:
                pts.append((pwm, meas))
        if len(pts) >= 2:
            n = len(pts)
            mx = sum(p for p, _ in pts) / n
            my = sum(v for _, v in pts) / n
            sxx = sum((p - mx) ** 2 for p, _ in pts)
            slope = sum((p - mx) * (v - my) for p, v in pts) / sxx if sxx > 0 else 0.0
            if slope > 0:
                off = mx - my / slope
                print(f"\n  fit: speed = {slope:.3f} * (pwm - {off:.1f})")
                print(f"  -> drive_protocol.h SpeedPI:  kff = {1.0 / slope:.3f}f;  offset = {off:.1f}f;")
                print(f"     top speed at pwm 255 about {slope * (255 - off):.0f} mm/s "
                      f"(config SPEED_LAPS23_MM_S = {config.SPEED_LAPS23_MM_S:.0f})")
        else:
            print("\n  not enough moving points to fit (is the motor powered?)")
        closed = [float(x) for x in a.closed.split(",") if x]
        if closed:
            print("\nCLOSED LOOP\n  target mm/s   settled mm/s   t90 s")
        for v in closed:
            rows, stopped = hold(link, drive, v, a.hold, closed=True)
            if stopped:
                print("stopped by the button")
                return 1
            meas, _, tv = speeds_of(rows, a.hold * 0.5)
            t90 = next((t for t, s in tv if abs(s) >= 0.9 * abs(v)), None)
            print(f"  {v:11.0f}   {meas if meas is None else round(meas):12}   {t90 if t90 is None else round(t90, 2)}")
            rows, _ = hold(link, drive, 0.0, 0.8, closed=True)
        return 0
    except KeyboardInterrupt:
        return 1
    finally:
        drive.run_over = True
        for _ in range(10):
            drive.send(STOP)
            time.sleep(0.02)
        drive.close()
        link.stop()


if __name__ == "__main__":
    sys.exit(main())

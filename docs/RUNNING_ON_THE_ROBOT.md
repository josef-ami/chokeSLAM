# Running the obstacle round on the robot

Step-by-step guide to running chokeSLAM's obstacle-round stack on the real car: the Raspberry Pi 5 runs localization, planning and the path follower (`run_mission.py`), and the STM32 Black Pill runs the drive firmware (`firmware/drive_bridge/`, pinout from the owner's `OpenRound.cpp`). The design and the reasons for every value are in [CHANGES.md](CHANGES.md) §16 (checkpoint E) and §17 (checkpoint F: firmware and the start button).

> **Status.** Everything here has been tested in simulation and with stand-in hardware only. The drive firmware has been compiled on a PC against stand-in Arduino headers, but never with the STM32 toolchain or on the STM32. The speed-loop values are **placeholders until measured** (step 5). Do the steps in order the first time, with the wheels off the ground until step 6.
>
> **Known issue (CHANGES §17.5).** With the measured turning radii (27 cm left, 25 cm right), the planner finds far fewer paths than it did with the old placeholder lock: in simulation, 49 % of the starts that initialise succeed, down from 100 %. A decision on the fix is pending.

## Contents

1. [What runs where](#1-what-runs-where)
2. [Flash the drive firmware](#2-flash-the-drive-firmware)
3. [Set up the Pi](#3-set-up-the-pi)
4. [Check the ports and the link](#4-check-the-ports-and-the-link)
5. [Calibrate and measure](#5-calibrate-and-measure-once-and-after-any-mechanical-change)
6. [First run on the field](#6-first-run-on-the-field)
7. [Competition setup: start on power-up](#7-competition-setup-start-on-power-up)
8. [Starting a round](#8-starting-a-round)
9. [Troubleshooting](#9-troubleshooting)
10. [Rehearse in simulation](#10-rehearse-in-simulation)

---

## 1. What runs where

| Part | Runs | Does |
|---|---|---|
| STM32 Black Pill | `firmware/drive_bridge/drive_bridge.ino` | Streams `$IMU` (BNO08x yaw + encoder, 100 Hz) and `$STA` (status, run state, 20 Hz) to the Pi over USB; **owns the run** (start button PB12: start / stop / restart); executes DRIVE frames (steering angle + speed loop) only while a run is going; stops the motor 250 ms after the last valid frame |
| Raspberry Pi 5 | `run_mission.py` | Between runs: initialises from the LIDAR whenever the car stands still and tells the STM32 it is ready. During a run: tracker, lap 1 lane by lane (look, plan, drive), laps 2–3 from the final map, stop in the start section. Runs until switched off: every run starts from nothing |
| RPLIDAR C1 | USB serial (`/dev/ttyUSB0`) | start scan and seat checks |
| OV5647 fisheye | Pi camera port (Picamera2) | pillar colours |

Only **one** program may own the STM32's USB port. Don't run the dashboard, `run_track.py` or `run_mission.py` at the same time.

## 2. Flash the drive firmware

1. **Before building**, open `firmware/drive_bridge/drive_protocol.h` and check the values. Leave them as they are for the first build; step 5 measures them.
   - `SteerMap`: servo straight 76.5°, left stop 20°, right stop 140° (OpenRound.cpp); lock 36.6° left, 40.5° right, from your measured radii (CHANGES §17.2). These must match `STEER_LOCK_*` / `SERVO_*` in `config.py`; `test_drive_firmware.py` checks that they do.
   - `SpeedPI`: `kff`, `offset`, `kp`, `ki`: **placeholders** (step 5.3 measures them)
2. Arduino IDE with the STM32duino core:
   - **Board:** Generic STM32F4 series → Board part number: **BlackPill F411CE**
   - **USB support:** **CDC (generic 'Serial' supersede U(S)ART)**
   - **Upload method:** STM32CubeProgrammer (DFU). Hold BOOT0, tap NRST.
   - **Libraries:** *SparkFun BNO08x Cortex Based IMU* and *Servo* (bundled with the core)
3. Open `firmware/drive_bridge/drive_bridge.ino`. `drive_protocol.h` must stay in the same folder. Compile and upload. **This is the first time this sketch is compiled with the STM32 toolchain**: if the build fails, send the error messages back.
4. **Wheels off the ground.** Power the car. Check the on-board LED (PC13):

   | LED | Meaning |
   |---|---|
   | fast blink (100 ms) | IMU fault (not found, or no report for 100 ms): no run can start |
   | blink 1 s on / 1 s off | no DRIVE frames: the Pi program is not running (normal right after power-on) |
   | blink 250 ms | the Pi is running but not ready: initialising, the car is moving, or initialisation failed (place the car again) |
   | solid | ready: a press starts a run. Also solid during a run |

   The motor must not turn and the servo must sit straight: the motor runs only during a run.

   **The start button (PB12)**, as you specified:

   | When | A press |
   |---|---|
   | ready (LED solid), before a run | starts a run |
   | during a run | **stops** the run: motor off at once, whatever the Pi sends |
   | after a run stopped or finished | starts a new run (**restart**). Put the car back in a start zone first and wait for the LED to go solid |
   | the Pi is not ready (LED blinking) | nothing; the firmware prints `# press ignored: Pi not ready` |

   The debounce is OpenRound's: 30 ms, presses closer than 400 ms are ignored, and a button held down at power-up must be released before a press counts. The floor colour sensor (TCS34725 / TCA9548A) and LED2 / LED3 are not used.

The old sketches (`obstacle_round_stream` / v7, `stm32_imu_bridge`) are kept for reference; `run_mission.py` needs `drive_bridge`.

## 3. Set up the Pi

```bash
git clone https://github.com/josef-ami/chokeSLAM.git
cd chokeSLAM
sudo apt install python3-opencv python3-picamera2    # OpenCV and the Pi camera (Picamera2 is not usable from PyPI)
pip install -r requirements.txt                       # numpy, flask, rplidarc1, pyserial (opencv already from apt)
sudo usermod -aG dialout $USER                        # serial-port access; log out and back in
```

Run the offline tests once to check the install (a few minutes; no hardware needed):

```bash
python3 test_vg_planner.py
python3 test_drive_firmware.py
python3 test_run_control.py
python3 test_lane_tracker.py
```

## 4. Check the ports and the link

1. Find the devices:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/
   ```
   The STM32 is a `ttyACM` device and the LIDAR a `ttyUSB` device. In `config.py`, set:
   - `IMU_PORT` (default `/dev/ttyACM0`)
   - `LIDAR_PORT` (default `/dev/ttyUSB0`)

   The `/dev/serial/by-id/...` paths are safer: `ACM` / `USB` numbers can change when devices are plugged in.
2. STM32 stream only (the motor stays off):
   ```bash
   python3 run_track.py --bench
   ```
   You should see about 100 Hz, `gaps 0`, `bad 0`.
   - **Turn the car clockwise by hand (seen from above): `heading` must increase.** If it decreases, flip `IMU_YAW_SIGN`.
   - Roll the car forward: `distance` must increase.

## 5. Calibrate and measure (once, and after any mechanical change)

The simulation showed which of these matter most: an encoder error of 2 % instead of 0.5 % takes the success rate from 97 % to 87 % (CHANGES §16.11).

### 5.1 Encoder scale (to 0.5 %)
In `python3 run_track.py --bench`, roll the car straight over a measured 2000 mm on the mat. `distance` must read 2000 ± 10 mm. If it doesn't:

```
ENCODER_TICKS_PER_CM = ENCODER_TICKS_PER_CM × (distance shown / 2000)
```

Repeat until it does. If you change it, also change `TICKS_PER_MM` in `drive_bridge.ino` (= ticks per cm / 10) and re-flash.

### 5.2 Steering: servo map and lock
The planner's turning radius comes from `STEER_LOCK_LEFT_DEG` / `STEER_LOCK_RIGHT_DEG`, and the firmware's servo map from `SteerMap`. They come from your full-lock radii (left 27 cm, right 25 cm, outer wheel to the turn centre), read as the **outer front** wheel (CHANGES §17.2). To command a fixed steering angle, save this as `steer_test.py` in the repo folder. It uses the same link code as `run_mission.py`; the motor only runs during a run, so **press the start button** when it asks. Press again to stop early.

```python
import sys, time
from stm32_link import Stm32Link
from drive_link import DriveLink, RUN_RUNNING
from follower import DriveCmd

steer, speed, seconds = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
link = Stm32Link()
link.start()
drv = DriveLink(link)
drv.ready = True                                 # PI_READY: the button may start a run
print("press the start button")
try:
    while drv.run_state()[0] != RUN_RUNNING:
        drv.send(DriveCmd(0.0, 0.0, stop=True))
        link.drain()
        time.sleep(0.05)
    drv.ready = False
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds and drv.run_state()[0] == RUN_RUNNING:
        drv.send(DriveCmd(steer, speed))         # steer: road-wheel degrees, + = LEFT; speed mm/s
        link.drain()
        time.sleep(0.02)
finally:
    drv.run_over = True                          # ends the run (FINISHED)
    drv.close()                                  # STOP frames
    link.stop()
print("refused (link not healthy):", drv.refused)
```

1. **Servo map (wheels off the ground, speed 0):**
   - `python3 steer_test.py 0 0 5`: the wheels must be straight. If they aren't, adjust `SERVO_STRAIGHT_DEG` and `SteerMap.straight` together.
   - `python3 steer_test.py 36.6 0 5` puts the servo at the left stop; `python3 steer_test.py -40.5 0 5` at the right stop.
2. **Lock radius (on the mat, clear space)**, to confirm the values:
   - `python3 steer_test.py 36.6 200 12` drives a full-lock left circle at 200 mm/s.
   - Measure the radius R_w of the circle traced by the **outer front wheel** (the right front wheel in a left turn). Then

     ```
     R = sqrt(R_w² − 135.9²) − 50.5          (rear-axle midpoint radius; 135.9 = wheelbase, 50.5 = half track)
     STEER_LOCK_LEFT_DEG = atan(135.9 / R)
     ```
   - Repeat to the right with `-40.5` for `STEER_LOCK_RIGHT_DEG`.

   Put the same values in `config.py` and `SteerMap.lockLeftDeg` / `lockRightDeg`, re-flash, and run `python3 test_drive_firmware.py` (it fails if they differ).

### 5.3 Speed loop
The firmware turns the speed the Pi asks for (mm/s) into motor PWM: feed-forward `kff × speed + offset`, plus a PI loop on the encoder speed. No speed / PWM pair of this motor has been measured, so `kff` and `offset` are placeholders. `drive_calibrate.py` measures them:

```bash
python3 drive_calibrate.py                                     # car on a stand, wheels free: first fit
python3 drive_calibrate.py --speeds 150,300,500 --closed 300,500,-200   # on the floor, clear 3 m straight
```

Press the start button when asked. It runs open loop at each speed, reads the PWM applied (from `$STA`) and the true speed (from the encoder), fits `speed = a × (pwm − offset)` and prints the `kff` and `offset` to put in `drive_protocol.h`. With `--closed` it then runs the PI loop at each speed and prints the settled speed and the time to reach 90 %. Re-flash after changing them. It also prints the top speed at PWM 255: if it is below `SPEED_LAPS23_MM_S` (800), lower that in `config.py`.

`$STA` also carries the PWM and measured speed during every run, so the current values can be read at any time with `link.status()['sta']`.

### 5.4 LIDAR
1. **Angle sign / zero** (measured −1 / 0 for the upside-down mount, CHANGES §11). Put one object about 30 cm to the car's **right**:
   ```bash
   python3 run_init.py --real --dump sign.json
   ```
   It must read about 90°. Re-check only if the LIDAR is remounted.
2. **Time offset:**
   ```bash
   python3 measure_lidar_delay.py --real --record delay.json
   ```
   Keep the car still, then turn it by hand ±30° about once a second when told. Put the printed `LIDAR_TIME_OFFSET_S` in `config.py`. Repeat if it says NOT RELIABLE.
3. **Lever arm:** `LIDAR_OFFSET_FORWARD_MM = 134.6`, `LIDAR_OFFSET_LATERAL_MM = −0.7`, from the CAD model; the pose point is the rear-axle midpoint. Check with a ruler that the LIDAR's spin axis really is 134.6 mm ahead of the rear axle.

### 5.5 Camera
The lens position comes from the camera mount in the CAD model: 139.9 mm ahead, centred, 127 mm high. Check it with a ruler. Then put one pillar 30° to the right, 500 mm from the lens:

```bash
python3 camera_check.py --real --bearing 30 --range 500 --out check.png
```

The box in `check.png` must sit on the pillar:

| Box position | Fix |
|---|---|
| mirrored to the other side | flip `CAMERA_BEARING_SIGN` |
| image upside-down | flip `CAMERA_ROTATE_180` |
| too high or too low | fix `CAMERA_HEIGHT_MM` |

Repeat with a red and a green pillar at 300, 700 and 1200 mm under the venue's lighting. It prints the colour fractions. The thresholds (`COLOR_*`) are placeholders until real frames are checked.

### 5.6 Start position check
Put the car in a start zone (the middle zone above the parking lot), switched on, standing still:

```bash
python3 run_init.py --real --dump start.json
```

The direction must be right; x is the distance from the outer wall to the rear axle, y the distance along the lane. The seat verdicts should match what you see. Keep `start.json`: `python3 run_init.py --replay start.json` re-runs it offline.

## 6. First run on the field

1. Field set up by the rulebook: pillars, parking lot, start in the middle zone above the lot, facing the round direction.
2. From an SSH session, **ready to stop**:
   ```bash
   python3 run_mission.py --log imu.log --dump scan.json
   ```
   Once the car has stood still for 1 s it initialises and prints `[run] ready: ...`; the LED goes solid.
3. Press the button. It prints the initialisation report and `[run] run 1 START`. The car stops at each corner for about 1–3 s on lap 1 (looking at the next lane), then drives laps 2–3 without stopping and stops in the start section. Every plan, stop, re-plan and reverse is printed as `[mission t] kind: detail`.
4. **To stop:** press the button (the STM32 cuts the motor at once). If the Pi dies or the cable comes out, the STM32 stops the motor by itself after 250 ms. Ctrl-C ends the program.
5. **To run again:** carry the car back to a start zone and let it stand. It re-initialises (`[run] ready`, LED solid); press to start run 2. Each run starts from nothing (a new initialisation and map). After a finish it becomes ready where it stopped; put it back in a start zone before pressing.
6. Keep `imu.log` and `scan_run<N>.json` (each run's start scan): `python3 run_track.py --replay-scan scan_run1.json --replay-imu imu.log` replays the tracking offline, and the files can be sent for analysis.

`--no-camera` runs without colour ID: every pillar is then passed "either side", so use it for driving tests only. (`--no-button` is gone: runs start only from the button now.)

## 7. Competition setup: start on power-up

The rules allow one power switch, then a waiting state, then one start button (9.10–9.11), and no wireless during rounds (11.10). So the program must start by itself when the Pi boots, and Wi-Fi / Bluetooth must be off. One way is a systemd service:

```ini
# /etc/systemd/system/chokeslam.service
[Unit]
Description=chokeSLAM obstacle round
After=multi-user.target

[Service]
User=<your user>
WorkingDirectory=/home/<your user>/chokeSLAM
ExecStart=/usr/bin/python3 run_mission.py --log /home/<your user>/runs/imu.log --dump /home/<your user>/runs/scan.json
Restart=on-failure
RestartSec=1

[Install]
WantedBy=multi-user.target
```

```bash
mkdir -p ~/runs
sudo systemctl daemon-reload
sudo systemctl enable chokeslam.service   # starts on every boot
journalctl -u chokeslam -f                # its output, during practice
sudo systemctl disable chokeslam.service  # back to manual runs
```

Turn off wireless for rounds, for example with `sudo rfkill block wifi bluetooth` (undo with `unblock`). Check the method with your organisers: the judges may inspect it (11.10).

`run_mission.py` keeps running between runs, so one program serves a whole practice session. If it crashes, systemd restarts it; a run the STM32 still has as "running" is then ended by the new program (the car does not move). Each program start overwrites `imu.log`, and each run's start scan is `scan_run<N>.json` (N counts from 1 after every STM32 power-on); copy them off after a round you want to keep.

## 8. Starting a round

1. Place the car in the start zone **switched off**, fully inside the zone, front wheels toward the next corner in the round direction (9.6–9.8).
2. Switch on. The Pi boots and `run_mission.py` starts. It initialises **while the car stands still** and the LED goes solid (ready). Don't touch the car after switching on. If the LED keeps blinking every 250 ms, initialisation is failing: re-place the car.
3. On the judge's "Go", press the start button once. **Don't press it again**: a second press stops the run.
4. The car drives 3 laps and stops inside the start section. Parking is not implemented yet.

## 9. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `no STM32 samples on /dev/ttyACM0 within 3 s` | wrong `IMU_PORT`, drive firmware not flashed, or the IMU is not found (LED fast blink) |
| Init report: `direction: UNDETERMINED` | the direction test could not see the opening past the island: a pillar hides it. It happens on about 17 % of rulebook starts (mostly CW starts with a pillar on the middle inner seat); nothing moves. Re-place the car within the zone (straighter, a little sideways) and let it stand: it retries every 0.5 s |
| Init report: `x rejected` / lane width | a side reading isn't a wall; check `LANE_WIDTH_MM` (1000) against the field, and the LIDAR mount |
| Nothing happens on a press | the LED says why: blinking 250 ms = the Pi is not ready (still initialising, the car moved, or initialisation failed: see the `[run]` lines); 1 s blink = the Pi program is not running; 100 ms = IMU fault |
| A run starts but nothing moves | the link is not healthy, so motion is refused and STOP frames are sent instead. Check that `$STA` arrives: `python3 -c "import time; from stm32_link import Stm32Link; l = Stm32Link(); l.start(); time.sleep(1); print(l.status()['sta']); l.stop()"` must print a status with a `run_state`, not `None` (no `run_state` = old firmware) |
| `[run] run N started but no initialisation is held` | the STM32 started a run the Pi was not ready for (it had just lost readiness); the run is ended at once. Let the car stand and press again |
| `mission FAILED` with `no path ...` | no drivable path was found (about 3 % of starts in simulation, all at the start: the car stands about 300 mm behind a pillar it must pass on the far side). Re-place the car further back in the zone |
| `mission FAILED` with `at the viewing pose but the tracker is in lane N` | the tracker counted a turn it shouldn't have. With the measured lock this is mostly the planner looping round (a 270° turn) where the corner is too tight: the known issue in CHANGES §17.5 |
| Car grazes pillars | tracking error larger than the 30 mm clearance: re-check the encoder scale (5.1), the lock radius (5.2) and the speed loop (5.3); `PLAN_CLEARANCE_MM` 40 with `PLAN_INFLATION_MM` 105 gives more margin (95.5 % vs 87 % under noise in simulation) |
| Wrong pillar colours / colours given up | camera check (5.5); colour thresholds |
| `test_run_track.py` / `test_dashboard.py` report 4 wrong seats | expected with `LIDAR_TIME_OFFSET_S` ≠ 0: their stand-in LIDAR doesn't apply the offset (CHANGES §16.13). They pass with 0 |

## 10. Rehearse in simulation

The same mission can be run on a simulated car with rulebook layouts, no hardware needed:

```bash
python3 sim_closed_loop.py --seed 3 --noise moderate            # one round, printed as it happens
python3 sweep_closed_loop.py --layouts 50 --noise moderate --out runs.csv   # many rounds, summary at the end
```

After changing a value in `config.py` (for example the measured lock), a sweep shows whether the change helps before you try it on the field.

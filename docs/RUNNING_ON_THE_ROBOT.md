# Calibrating and running chokeSLAM on the robot

This is the complete guide to getting the obstacle-round stack running on the real car: wiring, flashing, Pi setup, **every calibration in the order to do it**, practice with the live dashboard, the competition setup, and troubleshooting.

- The Raspberry Pi 5 runs localization, planning and the path follower (`run_mission.py`).
- The STM32 Black Pill runs the drive firmware (`firmware/drive_bridge/`, pinout from your `OpenRound.cpp`).
- The design and the evidence for every value are in [CHANGES.md](CHANGES.md): §16 is checkpoint E (planner and mission), §17 checkpoint F (firmware and start button), §18 checkpoint F2 (live tuning and the planned path).

> **Status.**
> - Everything has been tested in simulation and with stand-in hardware only.
> - The drive firmware compiles on a PC against stand-in Arduino headers, but has never been built with the STM32 toolchain or run on the STM32.
> - Do the steps in order the first time, with the wheels off the ground until §6.5.
>
> **Placeholders (to measure, §6).**
> - **Steering lock: 36.6° left, 40.5° right.** These are placeholders, kept on purpose. They were converted from your 27 / 25 cm full-lock radii as if measured at the outer front wheel. The radii came from encoder and IMU data, which measure a rear-axle radius, so the real lock is probably smaller: 31.8° / 34.3° (outer rear wheel) or 26.7° / 28.5° (rear-axle midpoint). See CHANGES §17.2.
> - **The speed loop's four values.**
> - **The colour thresholds.**
>
> None of these needs a re-flash: they live in `config.py` and on the dashboard.
>
> **Known issue (CHANGES §17.5).** With the lock placeholders the planner finds far fewer paths than it did with the old guess. In simulation without noise, 49 % of the starts that initialise succeed, where the old guess gave 100 %. A smaller measured lock makes this worse. Re-run the simulation sweep (§12) with your measured values before a competition; §6.14 says what to try.

## Contents

1. [What runs where](#1-what-runs-where)
2. [Wiring check](#2-wiring-check)
3. [Flash the drive firmware](#3-flash-the-drive-firmware)
4. [Set up the Pi](#4-set-up-the-pi)
5. [First link checks](#5-first-link-checks)
6. [Calibration, in order](#6-calibration-in-order)
7. [The dashboard](#7-the-dashboard)
8. [A practice session](#8-a-practice-session)
9. [Competition setup: start on power-up](#9-competition-setup-start-on-power-up)
10. [Starting a round](#10-starting-a-round)
11. [Troubleshooting](#11-troubleshooting)
12. [Rehearse in simulation](#12-rehearse-in-simulation)
13. [Reference](#13-reference)

---

## 1. What runs where

| Part | Runs | Does |
|---|---|---|
| STM32 Black Pill | `firmware/drive_bridge/drive_bridge.ino` | Streams `$IMU` (BNO08x yaw and encoder, 100 Hz) and `$STA` (status, run state, 20 Hz) over USB. **Owns the run:** the start button (PB12) starts, stops and restarts. Executes DRIVE frames (steering angle, speed loop) only during a run. Stops the motor 250 ms after the last valid frame. Takes its tuning values from the Pi (PARAM frames) |
| Raspberry Pi 5 | `run_mission.py` | Between runs: initialises from the LIDAR whenever the car stands still, and tells the STM32 it is ready. During a run: tracker, lap 1 lane by lane (look, plan, drive), laps 2–3 from the final map, stop in the start section. Every run starts from nothing. With `--dashboard` it also serves the practice page (§7) |
| RPLIDAR C1 | USB serial (`/dev/ttyUSB0`) | start scan and seat checks |
| OV5647 fisheye | Pi camera port (Picamera2) | pillar colours |

Only **one** program may own the STM32's USB port. Never run the dashboard, `run_track.py`, `drive_calibrate.py` and `run_mission.py` at the same time.

## 2. Wiring check

The pinout is `OpenRound.cpp`'s. Check it before the first power-up:

| Function | STM32 pin | Notes |
|---|---|---|
| Motor forward PWM | PA2 (TIM2_CH3) | BTS7960 RPWM |
| Motor reverse PWM | PA3 | BTS7960 LPWM |
| Encoder A / B | PA0 / PA1 (TIM5) | internal pull-ups; forward must count up (the firmware negates) |
| Steering servo | PA8 | 500–2500 µs |
| IMU (BNO08x, SPI1) | MOSI PA7, MISO PA6, SCK PA5, CS PA4, INT PB0, RST PB1 | 3 MHz |
| Start button | PB12 to GND | internal pull-up |
| Status LED | PC13 (on-board) | active LOW |
| Not used | PB6 / PB7 / PB8 (I2C, TCA9548A reset), PB13 / PB14 (LED2 / LED3) | the floor colour sensor is not used; left untouched |

The STM32 connects to the Pi by its USB-C port (native USB, CDC): the Pi sees it as `/dev/ttyACM*`.

## 3. Flash the drive firmware

1. **Nothing to edit.** The firmware's tuning values are set by the Pi from `config.py` over USB (PARAM frames), every time it connects and whenever they change: servo straight and stops, steering lock, speed loop, encoder scale. The defaults compiled into `drive_protocol.h` (`SteerMap`, `SpeedPI`) are used only until the Pi has sent its values, and `test_drive_firmware.py` checks they equal `config.py`.
2. **Arduino IDE** with the STM32duino core (Boards Manager: *STM32 MCU based boards*):
   - **Board:** Generic STM32F4 series; Board part number: **BlackPill F411CE**
   - **USB support:** **CDC (generic 'Serial' supersede U(S)ART)**
   - **Upload method:** STM32CubeProgrammer (DFU). STM32CubeProgrammer must be installed. To enter DFU, hold BOOT0, tap NRST, release BOOT0.
   - **Libraries:** *SparkFun BNO08x Cortex Based IMU* (Library Manager); *Servo* comes with the core.
3. Open `firmware/drive_bridge/drive_bridge.ino`. `drive_protocol.h` must stay in the same folder. Compile and upload. **This is the first build with the STM32 toolchain**: if it fails, keep the error messages.
4. **Wheels off the ground.** Power the car and watch the on-board LED (PC13):

   | LED | Meaning |
   |---|---|
   | fast blink (100 ms) | IMU fault (not found, or no report for 100 ms): no run can start |
   | 1 s on / 1 s off | no DRIVE frames: the Pi program is not running (normal right after power-on) |
   | 250 ms blink | the Pi is running but not ready: initialising, the car is moving, or initialisation failed (place the car again) |
   | solid | ready, and a press starts a run. Also solid during a run |

   The motor must not turn and the servo must sit straight: the motor runs only during a run.

**The start button (PB12):**

| When | A press |
|---|---|
| ready (LED solid), no run going | **starts** a run |
| during a run | **stops** the run: motor off at once, whatever the Pi sends |
| after a run stopped or finished | starts a new run (**restart**): carry the car back to a start zone first and wait for the solid LED |
| the Pi is not ready (LED blinking) | nothing (the firmware logs `# press ignored: Pi not ready`) |

The debounce is OpenRound's:
- a press must hold for 30 ms;
- a second press within 400 ms is ignored;
- a button held down at power-up must be released before a press counts.

## 4. Set up the Pi

```bash
git clone https://github.com/josef-ami/chokeSLAM.git
cd chokeSLAM
sudo apt install python3-opencv python3-picamera2    # OpenCV and the Pi camera (Picamera2 is not usable from PyPI)
pip install -r requirements.txt                       # numpy, flask, rplidarc1, pyserial (opencv already from apt)
sudo usermod -aG dialout $USER                        # serial-port access; log out and back in
```

Run the offline tests once to check the install. They take a few minutes and need no hardware:

```bash
python3 test_drive_firmware.py      # firmware logic (needs g++), link, parameter sync
python3 test_run_control.py         # start / stop / restart end to end (needs g++)
python3 test_lane_tracker.py
python3 test_dashboard.py
```

The known exceptions:
- `test_vg_planner.py`'s full-lap check fails with the lock placeholders (the known issue).
- `test_run_track.py` and `test_dashboard.py`'s real-mode part report a few wrong seats while `LIDAR_TIME_OFFSET_S` ≠ 0, because their stand-in LIDAR does not apply the offset (CHANGES §16.13).

## 5. First link checks

1. **Find the devices:**
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/
   ```
   The STM32 is a `ttyACM` device and the LIDAR a `ttyUSB` device. Set `IMU_PORT` and `LIDAR_PORT` in `config.py`. The `/dev/serial/by-id/...` paths are better, because `ACM` / `USB` numbers can change when devices are plugged in.
2. **STM32 stream** (the motor stays off):
   ```bash
   python3 run_track.py --bench
   ```
   You should see about 100 Hz, `gaps 0`, `bad 0`.
3. **Status and parameter echo:**
   ```bash
   python3 -c "
   import time; from stm32_link import Stm32Link; from drive_link import DriveLink
   l = Stm32Link(); l.start(); d = DriveLink(l); time.sleep(1)
   for _ in range(4): d.sync_params(); time.sleep(0.6)
   print('STA', l.status()['sta']); print('echo', l.status()['fw_params']); print('mismatch', d.param_mismatch()); l.stop()"
   ```
   It must print a `STA` with a `run_state` (0 = READY), ten echoed values, and `mismatch {}`. No `run_state`, or no echo, means the firmware is not the checkpoint-F2 `drive_bridge`.
4. **Button:** on the dashboard (§7, STM32 card) or by re-running the one-liner while holding the button: the BUTTON bit (4) of `status` must be set while it is held.

## 6. Calibration, in order

Do these once, then again after any mechanical change. The order matters: each step relies on the ones before it.
- **Where values go:** every value lives in `config.py`.
- **Trying values live:** most can be tried live on the dashboard's tuning panel (§7). Edits there stay in memory, so copy the final value into `config.py` and commit it.
- **Firmware values:** the Pi sends them to the STM32 within 0.5 s. Never re-flash to change them.
- **Record:** keep a calibration record (§6.15).

### 6.1 IMU yaw sign (`IMU_YAW_SIGN`)
- **How:** `python3 run_track.py --bench`. Turn the car **clockwise by hand** (seen from above). `heading` must **increase**.
- **Fix:** if it decreases, flip `IMU_YAW_SIGN` (currently −1: the chip reads clockwise as negative).
- **Why:** a wrong sign makes every turn go the wrong way.

### 6.2 Encoder scale (`ENCODER_TICKS_PER_CM`, to 0.5 %)
- **How:** still in `--bench`, roll the car straight over a measured 2000 mm on the mat. `distance` must read 2000 ± 10 mm. Otherwise:
  ```
  ENCODER_TICKS_PER_CM = ENCODER_TICKS_PER_CM × (distance shown / 2000)
  ```
  Repeat until it reads within ±10 mm.
- **Also:** the value is sent to the STM32 too, for its speed estimate.
- **Why:** in simulation, a 2 % scale error instead of 0.5 % takes success from 97 % to 87 % (CHANGES §16.11). This is the single most valuable calibration.

### 6.3 Servo straight (`SERVO_STRAIGHT_DEG`)
Save this as `steer_test.py` in the repo folder. It commands a fixed steering angle and speed for a few seconds. The motor only runs during a run, so **press the start button** when it asks; press again to stop early.

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
        drv.sync_params()                        # the STM32 gets config.py's values
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

- **Coarse, on the bench:** `python3 steer_test.py 0 0 5`, wheels off the ground. The wheels must point straight.
- **Fine, on the mat:** `python3 steer_test.py 0 300 4` drives about 1.2 m. The car must not curve by more than about 20 mm to either side.
- **Fix:** adjust `SERVO_STRAIGHT_DEG` (OpenRound: 76.5). Below straight steers left, so if the car curves left, raise it. It is also on the dashboard (§7), but the dashboard and `steer_test.py` cannot own the port at the same time: edit `config.py` between runs, or trim on the dashboard during a practice run (§8).

### 6.4 Servo stops (`SERVO_LEFT_STOP_DEG`, `SERVO_RIGHT_STOP_DEG`)
- **Check:** `python3 steer_test.py 36.6 0 5` and `python3 steer_test.py -40.5 0 5` put the servo at each stop (20 / 140). The servo must not buzz or strain against the steering's mechanical limit.
- **Fix:** if it does, move that stop 2–3° toward straight.
- **Then:** re-measure the lock for that side (§6.5), because the lock is the wheel angle **at the stop**.

### 6.5 Steering lock (`STEER_LOCK_LEFT_DEG`, `STEER_LOCK_RIGHT_DEG`): currently PLACEHOLDERS
This is the second most important value: the planner's tightest arcs and the servo map's scale both come from it.
1. On the mat, with clear space, drive a full-lock circle: `python3 steer_test.py 36.6 200 12` (left). The commanded angle only needs to reach the stop.
2. **Measure R, the radius of the rear-axle midpoint's circle.** Mark the floor under the rear-axle midpoint at two opposite points of the circle (or under each rear wheel and take the middle). Half the distance between the marks is R.
3. Compute
   ```
   STEER_LOCK_LEFT_DEG = atan(135.9 / R)              (135.9 mm = wheelbase)
   ```
   | R (mm) | 150 | 160 | 180 | 200 | 220 | 250 | 270 |
   |---|---|---|---|---|---|---|---|
   | lock (°) | 42.2 | 40.3 | 37.1 | 34.2 | 31.7 | 28.5 | 26.7 |
4. Repeat to the right (`python3 steer_test.py -40.5 200 12`) for `STEER_LOCK_RIGHT_DEG`.
5. **Cross-check with the encoder and IMU:** `R = distance ÷ heading change in radians` over one full circle (`run_track.py --bench` shows both). This is how the 27 / 25 cm were found. It matches the floor marks only if the encoder follows the rear-axle midpoint. The rear axle is solid (no differential), so the tyres scrub and the floor marks are the reference.
6. Put the values in `config.py`. The Pi sends them to the STM32. At the next flash, also update `SteerMap.lockLeftDeg` / `lockRightDeg` in `drive_protocol.h`, so the compiled defaults agree; `test_drive_firmware.py` reminds you.
7. **Re-run the simulation sweep** (§12) with the new values: the planner's success depends strongly on them (CHANGES §17.5).

### 6.6 Speed loop feed-forward (`SPEED_KFF`, `SPEED_OFFSET_PWM`)
The firmware turns the speed the Pi asks for (mm/s) into motor PWM: `SPEED_KFF × speed + SPEED_OFFSET_PWM`, plus a PI loop on the encoder speed. No speed / PWM pair of this motor has been measured yet.

```bash
python3 drive_calibrate.py                        # car on a stand, wheels free: first fit
python3 drive_calibrate.py --speeds 150,300,500   # on the floor, clear 3 m straight: the values to keep
```

Press the start button when asked. At each speed it drives open loop, reads the PWM applied (from `$STA`) and the true speed (from the encoder), fits `speed = a × (pwm − offset)`, and prints the `SPEED_KFF` and `SPEED_OFFSET_PWM` to put in `config.py`, plus the top speed at PWM 255.

### 6.7 Speed loop gains (`SPEED_KP`, `SPEED_KI`)

```bash
python3 drive_calibrate.py --speeds 300 --closed 300,500,800,-200    # on the floor
```

It prints the settled speed and the time to reach 90 % (t90) for each target.

| Result | Meaning | Fix |
|---|---|---|
| Settled speed within 3 %, t90 below 0.4 s, no audible hunting | good | nothing |
| Slow to settle, or settles low | too little correction | raise `SPEED_KI` (× 1.5 steps) |
| Oscillates or hunts | too much gain | lower `SPEED_KP` first, then `SPEED_KI` |

Put the values in `config.py`; they are also live on the dashboard during a practice run.

### 6.8 Cruise speeds (`SPEED_LAP1_MM_S` 500, `SPEED_LAPS23_MM_S` 800)
- **Limit:** both must stay below about 85 % of the top speed from §6.6, or the speed loop saturates and the follower loses its speed margin.
- **Start slower:** 300 / 500 for the first field runs (§8), then raise once tracking is good.

### 6.9 LIDAR angle sign and zero (`LIDAR_ANGLE_SIGN`, `LIDAR_ANGLE_ZERO_OFFSET_DEG`)
- **Measured for the upside-down mount:** −1 / 0 (CHANGES §11).
- **Check:** put one object about 30 cm to the car's **right** and run `python3 run_init.py --real --dump sign.json`. It must read about 90°.
- **Redo only if** the LIDAR is remounted.

### 6.10 LIDAR lever arm and blind wedge
- **Lever arm:** `LIDAR_OFFSET_FORWARD_MM` 134.6, `LIDAR_OFFSET_LATERAL_MM` −0.7, from CAD; the pose point is the rear-axle midpoint. Check with a ruler that the LIDAR's spin axis is 134.6 mm ahead of the rear axle.
- **Blind wedge:** `REAR_BLIND_ARC_WIDTH_DEG` (105°) is the sector blocked by the chassis. Check it in a scan: the live scan on the dashboard, tracking mode, before Initialise, shows the gap behind the car.

### 6.11 LIDAR time offset (`LIDAR_TIME_OFFSET_S`, currently −0.05)
- **How:** `python3 measure_lidar_delay.py --real --record delay.json`. Keep the car still, then turn it by hand ±30° about once a second when told.
- **Result:** put the printed `LIDAR_TIME_OFFSET_S` in `config.py`. Repeat if it says NOT RELIABLE.
- **Why:** about ±10 ms of error is harmless; 30 ms costs wrong entry verdicts.

### 6.12 Camera (`CAMERA_*`, `COLOR_*`)
- **Mount:** the lens position from the CAD mount is 139.9 mm ahead, centred, 127 mm high. Check it with a ruler.
- **Check:** put one pillar 30° to the right, 500 mm from the lens:
  ```bash
  python3 camera_check.py --real --bearing 30 --range 500 --out check.png
  ```
  The box in `check.png` must sit on the pillar:

  | Box position | Fix |
  |---|---|
  | mirrored to the other side | flip `CAMERA_BEARING_SIGN` |
  | image upside-down | flip `CAMERA_ROTATE_180` |
  | too high or too low | fix `CAMERA_HEIGHT_MM` |
- **Colour thresholds:** repeat with a red and a green pillar at 300, 700 and 1200 mm, under the venue's lighting. The tool prints the colour fractions. The thresholds are placeholders until checked with real frames:
  - the hue ranges `COLOR_RED_HUE` / `COLOR_GREEN_HUE`;
  - `COLOR_MIN_SAT` / `COLOR_MIN_VAL`;
  - `COLOR_ID_MIN_FRACTION`, and `COLOR_ID_MARGIN_RATIO`.

  All are live on the dashboard's "Pillar colour + camera" group, and the lanes card shows each verdict.

### 6.13 Initialisation at the start zone
- **Place the car:** in a start zone (the middle zone above the parking lot), switched on, standing still.
- **Run:**
  ```bash
  python3 run_init.py --real --dump start.json
  ```
- **Check:**
  - the direction is right;
  - x (distance from the outer wall to the rear axle) and y (along the lane) match a tape measure within about 20 mm;
  - the seat verdicts match what you see.
- **Keep `start.json`:** `python3 run_init.py --replay start.json` re-runs it offline.
- **Try several placements** (straight, a little skewed, a little sideways). If a placement is refused, the report says why.

### 6.14 Path following on the field
- **Run:** `python3 run_mission.py --dashboard` (§8), with speeds at 300 / 500.
- **Watch on the page:** the **magenta planned path** against the **cyan driven path**. The driven path is the tracker's own estimate, so also watch the real car against the mat's lines.
- **Rough guide for the follower** ("Path follower" group, live):

  | What you see | What to change |
  |---|---|
  | Weaves or oscillates around the path | raise `RWF_LENGTH_MM` (e.g. 150 → 200) or `RWF_DAMPING` |
  | Cuts inside corners, or is late into arcs | raise `RWF_PREVIEW_S` (servo lag and link latency) a little |
  | Drifts off and corrects slowly | lower `RWF_LENGTH_MM` |
  | Grazes pillars although it follows the path well | raise `PLAN_CLEARANCE_MM` (30 → 40) and `PLAN_INFLATION_MM` (95 → 105): 95.5 % instead of 87 % under noise in simulation |
- **If the planner often finds no path** ("no path" in the plan line, or the car loops round in a corner), the lock is the cause (CHANGES §17.5). In simulation, `PLAN_RADIUS_FACTOR` 1.0 (instead of 1.25) recovers most of it at the cost of steering margin. Try it only after §6.5 and §6.7 are done, check the sweep (§12), and watch that the car still follows arcs well.

### 6.15 Calibration record
Keep this table (a copy in the repo, or on paper) and fill it in:

| Value | config.py now | Measured | Date | How |
|---|---|---|---|---|
| `IMU_YAW_SIGN` | −1 | | | §6.1 |
| `ENCODER_TICKS_PER_CM` | 14.853 | | | §6.2, 2 m roll |
| `SERVO_STRAIGHT_DEG` | 76.5 | | | §6.3 |
| `SERVO_LEFT_STOP_DEG` / `RIGHT` | 20 / 140 | | | §6.4 |
| `STEER_LOCK_LEFT_DEG` / `RIGHT` | **36.6 / 40.5 (placeholder)** | | | §6.5, floor marks |
| `SPEED_KFF` / `SPEED_OFFSET_PWM` | **0.20 / 25 (placeholder)** | | | §6.6 |
| `SPEED_KP` / `SPEED_KI` | **0.10 / 0.50 (placeholder)** | | | §6.7 |
| top speed at PWM 255 | – | | | §6.6 |
| `LIDAR_ANGLE_SIGN` / `ZERO_OFFSET` | −1 / 0 | | | §6.9 |
| `LIDAR_TIME_OFFSET_S` | −0.05 | | | §6.11 |
| `CAMERA_*` / `COLOR_*` | see config | | | §6.12 |

## 7. The dashboard

The dashboard is a web page for practice: it shows what the car perceives and plans, and changes values live. **It uses Wi-Fi, so it is practice only:** the rules forbid wireless during rounds (11.10), and the competition service (§9) runs without it.

### Three ways to run it

| Command | Where | What |
|---|---|---|
| `python3 run_mission.py --dashboard` | on the robot | **Mission mode:** the competition program (the button starts and stops runs) plus the page. The one to use for field practice |
| `python3 dashboard_server.py --mission [--seed N] [--direction CW]` | anywhere, `config.MODE = "mock"` | Mission mode on a **simulated car** and rulebook layout (`mission_sim.py`). Buttons stand in for the start button and for carrying the car back, and pick a new layout. Runs at 1× only |
| `python3 dashboard_server.py` | robot (`MODE = "real"`) or mock | **Tracking mode:** press Initialise, then drive the car by hand, or watch the scripted mock robot. Shows perception and the planner preview; nothing drives |

Open `http://<pi address>:5056/` (or `http://localhost:5056/`).

### What the page shows
- **The map:** lanes, walls, and the seats as the car sees them (present red / green / colour pending, absent, unknown; ■ from initialisation, ◆ from the entry re-check); the car; the driven path (cyan); the live LIDAR scan (yellow); the start scan (purple). In the mock, it also shows the truth (pink, dashed).
- **The planned path (magenta), continuously:**
  - **Between runs, and in tracking mode: the planner preview.** Every 0.5 s (`PLAN_PREVIEW_S`) the planner re-plans from the tracked pose and the seats and colours seen so far, exactly as the mission would: on lap 1 to the next lane's viewing pose, on laps 2–3 the rest of the round and the finish. It runs in a separate process, so it never slows the tracker down.
  - **During a run: the mission's own current path**, redrawn on every re-plan.
  - **Drawn with it:** the planner's goal poses (yellow circles with a heading tick) and the pass-side lines of every pillar with a known colour (red / green, dashed). The forward-only lines are optional. The line under the map says what was planned, whether it succeeded, and how long it took.
- **The mission card** (mission mode): the STM32's run state (READY / RUNNING / STOPPED / FINISHED) and run number; whether the Pi is ready; the mission phase, target lane, plans and re-plans; each run's outcome; the mission's events and the run-control lines.
- **Cards** for the tracker, lanes and seats (with the reason for every verdict and colour), the initialisation report, the STM32 link, the LIDAR, the camera, and the events.

### The tuning panel
- **Size:** 110 values in 10 groups:
  - LIDAR mount and calibration;
  - initialisation thresholds;
  - tracker and IMU;
  - seat detector;
  - **steering and drive firmware (STM32)**;
  - planner;
  - mission and speeds;
  - path follower;
  - run control (start button);
  - pillar colour and camera.
- **Each value** has its meaning and a tag saying when it takes effect: **live**, **next plan** (the preview shows the effect within 0.5 s), **next Initialise** or **next initialisation at rest**, or **sent to the STM32 within 0.5 s**.
- **Firmware values:** each shows the STM32's echo next to it, red while the STM32 does not hold the value yet (or clamped it).
- **Entering values:**
  - **Lists** are JSON: `VIEW_X_MM` is `[500, 350, 650]`; the hue ranges are `[[0, 10], [170, 179]]`. A list must keep its shape.
  - **Booleans and choices** are drop-downs.
- **Edits stay in memory.** A changed value shows its `config.py` value next to it. **Copy what you keep into `config.py`.** "Reset to config.py" undoes every edit.

## 8. A practice session

1. **Field:** set up by the rulebook (pillars, parking lot). Place the car in a start zone facing the round direction.
2. **Start the program** from an SSH session, **ready to stop the car**:
   ```bash
   python3 run_mission.py --dashboard --log imu.log --dump scan.json
   ```
3. **Wait for ready.** Once the car has stood still for a second it initialises: the terminal prints `[run] ready: ...`, the LED goes solid, and the page shows the start scan and the planner preview. If the LED keeps blinking every 250 ms, initialisation is failing (the reason is printed): move the car a little and let it stand.
4. **Press the button.** The car looks at each corner for about 1–3 s on lap 1, then drives laps 2–3 without stopping and stops in the start section.
5. **To stop:** press the button; the STM32 cuts the motor at once. If the Pi dies or the cable comes out, the STM32 stops the motor by itself after 250 ms. Ctrl-C ends the program.
6. **To run again:** carry the car back to a start zone and let it stand. When the LED is solid, press. Every run starts from nothing. After a finish the car becomes ready where it stopped, so carry it back first.
7. **Logs:**
   - `imu.log` holds every STM32 line;
   - `scan_run<N>.json` is run N's start scan;
   - `python3 run_track.py --replay-scan scan_run1.json --replay-imu imu.log` replays the tracking offline.
   - Each program start overwrites `imu.log`; copy the logs off after a run you want to keep.
8. `--no-camera` runs without colour ID: every pillar is then passed "either side", so use it for driving tests only.

## 9. Competition setup: start on power-up

The rules allow one power switch, then a waiting state, then one start button (9.10–9.11), and no wireless during rounds (11.10). So `run_mission.py` must start by itself when the Pi boots, **without** `--dashboard`, and Wi-Fi / Bluetooth must be off. One way is a systemd service:

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

- **Wireless:** turn it off for rounds, for example with `sudo rfkill block wifi bluetooth` (undo with `unblock`). Check the method with your organisers: the judges may inspect it (11.10).
- **Crashes:** `run_mission.py` keeps running between runs. If it crashes, systemd restarts it, and a run the STM32 still has as "running" is ended by the new program (the car does not move).
- **Before a competition, commit `config.py` with every calibrated value.** Dashboard edits are not saved.

## 10. Starting a round

1. Place the car in the start zone **switched off**, fully inside the zone, front wheels toward the next corner in the round direction (9.6–9.8).
2. Switch on. The Pi boots and `run_mission.py` starts. It initialises **while the car stands still**, and the LED goes solid (ready). Don't touch the car. If the LED keeps blinking every 250 ms, re-place the car.
3. On the judge's "Go", press the start button once. **Don't press it again**: a second press stops the run.
4. The car drives 3 laps and stops inside the start section. Parking is not implemented yet.

## 11. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `no STM32 samples on /dev/ttyACM0 within 3 s` | wrong `IMU_PORT`, drive firmware not flashed, or the IMU is not found (LED fast blink) |
| The STM32 does not appear as `ttyACM` | USB support must be "CDC (generic 'Serial' supersede U(S)ART)"; re-flash |
| Nothing happens on a press | the LED says why: 250 ms blink = the Pi is not ready (see the `[run]` lines); 1 s blink = the Pi program is not running; 100 ms = IMU fault |
| A run starts but nothing moves | the link is not healthy (stale `$IMU` / `$STA`, watchdog), so STOP frames are sent instead. Check §5.3 |
| `[run] run N started but no initialisation is held` | the STM32 started a run the Pi had just stopped being ready for; the run is ended at once. Let the car stand and press again |
| Init report: `direction: UNDETERMINED` | a pillar hides the opening past the island. This happens on about 17 % of rulebook starts, mostly CW starts with a pillar on the middle inner seat. Re-place the car (straighter, a little sideways) and let it stand: it retries every 0.5 s |
| Init report: `x rejected` / lane width | a side reading is not a wall; check `LANE_WIDTH_MM` (1000) against the field, and the LIDAR mount |
| `mission FAILED` with `no path ...` | no drivable path was found. Mostly the steering lock (CHANGES §17.5, §6.14 here); at the start, it can also be a pillar about 300 mm ahead that must be passed on the far side (re-place further back) |
| `mission FAILED` with `at the viewing pose but the tracker is in lane N` | the planner looped round (a 270° turn) where a corner was too tight for the lock, and the tracker counted it as a turn: the known issue (CHANGES §17.5) |
| Car grazes pillars | tracking error larger than the clearance: check §6.2 (encoder), §6.5 (lock), §6.7 (speed loop), then the follower (§6.14) |
| Wrong pillar colours / colours given up | camera check and thresholds (§6.12) |
| Dashboard: a firmware value's STM32 echo stays red | the STM32 clamped it (servo stops must stay either side of straight, lock 5–80°, gains ≥ 0), or it runs older firmware (no echo at all: flash the current `drive_bridge`) |
| Dashboard: "planned path: preview error" | the planner raised an error; the line says which. The preview restarts itself |
| Wrong seat verdicts while the dashboard is open | check `LIDAR_TIME_OFFSET_S` (§6.11). The preview runs in its own process and should not affect tracking; if verdicts only go wrong with the page open, turn `PLAN_PREVIEW_ENABLED` off and report it |
| `test_run_track.py` / `test_dashboard.py` report a few wrong seats | expected with `LIDAR_TIME_OFFSET_S` ≠ 0: their stand-in LIDAR does not apply the offset (CHANGES §16.13) |

## 12. Rehearse in simulation

The same mission runs on a simulated car with rulebook layouts, with no hardware:

```bash
python3 sim_closed_loop.py --seed 3 --noise moderate                       # one round, printed as it happens
python3 sweep_closed_loop.py --layouts 200 --noise none --out runs.csv     # many rounds, summary at the end
python3 sweep_closed_loop.py --layouts 200 --noise moderate --set STEER_LOCK_LEFT_DEG=32 --set STEER_LOCK_RIGHT_DEG=34
```

- `--set NAME=VALUE` tries a value without editing `config.py`.
- **After every calibration that changes the car's geometry** (lock, encoder scale, speeds), run a 200-layout sweep with `--noise none` and `--noise moderate_cal`, and compare with CHANGES §17.5 and §18.
- Noise presets: `none`, `moderate`, `moderate_cal` (encoder at 0.5 %), `harsh`.
- For the dashboard on a simulated car, see §7.

## 13. Reference

- **Pi ↔ STM32 link** (details in `drive_protocol.h` and `stm32_link.py`):
  - `$IMU,<seq>,<t_ms>,<enc>,<yaw>` at 100 Hz.
  - `$STA,<seq_ack>,<status>,<run_state>,<run_id>,<pwm>,<speed_mmps>` at 20 Hz. Status bits: 1 ENABLED, 2 WATCHDOG, 4 BUTTON (held), 8 CLOSED_LOOP, 16 IMU_OK. Run states: 0 READY, 1 RUNNING, 2 STOPPED, 3 FINISHED.
  - `$PAR,<id>,<value>`: the STM32's echo of a tuning value.
  - `# ...` log lines when the run state changes.
  - DRIVE frame (Pi → STM32, 11 bytes, `AA 55`): steering (0.1°, + = left), speed (mm/s), flags (ENABLE, CLOSED_LOOP, mode, PI_READY = bit 4, RUN_OVER = bit 5).
  - PARAM frame (8 bytes, `AA 56`, id and float32). Ids:

    | Id | Value |
    |---|---|
    | 1 | servo straight |
    | 2 | servo left stop |
    | 3 | servo right stop |
    | 4 | lock left |
    | 5 | lock right |
    | 6 | kff |
    | 7 | offset |
    | 8 | kp |
    | 9 | ki |
    | 10 | ticks per mm |
    | 0xFF | report all |
- **Files:**
  - `run_mission.py`: the program.
  - `run_control.py`: start / stop / restart.
  - `mission.py`, `vg_planner.py`, `follower.py`: plan and drive.
  - `lane_tracker.py`, `lane_init.py`: localization.
  - `drive_link.py`, `stm32_link.py`: the link.
  - `dashboard_server.py`, `templates/dashboard.html`, `plan_view.py`: the dashboard.
  - `mission_sim.py`: the simulated car.
  - `drive_calibrate.py`: the speed loop.
  - `config.py`: every value.

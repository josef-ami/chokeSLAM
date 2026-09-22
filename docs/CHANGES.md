# chokeSLAM: changes and resulting design

This is the running record of every change made to this repository from September 2026 onwards, and of how the resulting product works. It covers every mechanism in detail, the reasoning behind each design choice, and the evidence for it. It is updated at every checkpoint.

| Checkpoint | Content | Status |
|---|---|---|
| **A** | Clockwise conventions, lane frame with x from the outer wall, removal of mat-level code, simulator heading fix, **direction (gap) test**, **initialisation of x / y / seats** | Implemented and tested. **Awaiting your approval.** |
| **B** | STM32 → Pi feed (BNO08x yaw + hall encoder), lane tracker, turn detection, lane switching, entry-corner seat re-check, simulated STM32 feed | Design agreed (§9). Not implemented. |
| **C** | Dashboard rewritten lane-by-lane | Design agreed (§10). Not implemented. |

Nothing has been committed to git. Every change is an uncommitted edit on top of your last commit `dd9c8f4 added lane frame`, so `git diff` shows exactly what changed.

---

## Contents

1. [What was asked](#1-what-was-asked)
2. [Ground truth: the rulebook facts used](#2-ground-truth-the-rulebook-facts-used)
3. [Decision log](#3-decision-log)
4. [Conventions](#4-conventions)
5. [Initialisation, the complete mechanism](#5-initialisation-the-complete-mechanism)
6. [Problems found in the existing code, and what happened to each](#6-problems-found-in-the-existing-code-and-what-happened-to-each)
7. [File-by-file changes (checkpoint A)](#7-file-by-file-changes-checkpoint-a)
8. [Verification (checkpoint A)](#8-verification-checkpoint-a)
9. [Checkpoint B: agreed design (not implemented)](#9-checkpoint-b-agreed-design-not-implemented)
10. [Checkpoint C: agreed design (not implemented)](#10-checkpoint-c-agreed-design-not-implemented)
11. [Things to measure or set on the real robot](#11-things-to-measure-or-set-on-the-real-robot)
12. [How to run](#12-how-to-run)

---

## 1. What was asked

1. A procedure that decides whether the round direction is **clockwise or counter-clockwise**. It works by finding which side has a gap of about 1000 mm in the wall that runs along the direction of travel.
2. A dashboard that shows **only a lane**, together with the obstacles present. This was clarified later: lanes are added one by one as turns are made, until the whole lap is drawn.
3. A mechanism for **IMU input from an STM32 connected over USB to the Raspberry Pi 5**. It traces the motion along the lane and detects turns. Each turn causes a new lane to be drawn.
4. The LIDAR position fix and obstacle check are **initialisation only**. All motion tracking after that comes from the IMU.

Standing instructions:

- Assume nothing.
- Don't use prior context.
- Ask whenever in doubt.
- Run every feature past the owner for approval.
- Keep this document.

## 2. Ground truth: the rulebook facts used

All facts come from the *WRO 2026 Future Engineers General Rules*.

| Fact | Source |
|---|---|
| Racetrack inner size 3000 × 3000 mm (± 5) | 13.1 |
| Obstacle Challenge: distance between track borders always 1000 mm (± 10 at the International Final) | §8, "Obstacle Challenge rounds" |
| Four corner sections, four straightforward sections; the island is the centred 1000 × 1000 square | §5, Fig. 2, Fig. 11 |
| Traffic-sign seats sit 400 mm and 600 mm from either wall (the `400 / 200 / 400` chain), on the section's two boundary lines and its midpoint. In the lane frame that is y = 1000, 1500, 2000 | Fig. 11, Fig. 3 |
| Seat 50 × 50 mm; pillar 50 × 50 × 100 mm; "moved" circle 85 mm | 13.13, 13.19, 13.15 |
| 1–2 traffic signs per straightforward section | Fig. 8c (cards) |
| The robot starts in a start zone of the starting section, or in the parking lot | §5, §8 step 4, 9.7 |
| Parking lot is 200 mm wide, against the outer wall, 1.5 × robot length long, limited by two 200 × 20 × 100 mm magenta elements | §5 Fig. 4, 13.25 |
| When the lot is placed, the starting section's signs move to the seats nearer the inner wall | Fig. 8e |
| The robot is oriented so its front axle is nearer the next corner in the round direction, i.e. it faces the round direction | 9.8 |
| Data can't be entered by physical adjustment, so the direction has to be sensed | 9.9 |
| Direction and start position are drawn at random each round | 9.3, 9.4 |

## 3. Decision log

These are the owner's answers, numbered in the order they were asked.

| # | Question | Decision |
|---|---|---|
| 1 | Angle convention in the code | **Clockwise everywhere**: 0 = forward / grid north, 90 = right, 270 = left |
| 2 | Which side is the outer wall | **CCW → right, so x = d(90°); CW → left, so x = d(270°)** |
| 3 | Lane x-axis and obstacle bearing | **x = distance from the outer wall in both directions.** Bearing = `360 − atan2(dx, dy)` for CCW, `atan2(dx, dy)` for CW |
| 4 | Robot yaw at initialisation | **Exactly 0°** |
| 5 | Gap test wall model | **Parallel lines, yaw 0.** Superseded by #27 |
| 6 | Gap decision thresholds | **Opening ≥ 500 mm on one side and ≤ 150 mm on the other** |
| 7 | Start positions to support | **Start zones and parking lot** |
| 8 | What the STM32 sends | **Nothing defined yet, so propose a format** |
| 9 | y measurement | **Front-wall fan** (§5.4) |
| 10 | x measurement | **Raw ray median at 90° / 270°** with the lane-sum check |
| 11 | Where this document lives | **Markdown in the repo** (this file) |
| 12 | Mock mode | **Keep it, fix the heading (P3), add a simulated STM32 feed** |
| 13 | STM32 hardware | **BNO08x IMU + hall encoder on the drive motor** |
| 14 | Turn trigger | **Left to Claude.** Proposal approved: #25 |
| 15 | Obstacles on lanes after the first turn | **Re-run the LIDAR seat check on entry to each new lane** (relaxes item 4 for this purpose) |
| 16 | Dashboard lane extent | **Full lane (1000 × 3000). Once the 1st lane is complete, draw the 2nd lane in its place, and so on for the whole lap** |
| 17 | STM32 message format | **Approved** (§9.1) |
| 18 | USB link | **STM32 native USB (CDC)**, so `/dev/ttyACM*` and the baud rate is ignored |
| 19 | When the entry re-check runs | **Every frame until y = 1000; the first decided verdict per seat is kept and later frames fill unknowns** |
| 20 | Laps 2 and 3 | **Keep the lap-1 result; no re-check** |
| 21 | Dashboard extras | **Frozen init scan, live scan, IMU-traced path, tuning panel** (all four) |
| 22 | Canvas framing | **Fixed full-loop frame** |
| 23 | Start lane on later laps | **Init result only** |
| 24 | Modules the new design no longer uses | **Delete.** They stay recoverable from git history (the zip does contain history; an earlier statement that it didn't was corrected) |
| 25 | Turn rule | **Approved:** ≥ 45° toward the round direction **and** tracked y ≥ 2000, or ≥ 80° alone as a failsafe |
| 26 | Re-check alignment gate | **Only frames within ±20° of the new lane's grid north** |
| 27 | Gap test vs placement yaw (after the WRONG finding, §5.2.6) | **Fit the wall tilt inside the gap test only.** x, y and seats keep yaw 0 |

Pending your approval at checkpoint A:

- the lane-sum precondition added to the gap test (§5.2, step 1)
- the single source of truth for the lever arm and blind wedge (§5.5)
- the `run_init.py` review tool (§12)
- the banner added to README.md

## 4. Conventions

### 4.1 Angles

Every angle in the codebase is **clockwise**.

- **Robot-relative LIDAR angle:** 0 = forward, 90 = right, 180 = back, 270 = left.
- **Robot-frame cartesian axes:** `fwd_mm` (+ ahead) and `right_mm` (+ right), with `angle = atan2(right, fwd)`. These are `ScanPoint.fwd_mm` / `right_mm` in `scan_processing.py`. They replaced the old `x_mm` / `y_mm`, which meant forward/left.
- **Raw sensor angle to robot angle:** this mapping happens in exactly one place, `scan_processing.clean_and_project`:
  `corrected = (LIDAR_ANGLE_SIGN × raw + LIDAR_ANGLE_ZERO_OFFSET_DEG) mod 360`
  `corrected` must come out clockwise with 0 = chassis forward. With a sensor whose raw angle already runs clockwise from forward, SIGN = +1 and OFFSET = 0, which is the current config. How to check this on the bench is in §11.

### 4.2 The lane frame (`lane_frame.py`)

Each lane has its own frame, anchored to the direction of travel:

- **y** runs along the lane in the direction of travel. y = 0 is the wall behind (the corner the robot entered from) and y = 3000 is the wall ahead.
- **x** is the distance from the **outer** wall, in both round directions. x = 0 is the outer wall and x = 1000 is the island wall.
- **Bearings** are clockwise from the lane's **grid north**, which is +y, the direction of travel.

Because x is always measured from the outer wall, the direction +x points depends on the round direction:

| Round | Island is on the… | Outer wall is on the… | +x points… | handedness *h* |
|---|---|---|---|---|
| CCW (left turns) | left | right, so x = d(90°) | left | −1 |
| CW (right turns) | right | left, so x = d(270°) | right | +1 |

A clockwise bearing *b* is the lane-frame unit vector `(h·sin b, cos b)`. A lane vector `(dx, dy)` has bearing `atan2(h·dx, dy)`, which works out as:

- **CCW:** `360° − atan2(dx, dy)`. This is the agreed "2π − atan" form.
- **CW:** `atan2(dx, dy)`.

The two-argument `atan2` matters. At initialisation the robot stands between the seat rows, so seats lie beside it and behind it (dy ≤ 0). A one-argument `atan(dx/dy)` would fold those forward or divide by zero.

Helpers in `lane_frame.py`:

- `handedness`, `outer_wall_side`, `outer_wall_lidar_angle`
- `bearing_of`, `unit_of_bearing`
- `offset_in_lane`: a robot-fixed point (lever arm) in lane coordinates
- `corner_transform`: see §9.4
- `lane_to_global` / `global_to_lane` / `grid_north_bearing`: only for building simulated worlds

### 4.3 The global mat frame (`mat_geometry.py`)

This frame is used only to build worlds for the simulator and the tests.

- Origin is the outer wall's bottom-left inside corner, +X east, +Y north.
- Headings are grid bearings, clockwise from +Y.
- Contents: `OUTER_SIZE_MM = 3000` (13.1), `LANE_WIDTH_MM = 1000` (§8), the island box and the outer box.

The robot's own decision path never uses this frame.

## 5. Initialisation, the complete mechanism

The code is in `lane_init.py` (pipeline), `direction_detect.py` (direction), `seat_occupancy.py` (seats) and `scan_processing.clean_and_project` (input).

### 5.1 Input

Initialisation takes **one LIDAR snapshot** with the robot stationary at its start pose, facing the round direction (9.8). The existing behaviour is kept: `lidar_source.RPLidarC1Source` keeps the latest reading per 1° bucket, and one snapshot of that table is used.

The snapshot passes through `clean_and_project`, which:

- drops ranges outside 60–6000 mm and low-quality returns
- applies the mount calibration (§4.1)

The result is a list of `ScanPoint`s with clockwise `angle_deg` and `dist_mm`.

The pipeline runs four steps in order. Each must succeed for the next to run:

**direction → x → y → seats**

Any failure produces `InitResult.ok = False` with a reason. The pipeline never guesses.

The robot's **yaw is taken as exactly 0** in every step (decision #4): LIDAR 0° is treated as the lane's grid north. The single exception is inside the gap test, which fits the walls' actual tilt for its own use only (decision #27, §5.2.6).

### 5.2 Direction: the gap test (`direction_detect.py`)

#### 5.2.1 Why there is a gap on exactly one side

The robot starts in a straightforward section, so y is roughly 1000–2000, with one wall on each side.

- **The outer wall is the boundary of the whole field.** It runs unbroken from y = 0 to y = 3000, and nothing can be seen through it.
- **The island wall is only 1000 mm long** (y = 1000–2000). Past its far end, from y = 2000 to 3000, there is no wall. That is the opening into the next lane: 1000 mm long, ending at the wall ahead.

So looking forward along each side, one side shows about 1000 mm of opening and the other shows none. The island is on the robot's left when driving CCW and on its right when driving CW:

- **gap on the LEFT (270° side) → CCW**
- **gap on the RIGHT (90° side) → CW**

The matching gap behind the robot (y = 0–1000) is ignored. It lies mostly inside the rear blind wedge.

#### 5.2.2 The procedure

Angles are clockwise. The robot frame is (f, s), with f = r·cos a ahead and s = r·sin a to the right.

**Step 1 – Side distances.**
- d_right = median range of the returns within ±`SIDE_RAY_HALF_WINDOW_DEG` (2°) of 90°. d_left is the same around 270°.
- If either is missing, the result is UNDETERMINED.
- **Lane-sum precondition** (added at checkpoint A, pending approval): d_left + d_right must equal 1000 ± `LANE_WIDTH_TOLERANCE_MM` (40), otherwise UNDETERMINED. If the sum is off, one side ray isn't reaching its wall. A pillar abeam, for example, would otherwise become the "wall", and every ray past it would count as passing through.

**Step 2 – Wall-tilt fit.** This step was approved after the finding in §5.2.6.
- For each side, take the returns within ±`GAP_FIT_HALF_DEG` (30°) of that side's 90°/270° whose s lies within `GAP_FIT_BAND_MM` (100) of +d_right (right side) or −d_left (left side).
  - This band drops pillars, which stand at least 400 mm from either wall.
  - It also drops rays that pass through a gap, and anything else that isn't the wall.
- Fit a total-least-squares line through those points.
- Drop points more than `GAP_FIT_INLIER_MM` (30) from the line and refit, up to three times.
- A side counts as fitted with at least `GAP_FIT_MIN_POINTS` (10) inliers.
- Combining the sides:
  - **Both fitted:** their directions must agree within `GAP_FIT_AGREE_DEG` (2°), because the walls are parallel. Otherwise UNDETERMINED. The lane direction is the mean of the two.
  - **One fitted:** the lane direction is that side's.
  - **Neither fitted:** UNDETERMINED.
- Each side wall is then modelled as the line along the lane direction through that side's inliers. If only the other side fitted, it goes through (0, ±d_side) instead.
- The fit uses raw returns, **not** `scan_processing` clusters, so the cluster corner-merge problem (P1, §6) cannot affect it.

**Step 3 – Classify the forward returns.** For every return in that side's forward quadrant (right: 0–90°, left: 270–360°):
- Skip rays within `GAP_MIN_ANGLE_FROM_FWD_DEG` (1°) of dead-ahead, and rays whose crossing lies behind the robot.
- Compute:
  - `r_line` = the range at which the ray meets the side-wall line
  - `f` = how far along the lane that crossing is
- Classify the return:
  - `r > r_line + GAP_MARGIN_MM` (80): **passed through** the wall line at f
  - `|r − r_line| ≤ 80`: **wall present** at f
  - `r < r_line − 80`: **blocked** by something nearer (pillar, parking-lot limitation), so no information

**Step 4 – Measure the opening.** `opening(side) = max(f) − min(f)` over that side's passed-through returns, or 0 if there are none.

**Step 5 – Decide.**

| Condition | Result |
|---|---|
| opening(left) ≥ `GAP_OPEN_MIN_MM` (500) and opening(right) ≤ `GAP_CLOSED_MAX_MM` (150) | **CCW** |
| opening(right) ≥ 500 and opening(left) ≤ 150 | **CW** |
| anything else | **UNDETERMINED** (`direction = None`) |

For information, the result also reports:
- the fitted wall angle, which is minus the placement yaw
- both openings, where each begins and ends
- the per-side wall / through / blocked counts

#### 5.2.3 Why "passed through" rather than "where the wall ends"

A pillar, or the magenta parking-lot limitation, can make the outer wall *look* as if it ends early, because its far part is hidden behind the obstacle. But no ray can pass *through* the outer wall, because it is the field boundary. So occlusion can only shrink the measured opening on the island side, which at worst gives UNDETERMINED. It can never create an opening on the outer side.

#### 5.2.4 Why the opening reads about 900 mm, not 1000

Past the island's end, the ray continues into the next lane and stops at the extension of the wall ahead. Close to that wall it passes the side-wall line by less than the 80 mm margin, so the last ~100 mm of the opening are classified as "wall present". This is why the threshold is ≥ 500 rather than ~1000.

Measured: the opening starts within 18 mm of the island's end in every noiseless case. Its length is 855–932 mm, with a median of about 900 across the sweeps.

#### 5.2.5 Failure behaviour

UNDETERMINED stops initialisation (`InitResult.ok = False`) and reports both openings. The existing behaviour is kept: the operator re-runs initialisation. In the simulation the gap test decides in 95–96% of starts, and UNDETERMINED in the rest. Nearly all UNDETERMINED cases are a pillar next to the robot hiding most of the real opening.

#### 5.2.6 The placement-yaw finding (why step 2 exists)

The first implementation modelled each wall as a line parallel to the robot's forward axis at d(90°) / d(270°). That was decision #5.

On 1,600 simulated starts per yaw range, that model gave:

| Placement yaw | Correct | UNDETERMINED | **WRONG** |
|---|---|---|---|
| 0 | 1546 | 54 | 0 |
| ±1° | 1459 | 141 | 0 |
| ±2° | 1264 | 335 | **1** |
| ±3° | 1141 | 457 | **2** |
| ±5° | 991 | 606 | **3** |

Mechanism of the wrong cases:
- The robot is turned a few degrees toward the island.
- The outer wall therefore tilts *away* from the parallel model.
- Far ahead, its returns come back beyond the model line and look "passed through", faking an opening on the outer side.
- In every wrong case, a pillar on the mid-inner seat just ahead of the robot hid most of the real opening at the same time.

A wrong direction mirrors x and every seat for the whole run, so the owner chose to fit the wall tilt inside the gap test (decision #27). With the tilt fit, results on the same worlds:

| Placement yaw | Correct | UNDETERMINED | WRONG |
|---|---|---|---|
| 0 | 1545 | 55 | 0 |
| ±1° | 1543 | 57 | 0 |
| ±2° | 1543 | 57 | 0 |
| ±3° | 1545 | 55 | 0 |
| ±5° | 1534 | 66 | 0 |
| ±8° / ±12° (960 each) | 915 / 891 | 45 / 69 | 0 |

The wrong cases are kept as a regression test, `test_previously_wrong_scenarios`. On those scans, the tilt-fitted test is wrong 0 times out of 50, while the old model is wrong 30 times out of 50.

### 5.3 x: distance from the outer wall (`lane_init.measure_x`)

- `d(90)` and `d(270)` are medians of the returns within ±2°, as in step 1 above.
- **Sanity check:** `|d(90) + d(270) − 1000| ≤ LANE_WIDTH_TOLERANCE_MM (40)`, otherwise x is **rejected** with a reason, not trusted. This catches a pillar or limitation between the robot and a wall. In practice the gap test's identical precondition fails first.
- The sensor's distance to the outer wall is `x_sensor = d(90)` for CCW and `d(270)` for CW.
- **Lever arm:** `x = x_sensor + h × LIDAR_OFFSET_LATERAL_MM`, with h = −1 for CCW and +1 for CW, and LATERAL positive to the robot's left.
  - Reasoning, CCW case: if the LIDAR is L to the left of the reference point, the reference point is L nearer the right-hand outer wall, so x_ref = x_sensor − L.

Measured: median error 1.1 mm, max 5.5 mm. Accepted in 95% of simulated starts, and no accepted value was ever more than 30 mm off.

### 5.4 y: along the lane (`lane_init.measure_y`)

This is still the agreed `y = 3000 − (distance to the wall ahead)`, but the distance is measured over a **fan**, not the single 0° ray:

```
for returns with |angle| <= FRONT_FAN_HALF_DEG (30):
    f = r * cos(angle)                       # forward distance
front = median of f over returns with  f >= max(f) - FRONT_BAND_MM (40)
y_sensor = 3000 - front
y = y_sensor - LIDAR_OFFSET_FORWARD_MM       # reference point
```

The wall ahead is the farthest thing straight ahead that is still inside the field, so anything nearer is something standing in front of it. Rays that see through the island-side opening end on the same wall-ahead line, because it extends across the next lane, so they agree with it.

- **Sanity check:** 0 < y < 3000.
- **Why not d(0°):** in simulation, the single 0° ray read a pillar in 42 of 440 zone starts and the parking-lot limitation in 88 of 88 lot starts. That is P2 (§6). The regression tests show d(0) giving y = 2325 when the truth is 1300 (pillar ahead), and y = 2775 when the truth is 1500 (limitation). The fan gives 1300 and 1500.

Measured: median error 0.4 mm and max 4.0 mm at yaw 0. Placement yaw biases it, because yaw is taken as 0: max 7 / 20 / 39 mm at ±1 / 2 / 3°.

### 5.5 Seats: present / absent / unknown (`seat_occupancy.py`)

The detector's logic is unchanged. Only its frame and bearing changed (§4.2).

**Seat table** (rulebook, Fig. 11), lane frame, x from the outer wall:

| index | name | x | y |
|---|---|---|---|
| 0 | near-outer | 400 | 1000 |
| 1 | near-inner | 600 | 1000 |
| 2 | mid-outer | 400 | 1500 |
| 3 | mid-inner | 600 | 1500 |
| 4 | far-outer | 400 | 2000 |
| 5 | far-inner | 600 | 2000 |

The old names (near-left / near-right, …) followed the old x-from-left-wall frame.

**For each seat:**

1. Move from the pose reference point to the **sensor**, using the lever arm (`lane_frame.offset_in_lane`).
2. `dx, dy` = seat − sensor.
3. **Bearing**: `bearing_to(dx, dy, direction)`, i.e. `360 − atan2(dx, dy)` for CCW and `atan2(dx, dy)` for CW. The LIDAR angle to look at is `(bearing − yaw) mod 360`. At initialisation yaw = 0, so the LIDAR angle equals the bearing. (The old code mirrored this with `−bearing` because its LIDAR frame was counter-clockwise; that conversion no longer exists.)
4. **Expected range to the pillar's near face** = distance to the seat − 25 mm.
5. **Search window**: the pillar's angular half-width at that range plus `angular_margin_deg` (4°).
6. **Range tolerance**: `range_tol_mm` (70) + 3% of range + `seat_position_slack_mm` (10).

**Verdict rules, in order:**

- **UNKNOWN** if the seat can't be observed. The first check that applies wins:
  - its near face is closer than `min_observable_face_mm` (120), the sensor dead zone
  - it lies inside the **rear blind wedge** (105° centred on 180°)
  - there are no returns in the window
- **PRESENT** if the window contains a contiguous run of at least `min_points` (2) returns whose range steps are ≤ 40 mm, whose mean range is within tolerance of the expected face, and which is no wider than `max_width_factor` (2.5) × the pillar's angular width.
- Otherwise, UNKNOWN if:
  - the nearest return is **short** of the seat by more than the tolerance (line of sight blocked)
  - a return sits **at the seat's range** but isn't pillar-shaped (ambiguous, not absence)
  - a pillar there would have given fewer than `min_expected_hits` (2) returns at this scan's measured angular resolution (a far pillar's one or two returns can be lost to dropout)
- **ABSENT** only if every return in the window is clearly *beyond* the seat, and a pillar there would have been resolvable.

UNKNOWN is never treated as absent anywhere.

**One source of truth (pending approval).** The detector used to have its own lever arm and blind-wedge defaults (`DetectParams.lidar_offset_*`, `blind_arc_*`), separate from `config.py`. Setting only one of them would make x/y and the seats disagree about where the sensor is. `lane_init.detect_params_from_config()` now fills them from `config.LIDAR_OFFSET_*` and `config.REAR_BLIND_ARC_*`.

At initialisation, the robot stands between the seat rows. Seats beside it or behind it therefore fall in the blind wedge or dead zone, and are UNKNOWN. In simulation, 43% of init verdicts were UNKNOWN, 0 were wrong. By decision #23, the start lane keeps this result all run.

### 5.6 Output

`InitResult` contains:

- `ok`, `reason`
- `direction`: the full `DirectionResult`
- `x`: `XReading` with d90, d270, lane sum and x before and after the lever arm
- `y`: `YReading` with front, farthest f, returns used and y before and after the lever arm
- `yaw_deg = 0`
- `seats`: 6 `SeatReading`s with the state, a human-readable reason and every intermediate number

## 6. Problems found in the existing code, and what happened to each

| ID | Problem | What happened |
|---|---|---|
| C1 | The code documented a **counter-clockwise** LIDAR frame (90 = left) with `LIDAR_ANGLE_SIGN = +1` passing raw angles through. The owner's readings are clockwise (90 = right). | Codebase converted to clockwise (decision #1). |
| C2 | `localization.compute_start_of_run_fix` took the outer wall to be on the **left** for CCW, but geometry puts it on the right. `lane_frame.py` had flagged this as "Finding 3". | Replaced by `lane_init.measure_x` with CCW → d(90) (decision #2); `localization.py` deleted. |
| C3 | `seat_occupancy.py` measured x from the **left** wall, but the owner's method measures it from the **outer** wall. | Converted (decision #3). |
| C4 | The bearing formula `2π − atan(dx/dy)`, taken literally, is correct only for CCW, and a one-argument atan folds seats beside or behind the robot. | Implemented as `360 − atan2` (CCW) / `atan2` (CW) (decision #3). A test checks it against global geometry, worst error 1.7 × 10⁻¹³°. |
| P1 | `localization._find_wall_near` (cluster-based side-wall finder) failed to find one side wall in 55 of 320 simulated empty-field starts (17%). The far side wall and the wall ahead merge across the corner into one non-flat cluster, which is then rejected. | Not needed any more. x uses raw rays (decision #10) and the gap-test fit uses raw returns. The clustering in `scan_processing.py` remains only to colour the dashboard's point cloud. Not fixed. |
| P2 | `y = 3000 − d(0°)` reads a pillar standing roughly in line ahead, or the parking-lot limitation when starting in the lot. | Replaced by the front-wall fan (decision #9). |
| P3 | The mock simulator's heading formula (shared with `localization.driving_heading_deg`, "Finding 2" in the old `lane_frame.py`) pointed the robot 180° away from its direction of travel. | Simulator rewritten. Heading = the lane's grid north + yaw, derived from geometry (decision #12). |
| P4 | Lever arm and blind wedge had two independent settings, one in `config.py` and one in `DetectParams`. | Unified (§5.5), pending approval. |
| P5 | The parallel-wall gap model can give the WRONG direction under 2°+ placement yaw when a pillar hides the opening (§5.2.6). | Tilt fit inside the gap test (decision #27). |
| — | `mat_geometry.all_slots()` placed the 24 seats at 250/750 mm × 500/1500/2500 mm, which disagrees with Fig. 11 on all 24. | Deleted with the rest of the mat-level code (decision #24). |

## 7. File-by-file changes (checkpoint A)

**Deleted** (decision #24; recoverable from git history at `dd9c8f4`):

- `localization.py`: broadside fix, pose estimator, old start-of-run fix, 8 start candidates
- `scan_prediction.py`: predicted-scan overlay for the candidates
- `assess_localization.py`: assessment script for the old localization

**New:**

- `direction_detect.py`: the gap test (§5.2)
- `lane_init.py`: the initialisation pipeline (§5.1, §5.3–5.6)
- `run_init.py`: command-line review tool (§12)
- `test_direction.py`, `test_init.py`, `test_lane_frame.py`: verification (§8)
- `docs/CHANGES.md`: this document

**Rewritten:**

- `mat_geometry.py`: only the rulebook field geometry needed to build worlds (outer square, island). Removed: the slot table, broadside headings, section order, safe-fix zone and local↔global helpers.
- `lane_frame.py`: x from the outer wall; handedness, bearing, unit-vector, lever-arm and corner-transform helpers. Removed: `lane_affine`, `convention_disagreements` (obsolete) and `robot_to_lane` / `sensor_origin` (replaced by `offset_in_lane`).
- `simulation.py`:
  - clockwise ray-casting
  - heading derived from geometry (P3 fixed)
  - pose kept in the lane frame
  - models the rear blind wedge from `config.REAR_BLIND_ARC_*` (the old mock cast a full 360° and zeroed the detector's wedge instead)
  - default pillars on rulebook seats
  - helpers `rulebook_pillars`, `parking_lot_boxes`
  - Motion and the STM32 feed come at checkpoint B.
- `config.py`:
  - clockwise documentation and a bench-check procedure for SIGN / OFFSET
  - removed parameters of deleted code: `BROADSIDE_HEADING_TOLERANCE_DEG`, `FRONT_BACK_SEARCH_WINDOW_DEG`, `SIDE_SEARCH_WINDOW_DEG`, `SECTION_LENGTH_TOLERANCE_MM`
  - added the initialisation parameters: `SIDE_RAY_HALF_WINDOW_DEG`, `GAP_*`, `FRONT_FAN_HALF_DEG`, `FRONT_BAND_MM`
  - `MODE` left at your `"real"`

**Modified:**

- `scan_processing.py`: clockwise robot frame, `ScanPoint.fwd_mm` / `right_mm` instead of `x_mm` / `y_mm`, docstrings. Clustering logic unchanged.
- `seat_occupancy.py`:
  - frame and bearing (§5.5)
  - `detect_seat_occupancy(scan, x, y, direction, yaw, …)` now **requires the round direction**
  - `bearing_to_lidar_angle` removed (the LIDAR is clockwise now)
  - seat names outer/inner
  - detection logic unchanged
- `test_seat_occupancy.py`: updated to the new frame and signature. Its ray-caster is clockwise. Two tests added or strengthened: the bearing formula checked against global geometry, and the lever arm checked in both directions with the sensor pose derived independently in global coordinates.
- `README.md`: a banner at the top saying it describes the pre-change design and pointing here. Nothing else touched.

**Unchanged:** `lidar_source.py`, `requirements.txt` (pyserial is already listed, which checkpoint B needs).

**Not runnable at this checkpoint:** `dashboard_server.py` and `templates/dashboard.html` still import the deleted modules and use the old conventions. They are rewritten at checkpoint C. Until then, `run_init.py` is the way to look at initialisation.

## 8. Verification (checkpoint A)

All four suites pass (`test_lane_frame.py`, `test_seat_occupancy.py`, `test_direction.py`, `test_init.py`).

The worlds are built in global coordinates from the rulebook geometry and ray-cast independently of the code under test. The tests' caster uses global grid bearings, not `lane_frame.bearing_of`.

Test-world assumptions shape only the scenarios, not the code:

- start poses y 1150–1850, x 200–800 from the outer wall
- 1–2 pillars per section (Fig. 8c), none within 150 mm of the robot
- parking lot 200 × 450 mm with the robot 100 mm from the outer wall, lot centre y 1300–1700, start-section pillars only on inner seats (Fig. 8e)
- 720 rays/rev, 4 mm noise, 2% dropout, 105° rear blind wedge

**`test_lane_frame.py`** (all 4 lanes × both directions, against global geometry)

| Test | Result |
|---|---|
| handedness / outer side | the outer wall is at 90° for CCW and 270° for CW, in every lane |
| bearing round trip | `bearing_of(unit_of_bearing(b)) = b`, worst 5.7 × 10⁻¹⁴° |
| lever arm | `offset_in_lane` matches global geometry, worst 6.4 × 10⁻¹³ mm |
| corner transform | same global point and heading in both lanes (worst 0 mm, 5.7 × 10⁻¹⁴°); the pose lands in the new lane's entry corner square |

**`test_direction.py`**

| Test | Result |
|---|---|
| physical sanity | CCW: the outer wall is at 90° (right); CW: at 270° (left). Ties the chain to the physical world. |
| opening geometry (noiseless) | Opening starts at the island's end (worst 17.7 mm); length 855–932 mm; outer side 0 everywhere |
| zones, yaw 0 (640) | 616 correct, 24 UNDETERMINED, **0 wrong** |
| zones, yaw 0, 360 rays/rev (320) | 305 / 15 / **0** |
| zones, ±1 / 2 / 3 / 5 / 8° (640 each) | 618 / 616 / 613 / 611 / 608 correct; **0 wrong** |
| parking, yaw 0 / ±5° (160 each) | 149 / 151 correct; **0 wrong** |
| previously-wrong scenarios (50) | tilt-fitted **0 wrong**; old parallel model 30 wrong |
| undetermined cases | No data, a missing side ray, or openings on both sides all give UNDETERMINED |

**`test_init.py`**

| Test | Result |
|---|---|
| x / y, zones (640) | init ok 608 (95%), direction wrong 0, \|x err\| median 1.1 / max 5.3 mm, \|y err\| median 0.4 / max 2.1 mm |
| x / y, parking (160) | init ok 152 (95%), \|x err\| max 5.5, \|y err\| max 4.0 |
| lever arm (sensor 110 mm ahead, 60 mm left; 8 lane/direction combinations) | reference x, y within 0.1 mm |
| pillar abeam | initialisation fails with the lane-sum reason, never an x |
| pillar dead ahead | d(0) alone would give y = 2325 (truth 1300); the fan gives 1300 |
| parking limitation ahead | d(0) alone would give y = 2775 (truth 1500); the fan gives 1500 |
| seat verdicts, end to end | 4602 verdicts: 675 present, 1930 absent, 1997 unknown (43%); **0 false present, 0 false absent** |
| placement yaw (info) | ±1 / 2 / 3°: x within 5.4 mm; y within 7 / 20 / 39 mm; 0 wrong seats |

**`test_seat_occupancy.py`** (existing suite, new conventions)

| Test | Result |
|---|---|
| frame to global | reproduces the 24 Fig. 11 seats, round-trips |
| bearing formula vs global geometry | worst 1.7 × 10⁻¹³° |
| empty field | 0 false OCCUPIED out of 2592 |
| accuracy sweep | 2160 verdicts, 0 FP / 0 FN |
| drive-through | 40 runs, all 6 seats correct, lead ≥ 600 mm |
| coarse 1° sampling | 0 false EMPTY out of 720 |
| lever arm, CW and CCW | 6/6 with the offset declared |
| pose-error budget | clean up to 50 mm / 2° |

## 9. Checkpoint B: agreed design (not implemented)

### 9.1 STM32 → Pi message (decisions #13, #17, #18)

The STM32 enumerates as USB CDC, `/dev/ttyACM*`; the baud rate is ignored. It sends one ASCII line per sample at 100 Hz, `\n`-terminated:

```
$IMU,<seq>,<t_ms>,<enc>,<yaw>
seq   uint32  +1 every line (the Pi counts dropped lines)
t_ms  uint32  STM32 HAL_GetTick() when sampled
enc   int32   cumulative hall-encoder count since power-on, forward = +, never reset
yaw   float   BNO08x Game Rotation Vector yaw, degrees, 2 decimals, as the chip reports it
e.g.  $IMU,1042,10420,15873,-12.37
```

- A cumulative encoder count means a lost line loses no distance.
- Game Rotation Vector uses no magnetometer, so the motor's magnetic field can't disturb it.

### 9.2 Pi side

New config values:

- `IMU_PORT`, default `/dev/ttyACM0` (confirm; prefer `/dev/serial/by-id/…`)
- `IMU_YAW_SIGN`
- `ENCODER_MM_PER_COUNT`

The mechanism:

- A reader thread parses and validates lines, and counts dropped and garbled ones.
- `heading = IMU_YAW_SIGN × yaw`, unwrapped, minus its value at initialisation, plus 0 (init yaw).
- `dist = (enc − enc_at_init) × ENCODER_MM_PER_COUNT`.
- For each sample, with `ds` the distance moved and `ψ` the mean heading of the step, relative to the current lane's grid north:
  - `y += ds·cos ψ`
  - `x += ds·sin ψ · (−1 for CCW, +1 for CW)`, since x is measured from the outer wall.

### 9.3 Turn rule (decisions #14, #25)

`ψ` is the heading relative to the current lane's grid north. A turn is made when either condition holds:

- **Primary:** ψ has rotated **≥ 45° toward the round direction** (left for CCW, right for CW) **and** tracked **y ≥ 2000**, which means past the island's end, in the corner square.
- **Failsafe:** **≥ 80°** alone.

Turns against the round direction are ignored. All thresholds are config values.

- **Why the y gate:** a car can't be 45° into a turn toward the island before the island ends. The gate also blocks false turns during the final parking manoeuvre in the start section.
- **Why switching late costs nothing:** switching frames is an exact change of coordinates (§9.4). The only real risk is a false turn.

### 9.4 Lane switch (`lane_frame.corner_transform`, already written; checked by `test_lane_frame.py`)

```
new_y   = old_x
new_x   = 3000 - old_y
new_yaw = old_yaw + 90   (CCW)   |   old_yaw - 90   (CW)
```

The corner square is shared by both lanes. The new lane's wall behind is the old lane's outer wall, and its outer wall is the old lane's wall ahead. The same formula holds for both directions because x is always measured from the outer wall.

### 9.5 Entry-corner seat re-check (decisions #15, #19, #20, #23, #26)

- **Lap 1, lanes 2–4:** after the lane switch, on every LIDAR frame while tracked y < 1000 **and** |ψ| ≤ 20°, run the seat check with the IMU-tracked pose, **including its yaw**. The first decided verdict per seat is kept; later frames only fill unknowns. Verdicts freeze at y ≥ 1000.
  - Why the ±20° gate: a snapshot spans up to one rotation of the LIDAR, and mid-turn the robot can rotate about 9° within it, more than the detector's 4° margin.
- **Start lane:** init result only, all run.
- **Laps 2–3:** no re-check; the lap-1 result is kept.

### 9.6 Mock mode

The simulator gains a motion model (driving along lanes, arcing through corners) and a simulated STM32 that emits the same `$IMU` lines through the same parser. It adds encoder and yaw noise, so the whole chain runs end to end: init, tracking, turn, new lane, re-check.

## 10. Checkpoint C: agreed design (not implemented)

Decisions #16, #21 and #22.

- **Canvas:** a fixed full-loop frame (3000 × 3000). Lane 1 is drawn with travel up the screen, at the spot it will hold when the loop closes: the right edge for CCW, the left edge for CW. Undriven lanes stay blank. Each turn adds the next lane (full 1000 × 3000) in its place, until the lap is drawn. Nothing rescales.
- **Per lane:** walls; the 6 seats in 3 states, with where each verdict came from (init or entry re-check).
- **Overlays:** robot marker, IMU-traced path, the frozen init scan on lane 1, the live scan drawn at the IMU-tracked pose.
- **Panel:**
  - initialisation results: direction with both openings, x / y with their raw readings, seat table with reasons
  - tracker: lane number, lap, x, y, ψ
  - IMU status: port, rate, dropped lines, raw yaw / encoder
  - trimmed tuning panel and re-initialise button
- **Removed:** mat view, start candidates, predicted overlays, broadside panel, convention-warning box.

## 11. Things to measure or set on the real robot

| What | Where | How |
|---|---|---|
| LIDAR angle sign / zero | `config.LIDAR_ANGLE_SIGN`, `LIDAR_ANGLE_ZERO_OFFSET_DEG` | Put an object dead ahead (must read about 0°) and one on the robot's right (must read about 90°). If right reads about 270°, flip SIGN. If ahead isn't about 0°, set OFFSET. `run_init.py --real` then prints d90 / d270, which must match a tape measure to the right and left walls. |
| LIDAR lever arm | `config.LIDAR_OFFSET_FORWARD_MM`, `_LATERAL_MM` (+ left) | Measure from the pose reference point you want x and y to describe. |
| Rear blind wedge | `config.REAR_BLIND_ARC_CENTER_DEG`, `_WIDTH_DEG` | The empty angular gap in a raw scan dump (`run_init.py --real --dump scan.json`). |
| Placement | — | Initialisation takes yaw = 0. y degrades by about 20 mm at 2° of placement yaw and about 40 mm at 3° (§5.4); the direction test is unaffected. |
| (Checkpoint B) IMU yaw sign, encoder scale | `IMU_YAW_SIGN`, `ENCODER_MM_PER_COUNT` | Turn the robot clockwise by hand: the heading must increase. Roll a known distance to get mm per count. |

## 12. How to run

```bash
python3 test_lane_frame.py         # lane-frame geometry vs global geometry
python3 test_seat_occupancy.py     # seat detector, new conventions
python3 test_direction.py          # gap test (a few minutes)
python3 test_init.py               # x, y, seats end to end

# initialisation, printed with every intermediate number:
python3 run_init.py --sim --lane E --direction CW --x 350 --y 1250 --pillars 1,4
python3 run_init.py --sim --lane N --direction CCW --x 100 --y 1500 --parking 1500
python3 run_init.py --real --dump scan.json   # on the Pi, real LIDAR; saves the scan
python3 run_init.py --replay scan.json        # re-run on a saved scan (e.g. send it for analysis)
```

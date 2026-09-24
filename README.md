> **Note (September 2026):** this README describes the design as it was *before* the changes recorded in [docs/CHANGES.md](docs/CHANGES.md): clockwise angles, lane-only initialisation, the CW/CCW gap test, IMU tracking with the de-skewed entry re-check, the lane-by-lane dashboard, pillar colour from the camera (checkpoint D), and the connection to the visibility-graph path planner with the drive link, drive firmware and closed-loop rulebook simulation (checkpoint E: `run_mission.py`, `mission.py`, `vg_planner.py`, `sim_closed_loop.py`). Files it mentions such as `localization.py` and `scan_prediction.py` have been deleted. Where the two disagree, docs/CHANGES.md is current. **To run the obstacle round on the robot, follow [docs/RUNNING_ON_THE_ROBOT.md](docs/RUNNING_ON_THE_ROBOT.md).**

# WRO Obstacle Challenge broadside localization + dashboard

Implements the wall-referenced localization design from our earlier
discussion: a one-time start-of-run broadside LIDAR fix, re-anchored after
each corner turn (once safely clear of the corner, see below), fused with
continuous encoder+IMU dead reckoning between fixes. Includes a live
Flask dashboard that draws the mat to scale with the classified point cloud
and estimated pose overlaid.

Everything here was built and tested in a sandbox with **no hardware and no
network access to PyPI**, so it runs in two modes:

- **mock** (default) -- a simulated robot + world, ray-cast LIDAR, noisy
  odometry. No hardware needed. This is what's been tested.
- **real** -- talks to an actual RPLidar C1 via the `rplidarc1` package.
  Written directly from that package's published API docs, but **could not
  be run against real hardware in this session** -- sanity check it against
  your existing `lidar_probe.py` experience before trusting it.

## Quick start (mock mode)

```
pip install -r requirements.txt   # just flask + numpy for mock mode
python3 dashboard_server.py
```

Open `http://localhost:5056/`. You'll see the mat, the 24 placeholder
slots, four demo pillars, the live classified point cloud, and both the
estimated pose (solid blue) and simulated ground truth (grey outline) --
they should track closely, with visible small corrections each time the
robot passes through a corner.

## Files

| File | What it does |
|---|---|
| `mat_geometry.py` | Field constants (outer size, lane width, island, 4 sections, 24 slots) and section-local <-> global coordinate conversion. **Verify `OUTER_SIZE_MM` against your actual mat** -- it was read off the mat artwork, not stated explicitly in the rules pages we reviewed. |
| `scan_processing.py` | Turns a raw scan into classified wall/pillar clusters (arc-length threshold, not point count; total-least-squares line fit for a denoised, sub-single-ray perpendicular distance). |
| `localization.py` | The broadside fix (front=outer wall; back is inferred from front, not read -- see below) and `PoseEstimator`, which fuses it with odometry. Also the one-time start-of-run fix (`compute_start_of_run_fix`, `candidate_start_positions`) -- see below. |
| `lidar_source.py` | Real-hardware RPLidar C1 interface (untested, see above). |
| `simulation.py` | Mock world + robot, used only in mock mode. |
| `dashboard_server.py` | Flask app: runs the pipeline in a background thread, serves the dashboard over Server-Sent Events. |
| `templates/dashboard.html` | The dashboard page (plain canvas, no external JS libraries). |
| `config.py` | Everything you need to fill in / tune -- read it top to bottom before deploying. |

## What you need to fill in before running in "real" mode

All in `config.py`:

- `LIDAR_PORT`, `LIDAR_BAUDRATE` -- your serial device.
- `LIDAR_ANGLE_SIGN`, `LIDAR_ANGLE_ZERO_OFFSET_DEG` -- calibrate these
  against your actual mount (spin to a known heading, see what angle a
  known object shows up at). The simulator sidesteps this by construction,
  so it can't validate your real values for you.
- `LIDAR_OFFSET_FORWARD_MM`, `LIDAR_OFFSET_LATERAL_MM` -- the lever-arm
  offset from the LIDAR to your path planner's reference point, which you
  said you already have measured.
- `MODE = "real"` when you're ready to run on the robot.

And in `dashboard_server.py`'s `_real_mode_loop()`: the actual wiring from
your STM32 UART telemetry into `estimator.update_heading()` /
`update_odometry()` / `on_corner_completed()`. That parsing lives on your
Pi already and couldn't be written here without your protocol -- the
integration points are commented inline.

## Back reading dropped -- front-only lateral fix

Real hardware testing found the LIDAR's rear is permanently blocked by the
robot's own chassis: a raw scan dump showed front, left, and right all
resolving to clean, smooth wall returns close to their expected angles,
while a ~105 degree arc centred almost exactly on 180 degrees
robot-relative was all single-digit-millimetre readings -- the sensor
pressed up against the chassis, not a wall. This isn't a near-corner
artifact like the ones documented below; it's present everywhere on the
mat, on every section, so `compute_broadside_fix()` (the original lateral
fix, used after **every** corner turn, not just at start-of-run) could
never have passed its old front+back≈1000mm check on this robot,
regardless of calibration.

Back is now **inferred** from front instead of independently read:
`back_d = LANE_WIDTH_MM - front_d`, since the two have to sum to the lane
width by definition of the lane. `compute_broadside_fix()` and
`compute_start_of_run_fix()` both only search for a front wall cluster
now (plus, for the latter, the two side rays -- unaffected, they're well
clear of the blocked arc). This trades away the old front+back
cross-check -- with back defined from front, that sum is now always
exactly `LANE_WIDTH_MM`, not a real validation -- for a plausibility bound
on front_d instead: a genuine outer-wall reading has to land inside the
lane (`0 < front_d < LANE_WIDTH_MM`, with `LANE_WIDTH_TOLERANCE_MM` as
noise margin either side). `BroadsideFix`/`StartOfRunFix` still report
`back_distance_mm` and `lane_sum_mm` for the dashboard, just inferred
rather than measured -- the dashboard now labels them as such rather than
implying a live cross-check that isn't happening.

If the chassis is ever modified to give the LIDAR a clear line of sight
behind -- even a narrow slot, the search window is only ±15 degrees --
this is straightforward to revert: restore the `back = _find_wall_near(...)`
search and the real front+back sum check in both functions (see git
history prior to this change).

## Start-of-run: resolving along-track position and the 4-leg ambiguity

The original design only ever resolved LATERAL (cross-lane) position from
the broadside fix -- along-track position (`along_mm`, how far along the
current section's edge you are) had to be typed in by hand
(`initial_along_mm`), and which of the 4 sections (S/E/N/W) you're even on
always has to be supplied (`initial_section`) -- neither is recoverable
from the front reading alone.

`compute_start_of_run_fix()` now also reads the LEFT (90 deg
robot-relative) and RIGHT (270 deg) rays at the same one-time start-of-run
moment. On every one of the 4 sections, a broadside robot's left/right axis
runs exactly along the section's own along-track axis -- left always points
toward the far corner, right toward the near corner (verified against
`mat_geometry._section_axes` for all 4) -- so `along_mm = right_raw -
LIDAR_OFFSET_LATERAL_MM`, cross-checked against `OUTER_SIZE_MM - left_raw -
LIDAR_OFFSET_LATERAL_MM`, with a left+right≈`OUTER_SIZE_MM` sanity check
(same spirit as the front plausibility bound above). This replaces the
manual `initial_along_mm` guess with a real reading, for whichever section
turns out to be the right one.

It still can't tell you *which* section that is -- that's the same
unresolvable gap the front fix always had, just now stated for
along-track too. `candidate_start_positions()` takes the along/lateral pair
and expands it into all 8 dashboard markers: one for each of the 4 sections
x 2 headings (the section's real broadside heading, and that +90 degrees,
purely so the dashboard can show both axis orientations at every
candidate) -- shown once at start-of-run so your team can visually confirm
which one matches where the robot was actually placed. This is a
one-time DISPLAY addition only: the live-tracked pose still needs
`initial_section` supplied manually, same as before, now just auto-filled
with a real `along_mm` instead of a guess.

**Read before relying on a specific starting spot.** Sweeping the mock
simulator across along/lateral combinations (noiseless, to isolate the
geometry from sensor noise) found the window where front, left, AND right
*all* resolve cleanly is workable across much more of each section now
that back is no longer part of the requirement -- e.g. at the lane's
lateral centre (500mm from the outer wall) it's open for roughly
70-650mm, 1380-1630mm, and 2880-2950mm along the section, versus only a
single ~250mm window before. It's still not the WHOLE section, and still
not simply "near the middle" -- at lateral=150mm it didn't open anywhere
in one sweep. Root cause for what's left: the near/far OUTER corners are
real sharp 90-degree corners too, so a side ray taken too close to one
still blends the perpendicular wall and the along-track wall into one
continuously-curving return, the same corner-blending effect documented
below for the back ray and the island -- clustering correctly refuses to
call that flat, and this fix correctly rejects it (confirmed: no silent
wrong answer), it's just a smaller effect now that only 3 of the 4
original readings need to simultaneously avoid it instead of 4. Verify
the workable range for your own geometry; `dashboard_server.py`'s mock
demo overrides the simulator's own default starting `along_mm` (50.0,
right next to a corner) for exactly this reason.

## Start-of-run candidate overlay (arrows + predicted LIDAR per leg)

Once `compute_start_of_run_fix()` succeeds, the dashboard now draws, for each
of the 8 candidates (`candidate_start_positions()` -- 4 legs x 2 axes), a
colour-coded arrow AND that pose's **predicted LIDAR scan** superimposed on
the mat: what the sensor *would* see if the robot were at that pose,
ray-cast against the known walls + island by `scan_prediction.predict_scan_global()`.
Each leg gets one colour (Okabe-Ito, colourblind-safe) shared by its arrow and
its predicted point cloud; the `START CANDIDATES` side panel lists all 8 with
their `(x, y)` and bearing. Compare the predicted clouds against the single
real start-of-run scan to pick which leg the robot is actually on -- that's
the one whose prediction lines up with the live points.

The prediction models the **rear chassis blind arc** on purpose
(`config.REAR_BLIND_ARC_CENTER_DEG` / `_WIDTH_DEG`, default 180 deg / 105 deg
from the "Back reading dropped" section): the same wedge that's dead on real
hardware is cut out of each prediction, so (a) a predicted cloud looks like a
real return from this robot, not a full 360 deg sweep, and (b) the two heading
variants at one leg differ (the blind wedge points a different way), making all
8 overlays visually distinct. **Measure your unit's real blind wedge** off a
raw scan dump (the empty angular gap in `_debug_dump_clusters` output) and set
those two config values to match -- the defaults are the README's stated
figures, not measured on your unit. Predictions are static (they depend only on
the fixed candidate poses), so they're computed once at start-of-run and reused
every frame -- not re-ray-cast at stream rate.

### Start orientation: along the lane, facing the direction of travel

The one-time start-of-run scan is taken with the robot in its REAL start
orientation: **parallel to the walls, facing the direction of travel** (down
the lane) -- NOT broadside. `compute_start_of_run_fix(clusters,
driving_direction)` resolves position from that view:

- The two **side rays** (90 deg = left, 270 deg = right) hit the OUTER and
  INNER lane walls, so they give the **cross-lane** position and sum to
  ~`LANE_WIDTH_MM` (~1000 mm). Which side is the outer wall is fixed by the
  driving direction, not the leg: **CCW keeps the outer wall on the left**
  (90 deg), CW on the right (270 deg).
- The **forward ray** (0 deg, straight down the lane) gives the range to the
  wall ahead, which resolves **along-track**: CCW -> `along = forward`,
  CW -> `along = OUTER_SIZE_MM - forward`. Forward is taken as the median
  range of the dead-ahead points (not a fitted wall), since straight down the
  lane the return is usually a corner-blend, not a flat wall.

Then `candidate_start_positions(along, lateral, driving_direction)` places all
4 legs, each facing its **driving heading** (`driving_heading_deg()` -- the
leg's broadside bearing turned 90 deg to run along the lane), plus the +90 deg
variant.

**Angle convention -- GRID BEARINGS.** All *world* headings (robot heading,
`BROADSIDE_HEADING_DEG`, driving headings, candidate bearings, the dashboard's
heading readout) are grid/compass bearings: **0 = grid north, 90 = east,
clockwise**, matching a compass/IMU. So a robot facing grid north reads 0 deg
(not 90). This is separate from the *robot-relative* LIDAR frame used by the
fix, which is unchanged (0 = forward, 90 = left, 180 = back, 270 = right). The
robot->world rotation converts once with `maths_angle = (90 - bearing)`; if you
switch back and forth, that's the only relation you need. Because bearings run
clockwise, the driving heading is `broadside + 90` for CCW and `broadside - 90`
for CW (the opposite sign from a maths-angle convention). Feed a north-
referenced IMU into `update_heading()` directly; add a fixed offset at that
boundary only if your IMU's zero isn't grid north.

This replaces an earlier version that assumed the robot faced the OUTER WALL
(broadside) at start, which put the side rays down the lane and expected
left+right ~= `OUTER_SIZE_MM` (~3000 mm). On the real robot the start
orientation is along-the-lane, so that version rejected every real scan with
"no wall cluster found for front". If the fix now reports
`left+right ~= 3000` it means the robot is broadside instead of along the
lane; ~1000 is the along-lane orientation it expects.

**Valid start zone (unchanged constraint):** the island only faces the middle
~1000-2000 mm of each 3000 mm edge, so the cross-lane fix only works when the
robot starts in that middle band -- outside it, a side ray sails past the
island's corner to a far wall and the lane-width check fails (correctly). You
said the robot always starts in a working position; this is what "working"
means geometrically.

## Editable dashboard (live tuning + pose)

The dashboard's **TUNING PARAMETERS** panel is fully editable at runtime -- no
restart, no file edits. Every tuning constant (config.py, mat_geometry.py,
scan_processing.py) and the live pose-estimation state are exposed as inputs
that POST to the running server:

- `GET /api/tuning` builds the panel from a registry that reads each value
  live; `POST /api/param {name, value}` applies one edit; `POST /api/refit`
  re-takes the start-of-run scan and rebuilds the fix + candidates + predicted
  overlay with the current parameters; `POST /api/reset` restores the file
  defaults captured at startup.
- **Live vs re-run:** calibration (`LIDAR_ANGLE_*`) and clustering
  (scan_processing) edits show up on the *next frame* -- watch the point cloud
  rotate / re-classify as you type. The start-of-run candidates are static, so
  click **Re-run start-of-run fix** to rebuild them after changing anything
  that affects them (geometry, tolerances, blind arc, driving direction,
  overlay density).
- **Field geometry** edits (`OUTER_SIZE_MM`, `LANE_WIDTH_MM`,
  `SAFE_FIX_MARGIN_MM`) re-derive the island + safe zone and redraw the mat;
  the derived values are shown read-only.
- **Pose estimate (live)** lets you set the estimator's `section`, `along_mm`,
  `lateral_mm`, `heading_deg` directly. In mock mode the simulator overwrites
  these every frame unless you tick `freeze_pose` (in real mode nothing feeds
  the estimator here, so edits stick).
- `MODE`, `DASHBOARD_HOST/PORT` are read-only (they need a restart to rebind).

These are debug controls on a single-viewer local dashboard -- there's no
auth; don't expose the port beyond your bench network.

## Findings from actually building and testing this (read this part)

A few things surfaced only once this got implemented and stress-tested in
simulation, that weren't obvious from the design discussion alone:

**The corner-turn fix needs to wait, not fire immediately.** The island is
a plain inward offset of the outer square, so it only directly faces the
*middle* portion of each 3000mm edge. Right at a corner-turn completion
(along-track position ~0), a perpendicular "back" ray toward the island
often sails past its corner and hits something much farther away instead of
the island's near face -- the front+back sanity check correctly *rejects*
this (confirmed: it never silently returns a bad answer), but that also
means a fix attempted right at the corner will usually just fail outright.
`PoseEstimator.in_safe_fix_zone()` tells you when you've driven far enough
into the new section for the geometry to work; wait for it before going
broadside. `mat_geometry.SAFE_FIX_MARGIN_MM` documents the margin and why
it needs to be a few hundred mm, not a token amount.

**The initial fix only resolves lateral position, not along-track
position.** If you don't tell `PoseEstimator` roughly where along the
starting section the robot was actually placed (`initial_along_mm`), your
x/y estimate will be off by that amount until the first corner turn
resynchronises along-track tracking to 0. The lateral (cross-lane) part is
still correct from the start either way -- it's specifically the
along-the-lane component that's affected.

**Wall-cluster matching has to use the cluster's actual angular coverage,
not its centroid angle.** A nearby, wide wall cluster can have a centroid
several degrees away from where you'd naively expect, especially once it's
been capped by `MAX_CLUSTER_SPAN_DEG` (added to stop a slowly-changing
sequence of points chaining two unrelated, non-collinear surfaces -- e.g.
a wall bending around a real corner -- into one cluster). Matching by "is
the target bearing inside this cluster's span" instead of "is the centroid
near the target" fixed a real bug where the correct wall was being found by
the clustering step but then discarded by the matching step.

**When two wall-like clusters both fall inside the search window, prefer
the nearer one.** This happens near corners, where a true nearby surface
and a much farther glimpsed-through-a-gap surface can both have a plausible
centroid angle; nearest-wins is both simpler and physically correct (the
sensor is blocked by whatever's actually closest).

None of this needed a fundamentally different design from what we
discussed -- it's all tuning/robustness that only shows up once you throw
real (simulated) noisy geometry at it, which is exactly why it's called out
here rather than left for you to rediscover.

## Testing performed

- Unit-level checks of clustering/classification/line-fit against
  hand-computed expected values.
- A ~600-simulated-second, multi-lap run (`python3` one-liners, not
  committed as a test file -- ask if you want these turned into a proper
  pytest suite) verifying: every broadside fix attempt in a valid position
  succeeds with front+back within a few mm of 1000mm, and position error
  converges to ~25mm and stays bounded across corners, vs. drifting
  unbounded when fixes are skipped.
- Playwright screenshot of the live dashboard confirming the point cloud
  visually aligns with the drawn walls/island and the estimated/true
  markers track together.
- `compute_start_of_run_fix()`: verified against all 4 sections at a
  centred along/lateral position (along/lateral estimate within ~1mm of
  truth against the simulator's ground truth, with default noise/dropout
  and default pillars; the matching candidate from `candidate_start_positions()`
  reproduces the true x/y/heading), ~99% success rate (198/200) at that
  same centred spot across repeated noisy trials, and the along/lateral
  sweep (noiseless) described above that found how narrow the workable
  window actually is.

Not tested (couldn't be, in this sandbox): the real `rplidarc1` hardware
path, and your actual STM32 telemetry integration.

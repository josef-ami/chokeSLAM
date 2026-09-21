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
| `localization.py` | The broadside fix (front=outer wall, back=inner wall, sanity-checks front+back≈1000mm) and `PoseEstimator`, which fuses it with odometry. |
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

Not tested (couldn't be, in this sandbox): the real `rplidarc1` hardware
path, and your actual STM32 telemetry integration.

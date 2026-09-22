"""
Values you need to fill in / verify for your actual robot and field.
Nothing here is safe to trust blindly -- see the comment on each one.

ANGLE CONVENTION (Sept 2026, agreed): every angle in this codebase is
CLOCKWISE. Robot-relative LIDAR angles: 0 = forward, 90 = right, 180 = back,
270 = left. Lane bearings: clockwise from the lane's grid north (the direction
of travel). See docs/CHANGES.md.
"""

# --- LIDAR hardware -----------------------------------------------------
LIDAR_PORT = "/dev/ttyUSB0"          # confirm with `ls /dev/ttyUSB*` on the Pi
LIDAR_BAUDRATE = 460800              # rplidarc1 default for the C1
LIDAR_SCAN_TIMEOUT_S = 0.2

# Mapping from the sensor's RAW angle to the robot frame used everywhere:
#     corrected = (LIDAR_ANGLE_SIGN * raw + LIDAR_ANGLE_ZERO_OFFSET_DEG) % 360
# and `corrected` must come out CLOCKWISE with 0 = the chassis' forward
# direction (90 = right, 270 = left). With raw angles that already run
# clockwise from forward, SIGN = +1 and OFFSET = 0. Check once on the bench:
# put an object dead ahead (must read ~0) and one on the robot's right (must
# read ~90). If right reads ~270, flip SIGN; if dead ahead isn't ~0, set OFFSET.
LIDAR_ANGLE_SIGN = 1                 # +1 or -1
LIDAR_ANGLE_ZERO_OFFSET_DEG = 0.0    # degrees added after the sign

# LIDAR mounting offset from the pose reference point (e.g. rear-axle
# midpoint), in the robot's own frame: FORWARD_MM is positive if the LIDAR
# sits ahead of the reference point, LATERAL_MM is positive to the robot's
# LEFT. Used by initialisation (x, y) and by the seat check (one source of
# truth since Sept 2026 -- lane_init passes these into DetectParams).
LIDAR_OFFSET_FORWARD_MM = 0.0        # <-- FILL IN
LIDAR_OFFSET_LATERAL_MM = 0.0        # <-- FILL IN

# --- Rear chassis blind arc ----------------------------------------------
# A wedge of the LIDAR's view centred on the robot's rear is blocked by the
# chassis. Seats inside it can only ever be UNKNOWN. Clockwise robot angle;
# the wedge is symmetric, so 180 is straight back in either convention.
# Measure the real wedge off a raw scan dump and set these to match.
REAR_BLIND_ARC_CENTER_DEG = 180.0   # robot-relative angle the wedge is centred on
REAR_BLIND_ARC_WIDTH_DEG = 105.0    # total angular width of the blocked wedge

# --- Initialisation (LIDAR, once, robot stationary at its start pose) -----
# All three steps assume the robot's yaw is exactly 0 (agreed): LIDAR 0 deg
# is taken to be the lane's grid north. See lane_init.py / direction_detect.py.

# Side rays: d(90) and d(270) are the MEDIAN of the returns within this many
# degrees of 90 / 270. Used by the direction test and for x.
SIDE_RAY_HALF_WINDOW_DEG = 2.0

# x sanity check: d(90) + d(270) must equal the lane width within this
# tolerance, otherwise something (a pillar abeam, a limitation) is standing
# between the robot and a wall and x is rejected rather than trusted.
LANE_WIDTH_TOLERANCE_MM = 40.0

# Direction (gap) test -- direction_detect.py.
GAP_MARGIN_MM = 80.0        # a return this much beyond the side-wall line = "passed through"
GAP_OPEN_MIN_MM = 500.0     # the gap side needs at least this much opening (along the lane)
GAP_CLOSED_MAX_MM = 150.0   # ...and the other side at most this much
GAP_MIN_ANGLE_FROM_FWD_DEG = 1.0   # rays closer to dead-ahead than this are skipped (sin ~ 0)
# Wall-tilt fit, used ONLY inside the gap test (approved after the yaw finding;
# x, y and the seat check still take yaw = 0). Each side wall is fitted to the
# raw returns within +/- GAP_FIT_HALF_DEG of 90 / 270 that lie within
# GAP_FIT_BAND_MM of that side's d(90)/d(270) -- which drops pillars (>= 400 mm
# from either wall) and rays passing through a gap.
GAP_FIT_HALF_DEG = 30.0
GAP_FIT_BAND_MM = 100.0
GAP_FIT_INLIER_MM = 30.0     # robust refit: drop returns farther than this from the line
GAP_FIT_MIN_POINTS = 10      # a side needs at least this many inliers to count as fitted
GAP_FIT_AGREE_DEG = 2.0      # the two walls are parallel: fitted tilts must agree within this

# y: front-wall fan -- lane_init.measure_y().
FRONT_FAN_HALF_DEG = 30.0   # returns within +/- this of 0 deg are considered
FRONT_BAND_MM = 40.0        # the front wall = returns within this of the farthest forward distance

# --- Dashboard / server ---------------------------------------------------
DASHBOARD_HOST = "0.0.0.0"
DASHBOARD_PORT = 5056
STREAM_HZ = 10

# --- Mode switch -----------------------------------------------------------
# "mock"  -> simulated LIDAR (and, from checkpoint B, a simulated STM32 feed),
#            no hardware needed.
# "real"  -> real rplidarc1 device on LIDAR_PORT.
MODE = "real"

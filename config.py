"""
Values you need to fill in / verify for your actual robot and field.
Nothing here is safe to trust blindly -- see the comment on each one.
"""

# --- LIDAR hardware -----------------------------------------------------
LIDAR_PORT = "/dev/ttyUSB0"          # confirm with `ls /dev/ttyUSB*` on the Pi
LIDAR_BAUDRATE = 460800              # rplidarc1 default for the C1
LIDAR_SCAN_TIMEOUT_S = 0.2

# Your LIDAR's angle convention relative to the chassis. RPLidar units
# typically increase clockwise looking from above, with 0 deg roughly at the
# cable-exit side rather than necessarily the chassis' forward direction.
# Calibrate these two once (spin the robot to a known heading and see what
# angle a known object shows up at) rather than guessing:
LIDAR_ANGLE_SIGN = 1                 # +1 or -1
LIDAR_ANGLE_ZERO_OFFSET_DEG = 0.0    # degrees added to raw angle before sign flip

# LIDAR mounting offset from your path planner's robot reference point
# (e.g. rear-axle midpoint), in the robot's own frame: FORWARD_MM is
# positive if the LIDAR sits ahead of the reference point, LATERAL_MM is
# positive to the robot's left. You told us you have exact numbers for
# this -- put them here.
LIDAR_OFFSET_FORWARD_MM = 0.0        # <-- FILL IN
LIDAR_OFFSET_LATERAL_MM = 0.0        # <-- FILL IN (only matters for along-lane reads, not the cross-lane fix)

# --- Localization tuning -------------------------------------------------
BROADSIDE_HEADING_TOLERANCE_DEG = 6.0   # how close to the target heading before we trust a fix
LANE_WIDTH_TOLERANCE_MM = 40.0          # back is inferred from front (LANE_WIDTH_MM - front_d), not
                                         # independently read -- see localization.compute_broadside_fix.
                                         # This is now the noise margin on the plausibility bound instead
                                         # of a front+back cross-check: front_d must land within
                                         # [0-this, LANE_WIDTH_MM+this] to be trusted as a genuine
                                         # outer-wall reading.
FRONT_BACK_SEARCH_WINDOW_DEG = 15.0     # how far from 0 deg / 180 deg (robot-relative) to look for the wall cluster.
                                         # Kept fairly tight, just past BROADSIDE_HEADING_TOLERANCE_DEG: a wide window
                                         # increases the chance of a ray near the edge of the window skimming past a
                                         # nearby corner (see mat_geometry.SAFE_FIX_*_MM) and picking up the wrong,
                                         # much farther surface -- found during testing, not theoretical.

# --- Start-of-run along-track fix (left/right, 90/270 deg robot-relative) --
# Only used once, at the very start of the run, to resolve along-track
# position from the two side rays -- see localization.compute_start_of_run_fix().
SIDE_SEARCH_WINDOW_DEG = 15.0           # same idea as FRONT_BACK_SEARCH_WINDOW_DEG, for the 90/270 deg rays
SECTION_LENGTH_TOLERANCE_MM = 60.0      # left+right distance sanity check: |sum - OUTER_SIZE_MM| must be under
                                         # this. Wider than LANE_WIDTH_TOLERANCE_MM since the rays travel much
                                         # farther (up to ~3000mm vs ~1000mm), so the same angular/range noise
                                         # translates into a bigger absolute error at the far end.

# --- Rear chassis blind arc (for the predicted-scan overlay) --------------
# On real hardware a wedge of the LIDAR's view centred on the robot's rear is
# permanently blocked by the chassis (see README "Back reading dropped": a
# ~105deg dead zone centred almost exactly on 180deg robot-relative). This is
# used by scan_prediction.predict_scan_global() to blank the same wedge out of
# each candidate's PREDICTED scan, so the overlay matches this robot's real
# (rear-occluded) field of view and the two heading variants at each leg look
# different. Measure the real wedge off a raw scan dump (the empty angular gap
# in dashboard_server._debug_dump_clusters output) and set these to match --
# these defaults are the README's stated figures, not measured on your unit.
REAR_BLIND_ARC_CENTER_DEG = 180.0   # robot-relative angle the wedge is centred on (180 = straight back)
REAR_BLIND_ARC_WIDTH_DEG = 105.0    # total angular width of the blocked wedge

# --- Dashboard / server ---------------------------------------------------
DASHBOARD_HOST = "0.0.0.0"
DASHBOARD_PORT = 5056
STREAM_HZ = 10

# --- Mode switch -----------------------------------------------------------
# "mock"  -> simulated LIDAR + simulated odometry, no hardware needed, for
#            testing this code and the dashboard on a laptop.
# "real"  -> real rplidarc1 device on LIDAR_PORT. You still need to wire up
#            real odometry (see odometry.py's OdometrySource) to your
#            STM32/encoder+IMU feed -- that part is stubbed, not simulated,
#            in "real" mode.
MODE = "real"

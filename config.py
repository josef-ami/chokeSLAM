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
LANE_WIDTH_TOLERANCE_MM = 40.0          # front+back distance sanity check: |sum - 1000| must be under this
FRONT_BACK_SEARCH_WINDOW_DEG = 15.0     # how far from 0 deg / 180 deg (robot-relative) to look for the wall cluster.
                                         # Kept fairly tight, just past BROADSIDE_HEADING_TOLERANCE_DEG: a wide window
                                         # increases the chance of a ray near the edge of the window skimming past a
                                         # nearby corner (see mat_geometry.SAFE_FIX_*_MM) and picking up the wrong,
                                         # much farther surface -- found during testing, not theoretical.

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

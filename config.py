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
# MEASURED 23 Sept 2026 (sign.json, now test_data/real_2026-09-23_sign_check.json):
# the LIDAR is mounted UPSIDE DOWN, and an object placed ~30 cm to the robot's
# RIGHT read raw 272 deg -> the raw angles run counter-clockwise -> SIGN = -1.
LIDAR_ANGLE_SIGN = -1                # +1 or -1
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

# --- Field -----------------------------------------------------------------
# Distance between the outer wall and the island wall. Rulebook (section 8,
# Obstacle Challenge): always 1000 mm (+/- 10 at the International Final).
# Agreed (23 Sept): use the RULEBOOK value, with a margin of error
# (LANE_WIDTH_TOLERANCE_MM) wide enough for real fields. Every
# "d(90) + d(270) = lane width" check uses it and the mock world is built with it.
LANE_WIDTH_MM = 1000.0
# d(90) + d(270) must equal LANE_WIDTH_MM within this margin, otherwise a side
# reading isn't the wall (or the robot isn't in a lane) and initialisation
# refuses rather than trusts it. 100 mm: the practice field measured 926-934 mm
# (23 Sept scan), i.e. ~70 mm under the rulebook, and still passes with ~25 mm
# to spare; a side reading that ISN'T the wall (a pillar -- they stand >= 400 mm
# from either wall -- or something seen through an opening) is off by several
# hundred mm and is still refused. (Was 40 mm before the real scans.)
LANE_WIDTH_TOLERANCE_MM = 100.0

# --- Initialisation (LIDAR, once, robot stationary at its start pose) -----
# x, y and the seat check take the robot's yaw as exactly 0 (agreed): LIDAR
# 0 deg is the lane's grid north. See lane_init.py / direction_detect.py.

# Side walls -- direction_detect.measure_side_wall(). Used for x, for the
# lane-width check and by the gap test (approved Sept 23, replacing the 2-deg
# ray median so a pillar beside the LIDAR can't stand in for the wall):
#   1. take the returns within +/- SIDE_WALL_HALF_DEG of 90 (right) / 270 (left);
#   2. histogram their perpendicular distance |s| in SIDE_WALL_BIN_MM bins;
#      the WALL is the FARTHEST peak with at least SIDE_WALL_MIN_POINTS returns
#      (a pillar is always nearer than the wall behind it);
#   3. fit a straight line (total least squares) to the returns within
#      SIDE_WALL_BAND_MM of that peak, dropping returns more than
#      SIDE_WALL_INLIER_MM off the line and refitting (up to 3 times);
#   4. d(90) / d(270) = where the 90 / 270 ray meets that line.
SIDE_WALL_HALF_DEG = 30.0
SIDE_WALL_BIN_MM = 20.0
SIDE_WALL_BAND_MM = 100.0
SIDE_WALL_INLIER_MM = 30.0
SIDE_WALL_MIN_POINTS = 10
# Diagnostics only (direction_detect.side_ray_distance): the old 2-deg median.
SIDE_RAY_HALF_WINDOW_DEG = 2.0

# Direction (gap) test -- direction_detect.py.
GAP_MARGIN_MM = 80.0        # a return this much beyond the side-wall line = "passed through"
GAP_OPEN_MIN_MM = 500.0     # the gap side needs at least this much opening (along the lane)
GAP_CLOSED_MAX_MM = 150.0   # ...and the other side at most this much
GAP_MIN_ANGLE_FROM_FWD_DEG = 1.0   # rays closer to dead-ahead than this are skipped (sin ~ 0)
GAP_FIT_AGREE_DEG = 2.0     # the two side walls are parallel: fitted tilts must agree within this

# y: front-wall fan -- lane_init.measure_y().
FRONT_FAN_HALF_DEG = 30.0   # returns within +/- this of 0 deg are considered
FRONT_BAND_MM = 40.0        # the front wall = returns within this of the farthest forward distance

# --- STM32 link: BNO08x heading + drive-motor hall encoder (checkpoint B) ----
# Format (decision #17): "$IMU,<seq>,<t_ms>,<enc>,<yaw>\n" at 100 Hz. See stm32_link.py.
IMU_PORT = "/dev/ttyACM0"    # STM32 native USB (CDC). Confirm with `ls /dev/ttyACM*`;
                             # /dev/serial/by-id/... is safer (ACM numbering can change).
IMU_BAUDRATE = 115200        # ignored by USB CDC; pyserial wants a number
IMU_STALE_S = 0.2            # no line for this long -> link status "stale"
# Owner (23 Sept): "IMU clockwise reads negative" -> the chip's yaw DEcreases when
# the robot turns clockwise; the tracker wants clockwise-positive, so -1.
IMU_YAW_SIGN = -1
# Owner (23 Sept): TICKS_PER_CM = 14.853 (the STM32's cumulative `enc` counts
# these ticks). 1 tick = 10 / 14.853 = 0.6733 mm.
ENCODER_TICKS_PER_CM = 14.853
# Plausibility guard: an encoder step implying more than this speed is treated
# as a glitch (or an STM32 restart) and not integrated; the event is logged.
MAX_SPEED_MM_S = 3000.0

# --- Lane tracker (checkpoint B) --------------------------------------------
# Starting heading: the placement yaw measured by the direction test's wall fit
# (decision #35, tracker only -- initialisation's x/y/seats keep yaw 0).
# Turn rule (decision #25): a turn is made when the heading has rotated at least
# TURN_MIN_DEG toward the round direction AND tracked y >= TURN_GATE_Y_MM, or at
# least TURN_FAILSAFE_DEG alone. Turns against the round direction are ignored.
TURN_MIN_DEG = 45.0
TURN_GATE_Y_MM = 2000.0
TURN_FAILSAFE_DEG = 80.0
# Entry-corner seat re-check (decisions #15, #19, #20, #23, #26): lap 1 only,
# lanes after the start lane, every LIDAR frame while tracked y < RECHECK_Y_MAX_MM
# and |heading| <= RECHECK_ALIGN_DEG; first decided verdict per seat is kept.
RECHECK_Y_MAX_MM = 1000.0
RECHECK_ALIGN_DEG = 20.0
# De-skew of the re-check's LIDAR frames (decision #38, deskew.py / timing.py).
# A frame is one ~100 ms revolution; each return is moved to where the current
# pose would see it, using the tracked pose at the time it was measured.
# DESKEW_HISTORY_S: how long the tracked-pose history is kept. Returns older
# than this (a bucket not refreshed since -- no return there any more) are dropped.
DESKEW_HISTORY_S = 0.5
# LIDAR_TIME_OFFSET_S: how much later a LIDAR return reaches the Pi than an
# STM32 line does, each counted from when it was measured (seconds; + = the
# LIDAR is later). The de-skew takes a return as measured at
# (its sweep time - this). MEASURE IT on the robot: python3 measure_lidar_delay.py --real
# (docs/CHANGES.md section 11). 0 until measured.
LIDAR_TIME_OFFSET_S = 0.0

# --- Pillar colour ID, OV5647 fisheye (checkpoint D, decisions #53-#65) -----
# The camera only answers "RED or GREEN?" for a seat the LIDAR already called
# PRESENT (colour_id.py). Everything below marked UNMEASURED must be set on the
# robot (docs/CHANGES.md section 15.8); until then colour ID is not trusted.
CAMERA_ENABLED = True               # real mode: open the camera (Picamera2) in the dashboard
# Lever arm (#53): lens position relative to the pose reference point, like
# LIDAR_OFFSET_*. UNMEASURED -- placeholder = the LIDAR's own offsets.
CAMERA_OFFSET_FORWARD_MM = LIDAR_OFFSET_FORWARD_MM   # <-- MEASURE
CAMERA_OFFSET_LATERAL_MM = LIDAR_OFFSET_LATERAL_MM   # <-- MEASURE (+ = LEFT, as the LIDAR)
# Lens height above the floor (#65). UNMEASURED -- placeholder.
CAMERA_HEIGHT_MM = 150.0                             # <-- MEASURE
# Mount (#54): optical axis horizontal, facing forward; body upside-down ->
# every frame is rotated 180 deg in exactly one place (colour_id.correct_frame).
CAMERA_ROTATE_180 = True
# +1: a seat to the robot's right (+bearing) appears right of centre in the
# corrected frame; -1: mirrored. UNMEASURED -- bench-check (section 15.8).
CAMERA_BEARING_SIGN = +1                             # <-- BENCH-CHECK
# Calibration (#58): cv2.fisheye (equidistant, 4 coefficients) at 640x480.
# The camera must run in the same mode it was calibrated in.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_K = [[383.20119761034823, 0.0, 330.52849471706367],
            [0.0, 383.4524308500294, 228.5909870627763],
            [0.0, 0.0, 1.0]]
CAMERA_D = [0.05460518934568049, -0.3105792203926291, 0.538008476718999, -0.3104152610068569]
# How much later a frame's timestamp is than the moment it shows (s), like
# LIDAR_TIME_OFFSET_S. UNMEASURED -- 0.
CAMERA_TIME_OFFSET_S = 0.0
# Retry policy (#56), proposed values: the window opens at the first frame in
# which the seat is in view (#63) and allows up to MAX_ATTEMPTS in-view frames
# within WINDOW_S; first confident read wins, otherwise UNKNOWN.
COLOR_ID_WINDOW_S = 0.3
COLOR_ID_MAX_ATTEMPTS = 5
# Classification (15.4) -- PLACEHOLDERS until real-lighting calibration.
COLOR_ID_MIN_FRACTION = 0.30        # winning colour's share of the ROI's pixels
COLOR_ID_MARGIN_RATIO = 2.0         # ... and at least this many times the other colour's
COLOR_ID_ROI_MARGIN_FACTOR = 1.5    # ROI = the pillar's projected box widened by this
# OpenCV HSV (H 0-179, S and V 0-255). Red wraps around 0/180.
COLOR_RED_HUE = ((0, 10), (170, 179))
COLOR_GREEN_HUE = ((40, 85),)
COLOR_MIN_SAT = 80
COLOR_MIN_VAL = 50

# --- Dashboard / server ---------------------------------------------------
DASHBOARD_HOST = "0.0.0.0"
DASHBOARD_PORT = 5056
STREAM_HZ = 10

# --- Mode switch -----------------------------------------------------------
# "mock"  -> simulated LIDAR (and, from checkpoint B, a simulated STM32 feed),
#            no hardware needed.
# "real"  -> real rplidarc1 device on LIDAR_PORT.
MODE = "real"

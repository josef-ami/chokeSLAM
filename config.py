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
# Checkpoint E (decision #80): the pose reference point is the REAR-AXLE
# MIDPOINT, and both values are derived from the CAD model (ASMB.3mf / ASMB.step,
# 23 Sept): the RPLIDAR C1's spin axis (circle fit on the upside-down turret)
# sits 134.6 mm ahead of the rear-axle midpoint and 0.7 mm to its right.
LIDAR_OFFSET_FORWARD_MM = 134.6      # from CAD
LIDAR_OFFSET_LATERAL_MM = -0.7       # from CAD (0.7 mm to the RIGHT)

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

# Checkpoint E PROPOSAL (awaiting approval; default False = decision #4, yaw 0):
# take the robot's yaw from the direction test's wall fit (already used for the
# tracker's starting heading, #35) for x, y and the seat check too. Found in the
# closed-loop simulation: with yaw 0, y is off by ~15 mm per degree of
# placement yaw (84 mm at 5.5 deg), which later moves the camera's box off pillars.
INIT_USE_WALL_YAW = False

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
LIDAR_TIME_OFFSET_S = -0.05

# --- Pillar colour ID, OV5647 fisheye (checkpoint D, decisions #53-#65) -----
# The camera only answers "RED or GREEN?" for a seat the LIDAR already called
# PRESENT (colour_id.py). Everything below marked UNMEASURED must be set on the
# robot (docs/CHANGES.md section 15.8); until then colour ID is not trusted.
CAMERA_ENABLED = True               # real mode: open the camera (Picamera2) in the dashboard
# Lever arm (#53): lens position relative to the pose reference point, like
# LIDAR_OFFSET_*. UNMEASURED -- placeholder = the LIDAR's own offsets.
# Checkpoint E (#80): from CAD. The camera module itself is not in the model; its
# mount is ("Camera mount": four holes at the Pi-camera pattern, 21 x 12.5 mm,
# around an 18 mm lens hole). Lens = the hole's centre on the mount's front face:
# 139.9 mm ahead of the rear-axle midpoint, on the centreline, 127 mm above the floor.
CAMERA_OFFSET_FORWARD_MM = 139.9                     # from CAD (mount)
CAMERA_OFFSET_LATERAL_MM = 0.0                       # from CAD (mount)
# Lens height above the floor (#65), from CAD (mount).
CAMERA_HEIGHT_MM = 127.0                             # from CAD (mount)
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

# --- Vehicle geometry (checkpoint E, decision #80: derived from ASMB.3mf / .step) ---
# CAD frame: y up (floor at y = -14.2), front toward -z, left toward -x (the
# "Left Front Tire" etc. names fix the handedness). Reference point: rear-axle
# midpoint (tyre centres x = -50.1 / +50.8, rear z = 140.35, front z = 4.5).
WHEELBASE_MM = 135.9                 # front axle z 4.5 -> rear axle z 140.35
TRACK_MM = 101.0                     # tyre centre to tyre centre
WHEEL_DIAMETER_MM = 54.0
# Collision footprint = every part below 100 mm (wall and pillar height), in the
# robot frame about the rear-axle midpoint: 228.7 x 114.1 mm.
BODY_FRONT_MM = 197.3                # ahead of the rear axle
BODY_REAR_MM = 31.5                  # behind it
BODY_HALF_WIDTH_MM = 57.2            # to either side (-56.8 .. +57.3 about x = 0.3)
# Full outline including the wing (169-180 mm high, passes over walls and
# pillars): used for the "projection inside the section" rules only.
OUTLINE_FRONT_MM = 197.3
OUTLINE_REAR_MM = 86.2               # wing trailing edge
OUTLINE_HALF_WIDTH_MM = 87.0         # wing span 174 mm
OUTLINE_LENGTH_MM = OUTLINE_FRONT_MM + OUTLINE_REAR_MM   # 283.5 -> parking lot 1.5 x = 425 mm
# STEERING LOCK -- PLACEHOLDER (decision #78). NOT in the CAD (a static model has
# no lock). Converted from the report's measured outer-body turning radii
# (270 mm left, 249.9 mm right) with THIS car's footprint (outer front corner
# 197.3 ahead, 57.2 out): rear-axle radius = sqrt(R_ob^2 - 197.3^2) - 57.2
# -> 127.4 mm left / 96.4 mm right -> lock = atan(135.9 / R) = 46.8 / 54.6 deg.
# MEASURE ON THE ROBOT (full-lock circle diameter each way) and replace.
STEER_LOCK_LEFT_DEG = 46.8           # PLACEHOLDER
STEER_LOCK_RIGHT_DEG = 54.6          # PLACEHOLDER
# Servo mapping for the drive firmware (v7's values: straight 76.5, left stop 20,
# right stop 140 servo degrees; below straight steers LEFT). Road-wheel angle is
# taken as linear in servo angle between straight and each stop -- a PLACEHOLDER
# until the wheel angle is measured at both stops.
SERVO_STRAIGHT_DEG = 76.5
SERVO_LEFT_STOP_DEG = 20.0
SERVO_RIGHT_STOP_DEG = 140.0

# --- Path planner (checkpoint E, vg_planner.py) -----------------------------
# The planner's minimum radius is the lock radius x this factor, so the follower
# keeps steering authority to correct errors on an arc.
PLAN_RADIUS_FACTOR = 1.25
PLAN_INFLATION_MM = 95.0             # node inflation: half-width 57.2 + clearance 30 + ~8 mm
PLAN_CLEARANCE_MM = 30.0             # swept-footprint check: body + this must be free
PLAN_INFLATION_STEP_MM = 15.0        # an obstacle the swept check hits is inflated by this and re-planned
PLAN_MAX_ITER = 8
PLAN_MAX_TURN_DEG = 150.0            # no single fillet turns more than this
# Parking lot (rulebook Fig. 4 / 8d, decision in CHANGES E): against the outer wall,
# 200 mm deep, 1.5 x OUTLINE_LENGTH_MM long between the two 20 mm limitations, at
# the end of the start section a CCW car reaches last:
#   CCW: y 2000 - 20 - lot .. 2000,  CW: y 1000 .. 1000 + 20 + lot   (lane frame)
PARKING_DEPTH_MM = 200.0
# Rulebook Fig. 8e: once the lot is placed, every sign of the start section is
# moved to the seat nearer the inner wall, so the start lane's OUTER seats
# (x = 400) are always empty. True: an UNKNOWN outer seat of the start lane is
# not treated as a possible pillar (decision #72 applies to every other seat).
START_LANE_OUTER_SEATS_EMPTY = True
PARKING_BARRIER_MM = 20.0

# --- Mission (checkpoint E, mission.py) --------------------------------------
SPEED_LAP1_MM_S = 500.0              # decision #79
SPEED_LAPS23_MM_S = 800.0
SPEED_MIN_MM_S = 120.0
LAT_ACCEL_MM_S2 = 1500.0             # arc speed limit v <= sqrt(a * R)
DECEL_MM_S2 = 1200.0                 # braking into a stop
VIEW_X_MM = (500.0, 350.0, 650.0)    # viewing-pose candidates (new lane's x, first = preferred)
VIEW_Y_MM = 500.0                    # viewing pose: new lane's y (the corner square's centre line)
LOOK_SETTLE_S = 0.6                  # at a viewing pose: minimum stationary time before planning
LOOK_TIMEOUT_S = 2.5                 # ... and the most it waits for verdicts / colours
COLOR_LOOK_DIST_MM = 800.0           # stop when a PRESENT pillar of unknown colour is this far ahead (rear axle)
COLOR_LOOK_RETRIES = 3               # colour re-requests before it is passed as "either" (logged)
REPLAN_DEVIATION_MM = 60.0           # laps 2-3: re-plan when the tracked pose is this far off the path
REVERSE_SPEED_MM_S = 200.0           # E-A4 (awaiting approval): backing up straight when no forward path exists
REVERSE_STEPS_MM = (150.0, 300.0, 450.0)
REVERSE_MAX_PER_STOP = 2
REVERSE_CLEARANCE_MM = 5.0           # a short straight line from a standstill (pose well known)
FINISH_MARGIN_MM = 30.0              # stop with the whole outline this far inside the start section
# Follower (pure pursuit on the rear axle, decision #67)
PP_LOOKAHEAD_S = 0.30                # lookahead = speed x this ...
PP_LOOKAHEAD_MIN_MM = 110.0          # ... clamped to this range
PP_LOOKAHEAD_MAX_MM = 260.0
DRIVE_HZ = 50.0                      # DRIVE frames per second to the STM32
# Steering law: "rwf" = rear-wheel feedback (curvature feed-forward + lateral and
# heading error feedback), "pp" = pure pursuit. See follower.py.
FOLLOWER_MODE = "rwf"
RWF_LENGTH_MM = 150.0                # a lateral error dies out over about this distance
RWF_DAMPING = 0.8
RWF_PREVIEW_S = 0.08                 # servo lag + link latency
# Entry re-check extension (Q7a, decision #81): past RECHECK_Y_MAX_MM the re-check
# stays open (lap 1) while a still-UNKNOWN seat is at least this far ahead.
RECHECK_AHEAD_MIN_MM = 250.0
RECHECK_EXTEND = True
# Start-lane re-check when lap 1 comes back to it (Q7b, decision #82).
RECHECK_START_LANE_ON_RETURN = True

# --- Dashboard / server ---------------------------------------------------
DASHBOARD_HOST = "0.0.0.0"
DASHBOARD_PORT = 5056
STREAM_HZ = 10

# --- Mode switch -----------------------------------------------------------
# "mock"  -> simulated LIDAR (and, from checkpoint B, a simulated STM32 feed),
#            no hardware needed.
# "real"  -> real rplidarc1 device on LIDAR_PORT.
MODE = "real"

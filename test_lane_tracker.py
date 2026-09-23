"""
Verification for lane_tracker.py (and, end to end, the whole checkpoint-B
chain: simulated STM32 lines -> stm32_link parser -> tracker -> turns ->
lane switches -> entry-corner re-checks), against ground truth.

Unit tests feed the tracker synthetic STM32 samples (rotate in place / drive
straight) from a real initialisation of a simulated start. End-to-end tests use
simulation.run_mock: a robot driving 3 laps of a rounded-square path around the
island with a gentle weave, the tracked pose mapped back to the mat and
compared with the true pose at every 10 ms step.

Run:  python3 test_lane_tracker.py
"""
from __future__ import annotations

# The simulated worlds are RULEBOOK fields.
import config
config.LANE_WIDTH_MM = 1000.0

import math

import lane_init as li
import simulation as sim
from lane_tracker import LaneTracker
from scan_processing import clean_and_project
from stm32_link import ImuSample


def real_init(direction="CCW", section="S", x=500.0, y=1400.0, yaw=0.0):
    robot = sim.MockRobotSimulator(section, direction, x, y, yaw, pillars=[], n_points=720,
                                   noise_std_mm=0.0, dropout_prob=0.0)
    res = li.initialise(clean_and_project(robot.current_scan(), config.LIDAR_ANGLE_SIGN,
                                          config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
    assert res.ok, res.reason
    return res


class Feeder:
    """Synthetic STM32: the robot rotates in place or drives straight; the chip
    yaw follows IMU_YAW_SIGN so the tracker sees clockwise-positive heading."""

    def __init__(self, heading0=0.0, chip_zero=170.0):
        self.seq, self.t, self.enc, self.heading, self.zero = 0, 0, 0.0, heading0, chip_zero

    def sample(self):
        self.seq += 1
        self.t += 10
        yaw = (config.IMU_YAW_SIGN * self.heading + self.zero + 180.0) % 360.0 - 180.0
        return ImuSample(self.seq, self.t, int(round(self.enc)), yaw)

    def rotate(self, trk, deg, step=1.0):
        n = max(1, int(abs(deg) / step))
        for _ in range(n):
            self.heading += deg / n
            trk.on_imu(self.sample())

    def drive(self, trk, mm, step=5.0):
        n = max(1, int(abs(mm) / step))
        for _ in range(n):
            self.enc += (mm / n) / 10.0 * config.ENCODER_TICKS_PER_CM
            trk.on_imu(self.sample())


def _new(direction, **kw):
    init = real_init(direction, **kw)
    f = Feeder()
    return LaneTracker(init, f.sample()), f, init


def test_straight_and_handedness():
    for d in ("CCW", "CW"):
        trk, f, init = _new(d)
        x0, y0 = trk.x, trk.y
        f.drive(trk, 400.0)
        assert abs(trk.y - (y0 + 400)) < 1.0 and abs(trk.x - x0) < 0.5, (trk.x, trk.y)
        f.rotate(trk, 10.0)                       # 10 deg clockwise = toward the robot's right
        f.drive(trk, 200.0)
        dx = trk.x - x0
        want = -200 * math.sin(math.radians(10)) if d == "CCW" else 200 * math.sin(math.radians(10))
        assert abs(dx - want) < 1.0, (d, dx, want)
    print("PASS  test_straight_and_handedness  straight: y += distance exactly; turning right moves x toward the "
          "outer wall for CCW (-35 mm) and toward the island for CW (+35 mm)")


def test_yaw_wraps_without_jumps():
    init = real_init("CCW")
    f = Feeder(chip_zero=178.0)                   # chip yaw sits just below +180 from the start
    trk = LaneTracker(init, f.sample())
    for _ in range(3):
        f.rotate(trk, 4.0)                        # crosses +180 -> -180 on the chip
        f.rotate(trk, -4.0)
    assert abs(trk.psi) < 1e-6, trk.psi
    print("PASS  test_yaw_wraps_without_jumps  chip yaw crossing +/-180 six times leaves the heading unchanged")


def test_turn_rule():
    rows = []
    for d in ("CCW", "CW"):
        toward = -1.0 if d == "CCW" else 1.0      # the round direction's turning sense
        # swerve 40 deg toward the island at y ~1400: no turn
        trk, f, _ = _new(d)
        f.rotate(trk, 40 * toward); f.rotate(trk, -40 * toward)
        rows.append((d, "40 deg at y 1400", trk.lane_index == 0))
        # 60 deg at y ~1400: below the y gate, below the failsafe -> no turn
        f.rotate(trk, 60 * toward); f.rotate(trk, -60 * toward)
        rows.append((d, "60 deg at y 1400 (y gate)", trk.lane_index == 0))
        # 90 deg AGAINST the round direction: never a turn
        f.rotate(trk, -90 * toward); f.rotate(trk, 90 * toward)
        rows.append((d, "90 deg against the round direction", trk.lane_index == 0))
        # drive past the island's end, then 50 deg toward: turn
        f.drive(trk, 2150 - trk.y)
        f.rotate(trk, 50 * toward)
        rows.append((d, "50 deg at y 2150", trk.lane_index == 1))
        # right after the switch: psi ~ +/-40 in the new lane, y ~ old x: no second turn
        f.rotate(trk, 40 * toward)
        rows.append((d, "finish the corner: no second turn", trk.lane_index == 1 and abs(trk.psi) < 1.0))
        # failsafe: 85 deg toward at y ~ 500 (inside the new lane's entry corner)
        trk2, f2, _ = _new(d)
        f2.rotate(trk2, 85 * toward)
        rows.append((d, "85 deg at y 1400 (failsafe)", trk2.lane_index == 1))
    bad = [r for r in rows if not r[2]]
    assert not bad, bad
    print(f"PASS  test_turn_rule                {len(rows)} cases, both directions: swerves (40/60 deg before the "
          f"island ends) and turns against the direction ignored; 50 deg past y 2000 and the 85 deg failsafe switch lanes once")


def test_start_heading_from_wall_fit():
    """Robot placed 3.5 deg clockwise of parallel; it straightens up, then drives
    2 m along the lane. With the wall-fit start heading x must not move; with
    psi0 = 0 (the old assumption) it would drift ~122 mm."""
    for d in ("CCW", "CW"):
        init = real_init(d, yaw=3.5)
        f = Feeder()
        trk = LaneTracker(init, f.sample())
        assert abs(trk.psi0 - 3.5) < 0.3, trk.psi0
        x0 = trk.x
        f.rotate(trk, -trk.psi0)                  # the robot straightens: true yaw 3.5 -> 0
        f.drive(trk, 2000.0)
        drift = abs(trk.x - x0)
        assert drift < 12.0, (d, drift)
        print(f"PASS  test_start_heading_from_wall  {d}: psi0 {trk.psi0:+.2f} (true +3.50); x drift over 2 m "
              f"{drift:.1f} mm (psi0 = 0 would give {2000 * math.sin(math.radians(3.5)):.0f} mm)")


def test_glitch_and_reset():
    trk, f, _ = _new("CCW")
    f.drive(trk, 300.0)
    x, y = trk.x, trk.y
    f.enc += 500.0 / 10.0 * config.ENCODER_TICKS_PER_CM        # a 500 mm jump in one 10 ms sample
    trk.on_imu(f.sample())
    assert abs(trk.y - y) < 1e-6 and trk.events[-1].kind == "glitch", trk.events[-1]
    f.seq = 0                                                   # the STM32 restarts
    f.enc = 0.0
    trk.on_imu(f.sample())
    assert trk.events[-1].kind == "reset" and abs(trk.y - y) < 1e-6
    f.drive(trk, 100.0)
    assert abs(trk.y - (y + 100)) < 1.0 and abs(trk.x - x) < 0.5, (trk.x, trk.y)
    print("PASS  test_glitch_and_reset         a 500 mm encoder jump and an STM32 restart are logged and not "
          "integrated; tracking continues from the kept pose")


def test_end_to_end():
    cases = []
    for d in ("CCW", "CW"):
        for slot in range(4):
            for seed, extra in ((1, {}), (2, {"placement_yaw_deg": 2.0}), (3, {"drop_prob": 0.05}),
                                (4, {"enc_scale_err": 0.01})):
                r = sim.run_mock(d, start_slot=slot, seed=seed + 10 * slot, **extra)
                assert r["ok"], (d, slot, seed, r["init"].reason)
                trk = r["tracker"]
                frozen = [e for e in trk.events if e.kind == "recheck_frozen"]
                wrong = sum(v["wrong"] for v in r["seats"].values())
                cases.append((d, slot, seed, extra, r["turns"], r["pos_err_max_mm"], r["pos_err_median_mm"],
                              wrong, len(frozen), r["seats"]))
                assert r["turns"] == 12, (d, slot, seed, r["turns"])
                assert wrong == 0, (d, slot, seed, r["seats"])
                assert len(frozen) == 3, (d, slot, seed, [e.detail for e in frozen])     # lanes 2-4 of lap 1 only
                assert trk.lanes[0].source == "init" and all(trk.lanes[k].source == "entry" for k in (1, 2, 3))
    worst = max(c[5] for c in cases)
    med = sorted(c[6] for c in cases)[len(cases) // 2]
    decided = sum(v["right"] for c in cases for k, v in c[9].items() if k != 0)
    unknown = sum(v["unknown"] for c in cases for k, v in c[9].items() if k != 0)
    print(f"PASS  test_end_to_end               {len(cases)} runs x 3 laps (both directions, all 4 start lanes; "
          f"placement yaw 2 deg / 5% lines lost / 1% encoder error variants): 12/12 turns every run, "
          f"0 wrong seats; entry re-checks decided {decided}, left {unknown} unknown; "
          f"tracked-vs-true position error median {med:.1f} mm, worst {worst:.1f} mm")
    return cases


def _lap1_rechecks(speed, **kw):
    right = wrong = unknown = 0
    for d in ("CCW", "CW"):
        for slot in range(4):
            for seed in (1, 2):
                r = sim.run_mock(d, start_slot=slot, seed=seed + 10 * slot, laps=1, speed_mm_s=speed, **kw)
                assert r["ok"], (d, slot, seed)
                for k, v in r["seats"].items():
                    if k != 0:
                        right += v["right"]; wrong += v["wrong"]; unknown += v["unknown"]
    return right, wrong, unknown


def test_deskew_end_to_end():
    """P9 / decision #38. The lap-1 entry re-checks (16 runs: both directions,
    all 4 start lanes, 2 seeds; 48 lanes x 6 seats = 288 verdicts) with each
    LIDAR frame as ONE REAL 100 ms REVOLUTION, the robot moving through it.
    Requirement: with the de-skew and correct timing, 0 wrong at 600 and 1000
    mm/s -- also when every frame reaches the tracker 250 ms after it was
    taken (a busy Pi): the frame is judged from the pose at its end; and at
    450 ms late, when the 0.5 s pose history no longer covers the whole frame,
    frames are skipped (UNKNOWN), never judged on a partial revolution. Shown for
    comparison: instantaneous frames (the old test model), no de-skew, and
    de-skew with the LIDAR time offset mis-measured."""
    rows = []
    for label, kw, must_be_clean in (
            ("instantaneous frame (old model)       ", {"lidar_sweep": False}, True),
            ("real sweep, NO de-skew                ", {"lidar_sweep": True, "deskew": False}, False),
            ("real sweep, de-skew                   ", {"lidar_sweep": True}, True),
            ("real sweep, de-skew, offset off +10 ms", {"lidar_sweep": True, "lidar_stamp_error_s": 0.010}, False),
            ("real sweep, de-skew, offset off -10 ms", {"lidar_sweep": True, "lidar_stamp_error_s": -0.010}, False),
            ("real sweep, de-skew, offset off +30 ms", {"lidar_sweep": True, "lidar_stamp_error_s": 0.030}, False),
            ("real sweep, de-skew, offset off -30 ms", {"lidar_sweep": True, "lidar_stamp_error_s": -0.030}, False),
            ("real sweep, de-skew, frames 250 ms late ", {"lidar_sweep": True, "lidar_delay_s": 0.25}, True),
            ("real sweep, de-skew, frames 450 ms late ", {"lidar_sweep": True, "lidar_delay_s": 0.45}, True)):
        for speed in (600.0, 1000.0):
            right, wrong, unknown = _lap1_rechecks(speed, **kw)
            if must_be_clean:
                assert wrong == 0, (label, speed, wrong)
            rows.append((label, speed, right, wrong, unknown))
    print("PASS  test_deskew_end_to_end       lap-1 entry re-checks, 16 runs, 288 verdicts per row; "
          "required: 0 wrong with the de-skew and correct timing, also with frames 250 / 450 ms late")
    for label, speed, right, wrong, unknown in rows:
        print(f"      {label}  {speed:5.0f} mm/s:  right {right:3d}  WRONG {wrong:3d}  unknown {unknown:3d}")
    return rows


def test_coverage_hole_is_not_empty():
    """P10: a sector with no returns over a seat (the revolution's seam while
    the robot turns) must not let the returns beside it call the seat EMPTY."""
    import seat_occupancy as so
    for d in ("CCW", "CW"):
        pillars = sim.rulebook_pillars("S", d, [4])                     # far-outer seat occupied
        robot = sim.MockRobotSimulator("S", d, 500.0, 900.0, 0.0, pillars=pillars, n_points=720,
                                       noise_std_mm=0.0, dropout_prob=0.0)
        pts = clean_and_project(robot.current_scan(), config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
        init = li.initialise(pts)
        trk = LaneTracker(init, Feeder().sample())
        p = trk.params
        full = so.detect_seat_occupancy(pts, 500.0, 900.0, d, 0.0, p)
        c = [r for r in full if r.seat.index == 4][0]
        assert c.state is so.Occupancy.OCCUPIED, c.reason
        # a 5 deg hole over the pillar (seam gap while turning at ~50 deg/s)
        holed = [q for q in pts if abs((q.angle_deg - c.predicted_lidar_angle_deg + 180) % 360 - 180) > 2.5]
        raw = so.detect_seat_occupancy(holed, 500.0, 900.0, d, 0.0, p)
        r4 = [r for r in raw if r.seat.index == 4][0]
        rec = type("Rec", (), {"empty_downgraded": 0})()                # stands in for the lane record
        checked = trk._coverage_check(holed, raw, rec)
        k4 = [r for r in checked if r.seat.index == 4][0]
        assert r4.state is so.Occupancy.EMPTY, r4.reason                 # the detector alone is fooled
        assert k4.state is so.Occupancy.UNKNOWN and rec.empty_downgraded >= 1, k4.reason
        # a complete frame of a truly empty seat keeps its EMPTY
        robot2 = sim.MockRobotSimulator("S", d, 500.0, 900.0, 0.0, pillars=[], n_points=720,
                                        noise_std_mm=0.0, dropout_prob=0.0)
        pts2 = clean_and_project(robot2.current_scan(), config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
        e = trk._coverage_check(pts2, so.detect_seat_occupancy(pts2, 500.0, 900.0, d, 0.0, p), rec)
        assert [r for r in e if r.seat.index == 4][0].state is so.Occupancy.EMPTY
    print("PASS  test_coverage_hole            a 5 deg hole over an occupied seat: the detector alone says EMPTY, the "
          "coverage check makes it UNKNOWN; a fully covered empty seat stays EMPTY (both directions)")


if __name__ == "__main__":
    test_straight_and_handedness()
    test_yaw_wraps_without_jumps()
    test_turn_rule()
    test_start_heading_from_wall_fit()
    test_glitch_and_reset()
    test_end_to_end()
    test_deskew_end_to_end()
    test_coverage_hole_is_not_empty()
    print("\nAll lane-tracker checks passed.")

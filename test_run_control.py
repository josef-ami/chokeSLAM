"""
Checkpoint F: start / stop / restart on the one start button, end to end.

    python3 test_run_control.py

The firmware's run logic is the REAL one: a virtual STM32 is compiled on the
host from firmware/drive_bridge/drive_protocol.h (Parser, Button, Run, decide,
formatStatus, exactly as drive_bridge.ino wires them) and stepped in lockstep
with the REAL Pi side (run_control.RunSupervisor -> drive_link -> lane_init,
LaneTracker, Mission) on a simulated car and rulebook layout (the simulator's
LIDAR, $IMU and camera). The button is a raw pin level, so presses go through
the debounce.

Scenario (one continuous session, as on the field):
    1. car at rest in its start zone -> the Pi initialises and reports READY
    2. a press while the car is being carried is ignored (Pi not ready)
    3. press -> run 1 drives; press again -> STOPPED: the motor is off in the
       same firmware step although the Pi is still sending motion
    4. the car is carried back (wheels off the floor: yaw changes, encoder
       does not) -> not ready while moving; re-initialised at rest -> READY
    5. press -> run 2 drives the whole round to DONE; the Pi sends RUN_OVER ->
       FINISHED; 3 laps judged from the true motion, stopped in the start section
    6. at rest after the finish -> READY again; press -> run 3 starts (restart
       from FINISHED); press -> STOPPED
    7. a fresh Pi program finding the STM32 RUNNING ends that run (RUN_OVER)
"""
from __future__ import annotations

import math
import os
import random
import shutil
import subprocess
import tempfile

import numpy as np

import camera_sim
import config
import field_map as fm
import lane_frame as lf
import lane_init as li
import layouts
import seat_occupancy as so
import simulation as sim
from drive_link import DriveLink, RUN_FINISHED, RUN_READY, RUN_RUNNING, RUN_STOPPED
from mission import Mission
from run_control import RunSupervisor
from scan_processing import clean_and_project
from stm32_link import ImuSample, parse_line

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "firmware", "drive_bridge")

# One line in per 20 ms step: "<t_ms> <pin level 1=released> <hex bytes or ->"
# one line out: "$STA,..." + " OUT <motorOn> <wheelDeg> <speed>" (+ " EV <event>")
EMU = r'''
#include "drive_protocol.h"
#include <stdio.h>
#include <string.h>
using namespace drive;
int main() {
  Parser parser; Command cmd; Button button; Run run;
  bool haveCmd = false; uint32_t lastCmdMs = 0;
  char hex[4096];
  unsigned long t; int level;
  setvbuf(stdout, NULL, _IOLBF, 0);
  while (scanf("%lu %d %4095s", &t, &level, hex) == 3) {
    int ev = EV_NONE;
    if (strcmp(hex, "-") != 0) {
      for (size_t i = 0; i + 1 < strlen(hex); i += 2) {
        unsigned b; sscanf(hex + i, "%2x", &b);
        Command c;
        if (parser.feed((uint8_t)b, c)) { cmd = c; haveCmd = true; lastCmdMs = t; if (run.frame(c)) ev = EV_FINISH; }
      }
    }
    // the button is sampled every 10 ms, like drive_bridge.ino's control tick
    for (int k = 0; k < 2; k++) {
      uint32_t tk = t - 10 + 10 * k;
      if (button.update(level, tk)) {
        bool piReady = haveCmd && tk - lastCmdMs <= WATCHDOG_MS && cmd.piReady;
        ev = run.press(piReady);
      }
    }
    Output o = decide(cmd, haveCmd, t - lastCmdMs, run.state == RUNNING);
    char line[96];
    formatStatus(line, sizeof line, cmd.seq, (o.motorOn ? ST_ENABLED : 0) | ST_IMU_OK, run.state, run.runId, 0, 0.0f);
    line[strlen(line) - 1] = 0;
    printf("%s OUT %d %.2f %.1f EV %d\n", line, o.motorOn, o.wheelDeg, o.speedMmps, ev);
  }
  return 0;
}
'''


class VirtualStm32:
    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.sta = None
        self.out = (False, 0.0, 0.0)
        self.ev = 0

    def step(self, t_ms, released, data: bytes):
        self.p.stdin.write(f"{t_ms} {1 if released else 0} {data.hex() if data else '-'}\n")
        self.p.stdin.flush()
        ln = self.p.stdout.readline().split()
        parts = ln[0].split(",")
        self.sta = [int(v) for v in parts[1:]]
        self.out = (ln[2] == "1", float(ln[3]), float(ln[4]))
        self.ev = int(ln[6])
        return self.sta

    def close(self):
        self.p.stdin.close()
        self.p.wait(timeout=5)


class FakeLink:
    """stm32_link.Stm32Link's surface, fed by the test."""
    def __init__(self):
        self.pending, self.written, self.sta = [], bytearray(), None

    def drain(self):
        out, self.pending = self.pending, []
        return out

    def write(self, data):
        self.written += data
        return True

    def status(self):
        sta = None
        if self.sta is not None:
            s = self.sta
            sta = {"seq_ack": s[0], "status": s[1], "age_s": 0.0, "run_state": s[2], "run_id": s[3],
                   "pwm": s[4], "speed_mm_s": s[5]}
        return {"stale": False, "sta": sta}


class World:
    """A rulebook layout, the car's true pose, and the simulated sensors."""
    def __init__(self, layout, seed):
        self.lay, self.rng = layout, random.Random(seed)
        d = layout.direction
        self.pillars, self.barriers = [], []
        for k in range(4):
            sec = layout.slot_section(k)
            for i, col in layout.pillars.get(sec, []):
                s = so.seats()[i]
                self.pillars.append(sim.Pillar(*lf.lane_to_global(sec, d, s.x_mm, s.y_mm), col))
        for (x0, y0, x1, y1) in fm.parking_barriers(d):
            a = lf.lane_to_global(layout.start_section, d, x0, y0)
            b = lf.lane_to_global(layout.start_section, d, x1, y1)
            self.barriers.append((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))
        self.place_at_start()
        self.dist, self.v, self.delta = 0.0, 0.0, 0.0
        self.stm = sim.SimStm32(self.rng, ticks_per_cm=config.ENCODER_TICKS_PER_CM, yaw_noise_deg=0.0,
                                yaw_drift_deg_s=0.0)
        self.cam_rng = np.random.default_rng(seed)
        self.path = []

    def place_at_start(self):
        d, sec = self.lay.direction, self.lay.start_section
        self.gx, self.gy = lf.lane_to_global(sec, d, self.lay.start_x, self.lay.start_y)
        self.brg = lf.yaw_to_heading(0.0, sec, d)

    def sensor(self):
        r = math.radians(self.brg)
        F, L = config.LIDAR_OFFSET_FORWARD_MM, config.LIDAR_OFFSET_LATERAL_MM
        return self.gx + F * math.sin(r) - L * math.cos(r), self.gy + F * math.cos(r) + L * math.sin(r)

    def drive(self, steer, speed, dt):
        self.v += (speed - self.v) * min(1.0, dt / 0.10)
        self.delta += (steer - self.delta) * min(1.0, dt / 0.05)
        if speed == 0.0 and abs(self.v) < 5.0:
            self.v = 0.0
        ds = self.v * dt
        dth = ds * math.tan(math.radians(self.delta)) / config.WHEELBASE_MM
        bm = math.radians(self.brg - math.degrees(dth) / 2)
        self.gx += ds * math.sin(bm)
        self.gy += ds * math.cos(bm)
        self.brg = (self.brg - math.degrees(dth)) % 360.0
        self.dist += ds
        self.path.append((self.gx, self.gy, self.brg))

    def imu(self, t):
        smp, _ = parse_line(self.stm.line(int(round(t * 1000)), self.dist, self.brg))
        return ImuSample(smp.seq, smp.t_ms, smp.enc, smp.yaw_deg, t)

    def scan_timed(self, t):
        lx, ly = self.sensor()
        raw = sim.simulate_scan(lx, ly, self.brg, self.pillars, self.barriers, n_points=720, rng_noise=self.rng,
                                blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                                blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
        # stamped like sim_closed_loop: the LIDAR's clock offset the tracker corrects for
        return [(sim.to_sensor_raw(a), dd, q, t + config.LIDAR_TIME_OFFSET_S) for a, dd, q in raw]

    def lane_pose(self):
        return lf.global_to_lane(self.lay.start_section, self.lay.direction, self.gx, self.gy)


class FakeLidar:
    def __init__(self, world, clock):
        self.w, self.clock = world, clock
        self.cache = (None, None)

    def get_latest_scan_timed(self):
        t = self.clock()
        if self.cache[0] != t:
            self.cache = (t, self.w.scan_timed(t))
        return self.cache[1]


class FakeCamera:
    def __init__(self, world, clock):
        self.w, self.clock = world, clock

    def get_latest_frame(self):
        w = self.w
        cx, cy = camera_sim.camera_global(w.gx, w.gy, w.brg)
        t = self.clock()
        return camera_sim.render(cx, cy, w.brg, w.pillars, w.cam_rng), t, t


MIS = []


def _mission(trk):
    MIS.append(Mission(trk, log=None))
    return MIS[-1]


class Session:
    DT = 0.02

    def __init__(self, exe, world, log):
        self.w, self.t, self.logs = world, 0.0, log
        self.fw = VirtualStm32(exe)
        self.link = FakeLink()
        self.drive = DriveLink(self.link)
        self.sup = self.new_supervisor()
        self.released = True
        self.carry = None             # (t_end, dx, dy, dbrg per step) while being carried

    def new_supervisor(self):
        return RunSupervisor(self.link, self.drive, FakeLidar(self.w, lambda: self.t),
                             FakeCamera(self.w, lambda: self.t),
                             make_mission=_mission, log=self.logs.append)

    def step(self):
        self.t = round(self.t + self.DT, 6)
        on, steer, speed = self.fw.out
        for k in range(2):
            if self.carry is not None:
                _, dx, dy, db = self.carry
                self.w.gx += dx / 2
                self.w.gy += dy / 2
                self.w.brg = (self.w.brg + db / 2) % 360.0
            else:
                self.w.drive(steer if on else 0.0, speed if on else 0.0, self.DT / 2)
            self.link.pending.append(self.w.imu(self.t - self.DT / 2 + k * self.DT / 2))
        if self.carry is not None and self.t >= self.carry[0]:
            self.carry = None
        self.sup.step(self.t)
        data, self.link.written = bytes(self.link.written), bytearray()
        self.link.sta = self.fw.step(int(round(self.t * 1000)), self.released, data)
        return self.fw.out

    def run_for(self, s):
        for _ in range(int(round(s / self.DT))):
            self.step()

    def press(self, hold_s=0.1):
        self.released = False
        self.run_for(hold_s)
        self.released = True
        self.run_for(0.1)

    def state(self):
        return self.link.sta[2], self.link.sta[3]

    def carry_to_start(self, s=2.0):
        """Lift the car and put it back in its start zone over s seconds (the
        wheels do not turn; the yaw wobbles)."""
        d, sec = self.w.lay.direction, self.w.lay.start_section
        tx, ty = lf.lane_to_global(sec, d, self.w.lay.start_x, self.w.lay.start_y)
        tb = lf.yaw_to_heading(0.0, sec, d)
        n = int(round(s / self.DT))
        db = ((tb - self.w.brg + 180) % 360 - 180) / n
        self.carry = (self.t + s, (tx - self.w.gx) / n, (ty - self.w.gy) / n, db)


def _build():
    gxx = shutil.which("g++")
    if gxx is None:
        return None
    d = tempfile.mkdtemp()
    src, exe = os.path.join(d, "emu.cpp"), os.path.join(d, "emu")
    open(src, "w").write(EMU)
    subprocess.check_call([gxx, "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-I", FW, src, "-o", exe])
    return exe


def _layout():
    """The first rulebook layout (fixed seeds) that initialises here and that the
    closed-loop simulator (sim_closed_loop, no noise) completes: this test is
    about starting and stopping runs, the sweeps are about driving them."""
    import sim_closed_loop as scl
    for seed in range(1, 60):
        lay = layouts.draw(random.Random(seed), None)
        w = World(lay, seed)
        raw = [(a, dd, q) for a, dd, q, _ in w.scan_timed(0.0)]
        init = li.initialise(clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
        if init.ok and scl.run(lay, seed=seed, noise="none").ok:
            return lay, seed
    raise AssertionError("no usable layout")


def test_start_stop_restart():
    exe = _build()
    if exe is None:
        print("SKIP  test_start_stop_restart (no g++)")
        return
    lay, seed = _layout()
    logs = []
    s = Session(exe, World(lay, seed), logs)

    # 1. at rest -> READY
    s.run_for(0.6)
    assert not s.drive.ready and s.state() == (RUN_READY, 0), (s.drive.ready, s.state())
    s.run_for(1.2)
    assert s.drive.ready, logs
    # 2. a press while the car is being carried: ignored
    s.carry = (s.t + 1.0, 0.0, 0.0, 0.3)              # turned by hand, 15 deg over 1 s
    s.run_for(0.4)
    assert not s.drive.ready
    s.press()
    assert s.state() == (RUN_READY, 0), s.state()
    s.carry_to_start(0.5)
    s.run_for(2.0)
    assert s.drive.ready, logs
    # 3. run 1, then stop it with the button
    s.press()
    assert s.state() == (RUN_RUNNING, 1) and s.sup.mode == "RUN", (s.state(), s.sup.mode)
    s.w.path = []
    moved = 0.0
    while moved < 300.0 and s.t < 60.0:                  # until it has driven 300 mm (after its first LOOK)
        s.step()
        moved = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(s.w.path[-2:], s.w.path[-1:])) + moved
    assert moved >= 300.0 and s.fw.out[0], (moved, logs[-5:])
    s.released = False
    stop_seen = None
    for _ in range(10):
        on, _, speed = s.step()
        if s.state()[0] == RUN_STOPPED:
            stop_seen = (on, s.sup.mode)
            break
    s.released = True
    assert stop_seen is not None and stop_seen[0] is False, stop_seen    # motor off in the same firmware step
    s.run_for(0.2)
    assert s.sup.mode == "WAIT" and s.sup.runs[-1] == (1, "stopped"), (s.sup.mode, s.sup.runs)
    s.run_for(1.0)
    assert abs(s.w.v) < 1.0
    # 4. carried back, re-initialised at rest
    s.carry_to_start(2.0)
    s.run_for(1.0)
    assert not s.drive.ready
    s.run_for(2.5)
    assert s.drive.ready, logs[-5:]
    # 5. run 2: the whole round
    s.press()
    assert s.state() == (RUN_RUNNING, 2), s.state()
    s.w.path = []
    laps_line = []
    while s.t < 400.0 and s.state()[0] == RUN_RUNNING:
        s.step()
        ly = s.w.lane_pose()
        laps_line.append(ly)
    assert s.state() == (RUN_FINISHED, 2), (s.state(), s.sup.runs, logs[-8:])
    assert s.sup.runs[-1] == (2, "DONE"), (s.sup.runs, [(round(e.t, 1), e.kind, e.detail[:90]) for e in MIS[-1].events][-12:], s.w.lane_pose(), s.w.brg)
    t_run2 = s.t
    # the true motion: about 3 laps of travel, stopped inside the start section
    lx, ly = s.w.lane_pose()
    assert 0 < lx < 1000 and 1000 < ly < 2000, (lx, ly)
    travelled = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(s.w.path, s.w.path[1:]))
    assert 3 * 6000 < travelled < 3 * 12000, travelled
    s.run_for(0.5)
    assert abs(s.w.v) < 1.0
    # 6. READY again where it finished; press -> run 3; press -> STOPPED
    s.run_for(2.0)
    assert s.sup.mode == "WAIT" and s.drive.ready, (s.sup.mode, logs[-5:])
    s.press()
    assert s.state() == (RUN_RUNNING, 3) and s.sup.mode == "RUN", s.state()
    s.run_for(1.0)
    s.press()
    assert s.state() == (RUN_STOPPED, 3) and not s.fw.out[0], s.state()
    s.run_for(0.2)
    assert s.sup.mode == "WAIT"
    # 7. a fresh program finds the STM32 RUNNING: it ends that run
    s.run_for(2.0)
    s.press()
    assert s.state() == (RUN_RUNNING, 4)
    s.sup = s.new_supervisor()
    s.drive.ready = s.drive.run_over = False
    s.run_for(0.3)
    assert s.state() == (RUN_FINISHED, 4) and not s.fw.out[0], s.state()
    s.run_for(0.3)
    assert s.sup.mode == "WAIT" and not s.drive.run_over
    s.fw.close()
    print(f"PASS  test_start_stop_restart  (layout seed {seed}, {lay.direction}) READY at rest; press while carried "
          f"ignored; run 1 stopped by the button with the motor off in the same step; re-initialised after being "
          f"carried back; run 2 DONE -> FINISHED in {t_run2:.0f} s of session time ({travelled / 1000:.1f} m, stopped "
          f"in the start section); READY after the finish, run 3 restarted and stopped; a stale RUNNING run ended")


if __name__ == "__main__":
    test_start_stop_restart()
    print("\nAll run-control checks passed.")

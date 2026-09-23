"""
Verification for the dashboard (checkpoint C): dashboard_server.py's runtime
and routes, live_sim.py, and the page's data, through Flask's test client.

  - mock mode, end to end: nothing until Initialise; then the simulated robot
    drives 3 laps (4x time) and the page's state shows every lane appearing
    at its turn, seats agreeing with the simulated truth, and the drawn robot
    on top of the true one; Re-initialise starts over; the stream serves
    events; Save run writes files that run_track.py --replay replays to the
    same turns.
  - the tuning panel: four groups, edits validated and applied (live ones
    reach the running tracker), Reset restores config.py.
  - real mode with stand-in hardware (child process): the STM32 through a real
    pseudo-terminal, a stand-in lidar_source giving real-sweep timed frames on
    the real clock; Initialise, 1 lap, lanes and seats.

Run:  python3 test_dashboard.py      (about a minute)
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import config
config.LANE_WIDTH_MM = 1000.0              # the simulated world is a rulebook field


def _wrong(state):
    n = 0
    for lane in state["lanes"]:
        truth = set(state["truth"]["seats_by_slot"][str(lane["slot"])])
        for s in lane["seats"]:
            if s["state"] != "unknown" and (s["state"] == "occupied") != (s["index"] in truth):
                n += 1
    return n


def test_mock_end_to_end():
    import dashboard_server as ds
    rt = ds.create_runtime("mock")
    c = ds.app.test_client()
    st = c.get("/api/state").get_json()
    assert not st["tracking"] and st["init"] is None and len(st["live"]) > 300 and st["live_frame"] == "robot", st
    assert c.post("/api/mock", json={"direction": "CW", "start_slot": 1, "seed": 3, "time_scale": 4}).get_json()["ok"]
    assert c.post("/api/mock", json={"direction": "sideways"}).status_code == 400
    r = c.post("/api/initialise").get_json()
    assert r["ok"], r
    lanes_seen, worst, turns_seen = [], 0.0, 0
    t0 = time.time()
    while time.time() - t0 < 40:
        time.sleep(0.2)
        st = c.get("/api/state").get_json()
        lanes_seen.append(len(st["lanes"]))
        a, b = st["robot"], st["truth"]["robot"]
        worst = max(worst, math.hypot(a["X"] - b["X"], a["Y"] - b["Y"]))
        if st["truth"]["finished"]:
            break
    time.sleep(0.5)
    st = c.get("/api/state").get_json()
    turns = [e for e in rt.trk.events if e.kind == "turn"]
    assert st["truth"]["finished"] and len(turns) == 12, (len(turns), st["tracker"])
    assert lanes_seen[0] == 1 and max(lanes_seen) == 4 and lanes_seen == sorted(lanes_seen), lanes_seen
    assert st["direction"] == "CW" and _wrong(st) == 0, st["lanes"]
    decided = sum(1 for lane in st["lanes"] for s in lane["seats"] if s["state"] != "unknown")
    assert worst < 30.0, worst
    # checkpoint D: colours of PRESENT seats vs the simulated truth
    col_right = col_wrong = col_other = 0
    for lane in st["lanes"]:
        tc = st["truth"]["colors_by_slot"][str(lane["slot"])]
        for s in lane["seats"]:
            if s["color"] in ("red", "green"):
                if tc.get(str(s["index"])) == s["color"]:
                    col_right += 1
                else:
                    col_wrong += 1
            elif s["color"] is not None:
                col_other += 1
    assert col_wrong == 0 and col_right >= 1, st["lanes"]
    assert st["camera"]["source"] == "simulated" and st["camera"]["frames_used"] > 0, st["camera"]
    static = c.get("/api/static").get_json()
    assert static["init_id"] == st["init_id"] and len(static["init_scan"]) > 300 and len(static["truth_pillars"]) >= 4
    # the stream: the first event is a full state
    resp = c.get("/stream")
    first = next(resp.response).decode()
    resp.close()
    assert first.startswith("data: ") and json.loads(first[6:])["tracking"]
    # Save run -> run_track replays the same turns
    with tempfile.TemporaryDirectory() as d:
        ds.RUNS_DIR = d
        sv = c.post("/api/save-run").get_json()
        assert sv["ok"], sv
        out = subprocess.run([sys.executable, "run_track.py", "--replay-scan", f"{sv['dir']}/scan.json",
                              "--replay-imu", f"{sv['dir']}/imu.log"], cwd=HERE, capture_output=True, text=True,
                             timeout=120).stdout
    replay = [ln.split("turn", 1)[1].strip() for ln in out.splitlines() if " turn " in ln]
    assert replay == [e.detail for e in turns], (replay[:2], [e.detail for e in turns][:2])
    # Re-initialise: everything starts again from lane 1
    old = st["init_id"]
    assert c.post("/api/initialise").get_json()["ok"]
    st = c.get("/api/state").get_json()
    assert st["init_id"] == old + 1 and len(st["lanes"]) == 1 and st["tracker"]["lane_index"] == 0
    print(f"PASS  test_mock_end_to_end          CW, start lane slot 1, 3 laps at 4x: lanes appeared one per turn "
          f"(1 -> 4), 12 turns, {decided} seat verdicts, 0 disagree with the truth; pillar colours {col_right} right, "
          f"0 wrong, {col_other} unknown; drawn robot within "
          f"{worst:.1f} mm of the true one; stream serves the state; Save run replays to the same 12 turns; "
          f"Re-initialise starts over at lane 1")
    return rt


def test_tuning(rt):
    import dashboard_server as ds
    c = ds.app.test_client()
    groups = c.get("/api/tuning").get_json()["groups"]
    assert [g["group"] for g in groups] == ["LIDAR mount + calibration", "Initialisation thresholds",
                                            "Tracker + IMU", "Seat detector"], groups
    n = sum(len(g["params"]) for g in groups)
    assert all(p["meaning"] and p["when"] for g in groups for p in g["params"])
    r = c.post("/api/param", json={"name": "TURN_MIN_DEG", "value": "50"}).get_json()
    assert r["ok"] and config.TURN_MIN_DEG == 50.0 and r["when"] == "live"
    assert c.post("/api/param", json={"name": "IMU_YAW_SIGN", "value": 2}).status_code == 400
    assert c.post("/api/param", json={"name": "GAP_OPEN_MIN_MM", "value": "abc"}).status_code == 400
    assert c.post("/api/param", json={"name": "GAP_OPEN_MIN_MM", "value": "nan"}).status_code == 400
    assert c.post("/api/param", json={"name": "NOPE", "value": 1}).status_code == 400
    assert c.post("/api/param", json={"name": "angular_margin_deg", "value": 5}).get_json()["ok"]
    assert rt.trk.params.angular_margin_deg == 5.0                 # reaches the running tracker
    changed = [p["name"] for g in c.get("/api/tuning").get_json()["groups"] for p in g["params"] if p["changed"]]
    assert sorted(changed) == ["TURN_MIN_DEG", "angular_margin_deg"], changed
    assert c.post("/api/tuning/reset").get_json()["restored"] == n
    assert config.TURN_MIN_DEG == 45.0 and rt.seat_params.angular_margin_deg == 4.0
    print(f"PASS  test_tuning                   4 groups, {n} parameters, each with its meaning and when it applies; "
          f"bad values refused; a live seat-detector edit reaches the running tracker; changed values are "
          f"flagged; Reset restores config.py")


def test_lagging_loop():
    """The dashboard's loop, replayed deterministically against live_sim (no
    threads, no wall clock): the simulated robot is stepped by hand, with
    random gaps between LIDAR reads and a random lag (up to 0.3 s) between
    reading a frame and taking the STM32 samples -- a Pi that falls behind.
    Required: 0 wrong. (With the coverage check of P10 switched off, this same
    test gives 8 wrong in 509.)"""
    import random
    import lane_init as li
    import live_sim
    from lane_tracker import LaneTracker
    from scan_processing import clean_and_project, clean_and_project_timed

    class ManualSim(live_sim.LiveSim):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self._stop.set()
            self._thread.join()

        def advance(self, seconds):
            for _ in range(int(round(seconds / 0.01))):
                self._step()

    rng = random.Random(7)
    decided = wrong = 0
    for run in range(40):
        d, slot, seed = rng.choice(["CCW", "CW"]), rng.randint(0, 3), rng.randint(1, 999)
        s = ManualSim(d, slot, seed, rng.choice([600.0, 1000.0]))
        s.advance(0.3)
        first = s.link.drain()[-1]
        init = li.initialise(clean_and_project(s.lidar.get_latest_scan(), config.LIDAR_ANGLE_SIGN,
                                               config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
        assert init.ok, init.reason
        trk = LaneTracker(init, first)
        s.drive(0.5)
        while not s.finished:
            s.advance(rng.choice([0.01, 0.02, 0.05, rng.uniform(0.01, 0.5)]))
            raw4 = s.lidar.get_latest_scan_timed() if trk.wants_lidar else None
            s.advance(rng.uniform(0.0, 0.3))
            for smp in s.link.drain():
                trk.on_imu(smp)
            if raw4:
                trk.on_lidar_frame(*clean_and_project_timed(raw4, config.LIDAR_ANGLE_SIGN,
                                                            config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
        tr = s.truth()
        for k, rec in trk.lanes.items():
            if k == 0:
                continue
            for i, st in rec.seats.items():
                if st.state != "unknown":
                    decided += 1
                    wrong += (st.state == "occupied") != (i in tr["seats"].get(tr["slot_sections"][k], []))
    assert wrong == 0 and decided > 100, (wrong, decided)
    print(f"PASS  test_lagging_loop             40 runs of the dashboard loop with up to 0.5 s between LIDAR reads "
          f"and up to 0.3 s lag before the STM32 samples are taken: {decided} entry verdicts, 0 wrong")


# =============================================================================
# real mode with stand-in hardware (child process)
# =============================================================================
def _child_real():
    import _thread  # noqa: F401
    import fcntl
    import random
    import select
    import struct
    import termios
    import threading
    import tty
    import types

    import numpy as np

    import lane_frame as lf
    import simulation as sim

    master, slave = os.openpty()
    tty.setraw(slave)
    slave_path = os.ttyname(slave)

    class Serial:
        def __init__(self, port, baudrate, timeout):
            assert port == config.IMU_PORT
            self.fd = os.open(slave_path, os.O_RDONLY | os.O_NOCTTY)
            self.timeout = timeout

        @property
        def in_waiting(self):
            return struct.unpack("i", fcntl.ioctl(self.fd, termios.FIONREAD, b"\0\0\0\0"))[0]

        def read(self, n):
            r, _, _ = select.select([self.fd], [], [], self.timeout)
            return os.read(self.fd, n) if r else b""

    m = types.ModuleType("serial")
    m.Serial = Serial
    sys.modules["serial"] = m

    direction, slot = "CCW", 2
    rng = random.Random(11)
    path, sec0, s0, pillars, truth = sim.make_world(direction, slot, 1300.0, rng)
    track = [(time.monotonic(), s0)]
    go = threading.Event()

    class RPLidarC1Source:                                  # stand-in: real-sweep frames on the real clock
        def __init__(self, port, baudrate, timeout):
            pass

        def start(self):
            pass

        def stop(self):
            pass

        def _pose(self, t):
            tr = list(track)
            return path.pose(float(np.interp(t, [a for a, _ in tr], [b for _, b in tr])))

        def get_latest_scan(self):
            return [(a, d, q) for a, d, q, _ in self.get_latest_scan_timed()]

        def get_latest_scan_timed(self):
            return sim.cast_revolution(self._pose, time.monotonic(), 0.1, pillars, rng)

        def status(self):
            return {"thread_alive": True, "error": None, "points_received_total": 0}

        def timing_status(self):
            return {"spin_hz": 10.0, "backwards_steps": 0, "last_return_age_s": 0.0}

    m = types.ModuleType("lidar_source")
    m.RPLidarC1Source = RPLidarC1Source
    sys.modules["lidar_source"] = m

    def feeder():
        stm = sim.SimStm32(random.Random(3))
        t0 = time.monotonic()
        t_ms, dist, s = 0, 0.0, s0
        px, py, _ = path.pose(s0)
        while True:                                          # after 1 lap the robot stops; lines keep coming
            t_ms += 10
            if go.is_set() and s < s0 + path.length + 300:
                s += 10.0                                    # 1000 mm/s
            g = path.pose(s)
            dist += math.hypot(g[0] - px, g[1] - py)
            px, py = g[0], g[1]
            os.write(master, (stm.line(t_ms, dist, g[2]) + "\n").encode())
            track.append((t0 + t_ms / 1000.0, s))     # the scheduled time: what the STM32's t_ms says
            time.sleep(max(0.0, t0 + t_ms / 1000.0 - time.monotonic()))

    threading.Thread(target=feeder, daemon=True).start()
    import dashboard_server as ds
    rt = ds.create_runtime("real")
    c = ds.app.test_client()
    time.sleep(1.0)
    before = c.get("/api/state").get_json()
    r = c.post("/api/initialise").get_json()
    go.set()
    time.sleep(9.5)
    st = c.get("/api/state").get_json()
    turns = [e for e in rt.trk.events if e.kind == "turn"] if rt.trk else []
    wrong = decided = 0
    for lane in st.get("lanes", []):
        tr = truth[path.section(slot + lane["slot"])]
        for s in lane["seats"]:
            if s["state"] != "unknown":
                decided += 1
                wrong += (s["state"] == "occupied") != (s["index"] in tr)
    print("RESULT " + json.dumps({
        "init": r, "before_link": before["imu"].get("lines_ok", 0), "before_tracking": before["tracking"],
        "turns": len(turns), "lanes": len(st.get("lanes", [])), "wrong": wrong, "decided": decided,
        "imu_ok": st["imu"].get("lines_ok", 0), "stale": st["imu"].get("stale"), "lidar_spin": st["lidar"].get("spin_hz"),
        "error": st["error"], "entry_frames": [lane["frames_used"] for lane in st.get("lanes", [])]}))


def test_real_mode_with_stand_in_hardware():
    p = subprocess.run([sys.executable, __file__, "--child-real"], cwd=HERE, capture_output=True, text=True,
                       timeout=120)
    assert p.returncode == 0, p.stderr[-3000:]
    r = json.loads([ln for ln in p.stdout.splitlines() if ln.startswith("RESULT ")][-1][7:])
    assert r["before_link"] > 0 and not r["before_tracking"], r          # live before Initialise, not tracking
    assert r["init"]["ok"] and r["turns"] == 4 and r["lanes"] == 4, r
    assert r["wrong"] == 0 and r["decided"] >= 15 and r["error"] is None and not r["stale"], r
    assert all(n > 0 for n in r["entry_frames"][1:]), r
    print(f"PASS  test_real_mode                real mode, STM32 through a pseudo-terminal, stand-in LIDAR giving "
          f"real-sweep timed frames: live before Initialise ({r['before_link']} STM32 lines), then "
          f"{r['init']['reason']}; 1 lap at 1000 mm/s: 4 turns, 4 lanes, {r['decided']} seat verdicts, 0 wrong; "
          f"entry re-check frames per lane {r['entry_frames'][1:]}")


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "--child-real":
        _child_real()
        sys.exit(0)
    rt = test_mock_end_to_end()
    test_tuning(rt)
    test_lagging_loop()
    test_real_mode_with_stand_in_hardware()
    print("\nAll dashboard checks passed.")

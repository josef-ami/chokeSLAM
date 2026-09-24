"""
Batch runs of sim_closed_loop over rulebook layouts (checkpoint E, step 3).

    python3 sweep_closed_loop.py --layouts 200 --trials 1 --noise none --out runs.csv
    python3 sweep_closed_loop.py --layouts 200 --trials 5 --noise moderate --set GAP_OPEN_MIN_MM=300

Layout k is layouts.draw(random.Random(k)); trial j of it uses seed
k * 1000 + j for every noise draw, so a row can be re-run alone with
    python3 sim_closed_loop.py --seed <layout> ...   (trial 0)
--set NAME=VALUE overrides a config value in every worker (for what-if runs).
"""
from __future__ import annotations

import argparse
import csv
import multiprocessing as mp
import random
import time
from collections import Counter


def _one(job):
    k, j, noise, sets = job
    import config
    for name, val in sets:
        setattr(config, name, type(getattr(config, name))(val) if not isinstance(getattr(config, name), str) else val)
    import layouts
    import sim_closed_loop as scl
    lay = layouts.draw(random.Random(k))
    t0 = time.time()
    try:
        r = scl.run(lay, seed=k * 1000 + j, noise=noise)
    except Exception as e:                                      # a crash is a result too
        return dict(layout=k, trial=j, direction=lay.direction, ok=False, reason="exception",
                    detail=f"{type(e).__name__}: {e}", t=0, laps=0, gap=None, gap_at="", err=None,
                    plans=0, replans=0, colour_stops=0, colour_given_up=0, wrong_seat=0, wrong_colour=0,
                    wall=time.time() - t0, desc=lay.describe())
    return dict(layout=k, trial=j, direction=lay.direction, ok=r.ok, reason=r.reason, detail=r.detail,
                t=round(r.t, 2), laps=r.laps, gap=None if r.min_pillar_gap_mm == float("inf") else round(r.min_pillar_gap_mm, 1),
                gap_at=str(r.min_gap_at), err=round(r.max_track_err_mm, 1), plans=r.plans, replans=r.replans,
                colour_stops=r.stops_for_colour, colour_given_up=r.colour_given_up, wrong_seat=r.wrong_seat,
                wrong_colour=r.wrong_colour, wall=round(time.time() - t0, 1), desc=lay.describe())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layouts", type=int, default=100)
    ap.add_argument("--first", type=int, default=0)
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--noise", default="none")
    ap.add_argument("--out", default="sweep.csv")
    ap.add_argument("--workers", type=int, default=mp.cpu_count())
    ap.add_argument("--set", action="append", default=[])
    a = ap.parse_args()
    sets = [tuple(s.split("=", 1)) for s in a.set]
    jobs = [(k, j, a.noise, sets) for k in range(a.first, a.first + a.layouts) for j in range(a.trials)]
    rows = []
    t0 = time.time()
    with mp.Pool(a.workers) as pool:
        for n, row in enumerate(pool.imap_unordered(_one, jobs), 1):
            rows.append(row)
            if n % 20 == 0:
                print(f"  {n}/{len(jobs)}  {time.time() - t0:.0f}s", flush=True)
    rows.sort(key=lambda r: (r["layout"], r["trial"]))
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    summarise(rows, a)


def summarise(rows, a=None):
    n = len(rows)
    ok = sum(r["ok"] for r in rows)
    started = [r for r in rows if r["reason"] not in ("init_failed", "init_wrong_direction")]
    oks = sum(r["ok"] for r in started)
    print(f"\n{n} runs ({a.noise if a else ''}): success {ok}/{n} = {100 * ok / n:.1f}%")
    if started:
        print(f"  initialised {len(started)}/{n}; success among them {oks}/{len(started)} = {100 * oks / len(started):.1f}%")
    for d in ("CW", "CCW"):
        rr = [r for r in started if r["direction"] == d]
        if rr:
            print(f"  {d}: {sum(r['ok'] for r in rr)}/{len(rr)} of initialised")
    c = Counter(r["reason"] for r in rows if not r["ok"])
    print("  failures:", dict(c))
    det = Counter((r["reason"], r["detail"][:60]) for r in rows if not r["ok"] and r["reason"] != "init_failed")
    for (k, dd), v in det.most_common(12):
        print(f"    {v:4d}  {k}: {dd}")
    good = [r for r in rows if r["ok"]]
    if good:
        ts = sorted(r["t"] for r in good)
        gs = sorted(r["gap"] for r in good if r["gap"] is not None)
        es = sorted(r["err"] for r in good)
        print(f"  successful runs: time median {ts[len(ts) // 2]:.1f}s max {ts[-1]:.1f}s; "
              f"closest pillar median {gs[len(gs) // 2]:.0f} mm min {gs[0]:.0f} mm; "
              f"tracking error max median {es[len(es) // 2]:.0f} mm worst {es[-1]:.0f} mm")
        print(f"  wrong seat verdicts {sum(r['wrong_seat'] for r in rows)}, wrong colours {sum(r['wrong_colour'] for r in rows)}, "
              f"colour stops {sum(r['colour_stops'] for r in rows)}, colours given up {sum(r['colour_given_up'] for r in rows)}")


if __name__ == "__main__":
    main()

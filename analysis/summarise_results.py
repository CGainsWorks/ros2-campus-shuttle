#!/usr/bin/env python3
"""Write the compact, committed summary of every number the README quotes.

`results/` holds raw run output and is not committed, with one exception:
`results/controller_comparison.json`, written by this script. It has three
sections, each from a different source:

bench
    The deterministic kinematic benchmark (scripts/run_bench.sh). Recomputed
    from results/bench/ on every run. The values are rounded exactly as
    docs/RESULTS.md prints them, so a regeneration on any machine reproduces
    the file byte for byte and CI can `git diff` it.

gazebo
    The Gazebo cross-check (scripts/run_gazebo_repeats.sh). Physics runs are
    not bit-reproducible, so this section is refreshed only when that script
    has just produced new repeats (--gazebo) and otherwise carried over from
    the committed file unchanged.

safety
    Stop times measured by the safety-layer launch test in simulation
    (--safety PATH to refresh), otherwise carried over.

scripts/check_readme.py then checks the README against this file.

    python3 analysis/summarise_results.py                # refresh bench only
    python3 analysis/summarise_results.py --check        # fail if bench is stale
    python3 analysis/summarise_results.py --gazebo results/gazebo
    python3 analysis/summarise_results.py --safety results/safety/safety_stop_times.json
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SUMMARY = os.path.join(REPO_ROOT, "results", "controller_comparison.json")
BENCH_DIR = os.path.join(REPO_ROOT, "results", "bench")

CONTROLLERS = ("pure_pursuit", "stanley")
SCENARIO_ORDER = ("straight_offset", "heading_offset", "circle", "corner",
                  "figure_eight", "campus_loop", "sharp_turns")
STEP_SCENARIOS = ("straight_offset", "heading_offset")
REFERENCE_SPEED = 1.0
BAND = 0.05  # m, the settling band docs/RESULTS.md uses


def read_csv(path: str) -> list[dict]:
    with open(path, encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def fnum(row: dict, key: str) -> float:
    return float(row[key])


def r4(x: float) -> float:
    return round(x, 4)


def mean(values) -> float:
    values = list(values)
    return sum(values) / len(values)


def speed_key(speed: float) -> str:
    return f"{speed:g}"


def transient(series_path: str) -> dict:
    """Settling distance and overshoot, as compare_controllers.transient_metrics."""
    rows = read_csv(series_path)
    err = [fnum(r, "cross_track") for r in rows]
    s = [fnum(r, "s") for r in rows]
    outside = [si for si, e in zip(s, err) if abs(e) > BAND]
    settle = max(outside) if outside else 0.0
    start_sign = 1.0 if err[0] >= 0 else -1.0
    overshoot = max(-start_sign * e for e in err)
    if overshoot <= 0.0:
        overshoot = 0.0  # never crossed the path; avoid reporting -0.0
    return {"settle_m": round(settle, 2), "overshoot_cm": round(100.0 * overshoot, 1)}


def series_path(ts_dir: str, controller: str, scenario: str, speed: float,
                noise: bool) -> str | None:
    """Same naming as compare_controllers.load_timeseries."""
    tag = f"{controller}_{scenario}_{speed:.1f}".replace(".", "p")
    path = os.path.join(ts_dir, f"{tag}_{'noise' if noise else 'clean'}.csv")
    return path if os.path.exists(path) else None


def bench_section(bench_dir: str) -> dict:
    rows = read_csv(os.path.join(bench_dir, "metrics.csv"))
    for r in rows:
        r["_speed"] = fnum(r, "target_speed")
        r["_noise"] = r["noise"] == "1"

    failed = [r for r in rows if r["goal_reached"] != "1" or r["diverged"] == "1"
              or r["stalled"] == "1"]
    speeds = sorted({r["_speed"] for r in rows})
    scenarios = [s for s in SCENARIO_ORDER if any(r["scenario"] == s for r in rows)]

    def pick(controller, scenario, speed, noise):
        hit = [r for r in rows if r["controller"] == controller and
               r["scenario"] == scenario and r["_speed"] == speed and
               r["_noise"] == noise]
        if len(hit) != 1:
            raise SystemExit(f"error: expected one bench row for {controller} "
                             f"{scenario} {speed} noise={noise}, found {len(hit)}")
        return hit[0]

    per_scenario = {}
    for sc in scenarios:
        entry = {}
        for c in CONTROLLERS:
            r = pick(c, sc, REFERENCE_SPEED, False)
            entry[c] = {"rms_m": r4(fnum(r, "ct_rms")), "max_m": r4(fnum(r, "ct_max_abs"))}
        entry["closer"] = ("pure_pursuit" if fnum(pick("pure_pursuit", sc, REFERENCE_SPEED, False),
                                                  "ct_rms")
                           < fnum(pick("stanley", sc, REFERENCE_SPEED, False), "ct_rms")
                           else "stanley")
        per_scenario[sc] = entry

    # How much tighter Pure Pursuit is on the curved routes, from unrounded RMS.
    curved = [s for s in scenarios if s not in STEP_SCENARIOS and s != "straight_offset"]
    ratios = {
        s: fnum(pick("stanley", s, REFERENCE_SPEED, False), "ct_rms")
        / fnum(pick("pure_pursuit", s, REFERENCE_SPEED, False), "ct_rms")
        for s in curved
    }

    # Robustness: clean against noisy RMS, averaged over every scenario and speed.
    noise = {}
    for c in CONTROLLERS:
        clean = mean(fnum(r, "ct_rms") for r in rows if r["controller"] == c and not r["_noise"])
        noisy = mean(fnum(r, "ct_rms") for r in rows if r["controller"] == c and r["_noise"])
        noise[c] = {"clean_rms_m": r4(clean), "noisy_rms_m": r4(noisy),
                    "penalty_pct": round(100.0 * (noisy - clean) / clean, 1)}

    ts_dir = os.path.join(bench_dir, "timeseries")
    steps = {}
    for sc in STEP_SCENARIOS:
        if sc not in scenarios:
            continue
        for speed in speeds:
            cell = {}
            for c in CONTROLLERS:
                path = series_path(ts_dir, c, sc, speed, False)
                if path is None:
                    raise SystemExit(f"error: no time series for {c} {sc} {speed}")
                cell[c] = transient(path)
            steps[f"{sc}@{speed_key(speed)}"] = cell

    return {
        "source": "results/bench/metrics.csv (scripts/run_bench.sh)",
        "label": "simulated: kinematic benchmark, deterministic",
        "runs": len(rows),
        "failed_runs": len(failed),
        "scenarios": len(scenarios),
        "speeds_mps": speeds,
        "reference_speed_mps": REFERENCE_SPEED,
        "rms_by_scenario": per_scenario,
        "curved_route_rms_ratio_stanley_over_pp": {
            "min": round(min(ratios.values()), 1),
            "min_scenario": min(ratios, key=ratios.get),
        },
        "noise": noise,
        "step_response": steps,
    }


def rms(values) -> float:
    values = list(values)
    return math.sqrt(sum(v * v for v in values) / len(values))


def gazebo_section(gazebo_dir: str, previous: dict | None) -> dict:
    repeat_dirs = sorted(glob.glob(os.path.join(gazebo_dir, "repeat_*")))
    if not repeat_dirs:
        raise SystemExit(f"error: no repeat_* directories under {gazebo_dir}")
    meta_path = os.path.join(gazebo_dir, "run_info.json")
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path, encoding="utf-8") as fh:
            meta = json.load(fh)

    repeats = []
    scenario = speed = None
    for d in repeat_dirs:
        rows = read_csv(os.path.join(d, "metrics.csv"))
        entry = {}
        for c in CONTROLLERS:
            hit = [r for r in rows if r["controller"] == c]
            if len(hit) != 1:
                raise SystemExit(f"error: {d}: expected one {c} row, found {len(hit)}")
            r = hit[0]
            scenario = r["scenario"]
            speed = fnum(r, "target_speed")
            entry[c] = {
                "rms_m": r4(fnum(r, "ct_rms")),
                "max_m": r4(fnum(r, "ct_max_abs")),
                "goal_reached": r["goal_reached"] == "1",
                "progress": round(fnum(r, "progress"), 3),
                "control_effort": round(fnum(r, "control_effort"), 2),
            }
        ekf = []
        for c in CONTROLLERS:
            path = os.path.join(d, f"ekf_evaluation_{c}.csv")
            if os.path.exists(path):
                erows = read_csv(path)
                # Split the drift along and across the direction of travel:
                # only the across-track part moves the robot off the line.
                along, across = [], []
                for r in erows:
                    yaw = fnum(r, "truth_yaw")
                    ex, ey = fnum(r, "err_x"), fnum(r, "err_y")
                    along.append(ex * math.cos(yaw) + ey * math.sin(yaw))
                    across.append(-ex * math.sin(yaw) + ey * math.cos(yaw))
                ekf.append({
                    "controller": c,
                    "position_rms_m": round(rms(fnum(r, "err_pos") for r in erows), 3),
                    "position_max_m": round(max(fnum(r, "err_pos") for r in erows), 3),
                    "heading_rms_rad": round(rms(fnum(r, "err_yaw") for r in erows), 4),
                    "along_track_rms_m": round(rms(along), 3),
                    "cross_track_rms_m": round(rms(across), 3),
                })
        entry["ekf"] = ekf
        repeats.append(entry)

    def span(values):
        values = list(values)
        return [min(values), max(values)]

    ekf_all = [e for rep in repeats for e in rep["ekf"]]
    return {
        "source": "scripts/run_gazebo_repeats.sh",
        "label": "simulated: Gazebo physics, not bit-reproducible",
        "simulator": meta.get("simulator", previous.get("simulator") if previous else None),
        "date": meta.get("date", previous.get("date") if previous else None),
        "scenario": scenario,
        "speed_mps": speed,
        "repeats": repeats,
        "rms_span_m": {c: span(rep[c]["rms_m"] for rep in repeats) for c in CONTROLLERS},
        "ekf_span": {
            "position_rms_m": span(e["position_rms_m"] for e in ekf_all),
            "position_max_m": span(e["position_max_m"] for e in ekf_all),
            "heading_rms_rad": span(e["heading_rms_rad"] for e in ekf_all),
            "along_track_rms_m": span(e["along_track_rms_m"] for e in ekf_all),
            "cross_track_rms_m": span(e["cross_track_rms_m"] for e in ekf_all),
        } if ekf_all else None,
    }


def write(summary: dict, path: str) -> str:
    text = json.dumps(summary, indent=2, sort_keys=False) + "\n"
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return text


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--bench", default=BENCH_DIR)
    p.add_argument("--gazebo", help="directory with repeat_*/ from run_gazebo_repeats.sh")
    p.add_argument("--safety", help="safety_stop_times.json from the safety launch test")
    p.add_argument("--output", default=SUMMARY)
    p.add_argument("--check", action="store_true",
                   help="regenerate and fail if the committed file differs")
    args = p.parse_args()

    previous = {}
    if os.path.exists(args.output):
        with open(args.output, encoding="utf-8") as fh:
            previous = json.load(fh)

    summary = {
        "about": ("Every number the README quotes about the controller comparison, "
                  "the Gazebo cross-check and the safety layer. All of it is "
                  "simulated. Written by analysis/summarise_results.py; checked "
                  "against the README by scripts/check_readme.py."),
        "bench": bench_section(args.bench),
        "gazebo": (gazebo_section(args.gazebo, previous.get("gazebo"))
                   if args.gazebo else previous.get("gazebo")),
        "safety": previous.get("safety"),
    }
    if args.safety:
        with open(args.safety, encoding="utf-8") as fh:
            measured = json.load(fh)
        summary["safety"] = {
            "source": "src/shuttle_bringup/test/test_safety_sim.py",
            "label": "simulated: Gazebo, ground-truth speed, simulated time",
            **measured,
        }

    if args.check:
        expected = json.dumps(summary, indent=2) + "\n"
        with open(args.output, encoding="utf-8") as fh:
            actual = fh.read()
        if actual != expected:
            print(f"error: {os.path.relpath(args.output, REPO_ROOT)} is stale. "
                  "Re-run analysis/summarise_results.py and commit the result.",
                  file=sys.stderr)
            return 1
        print(f"ok: {os.path.relpath(args.output, REPO_ROOT)} matches the regenerated "
              "benchmark summary")
        return 0

    write(summary, args.output)
    print(f"wrote {os.path.relpath(args.output, REPO_ROOT).replace(os.sep, '/')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

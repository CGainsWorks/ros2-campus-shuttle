#!/usr/bin/env python3
"""Check every number the README quotes against the committed results.

The README is written by hand, so nothing stops it drifting away from the data
except a check. This one reads

  results/controller_comparison.json  (analysis/summarise_results.py)
  calibration/extrinsics_report.json  (scripts/run_calibration.sh)

and fails if any figure the README states differs from the value in those
files, formatted the way the README formats it. Whitespace is normalised
first, so a number can wrap across lines in the Markdown source.

    python3 scripts/check_readme.py [--test-core-checks N]

With --test-core-checks, the C++ unit test count quoted in the README's test
table is checked too (CI passes the count test_core just printed).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
CONTROLLERS = ("pure_pursuit", "stanley")
CLOSER = {"pure_pursuit": "Pure Pursuit", "stanley": "Stanley"}


def load(rel: str) -> dict:
    with open(os.path.join(REPO_ROOT, rel), encoding="utf-8") as fh:
        return json.load(fh)


def normalise(text: str) -> str:
    return re.sub(r"\s+", " ", text)


class Checker:
    def __init__(self, name: str, text: str):
        self.name = name
        self.raw = text
        self.text = normalise(text)
        self.passed = 0
        self.failures: list[str] = []

    def contains(self, needle: str, what: str) -> None:
        if normalise(needle) in self.text:
            self.passed += 1
        else:
            self.failures.append(f"{self.name}: {what}: expected to find '{needle}'")

    def table_row(self, first_cell: str) -> list[str] | None:
        """Cells of the first table row whose first cell starts with first_cell."""
        for line in self.raw.splitlines():
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if line.lstrip().startswith("|") and cells and cells[0].startswith(first_cell):
                return cells
        return None

    def cell_value(self, cells: list[str], index: int, expected: str, what: str) -> None:
        actual = cells[index].replace("*", "").replace(" m", "").strip()
        if actual == expected:
            self.passed += 1
        else:
            self.failures.append(f"{self.name}: {what}: README has {actual}, results have "
                                 f"{expected}")


def check_bench(c: Checker, bench: dict) -> None:
    c.contains(f"Across {bench['runs']} runs", "total benchmark runs")
    speeds = len(bench["speeds_mps"])
    c.contains(f"{bench['scenarios']} scenarios × {speeds} speeds", "benchmark matrix")
    if bench["failed_runs"] == 0:
        c.contains("none of which failed to finish", "failed runs")

    for scenario, entry in bench["rms_by_scenario"].items():
        cells = c.table_row(f"`{scenario}`")
        if cells is None:
            c.failures.append(f"{c.name}: no row for `{scenario}` in the results table")
            continue
        c.cell_value(cells, 2, f"{entry['pure_pursuit']['rms_m']:.4f}",
                     f"{scenario} Pure Pursuit RMS")
        c.cell_value(cells, 3, f"{entry['stanley']['rms_m']:.4f}", f"{scenario} Stanley RMS")
        c.cell_value(cells, 4, CLOSER[entry["closer"]], f"{scenario} closer")

    ratio = bench["curved_route_rms_ratio_stanley_over_pp"]
    c.contains(f"at least {ratio['min']:.1f}× tighter", "smallest curved-route ratio")
    circle_mm = 1000.0 * bench["rms_by_scenario"]["circle"]["pure_pursuit"]["rms_m"]
    c.contains(f"`circle` comes out at {circle_mm:.1f} mm", "circle RMS")

    peaks = bench["rms_by_scenario"]
    c.contains(
        f"peaks of {1000 * peaks['campus_loop']['stanley']['max_m']:.1f} mm on "
        f"`campus_loop` and {1000 * peaks['sharp_turns']['stanley']['max_m']:.1f} mm on "
        f"`sharp_turns`, against Pure Pursuit's "
        f"{1000 * peaks['campus_loop']['pure_pursuit']['max_m']:.1f} mm and "
        f"{1000 * peaks['sharp_turns']['pure_pursuit']['max_m']:.1f} mm",
        "corner-exit peak errors")

    step = bench["step_response"]["straight_offset@1.5"]
    c.contains(
        f"Pure Pursuit is inside 5 cm after {step['pure_pursuit']['settle_m']:.2f} m "
        f"but overshoots by {step['pure_pursuit']['overshoot_cm']:.1f} cm; Stanley takes "
        f"{step['stanley']['settle_m']:.2f} m to get inside that band but overshoots only "
        f"{step['stanley']['overshoot_cm']:.1f} cm", "straight_offset step at 1.5 m/s")
    heading = bench["step_response"]["heading_offset@1"]
    if heading["stanley"]["settle_m"] == 0.0:
        c.contains("it never leaves a 5 cm band", "Stanley heading recovery")
    c.contains(f"Pure Pursuit swings {heading['pure_pursuit']['overshoot_cm']:.1f} cm past",
               "Pure Pursuit heading overshoot")

    noise = bench["noise"]
    c.contains(f"(+{noise['pure_pursuit']['penalty_pct']:.1f}% against "
               f"+{noise['stanley']['penalty_pct']:.1f}%)", "noise penalties")


def check_gazebo(c: Checker, gz: dict) -> None:
    c.contains(f"On the `{gz['scenario']}` route at {gz['speed_mps']:.1f} m/s",
               "Gazebo scenario and speed")
    c.contains(f"Across {len(gz['repeats'])} independent repeats", "Gazebo repeat count")
    for i, rep in enumerate(gz["repeats"], start=1):
        cells = c.table_row(f"repeat {i}")
        if cells is None:
            c.failures.append(f"{c.name}: no row for repeat {i} in the Gazebo table")
            continue
        c.cell_value(cells, 1, f"{rep['pure_pursuit']['rms_m']:.4f}", f"repeat {i} PP RMS")
        c.cell_value(cells, 2, f"{rep['stanley']['rms_m']:.4f}", f"repeat {i} Stanley RMS")
    if all(rep[ctrl]["goal_reached"] for rep in gz["repeats"] for ctrl in CONTROLLERS):
        c.contains("both controllers reach the goal every time", "Gazebo goals reached")

    span = gz["ekf_span"]
    if span:
        cells = c.table_row("Position error (RMS)")
        expected = f"{span['position_rms_m'][0]:.3f} to {span['position_rms_m'][1]:.3f}"
        if cells is None:
            c.failures.append(f"{c.name}: no EKF position RMS row")
        else:
            c.cell_value(cells, 1, expected, "EKF position RMS span")
        cells = c.table_row("Position error (max)")
        expected = f"{span['position_max_m'][0]:.3f} to {span['position_max_m'][1]:.3f}"
        if cells is None:
            c.failures.append(f"{c.name}: no EKF position max row")
        else:
            c.cell_value(cells, 1, expected, "EKF position max span")
        c.contains(
            f"{span['along_track_rms_m'][0]:.2f} to {span['along_track_rms_m'][1]:.2f} m "
            f"RMS along it and {span['cross_track_rms_m'][0]:.3f} to "
            f"{span['cross_track_rms_m'][1]:.3f} m across it", "EKF drift split")
        rms_all = [rep[ctrl]["rms_m"] for rep in gz["repeats"] for ctrl in CONTROLLERS]
        c.contains(f"({min(rms_all):.3f} to {max(rms_all):.3f} m)",
                   "span of Gazebo tracking error")
        cells = c.table_row("Heading error (RMS)")
        expected = (f"{span['heading_rms_rad'][0]:.4f} to {span['heading_rms_rad'][1]:.4f} "
                    "rad")
        if cells is None:
            c.failures.append(f"{c.name}: no EKF heading row")
        else:
            c.cell_value(cells, 1, expected, "EKF heading RMS span")


def check_safety(c: Checker, safety: dict) -> None:
    estop = safety["estop"]
    timeout = safety["command_timeout"]
    c.contains(
        f"from {estop['initial_speed_mps']:.2f} m/s to rest in "
        f"{estop['time_to_rest_s']:.2f} s", "measured e-stop stop time")
    c.contains(f"within {estop['bound_s']:.1f} s of the e-stop", "e-stop bound")
    c.contains(
        f"rest {timeout['time_to_rest_s']:.2f} s after the last command",
        "measured command-timeout stop time")
    c.contains(f"within {timeout['bound_s']:.1f} s of the last command",
               "command-timeout bound")
    c.contains(f"{timeout['cmd_timeout_s']:.1f} s command timeout", "command timeout")


def check_calibration(c: Checker, cal: dict) -> None:
    c.contains(f"with {cal['num_observations']} board poses and {cal['num_points']} points",
               "calibration data size")
    pairs = [
        ("Residual (RMS point-to-plane)", f"{1000 * cal['residual_rms_m']:.1f} mm"),
        ("Translation error vs truth",
         f"{1000 * cal['error_vs_ground_truth']['translation_m']:.1f} mm"),
        ("Rotation error vs truth", f"{cal['error_vs_ground_truth']['rotation_deg']:.2f}°"),
        ("Jacobian condition number", f"{cal['jacobian_condition_number']:.0f}"),
    ]
    for label, expected in pairs:
        cells = c.table_row(label)
        if cells is None:
            c.failures.append(f"{c.name}: no calibration row '{label}'")
            continue
        actual = cells[1].replace("*", "").strip()
        if actual == expected:
            c.passed += 1
        else:
            c.failures.append(f"{c.name}: {label}: README has {actual}, report has {expected}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--readme", default=os.path.join(REPO_ROOT, "README.md"))
    p.add_argument("--test-core-checks", type=int,
                   help="the check count test_core printed; verified against the README")
    args = p.parse_args()

    with open(args.readme, encoding="utf-8") as fh:
        readme = Checker("README.md", fh.read())
    with open(os.path.join(REPO_ROOT, "docs", "SAFETY.md"), encoding="utf-8") as fh:
        safety_doc = Checker("docs/SAFETY.md", fh.read())

    results = load("results/controller_comparison.json")
    check_bench(readme, results["bench"])
    if results.get("gazebo"):
        check_gazebo(readme, results["gazebo"])
    else:
        readme.failures.append("results file has no gazebo section")
    if results.get("safety"):
        check_safety(readme, results["safety"])
        check_safety(safety_doc, results["safety"])
    else:
        readme.failures.append("results file has no safety section")
    check_calibration(readme, load("calibration/extrinsics_report.json"))
    if args.test_core_checks is not None:
        readme.contains(f"`test_core` (C++, {args.test_core_checks} checks)",
                        "C++ unit test count")

    failures = readme.failures + safety_doc.failures
    passed = readme.passed + safety_doc.passed
    for failure in failures:
        print(f"FAIL {failure}")
    print(f"{passed} README figures match the results, {len(failures)} do not")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

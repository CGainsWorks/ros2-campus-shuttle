#!/usr/bin/env python3
"""Check the Python and C++ scenario generators still agree.

The reference paths exist twice: in C++ (used by the offline benchmark, which
must build without ROS) and in Python (used by the Gazebo path publisher, which
must run without a compiler). Duplicated geometry drifts silently, and a
comparison run against a subtly different reference path is worthless.

This test dumps the C++ paths and checks the Python ones match point for point.

Run:
    controller_bench --dump-paths /tmp/paths
    python3 tests/test_scenarios_match.py --paths /tmp/paths
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "src", "shuttle_localisation"),
)

from shuttle_localisation.scenario_paths import SCENARIOS  # noqa: E402

# The C++ path goes through Path::fromPolyline, which drops samples closer than
# 1e-6 m. Allow for that and for double formatting in the CSV.
TOLERANCE = 1e-4


def load_reference(path: str):
    with open(path, newline="", encoding="utf-8") as fh:
        return [(float(r["x"]), float(r["y"])) for r in csv.DictReader(fh)]


def dedupe(points):
    """Mirror the duplicate-point filtering in Path::fromPolyline."""
    out = []
    for x, y in points:
        if out and math.hypot(x - out[-1][0], y - out[-1][1]) < 1e-6:
            continue
        out.append((float(x), float(y)))
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--paths", default="results/paths",
                        help="directory of <scenario>.csv files from "
                             "controller_bench --dump-paths")
    args = parser.parse_args()

    if not os.path.isdir(args.paths):
        print(f"error: {args.paths} not found. Run "
              f"`controller_bench --dump-paths {args.paths}` first.",
              file=sys.stderr)
        return 2

    failures = 0
    checked = 0

    for name, generator in sorted(SCENARIOS.items()):
        ref_file = os.path.join(args.paths, f"{name}.csv")
        if not os.path.exists(ref_file):
            print(f"FAIL {name}: no C++ reference at {ref_file}")
            failures += 1
            continue

        reference = load_reference(ref_file)
        produced = dedupe(generator()[0])
        checked += 1

        if len(reference) != len(produced):
            print(f"FAIL {name}: C++ has {len(reference)} points, "
                  f"Python has {len(produced)}")
            failures += 1
            continue

        worst = 0.0
        worst_i = 0
        for i, ((rx, ry), (px, py)) in enumerate(zip(reference, produced)):
            d = math.hypot(rx - px, ry - py)
            if d > worst:
                worst, worst_i = d, i

        if worst > TOLERANCE:
            print(f"FAIL {name}: diverges by {worst:.6f} m at index {worst_i} "
                  f"(C++ {reference[worst_i]}, Python {produced[worst_i]})")
            failures += 1
        else:
            print(f"ok   {name}: {len(reference)} points, "
                  f"max difference {worst:.2e} m")

    print(f"\n{checked} scenarios checked, {failures} mismatched")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

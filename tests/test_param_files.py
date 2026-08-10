#!/usr/bin/env python3
"""Check every ROS 2 parameter file will actually load.

This exists because of a specific failure that cost real debugging time. The
EKF covariance matrices were written with a mix of `0` and `0.02`. That is valid
YAML and looks fine, but the ROS 2 parameter parser requires a sequence to be
homogeneous and rejects the file with:

    Sequence should be of same type. Value type 'integer' do not belong

The failure is not local to the offending parameter. The whole file fails to
parse, `ekf_node` aborts at startup, and the only visible symptom downstream is
a TF tree missing `odom -> base_footprint` and a controller that never moves.

Nothing in a build catches this, so it is checked here.

    python3 tests/test_param_files.py
"""

from __future__ import annotations

import os
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    print("error: PyYAML is required (apt install python3-yaml)", file=sys.stderr)
    sys.exit(2)

REPO_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SEARCH_DIRS = [
    os.path.join(REPO_ROOT, "src"),
]

FAILURES = []


def numeric_kind(value):
    """Return 'int', 'float' or None. bool is not numeric for this purpose."""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    return None


def check_sequence(path, key, seq):
    kinds = {numeric_kind(v) for v in seq}
    kinds.discard(None)
    if len(kinds) > 1:
        offenders = [v for v in seq if numeric_kind(v) == "int"]
        FAILURES.append(
            f"{os.path.relpath(path, REPO_ROOT)}: parameter '{key}' mixes "
            f"integers and floats; ROS 2 will reject the whole file. "
            f"First few integer entries: {offenders[:5]} "
            f"(write them as 0.0, 1.0, ...)"
        )
        return False
    return True


def walk(path, node, key_prefix=""):
    ok = True
    if isinstance(node, dict):
        for key, value in node.items():
            child = f"{key_prefix}.{key}" if key_prefix else str(key)
            ok = walk(path, value, child) and ok
    elif isinstance(node, list):
        if any(numeric_kind(v) is not None for v in node):
            ok = check_sequence(path, key_prefix, node) and ok
        for item in node:
            if isinstance(item, (dict, list)):
                ok = walk(path, item, key_prefix) and ok
    return ok


def main() -> int:
    files = []
    for root_dir in SEARCH_DIRS:
        for dirpath, _dirnames, filenames in os.walk(root_dir):
            for name in filenames:
                if name.endswith((".yaml", ".yml")):
                    files.append(os.path.join(dirpath, name))

    if not files:
        print("error: no parameter files found", file=sys.stderr)
        return 2

    print("ROS 2 parameter file checks\n")
    for path in sorted(files):
        rel = os.path.relpath(path, REPO_ROOT)
        try:
            with open(path, encoding="utf-8") as fh:
                data = yaml.safe_load(fh)
        except yaml.YAMLError as exc:
            FAILURES.append(f"{rel}: not valid YAML: {exc}")
            print(f"FAIL {rel}: invalid YAML")
            continue

        if data is None:
            print(f"ok   {rel} (empty)")
            continue

        before = len(FAILURES)
        walk(path, data)
        if len(FAILURES) == before:
            print(f"ok   {rel}")
        else:
            print(f"FAIL {rel}")

    print()
    for failure in FAILURES:
        print(f"  {failure}")
    print(f"\n{len(files)} files checked, {len(FAILURES)} problems")
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())

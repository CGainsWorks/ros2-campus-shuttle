#!/usr/bin/env python3
"""Reference path geometry for the controller comparison.

Deliberately free of any ROS import so it can be checked against the C++
generators without a ROS installation (tests/test_scenarios_match.py).

These functions mirror src/shuttle_control/core/src/scenarios.cpp point for
point. The duplication is unavoidable -- the C++ side has to build without ROS,
the Python side has to run without a compiler -- so it is tested rather than
trusted.
"""

from __future__ import annotations

import math

SAMPLE_STEP = 0.05


def _line(points, x0, y0, x1, y1):
    length = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(length / SAMPLE_STEP))
    for i in range(1, n + 1):
        t = i / n
        points.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))


def _arc(points, cx, cy, radius, a0, a1):
    sweep = a1 - a0
    length = abs(sweep) * radius
    n = max(1, int(length / SAMPLE_STEP))
    for i in range(1, n + 1):
        a = a0 + sweep * i / n
        points.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))


def _filleted(waypoints, radius):
    """Straights joined by circular fillets; mirrors makeFilleted() in C++."""
    points = [waypoints[0]]
    cx, cy = waypoints[0]
    for i in range(1, len(waypoints) - 1):
        px, py = waypoints[i - 1]
        x, y = waypoints[i]
        nx, ny = waypoints[i + 1]

        l1 = math.hypot(x - px, y - py)
        l2 = math.hypot(nx - x, ny - y)
        if l1 < 1e-9 or l2 < 1e-9:
            continue
        v1x, v1y = (x - px) / l1, (y - py) / l1
        v2x, v2y = (nx - x) / l2, (ny - y) / l2

        cross = v1x * v2y - v1y * v2x
        dot = v1x * v2x + v1y * v2y
        delta = math.atan2(cross, dot)
        if abs(delta) < 1e-9:
            continue

        half = abs(delta) / 2.0
        t_len = min(radius * math.tan(half), 0.5 * min(l1, l2))
        r_eff = t_len / math.tan(half)

        tix, tiy = x - v1x * t_len, y - v1y * t_len
        tox, toy = x + v2x * t_len, y + v2y * t_len

        _line(points, cx, cy, tix, tiy)

        turn = 1 if delta > 0 else -1
        ccx = tix + turn * (-v1y) * r_eff
        ccy = tiy + turn * (v1x) * r_eff
        a0 = math.atan2(tiy - ccy, tix - ccx)
        a1 = math.atan2(toy - ccy, tox - ccx)
        sweep = a1 - a0
        while turn > 0 and sweep < 0:
            sweep += 2 * math.pi
        while turn < 0 and sweep > 0:
            sweep -= 2 * math.pi
        _arc(points, ccx, ccy, r_eff, a0, a0 + sweep)
        cx, cy = tox, toy

    _line(points, cx, cy, waypoints[-1][0], waypoints[-1][1])
    return points


def straight_offset():
    pts = [(0.0, 0.0)]
    _line(pts, 0.0, 0.0, 30.0, 0.0)
    return pts, (0.0, 1.0, 0.0)


def heading_offset():
    pts = [(0.0, 0.0)]
    _line(pts, 0.0, 0.0, 30.0, 0.0)
    return pts, (0.0, 0.0, math.radians(30.0))


def circle():
    r = 6.0
    pts = [(r, 0.0)]
    _arc(pts, 0.0, 0.0, r, 0.0, 2 * math.pi)
    return pts, (r, 0.0, math.pi / 2)


def corner():
    r = 1.5
    pts = [(0.0, 0.0)]
    _line(pts, 0.0, 0.0, 8.0 - r, 0.0)
    _arc(pts, 8.0 - r, r, r, -math.pi / 2, 0.0)
    _line(pts, 8.0, r, 8.0, 8.0 - r)
    _arc(pts, 8.0 + r, 8.0 - r, r, math.pi, math.pi / 2)
    _line(pts, 8.0 + r, 8.0, 16.0, 8.0)
    return pts, (0.0, 0.0, 0.0)


def figure_eight():
    a = 8.0
    pts = []
    n = 2400
    for i in range(n + 1):
        t = 2 * math.pi * i / n
        pts.append((a * math.sin(t), a * math.sin(t) * math.cos(t)))
    return pts, (0.0, 0.0, math.pi / 4)


def campus_loop():
    r, w, h = 3.0, 40.0, 25.0
    pts = [(r, 0.0)]
    _line(pts, r, 0.0, w - r, 0.0)
    _arc(pts, w - r, r, r, -math.pi / 2, 0.0)
    _line(pts, w, r, w, h - r)
    _arc(pts, w - r, h - r, r, 0.0, math.pi / 2)
    _line(pts, w - r, h, r, h)
    _arc(pts, r, h - r, r, math.pi / 2, math.pi)
    _line(pts, 0.0, h - r, 0.0, r)
    _arc(pts, r, r, r, math.pi, 1.5 * math.pi)
    return pts, (r, 0.0, 0.0)


def sharp_turns():
    leg = 6.0
    wps = [(0.0, 0.0)]
    for i in range(4):
        y0 = 2.0 * leg * i
        wps += [(leg, y0), (leg, y0 + leg), (0.0, y0 + leg), (0.0, y0 + 2 * leg)]
    return _filleted(wps, 1.2), (0.0, 0.0, 0.0)


#: name -> callable returning (points, (start_x, start_y, start_yaw))
SCENARIOS = {
    "straight_offset": straight_offset,
    "heading_offset": heading_offset,
    "circle": circle,
    "corner": corner,
    "figure_eight": figure_eight,
    "campus_loop": campus_loop,
    "sharp_turns": sharp_turns,
}

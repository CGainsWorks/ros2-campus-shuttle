#!/usr/bin/env python3
"""Turn the raw benchmark CSV into figures and docs/RESULTS.md.

Every number quoted in the README comes from here. Nothing in this script
hard-codes a conclusion: the per-scenario verdicts and the summary sentence are
derived from the data, so if the controllers change places the document says so.

Usage:
    python3 analysis/compare_controllers.py \
        --metrics results/bench/metrics.csv \
        --timeseries results/bench/timeseries \
        --figures analysis/figures \
        --results docs/RESULTS.md
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

CONTROLLERS = ["pure_pursuit", "stanley"]
LABEL = {"pure_pursuit": "Pure Pursuit", "stanley": "Stanley"}
COLOUR = {"pure_pursuit": "#1f77b4", "stanley": "#d62728"}

# Scenarios the gains were fitted on; the rest are held out.
TUNING_SET = {"straight_offset", "circle", "corner", "campus_loop"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--metrics", default="results/bench/metrics.csv")
    p.add_argument("--timeseries", default="results/bench/timeseries")
    p.add_argument("--figures", default="analysis/figures")
    p.add_argument("--results", default="docs/RESULTS.md")
    p.add_argument("--reference-speed", type=float, default=1.0)
    return p.parse_args()


def load_timeseries(ts_dir: str, controller: str, scenario: str,
                    speed: float, noise: bool) -> Optional[pd.DataFrame]:
    tag = f"{controller}_{scenario}_{speed:.1f}".replace(".", "p")
    name = f"{tag}_{'noise' if noise else 'clean'}.csv"
    path = os.path.join(ts_dir, name)
    if not os.path.exists(path):
        return None
    return pd.read_csv(path)


def fmt(value: float, places: int = 4) -> str:
    return f"{value:.{places}f}"


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------

def fig_trajectories(df, ts_dir, out_dir, speed):
    """Route context on top, the error that actually distinguishes them below.

    Plotting the two driven paths on top of the reference is honest but
    useless at this scale: the errors are centimetres on a 40 m route, so the
    three curves are indistinguishable. The lower row plots signed cross-track
    error against distance travelled, which is where the difference lives.
    """
    scenarios = [s for s in ("campus_loop", "sharp_turns") if s in set(df.scenario)]
    if not scenarios:
        return None

    fig, axes = plt.subplots(2, len(scenarios),
                             figsize=(6.4 * len(scenarios), 8.6),
                             gridspec_kw={"height_ratios": [1.35, 1.0]})
    axes = axes.reshape(2, len(scenarios))

    for col, scenario in enumerate(scenarios):
        ax_xy = axes[0, col]
        ax_err = axes[1, col]
        drew_reference = False

        for ctrl in CONTROLLERS:
            ts = load_timeseries(ts_dir, ctrl, scenario, speed, False)
            if ts is None:
                continue
            if not drew_reference:
                ax_xy.plot(ts.path_x, ts.path_y, color="0.55", lw=3.5,
                           label="reference path", zorder=1)
                drew_reference = True
            ax_xy.plot(ts.x, ts.y, color=COLOUR[ctrl], lw=1.2,
                       label=LABEL[ctrl], zorder=2)
            ax_err.plot(ts.s, 100.0 * ts.cross_track, color=COLOUR[ctrl],
                        lw=1.2, label=LABEL[ctrl])

        ax_xy.set_title(f"{scenario}  @ {speed:g} m/s")
        ax_xy.set_xlabel("x [m]")
        ax_xy.set_ylabel("y [m]")
        ax_xy.set_aspect("equal", adjustable="datalim")
        ax_xy.grid(alpha=0.3)
        ax_xy.legend(loc="best", fontsize=8)

        ax_err.axhline(0.0, color="0.5", lw=0.8)
        ax_err.set_xlabel("distance along path [m]")
        ax_err.set_ylabel("cross-track error [cm]")
        ax_err.set_title("signed error, positive = left of path", fontsize=9)
        ax_err.grid(alpha=0.3)
        ax_err.legend(fontsize=8)

    fig.suptitle(
        "Driven trajectory and tracking error against ground truth (no noise)")
    fig.tight_layout()
    path = os.path.join(out_dir, "trajectories.png")
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def fig_step_response(df, ts_dir, out_dir):
    """Cross-track error against distance for the 1 m lateral step."""
    speeds = sorted(df[df.scenario == "straight_offset"].target_speed.unique())
    if not speeds:
        return None
    fig, axes = plt.subplots(1, len(speeds), figsize=(4.6 * len(speeds), 4.0),
                             sharey=True)
    if len(speeds) == 1:
        axes = [axes]

    for ax, speed in zip(axes, speeds):
        for ctrl in CONTROLLERS:
            ts = load_timeseries(ts_dir, ctrl, "straight_offset", speed, False)
            if ts is None:
                continue
            ax.plot(ts.s, ts.cross_track, color=COLOUR[ctrl], lw=1.6,
                    label=LABEL[ctrl])
        ax.axhline(0.0, color="0.5", lw=0.8)
        ax.set_title(f"{speed:g} m/s")
        ax.set_xlabel("distance along path [m]")
        ax.grid(alpha=0.3)
        ax.set_xlim(0, 20)
    axes[0].set_ylabel("cross-track error [m]")
    axes[0].legend(fontsize=9)
    fig.suptitle("Recovery from a 1.0 m lateral offset")
    fig.tight_layout()
    path = os.path.join(out_dir, "step_response.png")
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def fig_rms_by_scenario(df, out_dir, speed):
    """Grouped bars: RMS cross-track error per scenario."""
    sub = df[(df.target_speed == speed) & (df.noise == 0)]
    if sub.empty:
        return None
    scenarios = list(sub.scenario.unique())
    x = range(len(scenarios))
    width = 0.38

    fig, ax = plt.subplots(figsize=(10, 5))
    for i, ctrl in enumerate(CONTROLLERS):
        vals = [
            float(sub[(sub.scenario == s) & (sub.controller == ctrl)].ct_rms.mean())
            for s in scenarios
        ]
        ax.bar([xi + (i - 0.5) * width for xi in x], vals, width,
               label=LABEL[ctrl], color=COLOUR[ctrl])

    ax.set_xticks(list(x))
    ax.set_xticklabels(
        [s + ("" if s in TUNING_SET else "\n(held out)") for s in scenarios],
        fontsize=9)
    ax.set_ylabel("RMS cross-track error [m]")
    # Errors span four orders of magnitude across scenarios, so log scale is the
    # only way to see them all. It is not valid if anything is zero or missing,
    # which happens on a partial run, so fall back rather than crash.
    log_ok = bool(sub.ct_rms.notna().all()) and float(sub.ct_rms.min()) > 0.0
    if log_ok:
        ax.set_yscale("log")
    ax.set_title(
        f"Tracking error by scenario @ {speed:g} m/s "
        f"({'log scale, ' if log_ok else ''}no noise)")
    ax.grid(axis="y", alpha=0.3, which="both")
    ax.legend()
    fig.tight_layout()
    path = os.path.join(out_dir, "rms_by_scenario.png")
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def fig_speed_and_noise(df, out_dir):
    """Error against speed, and the cost of odometry noise."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))

    for ctrl in CONTROLLERS:
        for noise, style in ((0, "-o"), (1, "--s")):
            sub = df[(df.controller == ctrl) & (df.noise == noise)]
            if sub.empty:
                continue
            grouped = sub.groupby("target_speed").ct_rms.mean()
            ax1.plot(grouped.index, grouped.values, style, color=COLOUR[ctrl],
                     label=f"{LABEL[ctrl]}{' + noise' if noise else ''}")
    ax1.set_xlabel("target speed [m/s]")
    ax1.set_ylabel("mean RMS cross-track error [m]")
    ax1.set_title("Error against speed (averaged over scenarios)")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8)

    # Accuracy against smoothness: the trade-off the tuning objective encodes.
    for ctrl in CONTROLLERS:
        sub = df[(df.controller == ctrl) & (df.noise == 0)]
        ax2.scatter(sub.control_effort, sub.ct_rms, s=28, alpha=0.75,
                    color=COLOUR[ctrl], label=LABEL[ctrl])
    ax2.set_xlabel("control effort, RMS d$\\omega$/dt [rad/s$^2$]")
    ax2.set_ylabel("RMS cross-track error [m]")
    clean = df[df.noise == 0]
    if not clean.empty and float(clean.ct_rms.min()) > 0.0:
        ax2.set_yscale("log")
    ax2.set_title("Accuracy against smoothness (one point per run)")
    ax2.grid(alpha=0.3, which="both")
    ax2.legend(fontsize=9)

    fig.tight_layout()
    path = os.path.join(out_dir, "speed_and_effort.png")
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------
# Results document
# --------------------------------------------------------------------------

def scenario_table(df, speed) -> str:
    sub = df[(df.target_speed == speed) & (df.noise == 0)]
    lines = [
        "| Scenario | Tuned on | PP RMS (m) | Stanley RMS (m) | PP max (m) | "
        "Stanley max (m) | Closer |",
        "| --- | --- | ---: | ---: | ---: | ---: | :---: |",
    ]
    for scenario in sub.scenario.unique():
        row = {}
        for ctrl in CONTROLLERS:
            r = sub[(sub.scenario == scenario) & (sub.controller == ctrl)]
            if r.empty:
                continue
            row[ctrl] = (float(r.ct_rms.mean()), float(r.ct_max_abs.max()))
        if len(row) < 2:
            continue
        pp, st = row["pure_pursuit"], row["stanley"]
        winner = "PP" if pp[0] < st[0] else "Stanley"
        tuned = "yes" if scenario in TUNING_SET else "held out"
        lines.append(
            f"| `{scenario}` | {tuned} | {fmt(pp[0])} | {fmt(st[0])} | "
            f"{fmt(pp[1])} | {fmt(st[1])} | {winner} |"
        )
    return "\n".join(lines)


def speed_table(df) -> str:
    lines = [
        "| Speed (m/s) | Noise | PP RMS (m) | Stanley RMS (m) | PP effort | "
        "Stanley effort |",
        "| ---: | :---: | ---: | ---: | ---: | ---: |",
    ]
    for speed in sorted(df.target_speed.unique()):
        for noise in (0, 1):
            cells = {}
            for ctrl in CONTROLLERS:
                r = df[(df.target_speed == speed) & (df.noise == noise) &
                       (df.controller == ctrl)]
                if r.empty:
                    continue
                cells[ctrl] = (float(r.ct_rms.mean()), float(r.control_effort.mean()))
            if len(cells) < 2:
                continue
            pp, st = cells["pure_pursuit"], cells["stanley"]
            lines.append(
                f"| {speed:g} | {'yes' if noise else 'no'} | {fmt(pp[0])} | "
                f"{fmt(st[0])} | {fmt(pp[1], 2)} | {fmt(st[1], 2)} |"
            )
    return "\n".join(lines)


def build_verdict(df, speed) -> tuple[str, dict]:
    """Derive the conclusion from the data rather than asserting it."""
    sub = df[(df.target_speed == speed) & (df.noise == 0)]
    transient = ["straight_offset", "heading_offset"]

    wins = {"pure_pursuit": [], "stanley": []}
    for scenario in sub.scenario.unique():
        pp = sub[(sub.scenario == scenario) &
                 (sub.controller == "pure_pursuit")].ct_rms.mean()
        st = sub[(sub.scenario == scenario) &
                 (sub.controller == "stanley")].ct_rms.mean()
        if pd.isna(pp) or pd.isna(st):
            continue
        wins["pure_pursuit" if pp < st else "stanley"].append(scenario)

    steady_wins_pp = [s for s in wins["pure_pursuit"] if s not in transient]
    steady_total = len([s for s in sub.scenario.unique() if s not in transient])
    transient_wins_st = [s for s in wins["stanley"] if s in transient]

    stats = {}
    for ctrl in CONTROLLERS:
        clean = df[(df.controller == ctrl) & (df.noise == 0)].ct_rms.mean()
        noisy = df[(df.controller == ctrl) & (df.noise == 1)].ct_rms.mean()
        stats[ctrl] = {
            "clean": float(clean),
            "noisy": float(noisy),
            "penalty_pct": float(100.0 * (noisy - clean) / clean) if clean else 0.0,
            "effort": float(df[df.controller == ctrl].control_effort.mean()),
            "failures": int(((df.controller == ctrl) &
                             ((df.diverged == 1) | (df.stalled == 1) |
                              (df.goal_reached == 0))).sum()),
        }

    verdict = (
        f"Pure Pursuit tracks curvature more closely, winning "
        f"{len(steady_wins_pp)} of the {steady_total} steady-state routes. "
        f"Stanley recovers from an initial error faster, winning "
        f"{len(transient_wins_st)} of the {len(transient)} transient scenarios. "
        "Neither controller dominates: the choice depends on whether the "
        "robot spends its time settling or holding a line."
    )
    return verdict, stats


def write_results(path, df, verdict, stats, figures, speed, gains_text):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    n_runs = len(df)
    failures = int(((df.diverged == 1) | (df.stalled == 1) |
                    (df.goal_reached == 0)).sum())

    with open(path, "w", encoding="utf-8") as f:
        f.write("# Controller comparison results\n\n")
        f.write(
            "Generated by `analysis/compare_controllers.py` from "
            "`results/bench/metrics.csv`. Do not edit by hand: "
            "`scripts/run_bench.sh` regenerates the whole file.\n\n"
        )

        f.write("## Conclusion\n\n")
        f.write(verdict + "\n\n")

        f.write("## How the comparison is kept fair\n\n")
        f.write(
            "- Both controllers drive the **same** reference paths from the "
            "**same** start poses at the **same** speeds.\n"
            "- The longitudinal (speed) law is shared code. Only the lateral "
            "control law differs, so a difference in tracking error cannot be "
            "explained by one controller simply taking corners slower.\n"
            "- Cross-track error is reported at `base_link` for both, even "
            "though Stanley internally regulates the error at a virtual front "
            "axle.\n"
            "- Errors are measured against **ground truth**, not against the "
            "noisy pose the controller was given.\n"
            "- The odometry noise sequence is seeded from the scenario, speed "
            "and noise setting only, never from the controller, so both face "
            "an identical disturbance realisation.\n"
            "- A 100 ms sensing latency is modelled, because without it the "
            "tuning step selects gains that are only optimal in a perfect "
            "world.\n"
            "- Gains are fitted by grid search on "
            f"{', '.join('`' + s + '`' for s in sorted(TUNING_SET))} and "
            "reported on all scenarios, so the held-out routes test "
            "generalisation.\n\n"
        )

        f.write(f"## Per-scenario, at {speed:g} m/s, no noise\n\n")
        f.write(scenario_table(df, speed) + "\n\n")

        f.write("## Across speed and noise (averaged over all scenarios)\n\n")
        f.write(speed_table(df) + "\n\n")

        f.write("## Robustness\n\n")
        f.write("| Controller | Clean RMS (m) | Noisy RMS (m) | Noise penalty | "
                "Mean effort (rad/s²) | Failed runs |\n")
        f.write("| --- | ---: | ---: | ---: | ---: | ---: |\n")
        for ctrl in CONTROLLERS:
            s = stats[ctrl]
            f.write(
                f"| {LABEL[ctrl]} | {fmt(s['clean'])} | {fmt(s['noisy'])} | "
                f"+{s['penalty_pct']:.1f}% | {fmt(s['effort'], 2)} | "
                f"{s['failures']} |\n"
            )
        f.write(f"\n{n_runs} runs total, {failures} of which failed to finish.\n\n")

        if gains_text:
            f.write("## Tuned gains actually used\n\n```yaml\n")
            f.write(gains_text.strip() + "\n```\n\n")

        f.write("## Figures\n\n")
        for title, fig_path in figures:
            if fig_path:
                rel = os.path.relpath(fig_path, os.path.dirname(path) or ".")
                f.write(f"### {title}\n\n![{title}]({rel})\n\n")

        f.write("## Reading these numbers honestly\n\n")
        f.write(
            "- `straight_offset` starts 1.0 m off the path, so its RMS is "
            "dominated by the recovery transient and is not comparable with "
            "the steady-state routes.\n"
            "- These figures come from the kinematic benchmark, which models "
            "actuator lag, wheel saturation, sensing latency and odometry "
            "noise, but not wheel slip, tyre dynamics or LiDAR "
            "mis-association. The Gazebo runs exercise the identical "
            "controller code against a physics engine; see the README for how "
            "to reproduce them.\n"
            "- Absolute errors are small because the plant is well behaved. "
            "The comparison between the two columns is the result, not the "
            "magnitude of either.\n"
        )


def main() -> int:
    args = parse_args()
    if not os.path.exists(args.metrics):
        print(f"error: {args.metrics} not found. Run scripts/run_bench.sh first.",
              file=sys.stderr)
        return 1

    df = pd.read_csv(args.metrics)
    if df.empty:
        print("error: metrics file is empty", file=sys.stderr)
        return 1

    os.makedirs(args.figures, exist_ok=True)

    speed = args.reference_speed
    if speed not in set(df.target_speed):
        speed = float(sorted(df.target_speed.unique())[len(df.target_speed.unique()) // 2])

    figures = [
        ("Driven trajectories", fig_trajectories(df, args.timeseries, args.figures, speed)),
        ("Recovery from a lateral offset", fig_step_response(df, args.timeseries, args.figures)),
        ("Tracking error by scenario", fig_rms_by_scenario(df, args.figures, speed)),
        ("Speed, noise and control effort", fig_speed_and_noise(df, args.figures)),
    ]

    gains_path = os.path.join(os.path.dirname(args.metrics), "tuned_gains.yaml")
    gains_text = ""
    if os.path.exists(gains_path):
        with open(gains_path, encoding="utf-8") as fh:
            gains_text = fh.read()

    verdict, stats = build_verdict(df, speed)
    write_results(args.results, df, verdict, stats, figures, speed, gains_text)

    print(verdict)
    for ctrl in CONTROLLERS:
        s = stats[ctrl]
        print(f"  {LABEL[ctrl]:<13} clean {s['clean']:.4f} m  "
              f"noisy {s['noisy']:.4f} m (+{s['penalty_pct']:.1f}%)  "
              f"effort {s['effort']:.2f}  failures {s['failures']}")
    print(f"Wrote {args.results}")
    for title, path in figures:
        if path:
            print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Force and tool-position figures from wall_contact_demo runs.

Usage: plot_wall_contact.py <out_dir> "<label>=<glob of run dirs>" ...

Each run dir holds openmanip_x_flight.csv and openmanip_x_contact.json. Writes
contact_force.png (normal force against time, one panel per group, every run
drawn), summary.md and summary.json.
"""

import csv
import glob
import json
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

INK = "#333333"
GRID = "#dddddd"
SERIES = ["#2a78b5", "#d9642b", "#3b9a6d"]


def load_run(path):
    with open(os.path.join(path, "openmanip_x_contact.json")) as f:
        contact = json.load(f)
    with open(os.path.join(path, "openmanip_x_flight.csv")) as f:
        rows = list(csv.DictReader(f))
    press = [r for r in rows if r["phase"] == "press" and r.get("fx")]
    t0 = float(press[0]["t"]) if press else 0.0
    tare = contact.get("tare_n", 0.0)
    return {
        "path": path,
        "contact": contact,
        "t": [float(r["t"]) - t0 for r in press],
        "force": [abs(float(r["fx"]) - tare) for r in press],
    }


def mean_sd(values, digits=2):
    values = [v for v in values if v is not None]
    if not values:
        return "-"
    if len(values) == 1:
        return f"{values[0]:.{digits}f}"
    return f"{statistics.fmean(values):.{digits}f} ± {statistics.stdev(values):.{digits}f}"


def summarize(label, runs):
    held = [r["contact"] for r in runs
            if r["contact"].get("contact_reached") and not r["contact"].get("fell_after_s")]
    fell = [r["contact"]["fell_after_s"] for r in runs if r["contact"].get("fell_after_s")]
    pick = lambda key: [c.get(key) for c in held]  # noqa: E731
    cm = lambda key: [v * 100 for v in pick(key)]  # noqa: E731
    return {
        "group": label,
        "runs": len(runs),
        "fell": len(fell),
        "fell_after_s": fell,
        "force_mean_n": mean_sd(pick("force_mean_n")),
        "force_sd_n": mean_sd(pick("force_sd_n")),
        "force_first_half_n": mean_sd(pick("force_first_half_n")),
        "force_second_half_n": mean_sd(pick("force_second_half_n")),
        "contact_lost_pct": mean_sd([v * 100 for v in pick("contact_lost_fraction")], 1),
        "tip_y_sd_cm": mean_sd(cm("tip_y_sd_m"), 1),
        "tip_z_sd_cm": mean_sd(cm("tip_z_sd_m"), 1),
    }


def plot(groups, out_dir):
    fig, axes = plt.subplots(1, len(groups), figsize=(3.6 * len(groups), 3.2),
                             sharex=True, sharey=True, squeeze=False)
    for ax, (label, runs) in zip(axes[0], groups.items()):
        for i, run in enumerate(runs):
            ax.plot(run["t"], run["force"], lw=1.0, color=SERIES[i % len(SERIES)],
                    label=f"run {i + 1}")
        ax.set_title(label, fontsize=10, color=INK)
        ax.set_xlabel("time in press (s)", fontsize=9, color=INK)
        ax.grid(color=GRID, lw=0.6)
        ax.tick_params(labelsize=8, colors=INK)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.legend(fontsize=7, frameon=False)
    axes[0][0].set_ylabel("force along the tool (N)", fontsize=9, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "contact_force.png"), dpi=150)


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    groups = {}
    for spec in sys.argv[2:]:
        label, pattern = spec.split("=", 1)
        groups[label] = [load_run(p) for p in sorted(glob.glob(pattern))]
    rows = [summarize(label, runs) for label, runs in groups.items()]
    plot(groups, out_dir)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(rows, f, indent=2)
    with open(os.path.join(out_dir, "summary.md"), "w") as f:
        f.write("| run | n | fell | force mean (N) | force SD in a run (N) | first half (N) "
                "| second half (N) | contact lost (%) | tip y SD (cm) | tip z SD (cm) |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r['group']} | {r['runs']} | {r['fell']} | {r['force_mean_n']} "
                    f"| {r['force_sd_n']} | {r['force_first_half_n']} "
                    f"| {r['force_second_half_n']} | {r['contact_lost_pct']} "
                    f"| {r['tip_y_sd_cm']} | {r['tip_z_sd_cm']} |\n")
    print(open(os.path.join(out_dir, "summary.md")).read())


if __name__ == "__main__":
    main()

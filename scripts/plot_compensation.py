#!/usr/bin/env python3
"""Compare hover error across controllers: mean peak and RMS per arm phase.

Usage: plot_compensation.py <out_dir> <panel title> "<label>=<glob of *_summary.json>" ...
       [--panel <title> "<label>=<glob>" ...]

Writes compensation.png (one panel per --panel) and summary.md / summary.json.
"""

import glob
import json
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PHASES = [("hover", "hover"), ("arm_reach_a", "forward"), ("arm_reach_b", "side"),
          ("arm_fold", "back"), ("arm_stow", "stow")]
COLORS = ["#8a8a8a", "#2a78b5", "#d9642b"]
INK = "#333333"
GRID = "#dddddd"


def load(pattern):
    """Per phase: lists of RMS and peak horizontal error (cm) across good runs."""
    out = {p: {"rms": [], "peak": [], "z": []} for p, _ in PHASES}
    runs = 0
    for path in sorted(glob.glob(pattern)):
        with open(path) as f:
            data = json.load(f)
        if data.get("error"):
            continue
        runs += 1
        for phase, _ in PHASES:
            s = data["phases"].get(phase)
            if s and s["horizontal_error_m"]["n"]:
                out[phase]["rms"].append(s["horizontal_error_m"]["rms"] * 100)
                out[phase]["peak"].append(s["horizontal_error_m"]["max"] * 100)
                out[phase]["z"].append(s["vertical_error_m"]["rms"] * 100)
    return runs, out


def mean_sd(values):
    if len(values) < 2:
        return f"{values[0]:.1f}" if values else "-"
    return f"{statistics.fmean(values):.1f} ± {statistics.stdev(values):.1f}"


def main():
    args = sys.argv[2:]
    out_dir = sys.argv[1]
    os.makedirs(out_dir, exist_ok=True)
    panels = []
    for arg in args:
        if arg == "--panel":
            panels.append([None, []])
        elif panels[-1][0] is None:
            panels[-1][0] = arg
        else:
            label, pattern = arg.split("=", 1)
            panels[-1][1].append((label, load(pattern)))

    fig, axes = plt.subplots(1, len(panels), figsize=(5.2 * len(panels), 3.4), sharey=True,
                             squeeze=False)
    lines = ["| motions | controller | n | " + " | ".join(n for _, n in PHASES)
             + " |  (RMS / peak, cm) |", "|---|---|---|---|---|---|---|---|---|"]
    table = []
    for ax, (title, series) in zip(axes[0], panels):
        width = 0.8 / len(series)
        for i, (label, (runs, data)) in enumerate(series):
            xs = [j + i * width for j in range(len(PHASES))]
            ys = [statistics.fmean(data[p]["peak"]) if data[p]["peak"] else 0 for p, _ in PHASES]
            errs = [statistics.stdev(data[p]["peak"]) if len(data[p]["peak"]) > 1 else 0
                    for p, _ in PHASES]
            ax.bar(xs, ys, width * 0.92, yerr=errs, color=COLORS[i % len(COLORS)], label=label,
                   error_kw={"lw": 0.8, "ecolor": INK})
            cells = [f"{mean_sd(data[p]['rms'])} / {mean_sd(data[p]['peak'])}" for p, _ in PHASES]
            lines.append(f"| {title} | {label} | {runs} | " + " | ".join(cells) + " |")
            table.append({"motions": title, "controller": label, "runs": runs, "phases": data})
        ax.set_xticks([j + 0.4 - width / 2 for j in range(len(PHASES))], [n for _, n in PHASES])
        ax.set_title(title, fontsize=10, color=INK)
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(labelsize=8, colors=INK)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.legend(fontsize=8, frameon=False)
    axes[0][0].set_ylabel("peak horizontal error (cm)", fontsize=9, color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "compensation.png"), dpi=150)
    with open(os.path.join(out_dir, "summary.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(table, f, indent=2)
    print("\n".join(lines))


if __name__ == "__main__":
    main()

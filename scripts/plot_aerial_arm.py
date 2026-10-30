#!/usr/bin/env python3
"""Plot aerial-arm flight experiments written by `aerial_arm_demo`.

Usage:
    plot_aerial_arm.py <results_dir> [figure_dir]

For each ``<arm>_flight.csv`` in results_dir (and one subdirectory level
below, for control runs) writes ``<label>_flight.png``: hover position error
(top) and arm joint reference vs measured (bottom), with the experiment
phases shaded. With more than one run it also writes
``hover_error_by_arm.png`` (per-phase horizontal RMS and mean vertical
error), plus ``summary.md`` / ``summary.json`` with the numbers.
"""

import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Reference categorical order (light surface), never cycled.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#8a5cd1"]
INK, INK2, GRID, SURFACE = "#1f1f1e", "#5f5e58", "#e4e3de", "#fcfcfb"
FLIGHT_PHASES = ("hover", "arm_reach_a", "arm_reach_b", "arm_fold", "arm_stow")

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "xtick.color": INK2, "ytick.color": INK2, "grid.color": GRID,
    "axes.grid": True, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 9, "legend.frameon": False,
    "lines.linewidth": 1.6,
})


def _load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        d = {"phase": r["phase"]}
        for k, v in r.items():
            if k != "phase":
                d[k] = float(v) if v not in ("", None) else None
        out.append(d)
    return out


def _phase_spans(rows):
    spans, start, cur = [], rows[0]["t"], rows[0]["phase"]
    for r in rows[1:]:
        if r["phase"] != cur:
            spans.append((cur, start, r["t"]))
            start, cur = r["t"], r["phase"]
    spans.append((cur, start, rows[-1]["t"]))
    return spans


def plot_flight(rows, arm, out_png):
    rows = [r for r in rows if r["phase"] in FLIGHT_PHASES]
    t0 = rows[0]["t"]
    t = [r["t"] - t0 for r in rows]
    horiz = [math.hypot(r["x"] - r["sp_x"], r["y"] - r["sp_y"]) for r in rows]
    vert = [r["z"] - r["sp_z"] for r in rows]
    joints = [k[2:] for k in rows[0] if k.startswith("q_")]

    if joints:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5.2), sharex=True,
                                       gridspec_kw={"height_ratios": [1, 1.2]})
    else:  # baseline flight without an arm
        fig, ax1 = plt.subplots(figsize=(8, 2.8))
        ax2 = None
    for i, (phase, a, b) in enumerate(_phase_spans(rows)):
        for ax in (ax1, ax2) if ax2 else (ax1,):
            if i % 2:
                ax.axvspan(a - t0, b - t0, color="#f1f0ec", lw=0, zorder=0)
        ax1.text((a + b) / 2 - t0, 1.02, phase.replace("arm_", ""), ha="center",
                 va="bottom", fontsize=8, color=INK2, transform=ax1.get_xaxis_transform())

    ax1.plot(t, horiz, color=SERIES[0], label="horizontal |e_xy|")
    ax1.plot(t, vert, color=SERIES[1], label="vertical e_z")
    ax1.axhline(0, color=INK2, lw=0.8)
    ax1.set_ylabel("position error (m)")
    ax1.legend(loc="center right", ncol=2)
    ax1.set_title(f"{arm}: hover error while the arm moves", loc="left", pad=16)

    if ax2 is None:
        ax1.set_xlabel("time since hover start (s)")
    for i, j in enumerate(joints):
        c = SERIES[i % len(SERIES)] if len(joints) <= len(SERIES) else SERIES[0]
        ax2.plot(t, [r[f"ref_{j}"] for r in rows], color=c, lw=1.0, ls="--")
        ax2.plot(t, [r[f"q_{j}"] for r in rows], color=c, label=j)
    if ax2 is not None:
        ax2.set_ylabel("joint angle (rad)")
        ax2.set_xlabel("time since hover start (s)   solid = measured, dashed = reference")
        ax2.legend(loc="lower right", ncol=min(4, len(joints)), fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def phase_stats(rows):
    """Per-phase hover statistics straight from logged rows (cm)."""
    out = {}
    for p in FLIGHT_PHASES:
        rs = [r for r in rows if r["phase"] == p]
        if not rs:
            continue
        h = [math.hypot(r["x"] - r["sp_x"], r["y"] - r["sp_y"]) for r in rs]
        v = [r["z"] - r["sp_z"] for r in rs]
        je = [r["joint_err_norm"] for r in rs if r.get("joint_err_norm") is not None]
        out[p] = {
            "h_rms": 100 * math.sqrt(sum(x * x for x in h) / len(h)),
            "h_max": 100 * max(h),
            "v_mean": 100 * sum(v) / len(v),
            "v_rms": 100 * math.sqrt(sum(x * x for x in v) / len(v)),
            "j_rms": math.sqrt(sum(x * x for x in je) / len(je)) if je else None,
        }
    return out


def plot_comparison(stats, out_png):
    labels = list(stats)
    phases = [p for p in FLIGHT_PHASES if all(p in stats[k] for k in labels)]
    fig, axes = plt.subplots(2, 1, figsize=(8, 5.6), sharex=True)
    width = 0.8 / len(labels)
    for ax, key, ylabel in ((axes[0], "h_rms", "horizontal RMS error (cm)"),
                            (axes[1], "v_mean", "mean vertical error (cm)")):
        for i, label in enumerate(labels):
            vals = [stats[label][p][key] for p in phases]
            xs = [k + (i - (len(labels) - 1) / 2) * width for k in range(len(phases))]
            ax.bar(xs, vals, width=width - 0.02, color=SERIES[i % len(SERIES)],
                   label=label, zorder=2)
        ax.axhline(0, color=INK2, lw=0.8)
        ax.set_ylabel(ylabel)
        ax.grid(axis="x", visible=False)
    axes[0].set_title("Hover error per phase: arms vs no-arm controls", loc="left")
    axes[0].set_ylim(0, axes[0].get_ylim()[1] * 1.35)  # headroom for the legend
    axes[0].legend(ncol=3, loc="upper left", fontsize=8)
    axes[1].set_xticks(range(len(phases)))
    axes[1].set_xticklabels([p.replace("arm_", "") for p in phases])
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def write_table(stats, out_md):
    lines = ["| run | phase | horiz RMS (cm) | horiz max (cm) | vert mean (cm) | "
             "vert RMS (cm) | joint err RMS (rad) |", "|---|---|---|---|---|---|---|"]
    for label, ph in stats.items():
        for p, s in ph.items():
            j = f"{s['j_rms']:.4f}" if s["j_rms"] is not None else "-"
            lines.append(f"| {label} | {p} | {s['h_rms']:.1f} | {s['h_max']:.1f} | "
                         f"{s['v_mean']:.1f} | {s['v_rms']:.1f} | {j} |")
    with open(out_md, "w") as f:
        f.write("\n".join(lines) + "\n")


def discover(d):
    """(label, csv_path) for <arm>_flight.csv in d and one level below.

    A run in a subdirectory is labelled by the subdirectory (used for
    controls such as UAV=x500_d435 with arm none).
    """
    runs = []
    for f in sorted(os.listdir(d)):
        if f.endswith("_flight.csv"):
            arm = f[:-len("_flight.csv")]
            runs.append(("x500 (no arm)" if arm == "none" else arm, os.path.join(d, f)))
    for sub in sorted(os.listdir(d)):
        sd = os.path.join(d, sub)
        if os.path.isdir(sd):
            for f in sorted(os.listdir(sd)):
                if f.endswith("_flight.csv"):
                    runs.append((sub, os.path.join(sd, f)))
    return runs


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    d = argv[1]
    figdir = argv[2] if len(argv) > 2 else d
    os.makedirs(figdir, exist_ok=True)
    stats = {}
    for label, path in discover(d):
        rows = _load(path)
        name = label.split(" ")[0].replace("/", "_")
        plot_flight(rows, label, os.path.join(figdir, f"{name}_flight.png"))
        stats[label] = phase_stats(rows)
        print(f"wrote {name}_flight.png")
    if len(stats) > 1:
        plot_comparison(stats, os.path.join(figdir, "hover_error_by_arm.png"))
        print("wrote hover_error_by_arm.png")
    write_table(stats, os.path.join(figdir, "summary.md"))
    with open(os.path.join(figdir, "summary.json"), "w") as f:
        json.dump(stats, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

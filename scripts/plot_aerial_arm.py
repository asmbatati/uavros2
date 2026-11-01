#!/usr/bin/env python3
"""Plot and summarise aerial-arm flight experiments written by `aerial_arm_demo`.

Usage:
    plot_aerial_arm.py <results_dir> [figure_dir]
        One run per ``<arm>_flight.csv`` found in results_dir and one
        subdirectory level below (control runs), labelled by arm / subdir.

    plot_aerial_arm.py <figure_dir> <label>=<csv_glob>[,<csv_glob>...] ...
        Named groups of repeated runs, e.g.
        "x500 (no arm)=results/none_flight.csv,results/reps/none_rep*/none_flight.csv"

Writes into the figure directory:
  <name>_flight.png       per run (first run of each group): hover position
                          error and arm joint reference vs measured, with the
                          experiment phases shaded;
  hover_error_by_arm.png  per-phase horizontal RMS error, mean vertical error
                          and end-effector world error, mean +- SD over the
                          repeated runs of each group;
  summary.md / .json      the same numbers.

End-effector world error: the end effector's actual map-frame position
(drone pose composed with TF) against where it should be: the position
setpoint composed with the forward kinematics of the arm's reference
joints (level attitude, yaw 0 as commanded). It needs a sourced workspace
(uavros2.arm_kinematics + xacro); without one it is skipped.
"""

import csv
import glob
import json
import math
import os
import re
import shutil
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Reference categorical order (light surface), never cycled.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#8a5cd1"]
INK, INK2, GRID, SURFACE = "#1f1f1e", "#5f5e58", "#e4e3de", "#fcfcfb"
FLIGHT_PHASES = ("hover", "arm_reach_a", "arm_reach_b", "arm_fold", "arm_stow")
EE_LINKS = {"three_dof": "ee_link", "openmanip_x": "end_effector_link",
            "panda": "panda_hand", "ur5": "tool0"}
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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


def _arm_of(path):
    return os.path.basename(path)[:-len("_flight.csv")]


def _phase_spans(rows):
    spans, start, cur = [], rows[0]["t"], rows[0]["phase"]
    for r in rows[1:]:
        if r["phase"] != cur:
            spans.append((cur, start, r["t"]))
            start, cur = r["t"], r["phase"]
    spans.append((cur, start, rows[-1]["t"]))
    return spans


_URDF_CACHE = {}


def ee_errors(rows, arm):
    """Per-row end-effector world error (m), or None when it can't be computed."""
    if arm not in EE_LINKS:
        return None
    try:
        from uavros2.arm_kinematics import urdf_fk
    except ImportError:
        return None
    if arm not in _URDF_CACHE:
        xacro = shutil.which("xacro")
        path = os.path.join(PKG_DIR, "arms", arm, "urdf", f"{arm}.urdf.xacro")
        if xacro is None or not os.path.isfile(path):
            return None
        _URDF_CACHE[arm] = subprocess.check_output(
            [xacro, path, "sim_mode:=mock"], stderr=subprocess.DEVNULL).decode()
    urdf = _URDF_CACHE[arm]
    joints = [k[4:] for k in rows[0] if k.startswith("ref_")]
    fk_cache, out = {}, []
    for r in rows:
        if r.get("ee_x") is None or any(r.get(f"ref_{j}") is None for j in joints):
            out.append(None)
            continue
        q = tuple(round(r[f"ref_{j}"], 4) for j in joints)
        if q not in fk_cache:
            fk_cache[q] = urdf_fk(urdf, "base_link", EE_LINKS[arm], dict(zip(joints, q)))[:3, 3]
        p = fk_cache[q]
        out.append(math.dist((r["ee_x"], r["ee_y"], r["ee_z"]),
                             (r["sp_x"] + p[0], r["sp_y"] + p[1], r["sp_z"] + p[2])))
    return out


def plot_flight(rows, title, out_png):
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
    ax1.legend(loc="lower right", ncol=2, bbox_to_anchor=(1.0, 1.08))
    ax1.set_title(title, loc="left", pad=16)
    if ax2 is None:
        ax1.set_xlabel("time since hover start (s)")

    for i, j in enumerate(joints):
        c = SERIES[i % len(SERIES)]
        ax2.plot(t, [r[f"ref_{j}"] for r in rows], color=c, lw=1.0, ls="--")
        ax2.plot(t, [r[f"q_{j}"] for r in rows], color=c, label=j)
    if ax2 is not None:
        ax2.set_ylabel("joint angle (rad)")
        ax2.set_xlabel("time since hover start (s)   solid = measured, dashed = reference")
        ax2.legend(loc="lower right", ncol=min(4, len(joints)), fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def phase_stats(rows, ee_err=None):
    """Per-phase hover statistics of one run (cm, rad)."""
    out = {}
    for p in FLIGHT_PHASES:
        idx = [i for i, r in enumerate(rows) if r["phase"] == p]
        if not idx:
            continue
        rs = [rows[i] for i in idx]
        h = [math.hypot(r["x"] - r["sp_x"], r["y"] - r["sp_y"]) for r in rs]
        v = [r["z"] - r["sp_z"] for r in rs]
        je = [r["joint_err_norm"] for r in rs if r.get("joint_err_norm") is not None]
        ee = [ee_err[i] for i in idx if ee_err and ee_err[i] is not None]
        out[p] = {
            "h_rms": 100 * math.sqrt(sum(x * x for x in h) / len(h)),
            "h_max": 100 * max(h),
            "v_mean": 100 * sum(v) / len(v),
            "v_rms": 100 * math.sqrt(sum(x * x for x in v) / len(v)),
            "ee_rms": 100 * math.sqrt(sum(x * x for x in ee) / len(ee)) if ee else None,
            "ee_max": 100 * max(ee) if ee else None,
            "j_rms": math.sqrt(sum(x * x for x in je) / len(je)) if je else None,
        }
    return out


def aggregate(runs):
    """Mean and SD over runs of each per-phase statistic."""
    out = {}
    for p in FLIGHT_PHASES:
        per = [r[p] for r in runs if p in r]
        if not per:
            continue
        out[p] = {}
        for key in per[0]:
            vals = [s[key] for s in per if s[key] is not None]
            if not vals:
                out[p][key] = {"mean": None, "sd": None, "n": 0}
                continue
            m = sum(vals) / len(vals)
            sd = math.sqrt(sum((x - m) ** 2 for x in vals) / (len(vals) - 1)) if len(vals) > 1 else 0.0
            out[p][key] = {"mean": m, "sd": sd, "n": len(vals)}
    return out


def plot_comparison(groups, out_png):
    labels = list(groups)
    phases = [p for p in FLIGHT_PHASES if all(p in groups[k] for k in labels)]
    panels = [("h_rms", "horizontal RMS (cm)"), ("v_mean", "mean vertical (cm)")]
    if any(groups[k][p]["ee_rms"]["n"] for k in labels for p in phases):
        panels.append(("ee_rms", "end-effector RMS (cm)"))
    fig, axes = plt.subplots(len(panels), 1, figsize=(8, 2.3 * len(panels) + 0.8),
                             sharex=True)
    width = 0.8 / len(labels)
    for ax, (key, ylabel) in zip(axes, panels):
        for i, label in enumerate(labels):
            xs, ms, sds = [], [], []
            for k, p in enumerate(phases):
                s = groups[label][p][key]
                if s["mean"] is None:
                    continue
                xs.append(k + (i - (len(labels) - 1) / 2) * width)
                ms.append(s["mean"])
                sds.append(s["sd"])
            ax.bar(xs, ms, width=width - 0.02, color=SERIES[i % len(SERIES)],
                   label=label, zorder=2)
            if any(sds):
                ax.errorbar(xs, ms, yerr=sds, fmt="none", ecolor=INK2, elinewidth=1,
                            capsize=2, zorder=3)
        ax.axhline(0, color=INK2, lw=0.8)
        ax.set_ylabel(ylabel)
        ax.grid(axis="x", visible=False)
    n = max(groups[k][phases[0]]["h_rms"]["n"] for k in labels)
    axes[0].set_title("Hover error per phase" + (" (mean ± SD over repeated runs)"
                                                 if n > 1 else ""), loc="left")
    axes[0].set_ylim(0, axes[0].get_ylim()[1] * 1.35)  # headroom for the legend
    axes[0].legend(ncol=3, loc="upper left", fontsize=8)
    axes[-1].set_xticks(range(len(phases)))
    axes[-1].set_xticklabels([p.replace("arm_", "") for p in phases])
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def _fmt(s, nd=1):
    if s["mean"] is None:
        return "-"
    return f"{s['mean']:.{nd}f}" + (f" ± {s['sd']:.{nd}f}" if s["n"] > 1 else "")


def write_table(groups, out_md):
    lines = ["| run | n | phase | horiz RMS (cm) | horiz max (cm) | vert mean (cm) | "
             "EE RMS (cm) | joint err RMS (rad) |", "|---|---|---|---|---|---|---|---|"]
    for label, ph in groups.items():
        for p, s in ph.items():
            lines.append(f"| {label} | {s['h_rms']['n']} | {p.replace('arm_', '')} | "
                         f"{_fmt(s['h_rms'])} | {_fmt(s['h_max'])} | {_fmt(s['v_mean'])} | "
                         f"{_fmt(s['ee_rms'])} | {_fmt(s['j_rms'], 3)} |")
    with open(out_md, "w") as f:
        f.write("\n".join(lines) + "\n")


def discover(d):
    """{label: [csv]} for <arm>_flight.csv in d and one level below."""
    runs = {}
    for f in sorted(os.listdir(d)):
        if f.endswith("_flight.csv"):
            arm = _arm_of(f)
            runs["x500 (no arm)" if arm == "none" else arm] = [os.path.join(d, f)]
    for sub in sorted(os.listdir(d)):
        for f in sorted(glob.glob(os.path.join(d, sub, "*_flight.csv"))):
            runs[sub] = [f]
    return runs


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    if any("=" in a for a in argv[2:]):
        figdir = argv[1]
        groups_in = {}
        for spec in argv[2:]:
            label, globs = spec.split("=", 1)
            paths = sorted(p for g in globs.split(",") for p in glob.glob(g))
            if not paths:
                print(f"warning: no runs match {spec!r}")
                continue
            groups_in[label] = paths
    else:
        figdir = argv[2] if len(argv) > 2 else argv[1]
        groups_in = discover(argv[1])
    os.makedirs(figdir, exist_ok=True)

    groups, per_run = {}, {}
    for label, paths in groups_in.items():
        stats = []
        for k, path in enumerate(paths):
            rows = _load(path)
            stats.append(phase_stats(rows, ee_errors(rows, _arm_of(path))))
            per_run[path] = stats[-1]
            if k == 0:
                name = re.sub(r"[^A-Za-z0-9]+", "_", label).strip("_").lower()
                plot_flight(rows, f"{label}: hover error while the arm moves",
                            os.path.join(figdir, f"{name}_flight.png"))
                print(f"wrote {name}_flight.png")
        groups[label] = aggregate(stats)
    if len(groups) > 1:
        plot_comparison(groups, os.path.join(figdir, "hover_error_by_arm.png"))
        print("wrote hover_error_by_arm.png")
    write_table(groups, os.path.join(figdir, "summary.md"))
    with open(os.path.join(figdir, "summary.json"), "w") as f:
        json.dump({"groups": groups, "runs": per_run}, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

import csv
import sys
import os
import re
import glob
from collections import defaultdict
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ── patterns ────────────────────────────────────────────────────────────────

PHASE_START = {
    "Homing": re.compile(r"Gripper homing"),
    "Latch":  re.compile(r"Gripper latching"),
    "Home":   re.compile(r"Gripper releasing"),
    "Idle":   re.compile(r"Gripper moving to idle"),
}
PHASE_TOOK_RE = re.compile(r"Phase took (\d+) ms")

# ── paths ────────────────────────────────────────────────────────────────────

DATA_DIR  = os.path.join(os.path.dirname(__file__), "..", "Data")
PLOTS_DIR = os.path.join(DATA_DIR, "plots")

PHASE_COLORS = {
    "Homing": "#3498db",
    "Latch":  "#e74c3c",
    "Home":   "#2ecc71",
    "Idle":   "#e67e22",
}

# ── loading ──────────────────────────────────────────────────────────────────

def find_all_logs():
    return sorted(glob.glob(os.path.join(DATA_DIR, "*", "gripper_log.csv")))


def resolve_paths(args):
    """Accept CSV files or Data/<timestamp>/ directories."""
    paths = []
    for a in args:
        if a.endswith(".csv") and os.path.isfile(a):
            paths.append(a)
        else:
            p = os.path.join(a, "gripper_log.csv")
            if os.path.isfile(p):
                paths.append(p)
    return sorted(paths)


def load_run(csv_path):
    """Parse one CSV into a list of phase dicts."""
    phases = []
    active = None  # the phase currently being built

    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            elapsed = float(row["elapsed_ms"])

            if row["current_A"]:
                if active is not None:
                    active["rel_times"].append(elapsed - active["start_ms"])
                    active["currents"].append(float(row["current_A"]))

            elif row["event"]:
                event = row["event"]

                # phase start?
                for name, pat in PHASE_START.items():
                    if pat.search(event):
                        active = {
                            "name": name,
                            "start_ms": elapsed,
                            "duration_ms": None,
                            "rel_times": [],
                            "currents": [],
                        }
                        phases.append(active)
                        break

                # phase end?
                m = PHASE_TOOK_RE.search(event)
                if m and active is not None:
                    active["duration_ms"] = int(m.group(1))
                    active = None

    return phases


def load_all_runs(paths):
    runs = []
    for path in paths:
        phases = load_run(path)
        if phases:
            runs.append({"path": path, "phases": phases})
        else:
            print(f"  (skipped — no phase data): {path}")
    return runs

# ── plot 1: current per phase ─────────────────────────────────────────────────

CURRENT_PLOT_EXCLUDE = {"Homing"}

def plot_current(runs, out_dir):
    # Collect unique phase names in first-seen order, excluding Homing
    seen = set()
    phase_names = []
    for run in runs:
        for ph in run["phases"]:
            if ph["name"] not in seen and ph["name"] not in CURRENT_PLOT_EXCLUDE:
                phase_names.append(ph["name"])
                seen.add(ph["name"])

    if not phase_names:
        print("No current data to plot.")
        return

    n = len(phase_names)
    fig, axes = plt.subplots(1, n, figsize=(7 * n, 6), squeeze=False)
    axes = axes[0]

    for ax, pname in zip(axes, phase_names):
        color = PHASE_COLORS.get(pname, "#888888")
        all_t, all_c = [], []

        for run in runs:
            for ph in run["phases"]:
                if ph["name"] != pname or not ph["rel_times"]:
                    continue
                ax.plot(ph["rel_times"], ph["currents"],
                        linewidth=0.8, alpha=0.35, color=color)
                all_t.extend(ph["rel_times"])
                all_c.extend(ph["currents"])

        # Average line: bin into 20 ms buckets
        if all_t:
            bucket_ms = 20
            buckets = defaultdict(list)
            for t, c in zip(all_t, all_c):
                buckets[int(t // bucket_ms)].append(c)
            bkeys = sorted(buckets)
            ax.plot(
                [b * bucket_ms for b in bkeys],
                [np.mean(buckets[b]) for b in bkeys],
                linewidth=2.5, color=color, label="mean",
            )

        ax.axhline(0, color="#aaaaaa", linewidth=0.6, linestyle="--")
        ax.set_title(pname, fontsize=16, fontweight="bold")
        ax.set_xlabel("Time since phase start (ms)", fontsize=13)
        ax.set_ylabel("Current (A)", fontsize=13)
        ax.tick_params(axis="both", labelsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=12)

    n_runs = len(runs)
    fig.suptitle(f"Gripper current per phase", fontsize=16, fontweight="bold")
    plt.tight_layout()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    for ext, kw in [("png", {"dpi": 150}), ("svg", {})]:
        out = os.path.join(out_dir, f"current_{ts}.{ext}")
        plt.savefig(out, **kw)
        print(f"Saved: {out}")
    plt.close()

# ── plot 2: phase timing statistics ──────────────────────────────────────────

def plot_timing(runs, out_dir):
    durations = defaultdict(list)
    for run in runs:
        for ph in run["phases"]:
            if ph["duration_ms"] is not None:
                durations[ph["name"]].append(ph["duration_ms"])

    if not durations:
        print("No timing data to plot.")
        return

    phase_names = list(durations.keys())
    x = np.arange(len(phase_names))
    means = [np.mean(durations[p]) for p in phase_names]
    stds  = [np.std(durations[p])  for p in phase_names]
    mins  = [np.min(durations[p])  for p in phase_names]
    maxs  = [np.max(durations[p])  for p in phase_names]
    counts = [len(durations[p]) for p in phase_names]
    colors = [PHASE_COLORS.get(p, "#888888") for p in phase_names]

    fig, ax = plt.subplots(figsize=(max(6, len(phase_names) * 2.8), 6))

    ax.bar(x, means, yerr=stds, capsize=7, color=colors, alpha=0.75,
           error_kw={"linewidth": 1.5, "ecolor": "#444444"}, zorder=3)

    # Individual run dots
    for i, pname in enumerate(phase_names):
        ax.scatter([i] * len(durations[pname]), durations[pname],
                   color="black", s=25, alpha=0.55, zorder=5)

    # Annotations above each bar
    y_pad = (max(maxs) - min(mins)) * 0.04 + 20
    for i, pname in enumerate(phase_names):
        label = (
            f"n={counts[i]}\n"
            f"avg={means[i]:.0f} ms\n"
            f"min={mins[i]:.0f} ms\n"
            f"max={maxs[i]:.0f} ms\n"
            f"σ={stds[i]:.0f} ms"
        )
        ax.text(i, maxs[i] + y_pad, label,
                ha="center", va="bottom", fontsize=8, color="#222222",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#cccccc", alpha=0.8))

    # Extend ylim so the 5-line annotation boxes stay inside the axes area
    # and never push into the title
    ax.set_ylim(bottom=0, top=max(maxs) * 1.2 + y_pad)

    ax.set_xticks(x)
    ax.set_xticklabels(phase_names, fontsize=11)
    ax.set_ylabel("Duration (ms)")
    ax.set_title(
        f"Phase timing"
    )
    ax.grid(True, axis="y", alpha=0.3, zorder=0)
    plt.tight_layout()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    for ext, kw in [("png", {"dpi": 150}), ("svg", {})]:
        out = os.path.join(out_dir, f"timing_{ts}.{ext}")
        plt.savefig(out, **kw)
        print(f"Saved: {out}")
    plt.close()

# ── plot 3: PC-side round-trip timing ────────────────────────────────────────

def find_timing_logs():
    return sorted(glob.glob(os.path.join(DATA_DIR, "*", "timing_log.csv")))


def resolve_timing_paths(args):
    paths = []
    for a in args:
        if a.endswith("timing_log.csv") and os.path.isfile(a):
            paths.append(a)
        else:
            p = os.path.join(a, "timing_log.csv")
            if os.path.isfile(p):
                paths.append(p)
    return sorted(paths)


def load_timing_logs(paths):
    records = []
    trip_offset = 0
    for path in paths:
        batch_max = 0
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                trip = int(row["trip"]) + trip_offset
                records.append({
                    "trip":        trip,
                    "step":        int(row["step"]),
                    "label":       row["label"],
                    "command":     int(row["command"]),
                    "duration_ms": float(row["duration_ms"]),
                    "success":     row["success"] == "True",
                })
                batch_max = max(batch_max, int(row["trip"]))
        trip_offset += batch_max
    return records


def extract_phase_durations(current_log_path):
    """Return list of 'Phase took X ms' values in order from a current_log CSV."""
    durations = []
    with open(current_log_path, newline="") as f:
        for row in csv.DictReader(f):
            if row["event"]:
                m = PHASE_TOOK_RE.search(row["event"])
                if m:
                    durations.append(int(m.group(1)))
    return durations


def compute_latencies(timing_records, phase_durations):
    """
    latency = PC_total_duration - ESP32_phase_duration
    Subtracting the ESP32's own execution time leaves only the round-trip
    communication overhead between PC and the Nano.
    """
    n = min(len(timing_records), len(phase_durations))
    if len(timing_records) != len(phase_durations):
        print(f"  Warning: {len(timing_records)} timing entries vs "
              f"{len(phase_durations)} phase events — using first {n}")

    latencies = []
    for i, (rec, phase_ms) in enumerate(zip(timing_records[:n], phase_durations[:n])):
        latencies.append({
            "call_idx":    i + 1,
            "trip":        rec["trip"],
            "step":        rec["step"],
            "label":       rec["label"],
            "pc_ms":       rec["duration_ms"],
            "phase_ms":    phase_ms,
            "latency_ms":  rec["duration_ms"] - phase_ms,
        })
    return latencies


def plot_latency(latencies, out_dir):
    if not latencies:
        print("No latency data to plot.")
        return

    idx      = [r["call_idx"]   for r in latencies]
    lat      = [r["latency_ms"] for r in latencies]
    labels   = [r["label"]      for r in latencies]

    mean_v = np.mean(lat)
    std_v  = np.std(lat)
    min_v  = np.min(lat)
    max_v  = np.max(lat)

    fig, ax = plt.subplots(figsize=(max(8, len(idx) * 0.35 + 2), 5))

    # Thin connecting line
    ax.plot(idx, lat, color="#aaaaaa", linewidth=0.8, zorder=3)

    # Dots colored by phase for context, but phase is not the main point
    for label in dict.fromkeys(labels):
        mask = [i for i, l in enumerate(labels) if l == label]
        ax.scatter([idx[i] for i in mask], [lat[i] for i in mask],
                   color=PHASE_COLORS.get(label, "#888888"),
                   s=35, label=label, zorder=5, alpha=0.85)

    # Mean line + ±1σ band
    ax.axhline(mean_v, color="#222222", linewidth=1.5, linestyle="--", zorder=4)
    ax.axhspan(mean_v - std_v, mean_v + std_v, alpha=0.08, color="#222222")

    # Stats box (top-left, inside axes — never overlaps)
    stats = (
        f"n = {len(lat)}\n"
        f"avg = {mean_v:.1f} ms\n"
        f"min = {min_v:.1f} ms\n"
        f"max = {max_v:.1f} ms\n"
        f"σ  = {std_v:.1f} ms"
    )
    ax.text(0.02, 0.97, stats, transform=ax.transAxes,
            va="top", ha="left", fontsize=11, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="#cccccc", alpha=0.9))

    ax.set_xlabel("Command index", fontsize=13)
    ax.set_ylabel("Latency (ms)", fontsize=13)
    ax.set_title("Communication latency — PC ↔ ESP32 (Nano)", fontsize=15, fontweight="bold")
    ax.tick_params(labelsize=12)
    ax.legend(fontsize=11, title="Phase (context)", title_fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    for ext, kw in [("png", {"dpi": 150}), ("svg", {})]:
        out = os.path.join(out_dir, f"latency_{ts}.{ext}")
        plt.savefig(out, **kw)
        print(f"Saved: {out}")
    plt.close()

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    args = sys.argv[1:]
    current_paths = resolve_paths(args)        if args else find_all_logs()
    timing_paths  = resolve_timing_paths(args) if args else find_timing_logs()

    if not current_paths and not timing_paths:
        print("No CSV logs found. Run udp_csv_capture.py and/or run_trips.py first.")
        sys.exit(1)

    # Save plots in the shared folder if data spans multiple directories,
    # otherwise save next to the data files.
    all_paths   = current_paths + timing_paths
    parent_dirs = set(os.path.dirname(os.path.abspath(p)) for p in all_paths)
    out_dir     = parent_dirs.pop() if len(parent_dirs) == 1 else PLOTS_DIR
    os.makedirs(out_dir, exist_ok=True)

    if current_paths:
        print(f"Loading {len(current_paths)} current log(s)...")
        runs = load_all_runs(current_paths)
        if runs:
            print(f"  {len(runs)} run(s) with phase data.\n")
            plot_current(runs, out_dir)
            plot_timing(runs, out_dir)
        else:
            print("  No usable phase data.\n")

    if timing_paths and current_paths:
        # Group by parent directory — only compute latency where both files coexist
        current_by_dir = {os.path.dirname(os.path.abspath(p)): p for p in current_paths}
        timing_by_dir  = {os.path.dirname(os.path.abspath(p)): p for p in timing_paths}
        shared_dirs    = set(current_by_dir) & set(timing_by_dir)

        if shared_dirs:
            all_timing  = load_timing_logs([timing_by_dir[d]  for d in sorted(shared_dirs)])
            all_phase_d = []
            for d in sorted(shared_dirs):
                all_phase_d.extend(extract_phase_durations(current_by_dir[d]))

            print(f"Computing latencies ({len(all_timing)} commands, "
                  f"{len(all_phase_d)} phase events)...")
            latencies = compute_latencies(all_timing, all_phase_d)
            if latencies:
                print(f"  {len(latencies)} latency samples.\n")
                plot_latency(latencies, out_dir)
        else:
            print("  No matching folders — pass the same directory to both "
                  "udp_csv_capture.py and run_trips.py to enable latency plots.\n")
    elif timing_paths:
        print("  timing_log.csv found but no matching current_log.csv — "
              "latency plot requires both files in the same folder.\n")


if __name__ == "__main__":
    main()

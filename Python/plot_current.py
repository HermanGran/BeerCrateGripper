import csv
import sys
import os
import glob
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

EVENT_COLORS = [
    "#e74c3c", "#e67e22", "#2ecc71", "#9b59b6",
    "#1abc9c", "#f39c12", "#3498db", "#e91e63",
]


def find_latest_log():
    data_dir = os.path.join(os.path.dirname(__file__), "..", "Data")
    logs = sorted(glob.glob(os.path.join(data_dir, "*", "current_log.csv")))
    if not logs:
        print("No log files found in Data/")
        sys.exit(1)
    return logs[-1]


def load_csv(path):
    times, currents, events = [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            elapsed = float(row["elapsed_ms"])
            if row["current_A"]:
                times.append(elapsed)
                currents.append(float(row["current_A"]))
            elif row["event"]:
                events.append((elapsed, row["event"]))
    return times, currents, events


def plot(path):
    times, currents, events = load_csv(path)

    if not times:
        print("No current readings found in log.")
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(times, currents, linewidth=0.8, color="steelblue")
    ax.axhline(0, color="#aaaaaa", linewidth=0.6, linestyle="--")

    legend_handles = [mpatches.Patch(color="steelblue", label="Current (A)")]
    y_top = max(currents) if currents else 1.0
    y_bot = min(currents) if currents else -1.0
    y_span = y_top - y_bot or 1.0

    for i, (t, label) in enumerate(events):
        color = EVENT_COLORS[i % len(EVENT_COLORS)]
        ax.axvline(t, color=color, linewidth=1.2, linestyle="--", alpha=0.85)
        ax.text(
            t, y_top - 0.02 * y_span,
            label, rotation=90, fontsize=7,
            va="top", ha="right", color=color,
        )
        legend_handles.append(mpatches.Patch(color=color, label=label[:50]))

    ax.set_xlabel("Elapsed time (ms)")
    ax.set_ylabel("Current (A)")
    log_name = os.path.basename(os.path.dirname(path))
    ax.set_title(f"Gripper current — {log_name}")
    ax.legend(handles=legend_handles, fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else find_latest_log()
    print(f"Plotting: {path}")
    plot(path)

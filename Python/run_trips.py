import subprocess
import sys
import re
import csv
import os
import time
from datetime import datetime

SERVICE   = "/gripper_command"
SRV_TYPE  = "workcell_interfaces/srv/GripperCommand"
NUM_TRIPS = 10

# One trip: idle → latch → idle → home
TRIP_SEQUENCE = [
    (3, "Idle"),
    (1, "Latch"),
    (3, "Idle"),
    (2, "Home"),
]

SUCCESS_RE = re.compile(r"success=True")
DATA_DIR   = os.path.join(os.path.dirname(__file__), "..", "Data")


def create_log_dir():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DATA_DIR, timestamp)
    os.makedirs(path, exist_ok=True)
    return path


def call_gripper(command, label):
    print(f"    [{label}] command={command} ...", end=" ", flush=True)
    t0 = time.perf_counter()
    result = subprocess.run(
        ["ros2", "service", "call", SERVICE, SRV_TYPE, f"{{command: {command}}}"],
        capture_output=True,
        text=True,
    )
    duration_ms = (time.perf_counter() - t0) * 1000
    output = result.stdout + result.stderr
    success = bool(SUCCESS_RE.search(output))
    if success:
        print(f"OK  ({duration_ms:.0f} ms)")
    else:
        print("FAILED")
        print(f"      stdout: {result.stdout.strip()}")
        print(f"      stderr: {result.stderr.strip()}")
    return success, duration_ms


def main():
    log_dir  = create_log_dir()
    csv_path = os.path.join(log_dir, "timing_log.csv")

    print(f"Starting {NUM_TRIPS} trips: Idle → Latch → Idle → Home")
    print(f"Saving timing to: {csv_path}\n")

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["trip", "step", "label", "command", "duration_ms", "success"])

        for trip in range(1, NUM_TRIPS + 1):
            print(f"  Trip {trip}/{NUM_TRIPS}")
            for step, (command, label) in enumerate(TRIP_SEQUENCE, start=1):
                success, duration_ms = call_gripper(command, label)
                writer.writerow([trip, step, label, command, f"{duration_ms:.1f}", success])
                f.flush()
                if not success:
                    print(f"\nAborted at trip {trip}, step '{label}'.")
                    sys.exit(1)

    print(f"\nAll {NUM_TRIPS} trips completed. Log: {csv_path}")


if __name__ == "__main__":
    main()

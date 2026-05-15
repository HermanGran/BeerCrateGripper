import subprocess
import sys
import re

SERVICE  = "/gripper_command"
SRV_TYPE = "workcell_interfaces/srv/GripperCommand"
NUM_TRIPS = 10

# One trip: idle → latch → idle → home
TRIP_SEQUENCE = [
    (3, "Idle"),
    (1, "Latch"),
    (3, "Idle"),
    (2, "Home"),
]

SUCCESS_RE = re.compile(r"success=True")


def call_gripper(command, label):
    print(f"    [{label}] command={command} ...", end=" ", flush=True)
    result = subprocess.run(
        ["ros2", "service", "call", SERVICE, SRV_TYPE, f"{{command: {command}}}"],
        capture_output=True,
        text=True,
    )
    output = result.stdout + result.stderr
    if SUCCESS_RE.search(output):
        print("OK")
        return True
    print(f"FAILED")
    print(f"      stdout: {result.stdout.strip()}")
    print(f"      stderr: {result.stderr.strip()}")
    return False


def main():
    print(f"Starting {NUM_TRIPS} trips: Idle → Latch → Idle → Home\n")

    for trip in range(1, NUM_TRIPS + 1):
        print(f"  Trip {trip}/{NUM_TRIPS}")
        for command, label in TRIP_SEQUENCE:
            if not call_gripper(command, label):
                print(f"\nAborted at trip {trip}, step '{label}'.")
                sys.exit(1)

    print(f"\nAll {NUM_TRIPS} trips completed successfully.")


if __name__ == "__main__":
    main()

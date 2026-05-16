import socket
import csv
import os
import re
import time
from datetime import datetime

UDP_PORT = 4444

CURRENT_RE = re.compile(r"Current:\s*([-\d.]+)\s*A")

EVENT_KEYWORDS = (
    "Service request",
    "Gripper homing",
    "Gripper latching",
    "Gripper releasing",
    "Gripper moving",
    "Phase took",
    "Publishing response",
    "Latched",
    "Obstacle detected",
    "Failed to grip",
    "Contact in latch zone",
)


def create_log_dir():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = os.path.join(os.path.dirname(__file__), "..", "Data", timestamp)
    os.makedirs(root, exist_ok=True)
    return root


def main():
    log_dir = create_log_dir()
    csv_path = os.path.join(log_dir, "gripper_log.csv")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)

    start_ms = time.time() * 1000
    print(f"Listening on UDP port {UDP_PORT}")
    print(f"Saving to: {csv_path}")
    print("Press Ctrl+C to stop\n")

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_ms", "current_A", "event"])

        try:
            while True:
                try:
                    data, _ = sock.recvfrom(1024)
                    msg = data.decode("utf-8", errors="replace").strip()
                    elapsed = time.time() * 1000 - start_ms

                    m = CURRENT_RE.search(msg)
                    if m:
                        writer.writerow([f"{elapsed:.1f}", f"{float(m.group(1)):.4f}", ""])
                        f.flush()
                    elif any(kw in msg for kw in EVENT_KEYWORDS):
                        writer.writerow([f"{elapsed:.1f}", "", msg])
                        f.flush()
                        print(f"[{elapsed:8.0f} ms] {msg}")

                except socket.timeout:
                    continue

        except KeyboardInterrupt:
            print(f"\nCapture stopped. Saved: {csv_path}")


if __name__ == "__main__":
    main()

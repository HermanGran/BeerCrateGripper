# Beer Crate Gripper
##### Bachelor project 2025 — Herman Hårstad Gran, NTNU

Firmware for an Arduino Nano ESP32 (ESP32-S3) that controls a stepper-motor gripper designed to pick up beer crates. The gripper communicates with a ROS 2 workcell over WiFi using micro-ROS, exposes a `/gripper_command` service, and uses current sensing to detect contact with the crate.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | Arduino Nano ESP32 (ESP32-S3) |
| Motor driver | Step/Dir driver, 3200 steps/rev (microstepping) |
| Current sensor | ACS723 — 400 mV/A, ±5 A range, 3.3 V supply |
| Limit switch | Digital, active-low (homing) |

### Pin assignments

| Signal | Pin |
|--------|-----|
| Stepper EN | 5 |
| Stepper DIR | 12 |
| Stepper STEP | 11 |
| Limit switch | 4 |
| Current sensor | A2 |

---

## Software architecture

```
src/
├── main.cpp                    Entry point — setup/loop
├── Gripper/
│   └── Gripper.cpp             State machine + FreeRTOS task coordination
├── Actuators/
│   └── StepperMotor.cpp        AccelStepper wrapper
├── Sensors/
│   ├── CurrentSensor.cpp       ACS723 ADC reading + EMA filtering
│   └── LimitSwitch.cpp         Debounced digital input
├── ROS/
│   ├── MicroRosConnection.cpp  micro-ROS lifecycle state machine (WiFi + XRCE)
│   └── GripperNode.cpp         /gripper_command service handler
└── Debug/
    ├── UDPLogger.cpp           Wireless printf-style logging over UDP
    └── Timer.cpp               Hardware interrupt timer (ESP32 hw_timer)
```

### Gripper state machine

```
IDLE ──latch()──► LATCHING ──current spike (early)──► OBSTACLE_DETECTED
                      │
                      ├──current spike (latch zone)──► TIGHTENING ──► LATCHED
                      │
                      └──reached end, no contact──► FAILED

HOME ◄──home()──── any state
```

Contact detection uses an EMA-filtered current reading sampled at 100 µs intervals. A sustained reading above threshold for 150 ms is required before the state transitions — single spikes are ignored.

### micro-ROS connection state machine

```
WAITING_AGENT ──ping OK──► AGENT_AVAILABLE ──entities OK──► AGENT_CONNECTED
                                                                    │
WAITING_AGENT ◄── AGENT_DISCONNECTED ◄──────────ping lost──────────┘
```

---

## ROS 2 interface

**Service:** `/gripper_command` (`workcell_interfaces/srv/GripperCommand`)

| Command value | Action |
|---------------|--------|
| `0` | Home — move to home position |
| `1` | Latch — extend and grip a crate |
| `2` | Release — retract to home |

Example call from Linux:
```bash
ros2 service call /gripper_command workcell_interfaces/srv/GripperCommand "{command: 1}"
```

Response fields: `success` (bool), `message` (string).

---

## Setup

### Network configuration

| Device | IP |
|--------|----|
| ESP32 (static) | `192.168.0.104` |
| micro-ROS agent (Linux) | `192.168.0.100` |
| Agent UDP port | `8888` |
| ROS domain ID | `7` |

> The Linux machine must be reachable from the ESP32's subnet. Use a /24 subnet mask on the Linux NIC if connecting via Ethernet to the same router the ESP32 is on via WiFi.

### WiFi credentials

Edit `include/ROS/MicroRosConnection.hpp`:
```cpp
#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"
```

### Build and flash

```bash
# Build
pio run

# Flash over USB (first time)
pio run --target upload

# Flash over WiFi (OTA, subsequent uploads)
pio run --target upload   # uses upload_port = 192.168.0.104 with --auth=herman
```

OTA hostname: `esp32-gripper`, password: `herman` (set in `main.cpp`).

---

## Running

### 1. Start the micro-ROS agent on Linux

```bash
export ROS_DOMAIN_ID=7
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 2. Power the ESP32

The ESP32 connects to WiFi automatically and begins polling for the agent. Once the agent is up, it creates the node and service within a few seconds. You should see:

```
Ping OK! Creating entities...
All entities created OK
```

### 3. Verify the service is live

```bash
ros2 service list | grep gripper
# → /gripper_command
```

---

## Debug logging

Log messages are sent over UDP to a PC at `192.168.0.102` port `4444`. Run the receiver on that machine:

```bash
python3 Python/udp_log.py
```

The logger prints timestamped messages including state machine transitions, current readings, and ROS entity creation status.

---

## Dependencies (PlatformIO)

| Library | Source |
|---------|--------|
| AccelStepper | `waspinator/AccelStepper@^1.64` |
| micro_ros_platformio | `https://github.com/micro-ROS/micro_ros_platformio` |
| ArduinoOTA | Built-in (ESP32 Arduino core) |
| workcell_interfaces | `extra_packages/` (custom ROS 2 message package) |

ROS 2 distro: **Jazzy**

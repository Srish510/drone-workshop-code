# Documentation: RPi Drone Workshop Code

## Table of Contents
- [System Overview](#system-overview)
- [Architecture](#architecture)
- [Module Descriptions](#module-descriptions)
- [Configuration](#configuration)
- [Participant Mission Guide](#participant-mission-guide)
- [Failsafe Mechanisms](#failsafe-mechanisms)
- [Troubleshooting](#troubleshooting)

---

## System Overview
This project provides a modular codebase for controlling a drone using a Raspberry Pi and Pixhawk flight controller. It supports both simulation (SITL) and real hardware, and enables communication with a Ground Control Station (GCS) via serial, UDP, or TCP. The system is extensible for custom missions and includes failsafe features.

## Architecture
```
+-------------------+      +-------------------+      +-------------------+
|   Ground Control  |<---->|   Communication   |<---->|    Flight         |
|   Station (GCS)   |      |   Controller      |      |    Controller     |
+-------------------+      +-------------------+      +-------------------+
        ^                        |                          |
        |                        v                          v
+-------------------+      +-------------------+      +-------------------+
|   Image Handler   |<-----|   Camera Module   |      |   Failsafe        |
+-------------------+      +-------------------+      +-------------------+
```

## Module Descriptions

### main.py
- Entry point. Loads configuration, initializes controllers, and starts the system.

### comms/
- **serial_comm.py**: Handles serial communication with GCS.
- **udp_client.py**: Handles UDP communication.
- **tcp_client.py**: Handles TCP communication.

### controllers/
- **flight_controller.py**: Manages Pixhawk connection, arming, takeoff, landing, and waypoint navigation.
- **comm_controller.py**: Manages communication with GCS and relays commands to the flight controller.

### search/
- **image_rx.py**: Receives and processes images from the camera.

### failsafe/
- **battery_drop.py, disconnect.py, motor_loss.py, unstable_imu.py**: Failsafe modules for various emergency scenarios.

### participant_codes/
- **format.py**: Template for participant missions.
- **mission_planner.py**: Loads and executes participant missions.

### utils/
- **helpers.py**: Utility functions for config loading and coordinate calculations.
- **logging.py**: Logger setup.
- **message_parser.py**: Message encoding/decoding.

### config/
- **connections.yaml**: Communication and camera settings.
- **pixhawk_config.yaml**: Pixhawk and flight parameters.

## Configuration
- See `src/config/connections.yaml` and `src/config/pixhawk_config.yaml` for all configurable options.

## Participant Mission Guide
- Copy `src/participant_codes/format.py` to a new file named after your team.
- Implement the `mission_plan` function to return a list of waypoints.
- Do not modify other files.

## Failsafe Mechanisms
- The system includes modules to handle battery drops, disconnects, motor loss, and IMU instability. These modules can be extended as needed.

## Troubleshooting
- Ensure all dependencies are installed (`pip install -r requirements.txt`).
- Check configuration files for correct connection strings and parameters.
- Use the logger output for debugging system state and errors.

---

For further details, see the code comments and docstrings in each module.

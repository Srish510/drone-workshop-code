# RPi Drone Workshop Code

This repository contains the codebase for a Raspberry Pi-based drone system designed for educational workshops and competitions. The system supports both simulation (SITL) and real Pixhawk hardware, and enables communication with a Ground Control Station (GCS) via serial, UDP, or TCP. Participants can implement custom drone missions by defining waypoints and tasks.

## Features

- **Flight Control**: Connects to Pixhawk (SITL or physical), arms/disarms, takeoff, land, and waypoint navigation.
- **Communication**: Supports serial, UDP, and TCP links between the drone and GCS.
- **Image Handling**: Receives and processes images from a camera module (USB or RPi camera).
- **Mission Planning**: Participants can define custom missions by specifying waypoints.
- **Failsafe Mechanisms**: Includes modules for battery drop, disconnect, motor loss, and unstable IMU.
- **Extensible**: Modular structure for easy extension and customization.

## Directory Structure

```
src/
  main.py                # Entry point for the drone system
  comms/                 # Communication modules (serial, UDP, TCP)
  config/                # YAML configuration files
  controllers/           # Flight and communication controllers
  failsafe/              # Failsafe modules
  participant_codes/     # Participant mission templates and planners
  search/                # Image receiving and processing
  utils/                 # Helper functions, logging, message parsing
```


## Quick Start

### 0. Download the Code

Clone the repository from GitHub:

```bash
git clone https://github.com/Srish510/drone-workshop-code.git
cd drone-workshop-code
```


### 1. Install Dependencies

Install Python 3.8+ and the required packages:

```bash
pip install -r requirements.txt
```

### 2. Configure the System

Edit the configuration files in `src/config/` as needed:
- `connections.yaml`: Set up communication type and camera settings.
- `pixhawk_config.yaml`: Choose between SITL or physical Pixhawk and set flight parameters.

### 3. Run the System

```bash
python src/main.py
```

The system will connect to the Pixhawk, initialize communication, and wait for commands from the GCS.

## Participant Instructions

1. **Copy the Template:**
	- In `src/participant_codes/`, copy `format.py` to a new file named after your team (e.g., `alpha_team.py` if your team name is Alpha Team).
2. **Implement Your Mission:**
	- Edit only the `mission_plan` function in your team file to return a list of waypoints: `[(x1, y1, alt1), (x2, y2, alt2), ...]`.
	- Each waypoint is relative to the takeoff point (in meters).
3. **Do Not Modify Other Files:**
	- Only your team file should be changed. Other modifications may result in disqualification.
4. **Test Your Mission:**
	- The organizers will run your mission in simulation and on real hardware if selected.

See `src/participant_codes/format.py` for detailed instructions and requirements.
Submission details will be shared at the venue by the organizers.

## Configuration

- **connections.yaml**: Communication and camera settings.
- **pixhawk_config.yaml**: Pixhawk connection and flight parameters.

## Requirements

- Python 3.8+
- [pymavlink](https://www.ardusub.com/developers/pymavlink.html)
- pyserial
- PyYAML
- opencv-python-headless
- numpy

Install all dependencies with:

```bash
pip install -r requirements.txt
```

## Safety & Rules

- Always prioritize safety in mission design.
- Do not exceed operational bounds provided by organizers.
- Unsafe or non-compliant missions may be disqualified.

## Documentation

For detailed module descriptions, architecture, configuration, and troubleshooting, see the [full documentation](DOCUMENTATION.md).

## License

This project is for educational and competition use. See organizers for licensing details.

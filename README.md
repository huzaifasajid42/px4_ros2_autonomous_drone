# Autonomous Drone Control with ROS 2 and PX4

##  Demonstration Video
[Google Drive Link](https://drive.google.com/file/d/1q3vbi7ABRcCc4_g5LoWiZ6shfvhMZBp6/view?usp=drive_link)

---

##  Project Overview
This project implements a fully autonomous drone mission system using **ROS 2 Humble**, **PX4 SITL**, and **MAVROS**.  
The mission controller is written in Python and demonstrates:

- Autonomous flight in **OFFBOARD mode**
- Square pattern mission
- Automated landing
- Built-in failsafes (timeouts, hold logic, etc.)
- CSV telemetry logging for post-flight analysis

---

##  Features
- **Autonomous Mission Execution**  
  Takeoff → Square pattern → Landing  
- **One-Command Startup**  
  Run everything with `~/px4_ros_ws/src/drone_controller/scripts/launch_everything.sh`  
- **Failsafes**  
  Pose timeout, takeoff/waypoint timeouts, hysteresis at waypoints, AUTO.LAND handoff  
- **Telemetry Logging**  
  CSV log files stored in `~/drone_telemetry`  
- **Real-time Telemetry**  
  Logs state, position, PX4 mode, and arming status

---

##  Mission Profile
1. **Takeoff** to 5 m altitude  
2. **Square pattern**:  
   (0,0,5) → (5,0,5) → (5,5,5) → (0,5,5) → (0,0,5)  
3. **Landing** using PX4’s `AUTO.LAND` mode  
4. **Disarm** and shutdown of mission node  

---

##  System Architecture
- **PX4 SITL**  
  Tested with both **Gazebo** and **jMAVSim**.  
  Default in `start_px4.sh` is jMAVSim.  
  To use Gazebo, replace `jmavsim` with `gazebo`.  
- **MAVROS**  
  Bridge between PX4 and ROS 2.  
- **Mission Controller**  
  Python finite state machine (`drone_controller_node`).  
  Publishes local position setpoints at 50 Hz.  

---

##  Environment Specifications
- **OS**: Ubuntu 22.04.5 LTS  
- **PX4**: v1.13.3  
- **ROS 2**: Humble Hawksbill  
- **Python**: 3.10.12  
- **MAVROS**: 2.10.1  

---

## Quick Start

Run the all-in-one script:
~/px4_ros_ws/src/drone_controller/scripts/launch_everything.sh
This script will:
Start PX4 SITL
Start MAVROS
Launch the mission controller node

---

## Installation & Setup

# 1. Install ROS 2 Humble
sudo apt update && sudo apt install ros-humble-desktop

# 2. Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-msgs

# 3. Install PX4 Autopilot
sudo apt install git make python3-pip
git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
make px4_sitl jmavsim   # or: make px4_sitl gz_x500

# 4. Setup ROS environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Build workspace
cd ~/px4_ros_ws
colcon build --packages-select drone_controller
source install/setup.bash

## Failsafes & Safety Checks in the Node
The mission controller includes these safety mechanisms:

# Pose timeout
Logs an error if /mavros/local_position/pose stops updating.

# Setpoint streaming
Publishes at ~50 Hz to maintain OFFBOARD mode.

# Takeoff timeout (10 s)
Skip takeoff if altitude not reached.

# Waypoint timeout (12 s)
Skip waypoint if not reached in time.

# Waypoint hold + hysteresis
Require 3 s hold within 0.5 m of target. Reset if drift > 0.75 m.

# Landing handoff
Switch to PX4 AUTO.LAND for descent & disarm.

# Disarm detection
Once PX4 disarms, the node shuts down.

## Telemetry Logging
Location: ~/drone_telemetry
File name: mission_telemetry_YYYYMMDD_HHMMSS.csv

## Design Notes

# FSM Design
The mission logic is a finite state machine with states:
DISARMED → SETPOINT_STREAMING → ARMING → ARMING_WAIT → ARMED → TAKEOFF → MISSION → LANDING → LANDED.

# Control strategy
Position setpoints published continuously; PX4 handles inner-loop stabilization.

# Failsafe philosophy
Timeout-based recovery ensures the mission continues even with missing pose data.

# Telemetry
CSV logs intended for post-flight debugging and mission replay.

## Limitations & Known Issues

# Language mismatch
The project specification required Rust, but the implementation is in Python.

# No pose data
/mavros/local_position/pose does not publish updates in the current setup.
→ As a workaround, mission progression uses timeouts instead of feedback.

# CSV telemetry logging not functional
Because no pose data is received, CSV logs remain mostly empty.

## Future Work

1. Fix pose stream issue so mission uses true feedback instead of timeouts.

2. Make CSV logging fully functional with real telemetry data.

3. Replace Python mission node with a Rust implementation as originally specified.

4. Extend mission logic for dynamic waypoint updates and obstacle avoidance.
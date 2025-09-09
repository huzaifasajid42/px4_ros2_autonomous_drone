#!/bin/bash

# This is the script for our all-in-one startup of mission. Run this and our processes will start showing up in different windows

# Starting PX4 in a new terminal
echo "Starting PX4..."
x-terminal-emulator -e "bash -c '$HOME/px4_ros_ws/src/drone_controller/scripts/start_px4.sh; exec bash'" &
sleep 8

# Starting MAVROS in a new terminal

echo "Starting MAVROS..."
#x-terminal-emulator -e "bash -c 'ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557 --log-level debug; exec bash'" &
x-terminal-emulator -e "bash -c 'ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash'" &


sleep 15

# Starting our mission controller in a new terminal
echo "Starting mission controller..."
x-terminal-emulator -e "bash -c 'source ~/px4_ros_ws/install/setup.bash && ros2 run drone_controller drone_controller_node; exec bash'" &

echo "All processes started in separate terminals"

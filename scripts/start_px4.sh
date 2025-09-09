#!/bin/bash
# start_px4.sh
cd ~/PX4-workspaces/PX4-Autopilot

# I have used jmavsim for my simulation due to colorful demo.
# If you want to test on gazebo, simply replace the below command with 'make px4_sitl gazebo' (without commas ofcourse.) 
make px4_sitl jmavsim

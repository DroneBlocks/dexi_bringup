#!/bin/bash

source /home/dexi/ros2_jazzy/install/setup.bash
source /home/dexi/dexi_ws/install/setup.bash

# Detect hardware type from device tree model
HARDWARE_MODEL=$(cat /proc/device-tree/model 2>/dev/null || echo "unknown")

if [[ $HARDWARE_MODEL == *"Raspberry Pi Compute Module 4"* ]]; then
    echo "Detected CM4 hardware, launching dexi_bringup_ark_cm4.launch.py"
    ros2 launch dexi_bringup dexi_bringup_ark_cm4.launch.py
elif [[ $HARDWARE_MODEL == *"Raspberry Pi 5"* ]]; then
    echo "Detected Pi5 hardware, launching dexi_bringup_pi5.launch.py" 
    ros2 launch dexi_bringup dexi_bringup_pi5.launch.py
else
    echo "Unknown hardware: $HARDWARE_MODEL - no launch file specified for this platform"
fi
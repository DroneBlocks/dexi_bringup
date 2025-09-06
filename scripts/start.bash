#!/bin/bash

source /home/dexi/ros2_jazzy/install/setup.bash
source /home/dexi/dexi_ws/install/setup.bash

# Function to read YAML config values
get_config_value() {
    local node_name="$1"
    local default="$2"
    local config_file="/home/dexi/.dexi-config.yaml"
    
    if [ -f "$config_file" ]; then
        # Simple YAML parsing: find "nodes:" section, then find node, then get enabled value
        value=$(awk "
            /^nodes:/ { in_nodes=1; next }
            /^[a-zA-Z]/ && in_nodes { in_nodes=0 }
            in_nodes && /^  ${node_name}:/ { in_node=1; next }
            in_nodes && /^  [a-zA-Z]/ && in_node { in_node=0 }
            in_node && /^    enabled:/ { 
                gsub(/^    enabled: */, \"\")
                gsub(/ *$/, \"\")
                print
                exit
            }
        " "$config_file")
        echo "${value:-$default}"
    else
        echo "$default"
    fi
}

# Read configuration values from ~/.dexi-config.yaml
YOLO_ENABLED=$(get_config_value "yolo" "false")
APRILTAG_ENABLED=$(get_config_value "apriltag" "false")  
CAMERA_ENABLED=$(get_config_value "camera" "true")
ROSBRIDGE_ENABLED=$(get_config_value "rosbridge" "false")

# Detect hardware type from device tree model
HARDWARE_MODEL=$(cat /proc/device-tree/model 2>/dev/null || echo "unknown")

echo "Configuration loaded: yolo=$YOLO_ENABLED, apriltags=$APRILTAG_ENABLED, camera=$CAMERA_ENABLED, rosbridge=$ROSBRIDGE_ENABLED"

if [[ $HARDWARE_MODEL == *"Raspberry Pi Compute Module 4"* ]]; then
    echo "Detected CM4 hardware, launching dexi_bringup_ark_cm4.launch.py"
    ros2 launch dexi_bringup dexi_bringup_ark_cm4.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
elif [[ $HARDWARE_MODEL == *"Raspberry Pi 5"* ]]; then
    echo "Detected Pi5 hardware, launching dexi_bringup_pi5.launch.py"
    ros2 launch dexi_bringup dexi_bringup_pi5.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
else
    echo "Unknown hardware: $HARDWARE_MODEL - no launch file specified for this platform"
fi
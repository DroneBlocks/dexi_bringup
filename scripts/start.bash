#!/bin/bash

source /home/dexi/ros2_jazzy/install/setup.bash
source /home/dexi/dexi_ws/install/setup.bash

# Resolve this script's own directory so we can find sibling config files
# regardless of where dexi.service sets WorkingDirectory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/../config"

# Detect hardware platform from device-tree model.
HARDWARE_MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo "unknown")
if [[ $HARDWARE_MODEL == *"Raspberry Pi Compute Module 4"* ]]; then
    PLATFORM="cm4"
elif [[ $HARDWARE_MODEL == *"Raspberry Pi Compute Module 5"* ]]; then
    PLATFORM="cm5"
elif [[ $HARDWARE_MODEL == *"Raspberry Pi 5"* ]]; then
    PLATFORM="pi5"
else
    PLATFORM=""
fi

# Config file locations (search order: user -> platform -> base -> default).
USER_CONFIG="/home/dexi/.dexi-config.yaml"
PLATFORM_CONFIG="${CONFIG_DIR}/dexi_config_${PLATFORM}.yaml"
BASE_CONFIG="${CONFIG_DIR}/dexi_config.yaml"

# Flat nodes.<node>.<param> YAML lookup from a single file.
# Returns the value on stdout if found, nothing otherwise.
_yaml_lookup() {
    local file="$1"
    local node_name="$2"
    local param_name="$3"
    [ -f "$file" ] || return 1
    awk "
        /^nodes:/ { in_nodes=1; next }
        /^[a-zA-Z]/ && in_nodes { in_nodes=0 }
        in_nodes && /^  ${node_name}:/ { in_node=1; next }
        in_nodes && /^  [a-zA-Z]/ && in_node { in_node=0 }
        in_node && /^    ${param_name}:/ {
            gsub(/^    ${param_name}: */, \"\")
            gsub(/ *$/, \"\")
            gsub(/\"/, \"\")
            print
            exit
        }
    " "$file"
}

# Platform-aware config lookup.
# Searches each file in priority order and returns the first hit, falling
# back to the hardcoded default passed as $3 if nothing matches.
get_config_value() {
    local node_name="$1"
    local param_name="$2"
    local default="$3"
    local value
    local file

    for file in "$USER_CONFIG" "$PLATFORM_CONFIG" "$BASE_CONFIG"; do
        value=$(_yaml_lookup "$file" "$node_name" "$param_name")
        if [ -n "$value" ]; then
            echo "$value"
            return
        fi
    done
    echo "$default"
}

# Read configuration values
YOLO_ENABLED=$(get_config_value "yolo" "enabled" "false")
APRILTAG_ENABLED=$(get_config_value "apriltag" "enabled" "false")
CAMERA_ENABLED=$(get_config_value "camera" "enabled" "true")
CAMERA_WIDTH=$(get_config_value "camera" "width" "640")
CAMERA_HEIGHT=$(get_config_value "camera" "height" "480")
CAMERA_FORMAT=$(get_config_value "camera" "format" "XRGB8888")
CAMERA_JPEG_QUALITY=$(get_config_value "camera" "jpeg_quality" "60")
SERVO_ENABLED=$(get_config_value "servo" "enabled" "true")
GPIO_ENABLED=$(get_config_value "gpio" "enabled" "true")
OFFBOARD_ENABLED=$(get_config_value "offboard" "enabled" "false")
KEYBOARD_CONTROL_ENABLED=$(get_config_value "offboard" "keyboard_control" "false")
ROSBRIDGE_ENABLED=$(get_config_value "rosbridge" "enabled" "true")

echo "Configuration loaded (platform=${PLATFORM:-unknown}): yolo=$YOLO_ENABLED, apriltags=$APRILTAG_ENABLED, camera=$CAMERA_ENABLED (${CAMERA_WIDTH}x${CAMERA_HEIGHT}, $CAMERA_FORMAT, q$CAMERA_JPEG_QUALITY), servos=$SERVO_ENABLED, gpio=$GPIO_ENABLED, offboard=$OFFBOARD_ENABLED, keyboard_control=$KEYBOARD_CONTROL_ENABLED, rosbridge=$ROSBRIDGE_ENABLED"

# Launch the per-platform bringup
case "$PLATFORM" in
    cm4)
        echo "Detected CM4 hardware, launching dexi_bringup_ark_cm4.launch.py"
        ros2 launch dexi_bringup dexi_bringup_ark_cm4.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
        ;;
    cm5)
        echo "Detected CM5 hardware, launching dexi_bringup_cm5.launch.py"
        ros2 launch dexi_bringup dexi_bringup_cm5.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY gpio:=$GPIO_ENABLED servos:=$SERVO_ENABLED offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
        ;;
    pi5)
        echo "Detected Pi5 hardware, launching dexi_bringup_pi5.launch.py"
        ros2 launch dexi_bringup dexi_bringup_pi5.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY gpio:=$GPIO_ENABLED servos:=$SERVO_ENABLED offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
        ;;
    *)
        echo "Unknown hardware: $HARDWARE_MODEL - no launch file specified for this platform"
        ;;
esac

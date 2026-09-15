#!/bin/bash

source /home/dexi/ros2_jazzy/install/setup.bash
source /home/dexi/dexi_ws/install/setup.bash

# Resolve this script's own directory so we can find sibling config files
# regardless of where dexi.service sets WorkingDirectory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/../config"

# Platform selection. /etc/dexi-platform is written by the image build and is
# authoritative: the device-tree model cannot tell a CM5 on the DroneBlocks
# carrier from a CM5 on the ARK carrier, and they need different launch files.
# Fall back to model detection for images built before the marker existed.
#
# The marker holds the *build target* name, which is not always the same string
# as the platform arm below (ark_cm4 vs cm4). A marker naming a platform this
# script has no arm for must not take the whole stack down silently, so an
# unrecognized value degrades to model detection rather than launching nothing.
HARDWARE_MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo "unknown")

platform_is_known() {
    case "$1" in
        cm4|ark_cm4|cm5|ark_cm5|pi5) return 0 ;;
        *) return 1 ;;
    esac
}

platform_from_model() {
    case "$HARDWARE_MODEL" in
        *"Raspberry Pi Compute Module 4"*) echo "cm4" ;;
        *"Raspberry Pi Compute Module 5"*) echo "cm5" ;;
        *"Raspberry Pi 5"*) echo "pi5" ;;
        *) echo "" ;;
    esac
}

PLATFORM=$(tr -d '[:space:]' < /etc/dexi-platform 2>/dev/null || true)

if [ -n "$PLATFORM" ] && platform_is_known "$PLATFORM"; then
    echo "Platform from /etc/dexi-platform: $PLATFORM"
else
    if [ -n "$PLATFORM" ]; then
        echo "WARNING: /etc/dexi-platform names unknown platform '$PLATFORM' - falling back to device-tree detection"
    fi
    PLATFORM=$(platform_from_model)
    if [ -n "$PLATFORM" ]; then
        echo "Platform from device-tree model ($HARDWARE_MODEL): $PLATFORM"
    fi
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
    cm4|ark_cm4)
        echo "Detected CM4 hardware, launching dexi_bringup_ark_cm4.launch.py"
        ros2 launch dexi_bringup dexi_bringup_ark_cm4.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY gpio:=$GPIO_ENABLED servos:=$SERVO_ENABLED offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
        ;;
    ark_cm5)
        echo "ARK carrier + CM5, launching dexi_bringup_ark_cm5.launch.py"
        ros2 launch dexi_bringup dexi_bringup_ark_cm5.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY gpio:=$GPIO_ENABLED servos:=$SERVO_ENABLED offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
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
        echo "ERROR: no launch file for platform '${PLATFORM:-unknown}' (model: $HARDWARE_MODEL) - nothing will start"
        ;;
esac

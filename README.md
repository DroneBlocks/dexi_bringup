# DEXI Bringup

A comprehensive ROS2 bringup package for the DEXI drone system, designed for Debian Bookworm and ROS2 Jazzy. This package provides launch files and configuration to bring up all core DEXI components including camera, AprilTag detection, servo control, GPIO management, and communication services.

## Features

- **Multi-platform Support**: Automatic detection of Raspberry Pi CM4 vs Pi5 hardware
- **Modular Launch System**: Optional component activation (AprilTags, servos, GPIO)
- **Camera Integration**: Support for both CSI (CM4) and UVC (Pi5) cameras
- **Communication**: ROS bridge, micro-ROS agent, and web interfaces
- **Hardware Control**: Servo and GPIO control via PCA9685 I2C interface
- **Computer Vision**: AprilTag detection and YOLO object detection support
- **Systemd Service**: Automatic startup on boot

## Architecture Support

The system supports different Raspberry Pi architectures with specific launch files:

- **Raspberry Pi CM4**: Uses CSI camera via camera_ros package
- **Raspberry Pi 5**: Uses UVC camera via dexi_camera package

## Quick Start

### 1. Install Dependencies

```bash
# Install workspace dependencies
vcs import src < dexi.repos
rosdep install --from-paths src --ignore-src -y

# Build the workspace
colcon build
source install/setup.bash
```

### 2. Basic Launch

```bash
# Launch all components
ros2 launch dexi_bringup dexi_bringup.launch.py apriltags:=true servos:=true gpio:=true
```

### 3. Camera Setup (Docker)

The camera system runs in Docker for isolation and dependency management:

```bash
# Basic camera
docker run -d --restart unless-stopped --privileged --net=host \
  -v /dev:/dev/ -v /run/udev/:/run/udev/ --group-add video \
  droneblocks/dexi-pi-camera-ros ros2 run camera_ros camera_node \
  --ros-args -r __node:=cam0

# With calibration file
docker run -d --restart unless-stopped --privileged --net=host \
  -v /dev:/dev/ -v /run/udev/:/run/udev/ \
  -v /home/dexi/main_camera_2.1.yaml:/app/main_camera_2.1.yaml \
  --group-add video droneblocks/dexi-pi-camera-ros \
  ros2 run camera_ros camera_node --ros-args -r __node:=cam0 \
  -p camera_info_url:=file:///app/main_camera_2.1.yaml
```

Camera topics:
- `/cam0/image_raw` - Raw image feed
- `/cam0/image_raw/compressed` - Compressed image feed
- `/cam0/camera_info` - Camera calibration info

## Launch Options

### Core Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `apriltags` | `false` | Enable AprilTag detection |
| `servos` | `false` | Enable servo control |
| `gpio` | `false` | Enable GPIO control |
| `ros2` | `true` | Enable micro-ROS agent |
| `camera` | `true` | Enable camera system |

### Component Examples

```bash
# AprilTag detection only
ros2 launch dexi_bringup dexi_bringup.launch.py apriltags:=true

# Servo control only
ros2 launch dexi_bringup dexi_bringup.launch.py servos:=true

# Multiple components
ros2 launch dexi_bringup dexi_bringup.launch.py apriltags:=true servos:=true gpio:=true
```

### Architecture-Specific Launch Files

```bash
# Raspberry Pi CM4 (CSI camera)
ros2 launch dexi_bringup dexi_bringup_ark_cm4.launch.py

# Raspberry Pi 5 (UVC camera)
ros2 launch dexi_bringup dexi_bringup_pi5.launch.py
```

## System Integration

### Systemd Service

For automatic startup on boot:

```bash
# Install service
sudo cp scripts/dexi.service /etc/systemd/system/
sudo systemctl daemon-reload

# Enable and start
sudo systemctl enable dexi.service
sudo systemctl start dexi.service

# Monitor service
sudo systemctl status dexi.service
sudo journalctl -u dexi.service -f
```

### Configuration Files

Hardware-specific configurations are stored in `config/`:

- `config/cm4/config.txt` - Raspberry Pi CM4 boot configuration
- `config/pi5/config.txt` - Raspberry Pi 5 boot configuration
- `config/mavlink-router/` - MAVLink routing configuration

## Dependencies

### Core ROS2 Packages
- `dexi_led` - LED control service
- `dexi_cpp` - C++ servo and GPIO nodes
- `dexi_camera` - UVC camera interface (Pi5)
- `apriltag_ros` - AprilTag detection
- `rosbridge_server` - WebSocket bridge
- `micro_ros_agent` - Micro-ROS communication

### External Dependencies
- Docker (for camera system)
- I2C tools (for hardware control)
- OpenCV (for computer vision)

## Hardware Configuration

### I2C Setup
The system uses I2C for servo control via PCA9685. Ensure I2C is enabled:

```bash
# Enable I2C in raspi-config or add to /boot/config.txt
dtparam=i2c_arm=on
```

### Serial Configuration
Micro-ROS agent connects via UART:
- Device: `/dev/ttyAMA3`
- Baud rate: 921600

## Troubleshooting

### Camera Issues
1. Check Docker container status: `docker ps`
2. Verify camera permissions: `ls -la /dev/video*`
3. Test camera: `v4l2-ctl --list-devices`

### Service Issues
```bash
# Check service status
sudo systemctl status dexi.service

# View logs
sudo journalctl -u dexi.service --no-pager

# Restart service
sudo systemctl restart dexi.service
```

### Package-Specific Dependencies
Install dependencies for specific packages:

```bash
rosdep install --from-paths src/[package_name] --ignore-src -y
```

## Network Services

- **ROS Bridge**: WebSocket on port 9090 for web interfaces
- **Web Video Server**: HTTP video streaming
- **ROS API**: REST API for ROS topics and services

## Development

### Building
```bash
colcon build --packages-select dexi_bringup
source install/setup.bash
```

### Scripts
Located in `scripts/`:
- `dexi.service` - Systemd service configuration
- `start.bash` - Service startup script

## License

MIT License - see LICENSE file for details.

## Maintainer

Dennis Baldwin (db@droneblocks.io) - DroneBlocks

## Related Repositories

- [dexi_cpp](https://github.com/DroneBlocks/dexi_cpp) - C++ hardware control nodes
- [dexi_led](https://github.com/DroneBlocks/dexi_led) - LED control service
- [dexi_camera](https://github.com/DroneBlocks/dexi_camera) - Camera interface
- [dexi_yolo](https://github.com/droneblocks/dexi_yolo) - YOLO object detection
- [dexi_interfaces](https://github.com/DroneBlocks/dexi_interfaces) - Custom message definitions
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) - WebSocket bridge for web interfaces
- [micro_ros_agent](https://github.com/micro-ROS/micro_ros_agent) - Micro-ROS communication agent
- [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) - AprilTag detection package
- [camera_ros](https://github.com/christianrauch/camera_ros) - CSI camera interface for ROS2
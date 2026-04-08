# DEXI Bringup

A comprehensive ROS2 bringup package for the DEXI drone system, designed for Debian Bookworm and ROS2 Jazzy. This package provides launch files and configuration to bring up all core DEXI components including camera, AprilTag detection, color detection, servo control, GPIO management, and communication services.

## Features

- **Multi-platform Support**: Automatic detection of ARK CM4, CM5, and Pi5 hardware
- **Modular Launch System**: Optional component activation via config file
- **Camera Integration**: CSI camera (CM4/CM5) and UVC camera (Pi5)
- **Communication**: ROS bridge, micro-ROS agent, and web interfaces
- **Computer Vision**: AprilTag detection, YOLO object detection, and HSV color detection
- **Flight Control**: PX4 offboard manager with velocity and position control
- **Systemd Service**: Automatic startup on boot

## Supported Platforms

| Platform | Launch File | Camera | Serial | Baud |
|----------|-------------|--------|--------|------|
| **ARK CM4** | `dexi_bringup_ark_cm4.launch.py` | CSI (camera_ros) | `/dev/ttyAMA4` | 3000000 |
| **CM5** | `dexi_bringup_cm5.launch.py` | CSI (camera_ros) | `/dev/ttyAMA3` | 921600 |
| **Pi5** | `dexi_bringup_pi5.launch.py` | UVC (dexi_camera) | `/dev/ttyAMA3` | 921600 |
| **Unity Sim** | `dexi_bringup_unity_sim.launch.py` | N/A (Unity publishes) | N/A | N/A |

Hardware is auto-detected by `start.bash` via `/proc/device-tree/model`. You do not need to specify which launch file to use.

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

### 2. Launch

The DEXI service handles launching automatically on boot. To launch manually:

```bash
# Auto-detect hardware and launch (recommended)
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash && source /home/dexi/dexi_ws/install/setup.bash && /home/dexi/dexi_ws/src/dexi_bringup/scripts/start.bash'

# Or launch a specific platform directly
ros2 launch dexi_bringup dexi_bringup_ark_cm4.launch.py apriltags:=true offboard:=true
```

### Camera Topics

- `/cam0/image_raw` - Raw image feed
- `/cam0/image_raw/compressed` - Compressed image feed (used by web dashboard)
- `/cam0/camera_info` - Camera calibration info

## Launch Arguments

All hardware launch files accept the same arguments. Defaults come from `~/.dexi-config.yaml`.

| Argument | Default | Description |
|----------|---------|-------------|
| `apriltags` | `false` | Enable AprilTag detection |
| `camera` | `true` | Enable camera |
| `camera_width` | `640` | Camera width |
| `camera_height` | `480` | Camera height |
| `camera_format` | `XRGB8888` | Camera pixel format |
| `camera_jpeg_quality` | `60` | JPEG compression quality |
| `color_detection` | `true` | Enable HSV color detection |
| `offboard` | `false` | Enable PX4 offboard flight control |
| `keyboard_control` | `false` | Enable keyboard teleop |
| `rosbridge` | `true` | Enable ROS bridge WebSocket |
| `yolo` | `false` | Enable YOLO object detection |
| `servos` | `false` | Enable servo control (CM5/Pi5 only) |
| `gpio` | `false` | Enable GPIO control (CM5/Pi5 only) |

## Runtime Configuration

DEXI uses a YAML config file to control which nodes launch at boot. The config file lives at `/home/dexi/.dexi-config.yaml` and is read by `start.bash` on each service restart.

### Editing the Config

```bash
sudo nano /home/dexi/.dexi-config.yaml
```

The `nodes` section controls feature toggles:

```yaml
nodes:
  apriltag:
    enabled: true
    max_tags: 10
  camera:
    enabled: true
    width: 640
    height: 480
    format: "XRGB8888"
    jpeg_quality: 60
  gpio:
    enabled: true
  offboard:
    enabled: true
    keyboard_control: false
  rosbridge:
    enabled: true
  servo:
    enabled: true
  yolo:
    enabled: false
    confidence_threshold: 0.65
```

### Applying Changes

After editing the config, restart the DEXI service:

```bash
sudo systemctl restart dexi.service
```

Verify the new settings took effect:

```bash
journalctl -u dexi.service --no-pager -n 5
```

The startup log will show the resolved config, e.g.:

```
Configuration loaded: yolo=false, apriltags=true, camera=true ...
```

## System Integration

### Systemd Service

The DEXI service is installed automatically during provisioning. To manage it:

```bash
# Monitor service
sudo systemctl status dexi.service
sudo journalctl -u dexi.service -f

# Restart after config changes
sudo systemctl restart dexi.service
```

### Configuration Files

Hardware-specific configurations are stored in `config/`:

- `config/cm4/config.txt` - ARK CM4 boot configuration
- `config/cm5/config.txt` - CM5 boot configuration
- `config/pi5/config.txt` - Pi5 boot configuration
- `config/mavlink-router/` - MAVLink routing configuration
- `config/dexi_config.yaml` - Default runtime configuration template

## Dependencies

### Core ROS2 Packages
- `dexi_led` - LED control service
- `dexi_offboard` - PX4 offboard flight control manager
- `dexi_interfaces` - Custom message definitions
- `dexi_camera` - Camera interface (Pi5)
- `dexi_color_detection` - HSV color detection
- `dexi_yolo` - YOLO object detection
- `apriltag_ros` - AprilTag detection
- `camera_ros` - CSI camera interface (CM4/CM5)
- `rosbridge_server` - WebSocket bridge
- `micro_ros_agent` - Micro-ROS communication

## Troubleshooting

### Camera Issues
1. Check camera is detected: `v4l2-ctl --list-devices`
2. Verify camera node is running: check `journalctl -u dexi.service`
3. Test compressed output: `ros2 topic hz /cam0/image_raw/compressed` (run as root)

### Service Issues
```bash
# Check service status
sudo systemctl status dexi.service

# View logs
sudo journalctl -u dexi.service --no-pager

# Restart service
sudo systemctl restart dexi.service
```

### DDS Discovery
ROS2 CLI tools (e.g., `ros2 topic list`) must be run as root to discover topics from the DEXI service:

```bash
sudo bash -c 'source /home/dexi/ros2_jazzy/install/setup.bash && source /home/dexi/dexi_ws/install/setup.bash && ros2 topic list'
```

## Network Services

- **ROS Bridge**: WebSocket on port 9090 for web dashboard
- **ROS API**: Parameter and topic query service for web clients

## Development

### Building
```bash
colcon build --packages-select dexi_bringup
source install/setup.bash
```

### Scripts
Located in `scripts/`:
- `dexi.service` - Systemd service configuration
- `start.bash` - Service startup script (reads config, detects hardware, launches)
- `install.bash` - Installs the systemd service and default config

## License

MIT License - see LICENSE file for details.

## Maintainer

Dennis Baldwin (db@droneblocks.io) - DroneBlocks

## Related Repositories

- [dexi_cpp](https://github.com/DroneBlocks/dexi_cpp) - C++ servo and GPIO nodes
- [dexi_led](https://github.com/DroneBlocks/dexi_led) - LED control service
- [dexi_camera](https://github.com/DroneBlocks/dexi_camera) - Camera interface
- [dexi_offboard](https://github.com/DroneBlocks/dexi_offboard) - PX4 offboard flight control
- [dexi_color_detection](https://github.com/DroneBlocks/dexi_color_detection) - HSV color detection
- [dexi_yolo](https://github.com/droneblocks/dexi_yolo) - YOLO object detection
- [dexi_interfaces](https://github.com/DroneBlocks/dexi_interfaces) - Custom message definitions
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) - WebSocket bridge for web interfaces
- [micro_ros_agent](https://github.com/micro-ROS/micro_ros_agent) - Micro-ROS communication agent
- [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) - AprilTag detection package
- [camera_ros](https://github.com/christianrauch/camera_ros) - CSI camera interface for ROS2

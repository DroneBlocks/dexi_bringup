# DEXI Configuration

DEXI supports runtime configuration through a YAML configuration file that allows you to enable/disable nodes and customize their behavior without modifying code.

## Quick Setup

### 1. Copy Default Configuration (First Time Only)
```bash
cp ~/dexi_ws/src/dexi_bringup/config/dexi_config.yaml ~/.dexi-config.yaml
```

### 2. Edit Your Configuration
```bash
nano ~/.dexi-config.yaml
```

### 3. Apply Changes
```bash
# Restart DEXI service to apply changes (recommended)
sudo systemctl restart dexi.service

# OR reboot if needed
sudo reboot
```

## Configuration Options

### Node Control
- **`nodes.yolo.enabled`**: Enable/disable YOLO object detection
  - `true`: Start YOLO node for object detection
  - `false`: Skip YOLO node (default)
  - `confidence_threshold`: Detection confidence threshold (0.0-1.0)

- **`nodes.camera.enabled`**: Enable/disable camera node
  - `true`: Start camera node (default)
  - `false`: Skip camera node
  - `type`: Camera type (`auto`, `usb`, `csi`)

- **`nodes.apriltag.enabled`**: Enable/disable AprilTag detection
  - `true`: Start AprilTag detection node (default)
  - `false`: Skip AprilTag node
  - `max_tags`: Maximum tags to detect simultaneously

- **`nodes.led.enabled`**: Enable/disable LED control
  - `true`: Start LED controller node (default)
  - `false`: Skip LED node
  - `brightness`: LED brightness level (0.0-1.0)

### System Settings
- **`system.debug_mode`**: Enable debug logging
  - `true`: Verbose debug output
  - `false`: Normal logging (default)

- **`system.log_level`**: Set ROS log level
  - Options: `debug`, `info`, `warn`, `error`
  - Default: `info`

## Example Configurations

### Enable YOLO Detection
```yaml
nodes:
  yolo:
    enabled: true
    confidence_threshold: 0.7
```

### USB Camera Only
```yaml
nodes:
  camera:
    enabled: true
    type: "usb"
  apriltag:
    enabled: false  # Disable if no camera needed for tags
```

### Debug Mode
```yaml
system:
  debug_mode: true
  log_level: "debug"
```

### Minimal Configuration (LED only)
```yaml
nodes:
  yolo:
    enabled: false
  camera:
    enabled: false
  apriltag:
    enabled: false
  led:
    enabled: true
```

## Troubleshooting

### Configuration Not Applied
1. Check config file syntax: `yamllint ~/.dexi-config.yaml`
2. Verify file permissions: `ls -la ~/.dexi-config.yaml`
3. Check service logs: `journalctl -u dexi.service -f`

### Revert to Defaults
```bash
cp ~/dexi_ws/src/dexi_bringup/config/dexi_config.yaml ~/.dexi-config.yaml
sudo systemctl restart dexi.service
```

### Service Won't Start
1. Check service status: `sudo systemctl status dexi.service`
2. View detailed logs: `journalctl -u dexi.service --no-pager`
3. Verify all dependencies are built: `colcon build`

## Advanced Usage

### Per-Hardware Configs
You can maintain different configs for different hardware:
```bash
# Save Pi5 config
cp ~/.dexi-config.yaml ~/.dexi-config-pi5.yaml

# Save CM4 config  
cp ~/.dexi-config.yaml ~/.dexi-config-cm4.yaml

# Switch configs
cp ~/.dexi-config-pi5.yaml ~/.dexi-config.yaml
sudo systemctl restart dexi.service
```

### Automated Configuration
```bash
# Script to enable YOLO
#!/bin/bash
sed -i 's/enabled: false/enabled: true/' ~/.dexi-config.yaml
sudo systemctl restart dexi.service
```
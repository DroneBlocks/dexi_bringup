This is the master repo for the DEXI ROS2 project refactored to run with Debian Bookworm and ROS2 Jazzy.

1. Docker needs to be installed and manages the camera frames

docker run -d --restart unless-stopped --privileged --net=host -v /dev:/dev/ -v /run/udev/:/run/udev/ --group-add video droneblocks/dexi-pi-camera-ros ros2 run camera_ros camera_node --ros-args -r __node:=cam0

Run it with the calibration file:

docker run -it --rm --privileged --net=host -v /dev:/dev/ -v /run/udev/:/run/udev/ -v /home/dexi/main_camera_2.1.yaml:/app/main_camera_2.1.yaml --group-add video droneblocks/dexi-pi-camera-ros ros2 run camera_ros camera_node --ros-args -r __node:=cam0 -p camera_info_url:=file:///app/main_camera_2.1.yaml

This will publish to:

/cam0/image_raw

/cam0/image_raw/compressed

## Launch File Usage

The DEXI bringup launch file supports optional components that can be enabled as needed:

1. To start everything:
```bash
ros2 launch dexi_bringup dexi_bringup.launch.py apriltags:=true servos:=true gpio:=true
```

2. To start only specific components:
```bash
# Just AprilTags
ros2 launch dexi_bringup dexi_bringup.launch.py apriltags:=true

# Just servos
ros2 launch dexi_bringup dexi_bringup.launch.py servos:=true

# Just GPIO
ros2 launch dexi_bringup dexi_bringup.launch.py gpio:=true

# Any combination
ros2 launch dexi_bringup dexi_bringup.launch.py apriltags:=true servos:=true
```

Note: The rosbridge, rosapi, and LED service nodes will always start regardless of the arguments, as they are core components.

## Systemd Service Setup

To run the DEXI bringup launch file automatically on boot:

1. Copy the service file to systemd:
```bash
sudo cp systemd/dexi.service /etc/systemd/system/
```

2. Reload systemd to recognize the new service:
```bash
sudo systemctl daemon-reload
```

3. Enable the service to start on boot:
```bash
sudo systemctl enable dexi.service
```

4. Start the service:
```bash
sudo systemctl start dexi.service
```

5. Check the service status:
```bash
sudo systemctl status dexi.service
```

To view the service logs:
```bash
sudo journalctl -u dexi.service -f
```

To stop the service:
```bash
sudo systemctl stop dexi.service
```

To disable the service from starting on boot:
```bash
sudo systemctl disable dexi.service
```

Note: The service runs as root user and requires ROS2 Jazzy to be installed in the default location (/opt/ros/jazzy).


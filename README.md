This is the master repo for the DEXI ROS2 project refactored to run with Debian Bookworm and ROS2 Jazzy.

1. Docker needs to be installed and manages the camera frames

docker run -d --restart unless-stopped --privileged --net=host -v /dev:/dev/ -v /run/udev/:/run/udev/ --group-add video droneblocks/dexi-pi-camera-ros ros2 run camera_ros camera_node --ros-args -r __node:=cam0

This will publish to:

/cam0/image_raw

/cam0/image_raw/compressed


# ros2_ldlidar_d500

ROS2 driver for the LDLiDAR D500.

This package reads data from the LDLiDAR D500 through a serial port and publishes a standard `sensor_msgs/msg/LaserScan` message on `/scan`.

## Features

* ROS2 C++ node
* Serial decoder for LDLiDAR / LDROBOT packet format
* Publishes `sensor_msgs/msg/LaserScan`
* Configurable serial port
* Configurable frame ID
* Configurable scan direction
* Configurable angle offset
* Compatible with ROS2 Lyrical and ROS2 Jazzy

## Tested with

* ROS2 Lyrical
* ROS2 Jazzy
* Ubuntu
* LDLiDAR D500
* Baudrate: 230400

## Package structure

```text
ros2_ldlidar_d500/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE
├── config/
│   └── d500.yaml
├── launch/
│   └── d500.launch.py
└── src/
    └── d500_node.cpp
```

## Published topics

| Topic   | Type                        | Description   |
| ------- | --------------------------- | ------------- |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR scan |

## Parameters

| Parameter          | Default        | Description                            |
| ------------------ | -------------- | -------------------------------------- |
| `port`             | `/dev/ttyUSB0` | Serial port used by the LiDAR          |
| `baudrate`         | `230400`       | Serial baudrate                        |
| `frame_id`         | `lidar_link`   | Frame ID used in the LaserScan message |
| `topic_name`       | `/scan`        | Published scan topic                   |
| `range_min`        | `0.03`         | Minimum valid range in meters          |
| `range_max`        | `12.0`         | Maximum valid range in meters          |
| `scan_frequency`   | `10.0`         | Expected scan frequency in Hz          |
| `clockwise`        | `true`         | Reverse scan direction if needed       |
| `angle_offset_deg` | `0.0`          | Angular correction in degrees          |
| `bins`             | `720`          | Number of LaserScan bins               |

## Installation

Clone the package into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/SamuelSRI/ros2_ldlidar_d500.git
```

Build the package:

```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select ros2_ldlidar_d500
source install/setup.bash
```

For ROS2 Jazzy:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_ldlidar_d500
source install/setup.bash
```

## Serial permissions

The user must have permission to access the serial port.

Add the user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

After reboot, check:

```bash
groups
```

The `dialout` group should appear in the list.

## Configuration

Edit the configuration file:

```bash
nano config/d500.yaml
```

Example configuration:

```yaml
d500_node:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 230400

    frame_id: "lidar_link"
    topic_name: "/scan"

    range_min: 0.03
    range_max: 12.0
    scan_frequency: 10.0

    clockwise: true
    angle_offset_deg: 0.0
    bins: 720
```

Using `/dev/serial/by-path/` is recommended instead of `/dev/ttyUSB0`, because `/dev/ttyUSB0` can change after reboot or when another USB device is connected.

Find the stable serial path with:

```bash
ls -l /dev/serial/by-path/
```

Example:

```yaml
port: "/dev/serial/by-path/platform-xxxx-usb-0:1.2:1.0"
```

## Run

Launch the node:

```bash
ros2 launch ros2_ldlidar_d500 d500.launch.py
```

The node publishes the LiDAR scan on:

```text
/scan
```

## Test

List ROS2 topics:

```bash
ros2 topic list
```

Read one scan message:

```bash
ros2 topic echo /scan --once
```

Check the scan frequency:

```bash
ros2 topic hz /scan
```

## RViz visualization

Start RViz:

```bash
rviz2
```

Set the fixed frame to:

```text
lidar_link
```

Then add a LaserScan display:

```text
Add -> LaserScan -> Topic: /scan
```

## Robot frame integration

For use on a robot, the LiDAR frame should be connected to the robot base frame using a static transform or a fixed joint in the URDF.

Example URDF:

```xml
<link name="lidar_link"/>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.20 0.0 0.25" rpy="0 0 0"/>
</joint>
```

The values in `xyz` must be adapted to the real LiDAR position on the robot.

## Troubleshooting

If the scan is mirrored, change:

```yaml
clockwise: false
```

If the scan is rotated, adjust:

```yaml
angle_offset_deg: 90.0
```

or:

```yaml
angle_offset_deg: 180.0
```

If no scan is published, check the serial port:

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-path/
```

Check that the user has access to the serial port:

```bash
groups
```

Check that the package is sourced:

```bash
source ~/ros2_ws/install/setup.bash
```

## License

This project is licensed under the MIT License.

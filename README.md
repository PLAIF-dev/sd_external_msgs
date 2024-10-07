# sd_external_msgs

The ROS2 interface package for Samsung Display Expo

## Environment

Ubuntu 22.04, ROS2 Humble

## Installation

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/PLAIF-dev/sd_external_msgs.git
cd ..
colcon build --packages-select sd_external_msgs
source install/setup.bash
```

## Examples scripts

Test server

```bash
python3 examples/test_server.py
```

Test client

```bash
python3 examples/test_client.py
```

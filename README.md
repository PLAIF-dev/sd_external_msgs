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

Python dependencies:

```bash
sudo apt update
sudo apt install python3-rosdep ros-humble-vision-msgs
pip install empy==3.3.4 catkin-pkg lark rosdep
```

## Interfaces to use

Here are descriptions for each section:

### Action Client (Client SD -> Server Plaif)

- **Action name**: `task/execute`
  - **Action Type**: `sd_external_msgs.action.TaskExecution`
  - **Description**: Sends a goal request from the SD client to the Plaif server to execute a sequence of predefined tasks. These tasks may involve manipulation actions like picking, opening, or moving objects. The action server processes the request and sends periodic feedback and the final result back to the SD client.

### Server Clients (Client SD -> Server Plaif)

- **Service name**: `gripper/control`
  - **Service Type**: `sd_external_msgs.srv.GripperControl`
  - **Description**: Sends a service request to control the state of one or both grippers (left and right). The SD client specifies the desired gripper position (between 0 and 100), and the Plaif server processes the command.

- **Service name**: `task/pause`
  - **Service Type**: `sd_external_msgs.srv.Pause`
  - **Description**: Sends a request to pause the current task execution on the Plaif server.

- **Service name**: `task/reset`
  - **Service Type**: `sd_external_msgs.srv.Reset`
  - **Description**: Sends a request to reset the robot's state or task progress on the Plaif server ***while the task is paused***.

- **Service name**: `task/resume`
  - **Service Type**: `sd_external_msgs.srv.Resume`
  - **Description**: Sends a request to resume ***a paused task*** on the Plaif server. When not paused, this service call will be neglected.

### Publisher Topics (from SD -> to Plaif)

- **Topic name**: `bottles/bboxes`
  - **Msg Type**: `sd_external_msgs.msg.BoundingBox2DArray`
  - **Description**: Publishes an array of bounding boxes of detected objects to the Plaif server. Each bounding box includes `item_id` and `[center_x, center_y, size_x, size_y]`. TODO

### Subscriber Topics (from Plaif -> to SD)

- **Topic name**: `/camera/camera/color/image_raw/compressed`
  - **Msg Type**: `sensor_msgs.msg.CompressedImage`
  - **Description**: Subscribes to a topic providing color images in a compressed format.

- **Topic name**: `/camera/camera/aligned_depth_to_color/image_raw/compressedDepth`
  - **Msg Type**: `sensor_msgs.msg.CompressedImage`
  - **Description**: Subscribes to a topic providing depth images in a compressed format.

- **Topic name**: `/scales_data`
  - **Msg Type**: `std_msgs.msg.Int64MultiArray`
  - **Description**: Subscribes to data from two weight scales in an array format.

- **Topic name**: `/wrench/left`
  - **Msg Type**: `geometry_msgs.msg.Wrench`
  - **Description**: Subscribes to wrench data (force and torque) for the left end effector.

- **Topic name**: `/wrench/right`
  - **Msg Type**: `geometry_msgs.msg.Wrench`
  - **Description**: Subscribes to wrench data (force and torque) for the right end effector.

## Examples action server/client scripts

Test server

```bash
python3 examples/test_server.py
```

Test client

```bash
python3 examples/test_client.py
```

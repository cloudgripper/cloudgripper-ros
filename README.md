# CloudGripper ROS2 Integration
## Overview
This repository contains ROS2 packages for controlling and interacting with CloudGripper system.

## Installation
1. Clone this repo into your ROS2 workspace's `src` directory.
2. Navigate to your workspace root and run `colcon build`. 
3. Source the setup script: `source install/setup.bash`.

## Setup
Ensure `CLOUDGRIPPER_API_KEY` is set in your environment for authentication with the CloudGripper API.

## Usage
- Start the launch file with parameters: `ros2 launch cloudgripper_ros cloudgripper_launch.py -- robot_name:=robot1`. This example shows how to set the `robot_name` parameter to `robot1`. Adjust the parameter value as provided with API token.
- Subscribe to camera images: `/base_camera/image` for the base camera, `/top_camera/image` for the top camera.

### Control CloudGripper using ROS topics:
- **XY Position Control Topic (`/xy`)**: Publish `geometry_msgs/msg/Point` to set X (0.0 to 1.0) and Y (0.0 to 1.0) coordinates.
- **Z Position Control Topic (`/z`)**: Publish `std_msgs/msg/Float32` for Z movement (0.0 to 1.0).
- **Gripper Control Topic (`/gripper_value`)**: Publish `std_msgs/msg/Float32` to adjust the gripper' position (0.0 to 1.0).
- **Rotation Control Topic (`/rotation_angle`)**: Publish `std_msgs/msg/Int16`  to adjust the rotation angle (0 to 180 degrees).
- **Step Commands Topic (`/step_command`)**: Publish `std_msgs/msg/String` messages with values `"right"`, `"left"`, `"forward"`, or `"backward"` to step move the robot,


### Keyboard Control
- **Activate Keyboard Control**: Run the node with `ros2 run cloudgripper_ros keyboard_control`. This will enable real-time keyboard control for the Cloudgripper.
- **Key Bindings**:
  - **Movement**:
    - `W`: Move forward
    - `S`: Move backward
    - `A`: Move left
    - `D`: Move right
  - **Z-Axis Control**:
    - `Arrow Up`: Move up
    - `Arrow Down`: Move down
  - **Rotation Control**:
    - `Arrow Left`: Rotate left
    - `Arrow Right`: Rotate right
  - **Gripper Control**:
    - `Z`: Close gripper
    - `X`: Open gripper

Ensure the terminal window running the keyboard control node is active to receive key inputs.
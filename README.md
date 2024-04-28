# CloudGripper ROS2 Integration
## Overview
This repository contains ROS2 packages for controlling and interacting with CloudGripper system.

## Installation
1. Clone this repo into your ROS2 workspace's `src` directory.
2. Navigate to your workspace root and run `colcon build`. 
3. Source the setup script: `source install/setup.bash`.

## Setup
Ensure `CLOUDGRIPPER_API_KEY` is set in your environment for authentication with the CloudGripper API.
```bash
  export CLOUDGRIPPER_API_KEY="your_api_key_here"
  ```

## Usage
- Start the launch file with parameters: `ros2 launch cloudgripper_ros cloudgripper_launch.py -- robot_name:=robot1`. This example shows how to set the `robot_name` parameter to `robot1`. Adjust the parameter value as provided with API token.
- Subscribe to camera images: `/base_camera/image` for the base camera, `/top_camera/image` for the top camera.

### Access Camera Images via Service Calls
- **Base Camera**: Call the `get_camera_image` service with the `camera_type` parameter set to `base` to fetch the base camera image.
- **Top Camera**: Call the `get_camera_image` service with the `camera_type` parameter set to `top` to fetch the top camera image.

### Republishing Images on ROS Topics
The `image_re_publish.py` node subscribes to the image services and republishes the images to ROS topics.
- **Functionality**:
  - The node subscribes to the `get_camera_image` services for both base and top cameras.
  - It retrieves images once from each camera service and then publishes these current images to `/base_camera/image` and `/top_camera/image`.
  
- **Usage**:
  - To launch this node and start image republishing:
    ```bash
    ros2 run cloudgripper_ros image_re_publish
    ```


### Accessing Robot State via Service Call
- **Fetch State**: Call the `get_robot_state` service to fetch the current state of the CloudGripper. 

### Control CloudGripper using ROS topics:
- **XY Position Control Topic (`/xy`)**: Publish `geometry_msgs/msg/Point` to set X (0.0 to 1.0) and Y (0.0 to 1.0) coordinates.
- **Z Position Control Topic (`/z`)**: Publish `std_msgs/msg/Float32` for Z movement (0.0 to 1.0).
- **Gripper Control Topic (`/gripper_value`)**: Publish `std_msgs/msg/Float32` to adjust the gripper' position (0.0 to 1.0).
- **Rotation Control Topic (`/rotation_angle`)**: Publish `std_msgs/msg/Int16`  to adjust the rotation angle (0 to 180 degrees).
- **Step Commands Topic (`/step_command`)**: Publish `std_msgs/msg/String` messages with values `"right"`, `"left"`, `"forward"`, or `"backward"` to step move the robot.

### Keyboard Control
- **Activate Keyboard Control**: Run the node with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`. This will enable real-time keyboard control for the Cloudgripper.
- **Key Bindings**:
  - **Movement**:
    - `i`: Move forward
    - `k`: Move backward
    - `j`: Move left
    - `l`: Move right
  - **Z-Axis Control**:
    - `t`: Move up
    - `b`: Move down
  - **Rotation Control**:
    - `u`: Rotate left
    - `i`: Rotate right
  - **Gripper Control**:
    - `m`: Close gripper
    - `,`: Open gripper

Ensure the terminal window running the keyboard control node is active to receive key inputs.
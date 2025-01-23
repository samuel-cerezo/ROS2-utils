# D435 ROS2

The **D435 ROS2** repository provides an interface for the Intel RealSense D435 depth camera within the Robot Operating System 2 (ROS2). This project aims to facilitate the integration and utilization of the D435 camera in robotics applications by leveraging ROS2's capabilities.

## Overview

The Intel RealSense D435 is a versatile depth camera designed for various applications such as robotics, computer vision, and augmented reality. This repository offers a comprehensive ROS2 wrapper for the D435 camera, enabling users to access its features and functionalities seamlessly. The integration allows for the publication of camera data, including color and depth images, IMU data, and point clouds, making it suitable for a range of robotic applications.

## Features

- **Depth and Color Streaming**: Stream both depth and color images from the D435 camera.
- **IMU Integration**: Access inertial measurement unit (IMU) data for motion tracking and orientation sensing.
- **Point Cloud Generation**: Generate point clouds for 3D mapping and object recognition tasks.
- **Flexible Configuration**: Easily configure camera parameters, including resolution, frame rate, and filters.
- **Multi-Camera Support**: Support for multiple D435 cameras running concurrently, allowing for complex setups.

## Installation

### Prerequisites

- Ubuntu 20.04 or 22.04
- ROS2 (Humble or Iron) installed
- Intel RealSense SDK 2.0

### Steps

1. **Install Dependencies**: Ensure that the required packages and dependencies are installed. You can do this by running:
   ```bash
   sudo apt update
   sudo apt install ros-<ros_distro>-ros-base python3-colcon-common-extensions
   ```

2. **Clone the Repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/samuel-cerezo/d435-ROS2.git
   ```

3. **Install the RealSense SDK**:
   Follow the instructions from the [Intel RealSense SDK documentation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation/pc.md) to install the SDK.

4. **Build the Workspace**:
   Navigate back to your workspace directory and build the package:
   ```bash
   cd ~/ros2_ws
   rosdep install -i --from-path src --rosdistro <ros_distro> -y
   colcon build
   ```

5. **Source the Environment**:
   After building, source your ROS2 setup:
   ```bash
   source install/local_setup.bash
   ```

## Usage

To launch the D435 camera node, use the following command:
```bash
ros2 launch d435_camera rs_launch.py
```
You can also customize parameters such as resolution and frame rates by modifying the launch file or passing arguments via the command line.

### Published Topics

The D435 camera will publish various topics, including but not limited to:
- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/camera/imu`

Use the following command to list all available topics:
```bash
ros2 topic list
```

## Contributing

Contributions to the D435 ROS2 project are welcome! If you have suggestions for improvements or features, please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Thanks to the Intel RealSense team for providing the SDK and documentation.
- ROS2 community for their continued support and development.

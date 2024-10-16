#!/bin/bash

# Topic names
topic_imu_accel="/camera/camera/accel/sample"
topic_aligned_depth="/camera/camera/aligned_depth_to_color/image_raw"
topic_rgb="/camera/camera/color/image_raw"
topic_point_cloud="/camera/camera/depth/color/points"
topic_depth="/camera/camera/depth/image_rect_raw"
topic_imu_gyro="/camera/camera/gyro/sample"
topic_imu_aligned="/camera/camera/imu"
topic_rgbd_syncro="/camera/camera/rgbd"

# Paths and files
ros_ws="$HOME/dev/environment_modeling"
yaml_filename = "config"
bag_file_path="$HOME/Desktop"
bag_file_name="bag_test"

# Function to handle errors
error_exit() {
  echo "$1" 1>&2
  exit 1
}

# Source the ROS 2 workspace
#if [ -f "$ros_ws/install/setup.bash" ]; then
#  source "$ros_ws/install/setup.bash"
#else
#  error_exit "Error: ROS 2 workspace setup file not found."
#fi

# Change to the bag file directory
if cd "$bag_file_path"; then
  echo "Changed directory to $bag_file_path"
else
  error_exit "Error: Failed to change directory to $bag_file_path."
fi


# Record the topics
echo "ros2 bag record started..."
ros2 bag record -o $bag_file_name \
  $topic_imu_accel \
  $topic_aligned_depth \
  $topic_rgb \
  $topic_point_cloud \
  $topic_depth \
  $topic_imu_gyro \
  $topic_imu_aligned \
  $topic_rgbd_syncro     

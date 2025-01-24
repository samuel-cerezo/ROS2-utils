#!/bin/bash

# Topic names
#topic_imu_accel="/camera/camera/accel/sample"
#topic_aligned_depth="/camera/camera/aligned_depth_to_color/image_raw"
#topic_rgb="/camera/camera/color/image_raw"
#topic_point_cloud="/camera/camera/depth/color/points"
#topic_depth="/camera/camera/depth/image_rect_raw"
#topic_imu_gyro="/camera/camera/gyro/sample"
#topic_imu_aligned="/camera/camera/imu"
#topic_rgbd_syncro="/camera/camera/rgbd"
topic_joints="/joint_states"

# Paths and files
ros_ws="/home/samuel/dev/environment_modeling"
yaml_filename="config"
bag_file_path="/home/samuel/dev/environment_modeling/ROSBAGS"

# Check for bag file name argument
if [ -z "$1" ]; then
  echo "Usage: $0 <bag_file_name>"
  exit 1
fi

bag_file_name="$1"

# Function to handle errors
error_exit() {
  echo "$1" 1>&2
  exit 1
}

# Change to the bag file directory
if cd "$bag_file_path"; then
  echo "Changed directory to $bag_file_path"
else
  error_exit "Error: Failed to change directory to $bag_file_path."
fi

# Record the topics
echo "ros2 bag record started..."
ros2 bag record -o "$bag_file_name" $topic_joints

# usage:
#       bash /home/samuel/dev/environment_modeling/scripts/KUKA/create_trajectory/record_data.sh <my_bagfile_name>

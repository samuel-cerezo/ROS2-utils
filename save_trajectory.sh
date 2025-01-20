#!/bin/bash

# Check if the ROSBAG name is provided as an argument
if [ -z "$1" ]; then
  echo "You must provide the ROSBAG name as the first argument."
  exit 1
else
  BAG_NAME=$1
fi

# Window 1: Run the Docker command with ROS2
gnome-terminal -- bash -c "
echo 'Starting ROS2 Launch in Docker...';
docker run --network host -v /home/samuel/dev/environment_modeling/scripts/KUKA/create_trajectory/iiqka_driver_torque_config.yaml:/root/iiqka_driver_torque_config.yaml  --rm docker.pkg.rd.kuka.com/kuka/cr/ros2robot:0.1.4-humble ros2 launch kuka_tic startup_iiqka.launch.py -- robot_model:=lbr_iisy3_r760 client_ip:=192.168.1.90 controller_ip:=192.168.1.100 driver_config:=/root/iiqka_driver_torque_config.yaml;
exec bash
"

sleep 5

# Window 2: Execute the ros2 bag record script
gnome-terminal -- bash -c "

# Topic names
topic_imu_accel='/camera/camera/accel/sample'
topic_aligned_depth='/camera/camera/aligned_depth_to_color/image_raw'
topic_rgb='/camera/camera/color/image_raw'
topic_point_cloud='/camera/camera/depth/color/points'
topic_depth='/camera/camera/depth/image_rect_raw'
topic_imu_gyro='/camera/camera/gyro/sample'
topic_imu_aligned='/camera/camera/imu'
topic_rgbd_syncro='/camera/camera/rgbd'
topic_joints='/joint_states'

# Paths and files
ros_ws='/home/samuel/dev/environment_modeling'
yaml_filename='config'
bag_file_path='/home/samuel/dev/environment_modeling/ROSBAGS'

# Retrieve BAG_NAME from parent script
bag_file_name=\"$BAG_NAME\"

# Function to handle errors
error_exit() {
  echo \"\$1\" 1>&2;
  read -n 1 -p 'Press any key to exit...';
  exit 1;
}

# Change to the bag file directory
echo 'Changing to ROSBAG directory: \$bag_file_path';
if cd \"\$bag_file_path\"; then
  echo 'Changed directory to \$bag_file_path';
else
  error_exit 'Error: Failed to change directory to \$bag_file_path.';
fi

# Record the topics
echo 'Starting ROSBag recording...';
ros2 bag record -o \"\$bag_file_name\" \\
  \$topic_imu_accel \\
  \$topic_aligned_depth \\
  \$topic_rgb \\
  \$topic_point_cloud \\
  \$topic_depth \\
  \$topic_imu_gyro \\
  \$topic_imu_aligned \\
  \$topic_rgbd_syncro \\
  \$topic_joints;
if [ \$? -eq 0 ]; then
  echo 'ROSBag recording started successfully.';
else
  error_exit 'Error: Failed to start ROSBag recording.';
fi;

# Prevent terminal from closing automatically
echo 'Recording ROSBag. Press any key to exit...';
read -n 1
"

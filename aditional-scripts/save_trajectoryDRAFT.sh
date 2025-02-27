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

# Window 2: Configure and activate the robot_manager
gnome-terminal -- bash -c "
echo 'Configuring robot_manager...';
ros2 lifecycle set robot_manager configure;
if [ \$? -eq 0 ]; then
  echo 'Activating robot_manager...';
  ros2 lifecycle set robot_manager activate;
fi;
exec bash
"

# Window 3: Echo the /joint_states topic
gnome-terminal -- bash -c "
echo 'Displaying /joint_states topic...';
ros2 topic echo /joint_states;
exec bash
"

# Window 4: Wait for the robot_manager to activate and record the ROSBag
gnome-terminal -- bash -c "
echo 'Waiting for robot_manager to activate...';
while ! ros2 lifecycle get robot_manager | grep -q 'active'; do
  sleep 1;
done;
echo 'robot_manager is active, waiting for confirmation to start recording...';

read -p 'Press ENTER to start recording the ROSBag...';

topic_joints='/joint_states';
ros_ws='\$HOME/dev/environment_modeling';
bag_file_path='\$HOME/dev/environment_modeling/ROSBAGS';
bag_file_name='\$BAG_NAME';

# Change to the ROSBAG directory
echo 'Changing to ROSBAG directory: \$bag_file_path';
if cd \"\$bag_file_path\"; then
  echo 'Successfully changed to the ROSBAG directory.';
else
  echo 'Error: Could not change to the ROSBAG directory.' 1>&2;
  exit 1;
fi

# Record the topic in the ROSBag file
echo 'Starting the ROSBag recording...';
ros2 bag record -o \"\$bag_file_name\" \$topic_joints;
if [ \$? -eq 0 ]; then
  echo 'ROSBag recording started successfully.';
else
  echo 'Error: Failed to start ROSBag recording.' 1>&2;
fi;

echo 'Press any key to exit...';
read -n 1
"

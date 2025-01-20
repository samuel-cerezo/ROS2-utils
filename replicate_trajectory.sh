#!/bin/bash

# Check if sufficient arguments were provided
if [ "$1" == "virtual" ]; then
  VENTANA1_CMD="ros2 launch kuka_iiqka_eac_driver startup_with_rviz.launch.py"
elif [ "$1" == "real" ]; then
  VENTANA1_CMD="ros2 launch kuka_iiqka_eac_driver startup.launch.py -- robot_model:=lbr_iisy3_r760 client_ip:=192.168.1.90 controller_ip:=192.168.1.100 driver_config:=\$HOME/ros2_humble/src/kuka_drivers/kuka_iiqka_eac_driver/config/joint_trajectory_controller_config.yaml use_fake_hardware:=false"
else
  echo "Invalid argument. Use 'virtual' or 'real'."
  exit 1
fi

# Check if the dataset name was provided as the second argument
if [ -z "$2" ]; then
  echo "You must provide the dataset name as the second argument."
  exit 1
else
  TRAJ_DATA=$2
fi

# Window 1: Execute the ROS2 launch
gnome-terminal -- bash -c "
echo 'Starting ROS2 Launch...';
$VENTANA1_CMD
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
fi; exit
"

# Window 3: Wait for the processes in Window 2 to finish and then execute the Python script
gnome-terminal -- bash -c "
echo 'Waiting for robot_manager to activate...';
while ! ros2 lifecycle get robot_manager | grep -q 'active'; do
  sleep 1;
done;
echo 'Running script...';
/bin/python3.10 /home/samuel/dev/environment_modeling/scripts/KUKA/replicateTrajectory.py $TRAJ_DATA;
"

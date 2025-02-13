#!/bin/bash

VENTANA1_CMD="ros2 launch kuka_iiqka_eac_driver startup_with_rviz.launch.py"

# Check if the dataset name was provided as the second argument
if [ -z "$1" ]; then
  echo "You must provide the dataset name as the second argument."
  exit 1
else
  TRAJ_DATA=$1
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
python3 post_processing/fJoints2Pose.py --input $TRAJ_DATA;
"

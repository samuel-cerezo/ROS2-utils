#!/bin/bash

# Lanzar el nodo RealSense en una nueva terminal
gnome-terminal -- bash -c "ros2 launch realsense2_camera rs_launch_get_params_from_yaml.py config_file:=/home/samuel/dev/environment_modeling/scripts/KUKA/dataset_record/config_samuel.yaml; exec bash"

# Esperar 3 segundos para asegurarse de que el nodo está corriendo
sleep 3

# Desactivar y volver a activar el giroscopio y el acelerómetro en otra terminal
gnome-terminal -- bash -c "ros2 param set /camera/camera enable_gyro false; \
                        ros2 param set /camera/camera enable_accel false; \
                        sleep 2; \
                        ros2 param set /camera/camera enable_gyro true; \
                        ros2 param set /camera/camera enable_accel true; \
                        exit"

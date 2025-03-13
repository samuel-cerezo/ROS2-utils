import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.spatial.transform import Rotation as R
import os
import argparse
from transformations_utils import *


def main():
    parser = argparse.ArgumentParser(description='Compare poses (obtained by joint-robot vs gt)')
    parser.add_argument('--input', type=str, required=True, help='Folder containing dataset')
    args = parser.parse_args()
    
    dataset_path = os.path.join('/Volumes/SSD/archivos/KUKA_dev/environment_modeling/ROSBAGS', args.input)
    transforms = load_yaml_transformations(os.path.join(dataset_path,  "extrinsics.yaml"))

    #----- retrieving all transformations ----
    T_world_base_marker = transforms[('base_marker_ring', 'world')]                     # base markers --> world
    T_robot_base_base_marker = transforms[('base_marker_ring', 'robot_base')]           # base markers --> robot base
    T_robot_flange_flange_marker = transforms[('flange_marker_ring', 'robot_flange')]   # flange markers --> flange 
    T_robot_flange_rgb = transforms[('rgb_sensor', 'robot_flange')]                     # camera  --> flange
    
    # ---- calculation for useful transformations ---
    T_flange_marker_robot_flange = np.linalg.inv(T_robot_flange_flange_marker)          # flange  --> flange markers
    T_rgb_flange_markers = np.linalg.inv(T_robot_flange_rgb) @ np.linalg.inv(T_flange_marker_robot_flange)  # flange markers --> camera 
    T_base_marker_robot =  np.linalg.inv(T_robot_base_base_marker)
    T_world_robot_base = T_world_base_marker @ T_base_marker_robot                      # robot base --> world

    timestamps_flange, poses_flange = load_poses(os.path.join(dataset_path, "robot_data/flange_poses.txt"))
    timestamps_gt, poses_gt = load_poses(os.path.join(dataset_path, "groundtruth.txt"))

    camera_world = []
    camera_world_gt = []

    camera_world_q, camera_world_positions  = compute_camera_world_positions(poses_flange, T_world_base_marker, T_robot_base_base_marker, T_robot_flange_rgb)
    camera_world_q_gt, camera_world_positions_gt = compute_camera_world_positions_gt(poses_gt, T_rgb_flange_markers)

    print(len(camera_world_positions_gt))
    print(len(camera_world_positions))
    # Graficar
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(camera_world_positions[:, 0], camera_world_positions[:, 1], camera_world_positions[:, 2], label="Camera (obtained by robot-joints)", linestyle='--')
    ax.plot(camera_world_positions_gt[:, 0], camera_world_positions_gt[:, 1], camera_world_positions_gt[:, 2], label="Groundtruth ", linestyle=':')
    ax.legend()
    plt.show()

if __name__ == '__main__':
    main()








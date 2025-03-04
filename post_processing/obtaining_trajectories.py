import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.spatial.transform import Rotation as R
import os
import argparse


def load_yaml_transformations(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    
    transformations = {}
    for entry in data['T']:
        src = entry['source_frame']
        tgt = entry['target_frame']
        matrix = np.array(entry['matrix']['data']).reshape(4, 4)
        transformations[(src, tgt)] = matrix
    
    return transformations

def load_poses(txt_path):
    timestamps = []
    poses = []
    with open(txt_path, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            values = list(map(float, line.split()))
            timestamps.append(values[0])
            q = values[1:5]  # qx, qy, qz, qw
            t = values[5:8]  # tx, ty, tz
            poses.append((q, t))
    return np.array(timestamps), poses

def pose_to_homogeneous(q, t):
    rot_matrix = R.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot_matrix
    T[:3, 3] = t
    return T

def transform_poses(poses, transform):
    transformed_poses = []
    for q, t in poses:
        T = pose_to_homogeneous(q, t)
        T_transformed = transform @ T
        transformed_poses.append(T_transformed[:3, 3])
    return np.array(transformed_poses)



def main():
    parser = argparse.ArgumentParser(description='Align groundtruth and flange poses to image timestamps')
    parser.add_argument('--input', type=str, required=True, help='Folder containing dataset')
    args = parser.parse_args()
    
    #dataset_path = os.path.join('/home/samuel/dev/environment_modeling/ROSBAGS', args.input)
    dataset_path = os.path.join('/Volumes/SSD/archivos/KUKA_dev/environment_modeling/ROSBAGS', args.input)

    # Cargar transformaciones
    yaml_path = "extrinsics.yaml"


    transforms = load_yaml_transformations(os.path.join(dataset_path, yaml_path))

    # Matriz de transformación completa de flange a world
    T_world_base_marker = transforms[('base_marker_ring', 'world')]     # base markers --> world
    T_base_marker_robot_base = transforms[('base_marker_ring', 'robot_base')]   # base markers --> robot base
    T_flange_marker_robot_flange = transforms[('flange_marker_ring', 'robot_flange')]   # flange markers --> flange 
    T_rgb_robot_flange = transforms[('rgb_sensor', 'robot_flange')]  # camara kirill --> flange

    T_world_robot_base = T_world_base_marker @ np.linalg.inv(T_base_marker_robot_base)
    T_world_robot_flange = T_world_robot_base @ np.linalg.inv(T_flange_marker_robot_flange)
    T_world_camera = T_world_robot_flange @ np.linalg.inv(T_rgb_robot_flange)

    # Cargar poses
    timestamps_flange, poses_flange = load_poses(os.path.join(dataset_path, "flange_poses_aligned.txt"))
    timestamps_gt, poses_gt = load_poses(os.path.join(dataset_path, "groundtruth_aligned.txt"))
    # aca hay que cambiar, porq las poses son respecto del anillo, y necesito transformar hacia el sistema de coordenada de camara 
    # Transformar poses al sistema world
    flange_world_positions = transform_poses(poses_flange, T_world_robot_flange)
    camera_world_positions = transform_poses(poses_flange, T_world_camera)
    groundtruth_positions = np.array([t for _, t in poses_gt])

    # Graficar
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(flange_world_positions[:, 0], flange_world_positions[:, 1], flange_world_positions[:, 2], label="Flange (World)", linestyle='--')
    ax.plot(camera_world_positions[:, 0], camera_world_positions[:, 1], camera_world_positions[:, 2], label="Camera (World)", linestyle='-.')
    ax.plot(groundtruth_positions[:, 0], groundtruth_positions[:, 1], groundtruth_positions[:, 2], label="Ground Truth", linestyle=':')
    ax.legend()
    plt.show()

    
if __name__ == '__main__':
    main()








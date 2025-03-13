import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.spatial.transform import Rotation as R
import os
import argparse

def compute_ate_rmse(estimated_positions, groundtruth_positions):   
    """
    Computes the Absolute Trajectory Error (ATE) based on Root Mean Square Error (RMSE).

    Parameters:
    - estimated_positions: np.array of shape (N,3) with estimated positions (X, Y, Z)
    - groundtruth_positions: np.array of shape (N,3) with ground truth positions (X, Y, Z)

    Returns:
    - ATE RMSE (float)
    """
    assert estimated_positions.shape == groundtruth_positions.shape, "Trajectories must have the same length"

    errors = np.linalg.norm(estimated_positions - groundtruth_positions, axis=1)
    ate_rmse = np.sqrt(np.mean(errors**2))
    
    return ate_rmse

def load_yaml_transformations(yaml_path):
    """
    Loads transformation matrices from a YAML file.

    Parameters:
    - yaml_path (str): Path to the YAML file.

    Returns:
    - transformations (dict): Dictionary with frame pairs as keys and 4x4 transformation matrices as values.
    """
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
    """
    Loads poses from a text file.

    Parameters:
    - txt_path (str): Path to the text file.

    Returns:
    - timestamps (np.array): Array of timestamps.
    - poses (list of tuples): Each tuple contains a quaternion (qx, qy, qz, qw) and a translation vector (tx, ty, tz).
    """
    timestamps = []
    poses = []
    with open(txt_path, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            values = list(map(float, line.split()))
            timestamps.append(values[0])
            q = values[1:5]  # Quaternion (qx, qy, qz, qw)
            t = values[5:8]  # Translation (tx, ty, tz)
            poses.append((q, t))
    return np.array(timestamps), poses

def pose_to_homogeneous(q, t):
    """
    Converts a pose (quaternion and translation) into a 4x4 homogeneous transformation matrix.

    Parameters:
    - q (list): Quaternion (qx, qy, qz, qw)
    - t (list): Translation vector (tx, ty, tz)

    Returns:
    - T (np.array): 4x4 homogeneous transformation matrix.
    """
    rot_matrix = R.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot_matrix
    T[:3, 3] = t
    return T

def homogeneous_to_pose(T):
    """
    Converts a 4x4 homogeneous transformation matrix into a pose (quaternion and translation).

    Parameters:
    - T (np.array): 4x4 homogeneous transformation matrix.

    Returns:
    - q (np.array): Quaternion (qx, qy, qz, qw)
    - t (np.array): Translation vector (tx, ty, tz)
    """
    rot_matrix = T[:3, :3]
    t = T[:3, 3]
    q = R.from_matrix(rot_matrix).as_quat()
    return q, t

def transform_poses_left(poses, transform):
    """
    Applies a transformation matrix to a set of poses from the left.

    Parameters:
    - poses (list of tuples): List of (quaternion, translation) tuples.
    - transform (np.array): 4x4 transformation matrix.

    Returns:
    - transformed_poses (np.array): Transformed positions (N,3).
    """
    transformed_poses = []
    for q, t in poses:
        T = pose_to_homogeneous(q, t)
        T_transformed = transform @ T
        transformed_poses.append(T_transformed[:3, 3])
    return np.array(transformed_poses)

def transform_poses_right(poses, transform):
    """
    Applies a transformation matrix to a set of poses from the right.

    Parameters:
    - poses (list of tuples): List of (quaternion, translation) tuples.
    - transform (np.array): 4x4 transformation matrix.

    Returns:
    - transformed_poses (np.array): Transformed positions (N,3).
    """
    transformed_poses = []
    for q, t in poses:
        T = pose_to_homogeneous(q, t)
        T_transformed = T @ transform
        transformed_poses.append(T_transformed[:3, 3])
    return np.array(transformed_poses)

def compute_camera_world_positions(poses_flange, T_world_base_marker, T_robot_base_base_marker, T_robot_flange_rgb):
    """
    Computes the world positions and orientations (as quaternions) of a camera given a set of flange poses.

    Parameters:
        poses_flange (list of tuples): Each tuple contains rotation and translation information for a flange pose.
        T_world_base_marker (numpy.ndarray): 4x4 transformation matrix from the world to the base marker.
        T_robot_base_base_marker (numpy.ndarray): 4x4 transformation matrix from the robot base to the base marker.
        T_robot_flange_rgb (numpy.ndarray): 4x4 transformation matrix from the robot flange to the RGB camera.

    Returns:
        tuple: 
            - numpy.ndarray: Nx4 array of camera orientations as quaternions.
            - numpy.ndarray: Nx3 array of camera positions in the world frame.
    """
    camera_world_positions = []
    camera_world_orientations = []
    
    for f_pose in poses_flange:
        # Convert pose information to a homogeneous transformation matrix
        T_hom = pose_to_homogeneous(f_pose[0], f_pose[1])
        
        # Compute the transformed pose in the world frame
        T_transformed = (T_world_base_marker @ 
                         np.linalg.inv(T_robot_base_base_marker) @ 
                         T_hom @ 
                         T_robot_flange_rgb)
        
        # Extract the translation component (position)
        position = T_transformed[:3, 3]
        camera_world_positions.append(position)
        
        # Extract the rotation component and convert it to quaternion
        rotation_matrix = T_transformed[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()
        camera_world_orientations.append(quaternion)
    
    return np.array(camera_world_orientations), np.array(camera_world_positions)

def compute_camera_world_positions_gt(poses_gt, T_rgb_flange_markers):
    """
    Computes the world positions and orientations (as quaternions) of a camera given a set of ground truth poses.

    Parameters:
        poses_gt (list of tuples): Each tuple contains rotation and translation information for a ground truth pose.
        T_rgb_flange_markers (numpy.ndarray): 4x4 transformation matrix from the RGB camera to the flange markers.

    Returns:
        tuple:
            - numpy.ndarray: Nx4 array of camera orientations as quaternions.
            - numpy.ndarray: Nx3 array of camera positions in the world frame.
    """
    camera_world_positions_gt = []
    camera_world_orientations_gt = []
    
    for gt_pose in poses_gt:
        # Convert ground truth pose information to a homogeneous transformation matrix
        T_hom = pose_to_homogeneous(gt_pose[0], gt_pose[1])
        
        # Compute the transformed pose in the world frame
        T_transformed = T_hom @ np.linalg.inv(T_rgb_flange_markers)
        
        # Extract the translation component (position)
        position = T_transformed[:3, 3]
        camera_world_positions_gt.append(position)
        
        # Extract the rotation component and convert it to quaternion
        rotation_matrix = T_transformed[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()
        camera_world_orientations_gt.append(quaternion)
    
    return np.array(camera_world_orientations_gt),np.array(camera_world_positions_gt)

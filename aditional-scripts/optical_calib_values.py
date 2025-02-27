import numpy as np
import os

def read_poses_file(poses_file_path):
    """
    Reads the pose file and returns a list of homogeneous transformation matrices.
    
    Args:
        poses_file_path (str): Path to the poses file (homogeneous matrices).
        
    Returns:
        list: A list of 4x4 numpy arrays representing the homogeneous transformation matrices.
    """
    T_world_flange_list = []

    if not os.path.exists(poses_file_path):
        print(f"File {poses_file_path} not found.")
        return []

    with open(poses_file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line and not line.startswith('#'):  # Ignore empty lines and comments
                # Split the line by commas and convert to floats
                values = list(map(float, line.split(',')))
                
                if len(values) == 16:
                    # Reshape the flat list into a 4x4 matrix
                    T_world_flange = np.array(values).reshape(4, 4)
                    T_world_flange_list.append(T_world_flange)
                else:
                    print(f"Invalid line format, expected 16 values but got {len(values)}: {line}")
    
    return T_world_flange_list


if __name__ == "__main__":
    # Path to the file containing the homogeneous matrices
    calib_path = '/home/samuel/Desktop/base_robot_calibration/'
    #calib_path = os.path.expandvars("$HOME/Desktop/base_robot_calibration/")

    hom_poses_file_path = os.path.join(calib_path, 'robot_poses.txt')

    # Read the file and get the list of T_world_flange matrices
    T_world_flange_list = read_poses_file(hom_poses_file_path)

    if T_world_flange_list:
        print(f"Loaded {len(T_world_flange_list)} pose matrices.")
        for idx, T in enumerate(T_world_flange_list):
            print(f"\nPose {idx + 1}:\n{T}")

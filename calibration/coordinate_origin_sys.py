import numpy as np
import math
import os
import time
from include.transformations import euler_to_rotation_matrix  # Asegúrate de tener esta función correctamente definida


def check_pose_files(poses_file_path, hom_poses_file_path):
    if os.path.exists(poses_file_path):
        user_action = input('Pose file already exists. Overwrite? (y/n): ').strip().lower()
        if user_action == 'y':
            os.remove(poses_file_path)
            os.remove(hom_poses_file_path) if os.path.exists(hom_poses_file_path) else None
        elif user_action == 'n':
            new_name = input('Enter a new name for the pose file: ').strip()
            poses_file_path = os.path.join(os.path.dirname(poses_file_path), f"{new_name}_6D.txt")
            hom_poses_file_path = os.path.join(os.path.dirname(hom_poses_file_path), f"{new_name}.txt")
    return poses_file_path, hom_poses_file_path


def capture_poses(poses_file_path, hom_poses_file_path, max_poses=40):
    counter = 0

    # Add headers to the pose files
    with open(poses_file_path, "a") as pose_file:
        pose_file.write('# posx, posy, posz, angle1, angle2, angle3\n')
    with open(hom_poses_file_path, "a") as pose_file:
        pose_file.write('# R11, R12, R13, posX, R21, R22, R23, posY, R31, R32, R33, posZ, 0,0,0,1\n')

    try:
        while counter < max_poses:
            user_entry = input('Enter robot pose (x,y,z,angle1,angle2,angle3) or type "quit" to finish: ').strip().lower()

            if user_entry == 'quit':
                break

            # Parse and validate the user input
            try:
                float_list = [float(value) for value in user_entry.split(',')]
                
                # Ensure there are exactly 6 values (3 for position, 3 for rotation angles)
                if len(float_list) != 6:
                    print("Invalid input. Please enter exactly 6 numbers (x,y,z,angle1,angle2,angle3).")
                    continue
                
                position_float = float_list[:3]
                angles_d_float = float_list[3:]
            except ValueError:
                print("Invalid input. Please enter valid numbers separated by commas.")
                continue
            
            # Convert angles to rotation matrix
            rotation_matrix = euler_to_rotation_matrix(angles_d_float, order='ZYX')

            # Prepare the homogeneous pose list
            robot_pose_hom = [
                rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2], position_float[0],
                rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2], position_float[1],
                rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2], position_float[2],
                0, 0, 0, 1
            ]

            # Write the robot pose to the text files
            with open(poses_file_path, "a") as pose_file:
                pose_file.write(','.join(map(str, float_list)) + '\n')

            with open(hom_poses_file_path, "a") as pose_file:
                pose_file.write(','.join(map(str, robot_pose_hom)) + '\n')

            counter += 1
            print(f"Saved pose: {counter}")

    finally:
        print("Pose capture finished.")


if __name__ == "__main__":
    calib_path = '/home/samuel/Desktop/base_robot_calibration/'
    
    # Ensure the calibration path exists
    if not os.path.exists(calib_path):
        os.makedirs(calib_path)

    poses_txt_name = 'robot_poses'
    poses_file_path = os.path.join(calib_path, f"{poses_txt_name}_6D.txt")
    hom_poses_file_path = os.path.join(calib_path, f"{poses_txt_name}.txt")

    # Optional: check if pose files already exist and ask for overwrite or rename
    poses_file_path, hom_poses_file_path = check_pose_files(poses_file_path, hom_poses_file_path)

    capture_poses(poses_file_path, hom_poses_file_path)

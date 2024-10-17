import numpy as np
from include.posesYAML import extract_transformation_matrices  # Import custom functions

def main():
    # Path to the cleaned YAML file
   # yaml_file = '/home/samuel/dev/environment_modeling/scripts/KUKA/calibration/transformations.yaml'
    yaml_file = '/Users/samucerezo/dev/src/my-github/KUKA/calibration/transformations.yaml'

    # Extract transformation matrices
    T = extract_transformation_matrices(yaml_file)

    # Example point in the robot_flange frame (homogeneous coordinates, 4D)
    P_flange = np.array([1, 2, 3, 1])  # Replace x, y, z with your point coordinates
    # Transformation from robot_flange to color_optical_frame
    # Inverse transformation is needed because we are going from robot_flange to color_optical_frame
    T_flange_to_color = np.linalg.inv(T['color_optical_to_robot_flange'])

    # Transform the point to the d400_color_optical_frame
    P_color = T_flange_to_color @ P_flange
    print(f"Point in color_optical_frame: {P_color[:3]}")

    # Transformation from robot_flange to mocap_markers
    # Inverse transformation is needed because we are going from robot_flange to mocap_markers
    T_flange_to_mocap = np.linalg.inv(T['mocap_markers_to_robot_flange'])

    # Transform the point to the mocap_markers frame
    P_mocap = T_flange_to_mocap @ P_flange
    print(f"Point in mocap_markers frame: {P_mocap[:3]}")


if __name__ == "__main__":

    main()

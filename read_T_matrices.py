import numpy as np
import yaml
import os

def extract_transformation_matrices(yaml_file):
    # Load the YAML file
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    # Extract transformations and save them in variables
    transformations = data.get('T', [])

    # Dictionary to hold matrices with variable names
    T_matrices = {}

    for transformation in transformations:
        source_frame = transformation['source_frame']
        destination_frame = transformation['destination_frame']
        matrix_data = transformation['matrix']['data']
        
        # Create a unique variable name based on source and destination frames
        variable_name = f"{source_frame}_to_{destination_frame}".replace('-', '_')  # Replace any dashes with underscores
        matrix_array = np.array(matrix_data).reshape(4, 4)

        T_matrices[variable_name] = matrix_array

    return T_matrices #return a dictionary. Every T is save with a key-value correspondence

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

import numpy as np
import yaml
import os


def extract_transformation_matrices(yaml_file):
    """
    Extract transformation matrices from a YAML file.

    This function reads a YAML file that contains transformation matrices
    between different frames (e.g., camera, robot, etc.) and returns 
    these matrices as a dictionary where the keys are formatted as 
    'source_frame_to_destination_frame'.

    Args:
        yaml_file (str): The path to the YAML file containing transformation matrices.
        
    Returns:
        dict: A dictionary containing transformation matrices. The keys are formatted 
              as 'source_frame_to_destination_frame', and the values are 4x4 transformation 
              matrices represented as NumPy arrays.
    
    Raises:
        FileNotFoundError: If the specified YAML file does not exist.
        yaml.YAMLError: If the YAML file cannot be parsed or is invalid.
    
    Example:
        >>> matrices = extract_transformation_matrices('transformations.yaml')
        >>> print(matrices['color_optical_to_robot_flange'])
        [[ 0.0202555  0.999719   0.0122748 -0.00040487]
         [-0.99961    0.0200144  0.0194587 32.9645   ]
         [ 0.0192076 -0.0126642  0.999735  44.6762   ]
         [ 0.         0.         0.         1.       ]]
    """

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

    return T_matrices  # Return a dictionary. Every transformation matrix is saved with a key-value correspondence.


def main():
    # yaml_file = '/home/samuel/dev/environment_modeling/scripts/KUKA/calibration/transformations.yaml'
    yaml_file = '/Users/samucerezo/dev/src/my-github/KUKA/calibration/transformations.yaml'
    # Extract transformation matrices and save in a dictionary
    T = extract_transformation_matrices(yaml_file)

    # example of usage:
    P_flange = np.array([1, 2, 3, 1])  # point in the robot_flange frame (homogeneous coordinates, 4D)
    T_flange_to_color = np.linalg.inv(T['color_optical_to_robot_flange'])   # Inverse transformation is needed because we are going from robot_flange to color_optical_frame
    P_color = T_flange_to_color @ P_flange      # Transform the point to the color_optical_frame
    print(f"Point in color_optical_frame: {P_color[:3]}")

if __name__ == "__main__":
    main()

import numpy as np
import yaml

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
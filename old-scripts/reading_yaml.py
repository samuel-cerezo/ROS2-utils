import yaml
import numpy as np
import os
import numpy as np

def read_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def extract_transformation_matrices(data):
    trans_matrices = []
    
    for item in data['T']:
        source_frame = item['source_frame']
        destination_frame = item['destination_frame']
        matrix_data = item['matrix']['data']
        
        # Reshape the flat list into a 4x4 matrix
        matrix = np.array(matrix_data).reshape(4, 4)
        
        trans_matrices.append({
            'source_frame': source_frame,
            'destination_frame': destination_frame,
            'matrix': matrix
        })
    
    return trans_matrices

def main():
    # Path to the cleaned YAML file
   # yaml_file = '/home/samuel/dev/environment_modeling/scripts/KUKA/calibration/transformations.yaml'
    yaml_file = '/Users/samucerezo/dev/src/my-github/KUKA/calibration/transformations.yaml'
    yaml_file = os.path.expandvars("$HOME/dev/environment_modeling/scripts/KUKA/calibration/transformations.yaml")

    current_directory = os.getcwd()

    # Read the YAML file
    data = read_yaml(yaml_file)
    
    # Extract transformation matrices
    transformation_matrices = extract_transformation_matrices(data)
    
    # Example usage: printing the matrices
    for T in transformation_matrices:
        print(f"Transformation from {T['source_frame']} to {T['destination_frame']}:")
        print(T['matrix'], "\n")
        matrix_data = T['matrix']
        # Convert to NumPy array and reshape to 4x4
        matrix_array = np.array(matrix_data).reshape(4, 4)
        transformation_matrices.append(matrix_array)

if __name__ == "__main__":

    main()

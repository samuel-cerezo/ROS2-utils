import yaml
import numpy as np

def read_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def extract_transformation_matrices(data):
    trans_matrices = []
    
    for item in data['T']:
        parent_frame = item['parent_frame']
        child_frame = item['child_frame']
        matrix_data = item['matrix']['data']
        
        # Reshape the flat list into a 4x4 matrix
        matrix = np.array(matrix_data).reshape(4, 4)
        
        trans_matrices.append({
            'parent_frame': parent_frame,
            'child_frame': child_frame,
            'matrix': matrix
        })
    
    return trans_matrices

def main():
    # Path to the cleaned YAML file
    yaml_file = 'transformations.yaml'
    
    # Read the YAML file
    data = read_yaml(yaml_file)
    
    # Extract transformation matrices
    transformation_matrices = extract_transformation_matrices(data)
    
    # Example usage: printing the matrices
    for T in transformation_matrices:
        print(f"Transformation from {T['parent_frame']} to {T['child_frame']}:")
        print(T['matrix'], "\n")

if __name__ == "__main__":
    main()

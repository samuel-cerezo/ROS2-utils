import csv
import numpy as np

# Path to the CSV file containing the robot poses
csv_file = '/home/samuel/dev/environment_modeling/scripts/KUKA/Robot_base_calib.csv'

# List to store the poses of the RigidBody
poses_rigidbody = []

# Column indices for the relevant data in the CSV file
indices_rigidbody = {
    'rotation_z': 2,
    'rotation_y': 3,
    'rotation_x': 4,
    'position_x': 5,
    'position_y': 6,
    'position_z': 7
}

def euler_to_rotation_matrix(angles, order='ZYX'):
    """
    Converts Euler angles from the ZYX order to a rotation matrix.

    Parameters:
    angles (list): A list of three angles [rotation_z, rotation_y, rotation_x] in degrees.
    order (str): The order of rotations (default is 'ZYX').

    Returns:
    numpy.ndarray: A 3x3 rotation matrix corresponding to the input Euler angles.
    """
    z, y, x = np.radians(angles)

    Rz = np.array([[np.cos(z), -np.sin(z), 0],
                   [np.sin(z), np.cos(z), 0],
                   [0, 0, 1]])

    Ry = np.array([[np.cos(y), 0, np.sin(y)],
                   [0, 1, 0],
                   [-np.sin(y), 0, np.cos(y)]])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(x), -np.sin(x)],
                   [0, np.sin(x), np.cos(x)]])

    return Rz @ Ry @ Rx  # Multiply rotation matrices

# Read the CSV file
with open(csv_file, 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Skip the first header row
    next(csv_reader)  # Skip the second header row

    # Iterate over each row in the CSV file
    for row in csv_reader:
        try:
            # Extract the pose of the RigidBody (position and rotation)
            pose = {
                'rotation_z': float(row[indices_rigidbody['rotation_z']]),
                'rotation_y': float(row[indices_rigidbody['rotation_y']]),
                'rotation_x': float(row[indices_rigidbody['rotation_x']]),
                'position_x': float(row[indices_rigidbody['position_x']]),
                'position_y': float(row[indices_rigidbody['position_y']]),
                'position_z': float(row[indices_rigidbody['position_z']])
            }

            # Convert to rotation matrix
            rotation_matrix = euler_to_rotation_matrix([
                pose['rotation_z'],
                pose['rotation_y'],
                pose['rotation_x']
            ])

            # Create the homogeneous transformation matrix T
            robot_pose_hom = np.array([
                rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2], pose['position_x'],
                rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2], pose['position_y'],
                rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2], pose['position_z'],
                0, 0, 0, 1
            ]).reshape(4, 4)

            # Add the pose to the list
            poses_rigidbody.append(robot_pose_hom)

        except (ValueError, IndexError):
            # Handle errors in value conversion or out-of-range index
            continue

def remove_outliers(poses, threshold=200.0):
    """
    Removes outlier poses based on the distance between consecutive poses.

    Parameters:
    poses (list): A list of poses represented as 4x4 homogeneous transformation matrices.
    threshold (float): The multiplier for the standard deviation to define outliers (default is 200.0).

    Returns:
    list: A list of filtered poses with outliers removed.
    """
    poses = np.array(poses)
    distances = np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1)  # Calculate distances between poses
    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    filtered_poses = []

    for i, pose in enumerate(poses):
        # Keep the first pose and poses that are within the defined threshold
        if i == 0 or (abs(np.linalg.norm(pose[:3]) - mean_distance) <= threshold * std_distance):
            filtered_poses.append(pose)
    
    return filtered_poses

# Filter the poses to remove outliers
filtered_poses = remove_outliers(poses_rigidbody)

# Print the filtered poses
for pose in filtered_poses:
    print(pose)
print(f'Poses: {len(poses_rigidbody)}')
print(f'Filtered poses: {len(filtered_poses)}')

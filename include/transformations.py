import numpy as np

def euler_to_rotation_matrix(euler_angles, order='ZYX'):
    """
    Convert Euler angles to a rotation matrix.

    Parameters:
    euler_angles (list or np.ndarray): A list or array of three Euler angles (in radians).
                                        Format: [angle1, angle2, angle3]
                                        For ZYX order, these correspond to:
                                        - angle1: Yaw (rotation around z-axis)
                                        - angle2: Pitch (rotation around y-axis)
                                        - angle3: Roll (rotation around x-axis)
    order (str): The order of rotations. Supported orders are 'ZYX', 'XYZ', 'YXZ', etc.

    Returns:
    np.ndarray: A 3x3 rotation matrix corresponding to the input Euler angles.
    """
    
    # Unpack Euler angles
    angle1, angle2, angle3 = euler_angles

    # Initialize rotation matrices
    R = np.eye(3)  # Start with the identity matrix

    if order == 'ZYX':
        # Rotation around Z-axis (yaw)
        Rz = np.array([[np.cos(angle1), -np.sin(angle1), 0],
                       [np.sin(angle1), np.cos(angle1), 0],
                       [0, 0, 1]])

        # Rotation around Y-axis (pitch)
        Ry = np.array([[np.cos(angle2), 0, np.sin(angle2)],
                       [0, 1, 0],
                       [-np.sin(angle2), 0, np.cos(angle2)]])

        # Rotation around X-axis (roll)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(angle3), -np.sin(angle3)],
                       [0, np.sin(angle3), np.cos(angle3)]])

        # Combine the rotations: R = Rz * Ry * Rx
        R = Rz @ Ry @ Rx

    elif order == 'XYZ':
        # Rotation around X-axis (roll)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(angle1), -np.sin(angle1)],
                       [0, np.sin(angle1), np.cos(angle1)]])

        # Rotation around Y-axis (pitch)
        Ry = np.array([[np.cos(angle2), 0, np.sin(angle2)],
                       [0, 1, 0],
                       [-np.sin(angle2), 0, np.cos(angle2)]])

        # Rotation around Z-axis (yaw)
        Rz = np.array([[np.cos(angle3), -np.sin(angle3), 0],
                       [np.sin(angle3), np.cos(angle3), 0],
                       [0, 0, 1]])

        # Combine the rotations: R = Rx * Ry * Rz
        R = Rx @ Ry @ Rz

    else:
        raise ValueError("Unsupported rotation order. Please use 'ZYX' or 'XYZ'.")

    return R

'''
    # Define Euler angles in radians: [yaw, pitch, roll]
    euler_angles = [np.radians(30), np.radians(45), np.radians(60)]  # Example angles

    # Convert to rotation matrix with ZYX order
    rotation_matrix = euler_to_rotation_matrix(euler_angles, order='ZYX')
    print("Rotation Matrix R (ZYX order):")
    print(rotation_matrix)

    # Convert to rotation matrix with XYZ order
    rotation_matrix_xyz = euler_to_rotation_matrix(euler_angles, order='XYZ')
    print("\nRotation Matrix R (XYZ order):")
    print(rotation_matrix_xyz)
'''

import numpy as np

# Quaternion and position data provided by the MoCap CSV file
qX, qY, qZ, qW = 0.504544, 0.494481, 0.498329, -0.502585  # Quaternion components
pX, pY, pZ = -143.087402, 912.757263, -619.818909  # Position in 3D space (mm)

# Normalize the quaternion to ensure it represents a valid rotation
norm = np.sqrt(qX**2 + qY**2 + qZ**2 + qW**2)
qX, qY, qZ, qW = qX / norm, qY / norm, qZ / norm, qW / norm

# Compute the 3x3 rotation matrix from the normalized quaternion
rotation_matrix = np.array([
    [1 - 2 * (qY**2 + qZ**2), 2 * (qX * qY - qZ * qW), 2 * (qX * qZ + qY * qW)],
    [2 * (qX * qY + qZ * qW), 1 - 2 * (qX**2 + qZ**2), 2 * (qY * qZ - qX * qW)],
    [2 * (qX * qZ - qY * qW), 2 * (qY * qZ + qX * qW), 1 - 2 * (qX**2 + qY**2)]
])

# Create the 4x4 homogeneous transformation matrix
# The top-left 3x3 section is the rotation matrix
# The last column represents the translation (position)
transformation_matrix = np.eye(4)  # Initialize as an identity matrix
transformation_matrix[:3, :3] = rotation_matrix  # Set the rotation part
transformation_matrix[:3, 3] = [pX, pY, pZ]  # Set the translation part

# Print the resulting transformation matrix
print(transformation_matrix)

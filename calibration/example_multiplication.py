import numpy as np

# Define the transformation matrix from d400_color_optical_frame to robot_flange
T_d400_to_flange = np.array([
    [0.0202555, 0.999719, 0.0122748, -0.000404874],
    [-0.99961, 0.0200144, 0.0194587, 32.9645],
    [0.0192076, -0.0126642, 0.999735, 44.6762],
    [0, 0, 0, 1]
])

# Define the transformation matrix from mocap_markers to robot_flange
T_mocap_to_flange = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 18.60],
    [0, 0, 0, 1]
])

# Example point in the robot_flange frame (homogeneous coordinates, 4D)
P_robot_flange = np.array([1, 2, 3, 1])  

# Transformation from robot_flange to d400_color_optical_frame
# Inverse transformation is needed because we are going from robot_flange to d400_color_optical_frame
T_flange_to_d400 = np.linalg.inv(T_d400_to_flange)

# Transform the point to the d400_color_optical_frame
P_d400 = T_flange_to_d400 @ P_robot_flange
print(f"Point in d400_color_optical_frame: {P_d400[:3]}")

# Transformation from robot_flange to mocap_markers
# Inverse transformation is needed because we are going from robot_flange to mocap_markers
T_flange_to_mocap = np.linalg.inv(T_mocap_to_flange)

# Transform the point to the mocap_markers frame
P_mocap = T_flange_to_mocap @ P_robot_flange
print(f"Point in mocap_markers frame: {P_mocap[:3]}")

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
from include.transformations import *  # Import custom transformation functions

# ----------------- Folders Management ------------------
calib_path = '/home/samuel/Desktop/d435_calibration/'  # Path for calibration files
poses_txt_name = 'robot_poses'  # Base name for the poses text file

if __name__ == "__main__":
    """
    Main execution block to capture images from a RealSense camera 
    and log robot poses to a text file.
    """

    # Create a directory to save images if it doesn't exist
    file_path = os.path.join(calib_path, 'images/')
    if not os.path.exists(file_path):
        os.makedirs(file_path)

    # Check if the poses file already exists and prompt user for action
    poses_file_path = os.path.join(calib_path, f"{poses_txt_name}_6D.txt")
    hom_poses_file_path = os.path.join(calib_path, f"{poses_txt_name}_hom.txt")
   
    if os.path.exists(poses_file_path):
        userAction = input('Robot pose file already exists. Want to remove it? (y/n): ')
        while userAction.lower() not in ['y', 'n']:
            userAction = input('Robot pose file already exists. Want to remove it? (y/n): ')

        if userAction.lower() == 'y':
            os.remove(poses_file_path)
            if os.path.exists(hom_poses_file_path):
                os.remove(hom_poses_file_path)
        elif userAction.lower() == 'n':
            poses_txt_name = input('Enter a new name for the Robot pose file: ')
            poses_file_path = os.path.join(calib_path, f"{poses_txt_name}_6D.txt")
            hom_poses_file_path = os.path.join(calib_path, f"{poses_txt_name}_hom.txt")

    # ----------------------- Fresh Reset ---------------------------------
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Enable color stream

    print("Resetting...")
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()  # Reset device hardware
        time.sleep(4)  # Wait for reset to complete
    print("Fresh reset done.\n")

    # Verify the device has a color sensor
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires a Depth camera with a Color sensor")
        exit(0)

    # Start streaming
    pipeline.start(config)

    MaxNrImages = 40  # Maximum number of images to capture
    counter = 0  # Initialize counter for saved images

    # Write a header for showing the format in robot pose file txt
    with open(poses_file_path, "a") as pose_file:
        pose_file.write('# posx, posy, posz, angle1, angle2, angle3' + '\n')
    
    with open(hom_poses_file_path, "a") as pose_file:
        pose_file.write('# R11, R12, R13, posx,R21, R22, R23, posy, R31, R32, R33, posz, 0,0,0,1' + '\n')
    
    try:
        while counter < MaxNrImages:
            userEntry = input('Enter the robot pose or (quit) to finish: ')

            if userEntry.lower() == 'quit':
                break
            
            # Split the string into individual values
            string_list = userEntry.split(',')
            # Convert each value to a float
            float_list = [float(value) for value in string_list]
            position_float = float_list[0:3]
            angles_float = float_list[3:]

        #    R11, R12, R13, posx,R21, R22, R23, posy, R31, R32, R33, posz, 0,0,0,1 

            # Convert angles to rotation matrix with ZYX order
            rotation_matrix = euler_to_rotation_matrix(angles_float, order='ZYX')   # use 'ZYX' or 'XYZ'
            #print(rotation_matrix)
            robot_pose_hom = [rotation_matrix[0,0] , rotation_matrix[0,1] , rotation_matrix[0,2], position_float[0],
                              rotation_matrix[1,0] , rotation_matrix[1,1] , rotation_matrix[1,2], position_float[1],
                              rotation_matrix[2,0] , rotation_matrix[2,1] , rotation_matrix[2,2], position_float[2],
                                        0,                  0,                      0,                  1           ]

            # Write the robot pose to the text file (homogenous format)
            with open(hom_poses_file_path, "a") as pose_file:
                pose_file.write(str(robot_pose_hom) + '\n')

            # Write the robot pose to the text file (6D format)
            with open(poses_file_path, "a") as pose_file:
                pose_file.write(userEntry + '\n')

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()  # Get color frame

            # Convert the color frame to a numpy array
            color_image = np.asanyarray(color_frame.get_data())

            # Save the color image with a formatted counter
            counter_str = "{:02}".format(counter)  # Format counter as two digits
            cv2.imwrite(os.path.join(file_path, f"{counter_str}.png"), color_image)  # Save image
            counter += 1
            print(f"{file_path}{counter_str}.png saved.")

        print('Process finished.')

    finally:
        # Stop streaming when done
        pipeline.stop()

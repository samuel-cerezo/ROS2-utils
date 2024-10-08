## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
from include.transformations import * 

# ----------------- folders managing ------------------
calib_path = '/home/samuel/Desktop/d435_calibration/'
poses_txt_name = 'robot_poses'


if __name__ == "__main__":

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


    file_path = calib_path + 'images/'
    if not os.path.exists(file_path):
        os.makedirs(file_path)

    if os.path.exists(calib_path + poses_txt_name + '.txt'):
        userAction = input('Robot pose file already exist. Want to remove it? (y/n):')
        while (userAction.lower() != 'y') and ((userAction.lower() != 'n')):
            userAction = input('Robot pose file already exist. Want to remove it? (y/n):')

        if userAction.lower() == 'y':
            os.remove(calib_path + poses_txt_name + '.txt')
        elif userAction.lower() == 'n':
            poses_txt_name = input('Enter new name for Robot pose file:')

    # ---------------------------------------------------


    # -----------------------fresh reset ---------------------------------
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    print("resetting...")
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()
        time.sleep(4)
    print("fresh reset done")

    # --------------------------------------------------------------------


    # Get device product line for setting a supporting resolution
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
        print("The demo requires Depth camera with Color sensor")
        exit(0)


    #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)


    MaxNrImages = 40
    counter = 0

    try:
            while counter<MaxNrImages:

                userValue = input('Enter the robot pose or (quit) for finish:')

                if userValue.lower() == 'quit':
                    break

                # ------------------------ write the Robot pose in txt file --------
                pose_file = open(calib_path + poses_txt_name + '.txt', "a")
                pose_file.write(userValue + '\n')
                pose_file.close()
                # -----------------------------------------------------------------

                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                #depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                #if not depth_frame or not color_frame:

                # Convert images to numpy arrays
                #depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                #depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                '''
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))
                '''

                # Using cv2.imwrite() method
                # Saving the image
                #cv2.imwrite(file_path + file_name + '.png', color_image)
                counter_str = "{:02}".format(counter)   # convert to string of 2 digits: "05.png"
                cv2.imwrite(file_path + counter_str+'.png', color_image)
                counter += 1
                print(file_path + counter_str+'.png', 'saved.')
                
                # Show images
                #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                #cv2.imshow('RealSense', color_image)
                #cv2.waitKey(1)
            print('Process finished.')

    finally:

        # Stop streaming
        pipeline.stop()
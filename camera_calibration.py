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

    # Define Euler angles in radians for rotation matrix conversion
    euler_angles = [np.radians(30), np.radians(45), np.radians(60)]  # Example angles
    rotation_matrix = euler_to_rotation_matrix(euler_angles, order='ZYX')  # Convert to rotation matrix (ZYX order)
    print("Rotation Matrix R (ZYX order):")
    print(rotation_matrix)

    rotation_matrix_xyz = euler_to_rotation_matrix(euler_angles, order='XYZ')  # Convert to rotation matrix (XYZ order)
    print("\nRotation Matrix R (XYZ order):")
    print(rotation_matrix_xyz)

    # Create a directory to save images if it doesn't exist
    file_path = os.path.join(calib_path, 'images/')
    if not os.path.exists(file_path):
        os.makedirs(file_path)

    # Check if the poses file already exists and prompt user for action
    poses_file_path = os.path.join(calib_path, f"{poses_txt_name}.txt")
    if os.path.exists(poses_file_path):
        userAction = input('Robot pose file already exists. Want to remove it? (y/n): ')
        while userAction.lower() not in ['y', 'n']:
            userAction = input('Robot pose file already exists. Want to remove it? (y/n): ')

        if userAction.lower() == 'y':
            os.remove(poses_file_path)
        elif userAction.lower() == 'n':
            poses_txt_name = input('Enter a new name for the Robot pose file: ')

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
    print("Fresh reset done")

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

    try:
        while counter < MaxNrImages:
            userEntry = input('Enter the robot pose or (quit) to finish: ')

            if userEntry.lower() == 'quit':
                break

            # Write the robot pose to the text file
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

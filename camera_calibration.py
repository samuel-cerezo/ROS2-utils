import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
from include.transformations import euler_to_rotation_matrix  # Import custom transformation functions


def create_directories(calib_path, images_folder='images'):
    """
    Creates necessary directories for storing images and calibration files.

    Args:
        calib_path (str): The base path for calibration data.
        images_folder (str): Folder name to store captured images.
    
    Returns:
        str: Path to the image folder.
    """
    # Create a directory to save images if it doesn't exist
    file_path = os.path.join(calib_path, images_folder)
    if not os.path.exists(file_path):
        os.makedirs(file_path)
    return file_path


def check_pose_files(poses_file_path, hom_poses_file_path):
    """
    Checks if pose files already exist and prompts the user for action.

    Args:
        poses_file_path (str): Path for the 6D pose file.
        hom_poses_file_path (str): Path for the homogeneous pose file.
    
    Returns:
        str, str: Updated paths for the pose files.
    """
    if os.path.exists(poses_file_path):
        user_action = input('Pose file already exists. Overwrite? (y/n): ').strip().lower()
        if user_action == 'y':
            os.remove(poses_file_path)
            os.remove(hom_poses_file_path) if os.path.exists(hom_poses_file_path) else None
        elif user_action == 'n':
            new_name = input('Enter a new name for the pose file: ').strip()
            poses_file_path = os.path.join(os.path.dirname(poses_file_path), f"{new_name}_6D.txt")
            hom_poses_file_path = os.path.join(os.path.dirname(hom_poses_file_path), f"{new_name}_hom.txt")
    return poses_file_path, hom_poses_file_path


def reset_device():
    """
    Resets the RealSense device hardware and ensures a fresh start.
    
    Returns:
        rs.pipeline: The configured RealSense pipeline object.
    """
    print("Resetting the device...")
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()
        time.sleep(4)
    print("Device reset complete.\n")
    
    # Initialize and configure RealSense streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    return pipeline, config


def validate_rgb_sensor(device):
    """
    Validates that the RealSense device has an RGB camera.

    Args:
        device (rs.device): RealSense device object.
    
    Returns:
        bool: True if RGB camera is found, False otherwise.
    """
    for sensor in device.sensors:
        if sensor.get_info(rs.camera_info.name) == 'RGB Camera':
            return True
    print("This script requires a Depth camera with an RGB sensor.")
    return False


def capture_images(pipeline, file_path, poses_file_path, hom_poses_file_path, max_images=40):
    """
    Captures images from the RealSense camera and logs robot poses.

    Args:
        pipeline (rs.pipeline): The RealSense pipeline for image capture.
        file_path (str): Directory path to save the captured images.
        poses_file_path (str): Path to save the 6D robot poses.
        hom_poses_file_path (str): Path to save the homogeneous robot poses.
        max_images (int): Maximum number of images to capture.
    
    Returns:
        None
    """
    counter = 0  # Initialize image counter

    # Add headers to the pose files
    with open(poses_file_path, "a") as pose_file:
        pose_file.write('# posx, posy, posz, angle1, angle2, angle3\n')
    with open(hom_poses_file_path, "a") as pose_file:
        pose_file.write('# R11, R12, R13, posx,R21, R22, R23, posy, R31, R32, R33, posz, 0,0,0,1\n')

    try:
        while counter < max_images:
            user_entry = input('Enter robot pose or type "quit" to finish: ').strip().lower()

            if user_entry == 'quit':
                break

            # Parse and validate the user input
            try:
                float_list = [float(value) for value in user_entry.split(',')]
                position_float = float_list[:3]
                angles_float = float_list[3:]
            except ValueError:
                print("Invalid input. Please enter valid numbers separated by commas.")
                continue

            # Convert angles to rotation matrix
            rotation_matrix = euler_to_rotation_matrix(angles_float, order='ZYX')

            # Prepare the homogeneous pose list
            robot_pose_hom = [
                rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2], position_float[0],
                rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2], position_float[1],
                rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2], position_float[2],
                0, 0, 0, 1
            ]

            # Write the robot pose to the text files
            with open(poses_file_path, "a") as pose_file:
                pose_file.write(user_entry + '\n')
            with open(hom_poses_file_path, "a") as pose_file:
                pose_file.write(str(robot_pose_hom) + '\n')

            # Capture and save image
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            image_name = f"{counter:02}.png"
            cv2.imwrite(os.path.join(file_path, image_name), color_image)
            counter += 1
            print(f"Saved image: {image_name}")

    finally:
        # Stop streaming when finished
        pipeline.stop()
        print("Image capture finished.")


if __name__ == "__main__":
    """
    Main execution block to capture images from a RealSense camera and log robot poses.
    """
    # Set calibration folder path and file names
    calib_path = '/home/samuel/Desktop/d435_calibration/'
    poses_txt_name = 'robot_poses'

    # Prepare directories and file paths
    image_folder = create_directories(calib_path)
    poses_file_path = os.path.join(calib_path, f"{poses_txt_name}_6D.txt")
    hom_poses_file_path = os.path.join(calib_path, f"{poses_txt_name}_hom.txt")
    poses_file_path, hom_poses_file_path = check_pose_files(poses_file_path, hom_poses_file_path)

    # Reset and configure the device
    pipeline, config = reset_device()
    device = config.resolve(rs.pipeline_wrapper(pipeline)).get_device()

    if validate_rgb_sensor(device):
        # Start capturing images
        pipeline.start(config)
        capture_images(pipeline, image_folder, poses_file_path, hom_poses_file_path)

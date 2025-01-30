import cv2
import os
import numpy as np
import argparse
import random

def create_paths(base_path, folder_name):
    """
    Generate file paths for RGB and depth data.
    """
    rgb_path = os.path.join(base_path, folder_name + '_data', 'rgb')
    depth_path = os.path.join(base_path, folder_name + '_data', 'depth')
    return rgb_path, depth_path

def load_depth_image(depth_image_path):
    """
    Load a depth image using OpenCV.
    """
    if not os.path.exists(depth_image_path):
        raise FileNotFoundError(f"Depth image file not found: {depth_image_path}")
    
    img = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError(f"Failed to load image. Check if the file is a valid image: {depth_image_path}")
    
    return img

def convert_to_meters(depth_image):
    """
    Convert depth image values to meters (assuming the image is in millimeters).
    """
    return np.array(depth_image, dtype=float) * 0.001

def get_random_depth_image(depth_path):
    """
    Select a random depth image from the given path.
    """
    if not os.path.exists(depth_path):
        raise FileNotFoundError(f"Depth directory not found: {depth_path}")
    
    images = [f for f in os.listdir(depth_path) if f.endswith('.png')]
    if not images:
        raise FileNotFoundError("No depth images found in the directory.")
    
    return random.choice(images)

def main():
    """
    Main function to manage loading and processing of depth images.
    """
    parser = argparse.ArgumentParser(description="Process a depth image and convert values to meters.")
    parser.add_argument("--input", required=True, help="Name of the ROS file (e.g., d435_test1)")
    args = parser.parse_args()
    
    # Base file path
    file_path = "/home/samuel/dev/environment_modeling/ROSBAGS"
    
    # Generate paths for RGB and Depth images
    rgb_path, depth_path = create_paths(file_path, args.input)
    
    try:
        depth_image_filename = get_random_depth_image(depth_path)
        depth_image_path = os.path.join(depth_path, depth_image_filename)
        
        depth_image = load_depth_image(depth_image_path)
        depth_in_meters = convert_to_meters(depth_image)
        
        print(f'Processing depth image: {depth_image_filename}')
        print('Depth values in meters:')
        print(depth_in_meters[1:100])
    
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

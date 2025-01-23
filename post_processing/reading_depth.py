import cv2
import os
import numpy as np

# the idea is read the depth values in mm

def create_paths(base_path, folder_name):
    """
    Generate file paths for RGB and depth data.

    Args:
        base_path (str): The base directory where the data folder is located.
        folder_name (str): The specific folder name where the RGB and depth data are stored.

    Returns:
        tuple: A tuple containing the RGB path and depth path strings.
    """
    # Construct paths for RGB and Depth data
    rgb_path = os.path.join(base_path, folder_name + '_data', 'rgb')
    depth_path = os.path.join(base_path, folder_name + '_data', 'depth')
    
    return rgb_path, depth_path

def load_depth_image(depth_image_path):
    """
    Load a depth image using OpenCV.

    Args:
        depth_image_path (str): The full file path of the depth image to load.

    Returns:
        np.ndarray: A NumPy array containing the depth image data.

    Raises:
        FileNotFoundError: If the image file is not found.
    """
    if not os.path.exists(depth_image_path):
        raise FileNotFoundError(f"Depth image file not found: {depth_image_path}")
    
    # Read the image in unchanged format (to preserve depth information)
    img = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    
    if img is None:
        raise ValueError(f"Failed to load image. Check if the file is a valid image: {depth_image_path}")
    
    return img

def convert_to_meters(depth_image):
    """
    Convert depth image values to meters (assuming the image is in millimeters).

    Args:
        depth_image (np.ndarray): The depth image array with values in millimeters.

    Returns:
        np.ndarray: A NumPy array with depth values converted to meters.
    """
    # Convert depth values to float and scale to meters
    float_array = np.array(depth_image, dtype=float) * 0.001
    return float_array

def main():
    """
    Main function to manage loading and processing of depth images.
    """
    # Base file path and folder names
    rosfile_name = 'd435_test1'
    #file_path = "/home/samuel/Desktop/"
    file_path = os.path.expandvars("$HOME//Desktop/")

    # Generate paths for RGB and Depth images
    rgb_path, depth_path = create_paths(file_path, rosfile_name)
    
    # Depth image filename (can be parameterized or looped over a set of images)
    depth_image_filename = '1728286188895709757.png'
    depth_image_path = os.path.join(depth_path, depth_image_filename)
    
    try:
        # Load the depth image
        depth_image = load_depth_image(depth_image_path)
        
        # Convert depth values from millimeters to meters
        depth_in_meters = convert_to_meters(depth_image)
        
        # Output depth values (in meters)
        print('Depth values in meters:')
        print(depth_in_meters)
    
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.image import image_to_cvimage
import cv2
import os
from collections import defaultdict

def create_output_directories(base_dir):
    """
    Creates directories for storing extracted RGB and depth images.
    
    Args:
        base_dir (str): Base directory where the 'rgb' and 'depth' folders will be created.
    
    Returns:
        tuple: Paths for RGB and depth image directories.
    """
    rgb_path = os.path.join(base_dir, 'rgb')
    depth_path = os.path.join(base_dir, 'depth')
    
    # Create the directories if they do not exist
    os.makedirs(rgb_path, exist_ok=True)
    os.makedirs(depth_path, exist_ok=True)
    
    return rgb_path, depth_path

def find_closest_timestamps(rgb_timestamps, depth_timestamps):
    """
    Finds the closest matching timestamps between RGB and depth images.
    
    Args:
        rgb_timestamps (list): List of timestamps for RGB images
        depth_timestamps (list): List of timestamps for depth images
    
    Returns:
        list: List of tuples containing matched (rgb_timestamp, depth_timestamp) pairs
    """
    associations = []
    for rgb_ts in rgb_timestamps:
        # Find the depth timestamp that's closest to the RGB timestamp
        closest_depth_ts = min(depth_timestamps, key=lambda x: abs(x - rgb_ts))
        associations.append((rgb_ts, closest_depth_ts))
    return associations

def extract_and_save_images(reader, typestore, rgb_path, depth_path, topics):
    """
    Reads a ROSBAG and extracts RGB and depth images, saving them to the specified directories.
    
    Args:
        reader (Reader): ROSBAG reader instance.
        typestore (TypeStore): ROSBAG TypeStore for deserializing messages.
        rgb_path (str): Path to save the extracted RGB images.
        depth_path (str): Path to save the extracted depth images.
        topics (dict): A dictionary containing the relevant topics for color and depth images.
    
    Returns:
        tuple: Lists of RGB and depth timestamps
    """
    msg_count = 0    
    rgb_timestamps = []
    depth_timestamps = []

    for connection, timestamp, rawdata in reader.messages():
        # Process color images
        if connection.topic == topics['color_images']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = image_to_cvimage(msg, 'bgr8')  # Convert to BGR color space
            img_name = f'{timestamp}.png'
            cv2.imwrite(os.path.join(rgb_path, img_name), img)
            rgb_timestamps.append(timestamp)
            print(f"Saved RGB image: {img_name}")
        
        # Process depth images
        elif connection.topic == topics['depth_images']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = image_to_cvimage(msg)  # Depth image does not need color conversion
            img_name = f'{timestamp}.png'
            cv2.imwrite(os.path.join(depth_path, img_name), img)
            depth_timestamps.append(timestamp)
            print(f"Saved Depth image: {img_name}")

        msg_count += 1

    return rgb_timestamps, depth_timestamps

def create_associations_file(output_dir, associations):
    """
    Creates an associations.txt file containing matched RGB and depth image timestamps.
    
    Args:
        output_dir (str): Directory where to save the associations file
        associations (list): List of tuples containing matched (rgb_timestamp, depth_timestamp) pairs
    """
    associations_path = os.path.join(output_dir, 'associations.txt')
    with open(associations_path, 'w') as f:
        for rgb_ts, depth_ts in associations:
            f.write(f"{rgb_ts} rgb/{rgb_ts}.png {depth_ts} depth/{depth_ts}.png\n")
    print(f"\nAssociations file created at: {associations_path}")

def list_topics(reader):
    """
    Lists the available topics in a ROSBAG file.
    
    Args:
        reader (Reader): ROSBAG reader instance.
    
    Returns:
        None
    """
    print("Available Topics in the ROSBAG file:")
    for i, connection in enumerate(reader.connections, start=1):
        print(f"Topic {i}: {connection.topic}, Message Type: {connection.msgtype}")

def main(rosbag_path, output_dir):
    """
    Main function to extract and save RGB and depth images from a ROSBAG file.
    
    Args:
        rosbag_path (str): Path to the ROSBAG file.
        output_dir (str): Destination path where extracted images will be saved.
    
    Returns:
        None
    """
    # Topics to extract
    topics = {
        'color_images': '/camera/camera/color/image_raw',
        'depth_images': '/camera/camera/aligned_depth_to_color/image_raw'
    }

    # Create typestore for message deserialization
    typestore = get_typestore(Stores.LATEST)
    
    # Create output directories for RGB and depth images
    rgb_path, depth_path = create_output_directories(output_dir)
    
    try:
        # Open the ROSBAG file using a Reader
        with Reader(rosbag_path) as reader:
            # List all topics in the bag file
            list_topics(reader)
            
            print("\nReading and extracting images from the ROSBAG...\n")
            
            # Extract and save images, getting timestamps
            rgb_timestamps, depth_timestamps = extract_and_save_images(
                reader, typestore, rgb_path, depth_path, topics)
            
            # Find closest matching timestamps and create associations file
            associations = find_closest_timestamps(rgb_timestamps, depth_timestamps)
            create_associations_file(output_dir, associations)
            
            print(f"\nProcessing completed. Total RGB images: {len(rgb_timestamps)}")
            print(f"Total depth images: {len(depth_timestamps)}")

    except FileNotFoundError:
        print(f"Error: The ROSBAG file at {rosbag_path} was not found.")
    except Exception as e:
        print(f"An unexpected error occurred: {str(e)}")

if __name__ == "__main__":
    # Define the path to the ROSBAG file and output directory
    rosbag_path = "/home/samuel/Desktop/samuel_test2"
    output_dir = "/home/samuel/Desktop/samuel_test2_data"
    
    # Call the main function
    main(rosbag_path, output_dir)
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.image import image_to_cvimage
import cv2
import os

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
        int: The total number of processed messages.
    """
    msg_count = 0    

    for connection, timestamp, rawdata in reader.messages():
        # Process color images
        if connection.topic == topics['color_images']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = image_to_cvimage(msg, 'bgr8')  # Convert to BGR color space
            img_name = f'{timestamp}.png'
            cv2.imwrite(os.path.join(rgb_path, img_name), img)
            print(f"Saved RGB image: {img_name}")
        
        # Process depth images
        elif connection.topic == topics['depth_images']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = image_to_cvimage(msg)  # Depth image does not need color conversion
            img_name = f'{timestamp}.png'
            cv2.imwrite(os.path.join(depth_path, img_name), img)
            print(f"Saved Depth image: {img_name}")

        msg_count += 1

    return msg_count

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
            
            # Extract and save images
            total_messages = extract_and_save_images(reader, typestore, rgb_path, depth_path, topics)
            
            print(f"\nProcessing completed. Total messages processed: {total_messages}")

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

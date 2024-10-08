from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.image import image_to_cvimage
import cv2
import os

# --------------------- Folders Management -------------------------
rosfile_path = "/home/samuel/Desktop/d435_test1"  # Path to the ROSBAG file
rosfile_data_destiny = '/home/samuel/Desktop/d435_test2_data'  # Destination path for extracted images

# Define paths for RGB and depth images
rgb_path = os.path.join(rosfile_data_destiny, 'rgb')
depth_path = os.path.join(rosfile_data_destiny, 'depth')

# Create directories if they do not exist
if not os.path.exists(rgb_path):
    os.makedirs(rgb_path)
if not os.path.exists(depth_path):
    os.makedirs(depth_path)

# ----------------- Subscribe to These Topics --------------------
search_topic = '/camera/camera/color/camera_info'
color_images_topic = '/camera/camera/color/image_raw'
depth_images_topic = '/camera/camera/aligned_depth_to_color/image_raw'
info_depth_topic = '/camera/camera/depth/camera_info'

# Create a typestore to deserialize messages
typestore = get_typestore(Stores.LATEST)

if __name__ == "__main__":
    """
    Main execution block to read a ROSBAG file and extract RGB and depth images.
    It saves the images in the specified destination folders.
    """
    
    # Create a reader instance to read from the ROSBAG file
    with Reader(rosfile_path) as reader:
        # Output the available topics and their message types
        counter = 1
        for connection in reader.connections:
            print("Topic N°", counter, "--> Name:", connection.topic, "msgtype:", connection.msgtype)
            counter += 1
        
        print('The ROSBAG file has', counter, 'topics.\n')
        print('Reading and saving the images...\n')

        counter = 0  # Reset counter for processed messages

        # Iterate over messages in the ROSBAG
        for connection, timestamp, rawdata in reader.messages():
            # --------- Process Color Images ----------------
            if connection.topic == color_images_topic:
                # Deserialize the message to get image data
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                img = image_to_cvimage(msg)  # Get image in the original color space
                img = image_to_cvimage(msg, 'bgr8')  # Convert image to BGR color space
                img_name = str(timestamp) + '.png'  # Name the image file based on timestamp
                cv2.imwrite(os.path.join(rgb_path, img_name), img)  # Save the image

            # ----------- Process Depth Images ---------------
            if connection.topic == depth_images_topic:
                # Deserialize the message to get depth image data
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                img = image_to_cvimage(msg)  # Get image in the original color space
                img_name = str(timestamp) + '.png'  # Name the depth image file
                cv2.imwrite(os.path.join(depth_path, img_name), img)  # Save the depth image

            # ----------- Process Info Depth Images ---------------
            if connection.topic == info_depth_topic:
                # Deserialize the message to get depth camera info
                info_msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            counter += 1  # Increment message counter

        # Output the total number of processed messages
        print("Work done. Total messages: ", counter)

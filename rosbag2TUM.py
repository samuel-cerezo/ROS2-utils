from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.image import image_to_cvimage
import cv2
import os

# --------------------- folders managing -------------------------
rosfile_path = "/home/samuel/Desktop/d435_test1"
rosfile_data_destiny = '/home/samuel/Desktop/d435_test2_data'

rgb_path = rosfile_data_destiny + '/rgb'
depth_path = rosfile_data_destiny + '/depth'
if not os.path.exists(rgb_path):
    os.makedirs(rgb_path)
if not os.path.exists(depth_path):
    os.makedirs(depth_path)

# ----------------- subscribe to these topics --------------------
search_topic= '/camera/camera/color/camera_info'
color_images_topic= '/camera/camera/color/image_raw'
depth_images_topic = '/camera/camera/aligned_depth_to_color/image_raw'
info_depth_topic = '/camera/camera/depth/camera_info'

# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)

if __name__ == "__main__":

    # Create reader instance and open for reading.
    with Reader(rosfile_path) as reader:
        # Topic and msgtype information is available on .connections list.
        counter = 1
        for connection in reader.connections:
            print("Topic N°", counter, "--> Name:",connection.topic, "msgtype:",connection.msgtype)
            counter+=1
        print('The ROSBAG file has', counter, 'topics.\n')
        print('Reading and saving the images...\n')
        counter = 0
        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            # --------- color images ----------------
            if connection.topic == color_images_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                img = image_to_cvimage(msg) # get image in source color space
                img = image_to_cvimage(msg, 'bgr8') # get image and convert to specific color space
                img_name = str(timestamp) + '.png'
                cv2.imwrite(os.path.join(rgb_path , img_name), img)

            # ----------- depth images ---------------
            if connection.topic == depth_images_topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                img = image_to_cvimage(msg) # get image in source color space
                #img = image_to_cvimage(msg, 'bgr8') # get image and convert to specific color space
                img_name = str(timestamp) + '.png'
                cv2.imwrite(os.path.join(depth_path , img_name), img)  

            # ----------- info depth images ---------------
            if connection.topic == info_depth_topic:
                info_msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            counter+=1

        #print("Message depth:",info_msg,"Timestamp:",timestamp)    
        print("Work done. Total messages: ",counter)
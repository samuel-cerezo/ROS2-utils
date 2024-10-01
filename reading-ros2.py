from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.image import image_to_cvimage
import cv2
import os


#rosfile_path = "/home/samuel/Desktop/d435/rosbag-01-oct"
#search_topic = '/chatter'


rosfile_path = "/home/samuel/Desktop/pics_samuel"

images_path = '/home/samuel/Desktop/d435/color-images'

search_topic = '/camera/camera/color/camera_info'
search_topic = '/camera/camera/color/image_raw'

# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)

# Create reader instance and open for reading.
with Reader(rosfile_path) as reader:
    # Topic and msgtype information is available on .connections list.
    counter = 1
    for connection in reader.connections:
        print("Connection N°", counter, "--> Topic:",connection.topic, "msgtype:",connection.msgtype)
        counter+=1
    
    counter = 0
    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == search_topic:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = image_to_cvimage(msg) # get image in source color space
            img = image_to_cvimage(msg, 'bgr8') # get image and convert to specific color space
            img_name = str(timestamp) + '.jpg'
            cv2.imwrite(os.path.join(images_path , img_name), img, [cv2.IMWRITE_JPEG_QUALITY, 90])   
        counter+=1


    '''
    # ----------------------- Displays image inside a window
    cv2.imshow('color image',img)  
    # Waits for a keystroke
    cv2.waitKey(0)  
    # Destroys all the windows created
    cv2.destroyAllwindows()
    # ---------------------------------------------------------
    '''
    print("Message:",msg,"Timestamp:",timestamp)    
    print("Total messages: ",counter)

'''
    # The .messages() method accepts connection filters.
    connections = [x for x in reader.connections if x.topic == '/imu_raw/Imu']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        print(msg.header.frame_id)
'''





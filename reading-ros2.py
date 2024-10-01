from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


#rosfile_path = "/home/samuel/Desktop/d435/rosbag-01-oct"
#search_topic = '/chatter'


rosfile_path = "/home/samuel/Desktop/pics_samuel"
search_topic = '/camera/camera/color/camera_info'


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
        counter+=1
    print("Message:",msg,"Timestamp:",timestamp)    
    print("Total messages: ",counter)

'''
    # The .messages() method accepts connection filters.
    connections = [x for x in reader.connections if x.topic == '/imu_raw/Imu']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        print(msg.header.frame_id)
'''

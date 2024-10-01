from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)

# Create reader instance and open for reading.
with Reader('/home/samuel/Desktop/d435/rosbag-01-oct') as reader:
    # Topic and msgtype information is available on .connections list.
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/imu_raw/Imu':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            print(msg.header.frame_id)

    # The .messages() method accepts connection filters.
    connections = [x for x in reader.connections if x.topic == '/imu_raw/Imu']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        print(msg.header.frame_id)
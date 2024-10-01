from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)

# Create reader instance and open for reading.
with Reader('/home/samuel/Desktop/d435/rosbag-01-oct') as reader:
    # Topic and msgtype information is available on .connections list.
    for connection in reader.connections:
        print("Conection: ", connection.topic, connection.msgtype)

    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        #print(msg.header.frame_id)
        print(msg.data)

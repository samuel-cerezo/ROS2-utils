from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)
String = typestore.types['std_msgs/msg/String']

# Create writer instance and open for writing.
with Writer('/home/samuel/Desktop/d435/rosbag-01-oct') as writer:

    # Add new connection.
    topic = '/chatter'
    msgtype = String.__msgtype__
    connection = writer.add_connection(topic, msgtype, typestore=typestore)

    # Serialize and write message.
    timestamp = 42
    message = String('hello world')
    writer.write(connection, timestamp, typestore.serialize_cdr(message, msgtype))
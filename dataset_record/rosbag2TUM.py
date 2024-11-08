import cv2
import os
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg, register_types, get_types_from_idl
from rosbags.typesys.stores.ros2_humble import *
from rosbags.image import image_to_cvimage
from collections import defaultdict
from pathlib import Path

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

# Function necessary for reading rgbd messages
def load_custom_types(path_to_RGBDmsg):

    typestore = get_typestore(Stores.LATEST)
    add_types = {}

    for pathstr in [path_to_RGBDmsg]:
        msgpath = Path(pathstr)
        msgdef = msgpath.read_text(encoding='utf-8')
        add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

    typestore.register(add_types)
    return typestore

# Function to create output directories
def create_output_directories(base_dir, available_topics, topics):
    rgb_path = os.path.join(base_dir, 'rgb')
    depth_path = os.path.join(base_dir, 'depth')
    joint_data_path = os.path.join(base_dir, 'joint_data')

    if (topics['color_images'] in available_topics) or (topics['rgbd'] in available_topics):
        os.makedirs(rgb_path, exist_ok=True)
    

    if (topics['depth_images'] in available_topics) or (topics['rgbd'] in available_topics):
        os.makedirs(depth_path, exist_ok=True)

    #if topics['joint_states'] in available_topics:
    os.makedirs(joint_data_path, exist_ok=True)
    
    imu_data_path = base_dir

    return rgb_path, depth_path, joint_data_path, imu_data_path

# Function to create joint data files
def create_joint_data_files(joint_data_path):
    position_file = os.path.join(joint_data_path, 'joint_positions.txt')
    velocity_file = os.path.join(joint_data_path, 'joint_velocities.txt')
    effort_file = os.path.join(joint_data_path, 'joint_efforts.txt')
    
    header = "#timestamp A1 A2 A3 A4 A5 A6\n"
    for file_path in [position_file, velocity_file, effort_file]:
        with open(file_path, 'w') as f:
            f.write(header)
    
    return position_file, velocity_file, effort_file

# Function to create IMU data file
def create_imu_data_file(joint_data_path):
    imu_file = os.path.join(joint_data_path, 'imu_data.txt')
    
    imu_header = "#timestamp accel_x(m/s^2) accel_y(m/s^2) accel_z(m/s^2) gyro_x(rad/s) gyro_y(rad/s) gyro_z(rad/s)\n"
    with open(imu_file, 'w') as f:
        f.write(imu_header)
    
    return imu_file

# Function to write joint data
def write_joint_data(file_path, timestamp, data):
    with open(file_path, 'a') as f:
        data_str = ' '.join(map(str, data))
        f.write(f"{timestamp} {data_str}\n")

# Function to write IMU data
def write_imu_data(imu_file, timestamp, accel_data, gyro_data):
    accel_str = ' '.join(map(str, accel_data))
    gyro_str = ' '.join(map(str, gyro_data))
    with open(imu_file, 'a') as f:
        f.write(f"{timestamp} {accel_str} {gyro_str}\n")

# Function to find closest timestamps
def find_closest_timestamps(rgb_timestamps, depth_timestamps):
    associations = []
    for rgb_ts in rgb_timestamps:
        closest_depth_ts = min(depth_timestamps, key=lambda x: abs(x - rgb_ts))
        associations.append((rgb_ts, closest_depth_ts))
    return associations

# Function to extract and save data from the ROSBAG
def extract_and_save_data(reader, typestore, rgb_path, depth_path, joint_data_path, imu_data_path, available_topics, topics):
    msg_count = 0    
    rgb_timestamps = []
    depth_timestamps = []

    # Create joint and IMU data files
    position_file, velocity_file, effort_file = create_joint_data_files(joint_data_path)
    imu_file = create_imu_data_file(imu_data_path)

    for connection, timestamp, rawdata in reader.messages():

        timestamp = timestamp/1e9   # convert the time: nanosec -> sec

        # Save color image
        if connection.topic == topics['color_images']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = image_to_cvimage(msg, 'bgr8')
            img_name = f'{timestamp}.png'
            cv2.imwrite(os.path.join(rgb_path, img_name), img)
            rgb_timestamps.append(timestamp)
            print(f"Saved RGB image: {img_name}")

        # Save depth image
        if connection.topic == topics['depth_images']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = image_to_cvimage(msg)
            img_name = f'{timestamp}.png'
            cv2.imwrite(os.path.join(depth_path, img_name), img)
            depth_timestamps.append(timestamp)
            print(f"Saved Depth image: {img_name}")

        # Save RGBD image
        if connection.topic == topics['rgbd']:
            '''
            message Info: 

             rgb=sensor_msgs__msg__Image(header=std_msgs__msg__Header(stamp=builtin_interfaces__msg__Time(sec=1728571428, nanosec=866459717, __msgtype__='builtin_interfaces/msg/Time'), frame_id='camera_color_optical_frame', __msgtype__='std_msgs/msg/Header'), height=480, width=848, encoding='rgb8', is_bigendian=0, step=2544, data=array([162, 170, 169, ...,  45,  73, 108], dtype=uint8), __msgtype__='sensor_msgs/msg/Image'), 
             depth=sensor_msgs__msg__Image(header=std_msgs__msg__Header(stamp=builtin_interfaces__msg__Time(sec=1728571428, nanosec=866459717, __msgtype__='builtin_interfaces/msg/Time'), frame_id='camera_depth_optical_frame', __msgtype__='std_msgs/msg/Header'), height=480, width=848, encoding='16UC1', is_bigendian=0, step=1696, data=array([168,   8, 168, ...,   3, 182,   3], dtype=uint8), __msgtype__='sensor_msgs/msg/Image'),
             
             __msgtype__='realsense2_camera_msgs/msg/RGBD')
            '''
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            # Extract the RGB and depth image data
            rgb_image_msg = msg.rgb  # The RGB image data
            depth_image_msg = msg.depth  # The depth image data
            # rgb image
            rgb_image_cv = image_to_cvimage(rgb_image_msg, 'bgr8')
            rgb_img_name = f'{timestamp:.9f}.png'
            cv2.imwrite(os.path.join(rgb_path, rgb_img_name), rgb_image_cv)
            print(f"Saved RGB image: {rgb_img_name}")
            rgb_timestamps.append(timestamp)

            # depth image
            depth_img_cv = image_to_cvimage(depth_image_msg)
            depth_img_name = f'{timestamp:.9f}.png'
            cv2.imwrite(os.path.join(depth_path, depth_img_name), depth_img_cv)
            print(f"Saved Depth image: {depth_img_name}")
            depth_timestamps.append(timestamp)

            
        # Save joint states
        if connection.topic == topics['joint_states']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            if hasattr(msg, 'position') and len(msg.position) > 0:
                write_joint_data(position_file, timestamp, msg.position)
                print(f"Saved joint positions at timestamp: {timestamp}")
            if hasattr(msg, 'velocity') and len(msg.velocity) > 0:
                write_joint_data(velocity_file, timestamp, msg.velocity)
                print(f"Saved joint velocities at timestamp: {timestamp}")
            if hasattr(msg, 'effort') and len(msg.effort) > 0:
                write_joint_data(effort_file, timestamp, msg.effort)
                print(f"Saved joint efforts at timestamp: {timestamp}")

        # Save IMU data
        if connection.topic == topics['imu']:
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            if hasattr(msg, 'linear_acceleration') and hasattr(msg, 'angular_velocity'):
                accel_data = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
                gyro_data = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
                write_imu_data(imu_file, timestamp, accel_data, gyro_data)
                print(f"Saved IMU data at timestamp: {timestamp:.9f}")

        msg_count += 1

    return rgb_timestamps, depth_timestamps

# Function to create associations file
def create_associations_file(output_dir, associations):
    associations_path = os.path.join(output_dir, 'associations.txt')
    assoc_header = "#rgb_timestamp rgb_file depth_timestamp depth_file\n"
    with open(associations_path, 'w') as f:
        f.write(assoc_header)
        for rgb_ts, depth_ts in associations:
            # Formatear los timestamps con precisión de 9 decimales
            rgb_ts_sec = f"{rgb_ts:.9f}"
            depth_ts_sec = f"{depth_ts:.9f}"
            f.write(f"{rgb_ts_sec} rgb/{rgb_ts_sec}.png {depth_ts_sec} depth/{depth_ts_sec}.png\n")
    print(f"\nAssociations file created at: {associations_path}")
    '''
    with open(associations_path, 'w') as f:
        f.write(assoc_header)
        for rgb_ts, depth_ts in associations:
            f.write(f"{rgb_ts} rgb/{rgb_ts}.png {depth_ts} depth/{depth_ts}.png\n")
    print(f"\nAssociations file created at: {associations_path}")
    '''

# Function to list available topics in ROSBAG
def list_topics(reader):
    available_topics = []
    print("Available Topics in the ROSBAG file:")
    for i, connection in enumerate(reader.connections, start=1):
        print(f"Topic {i}: {connection.topic}, Message Type: {connection.msgtype}")
        available_topics.append(connection.topic)
    return available_topics

# Main function
def main(rosbag_path, output_dir):

    topics = {
        'color_images': ' ',        #need to be complete, for example... 'color_images': '/camera/camera/color/image_raw',
        'rgbd':'/camera/camera/rgbd',   # delete if it is not needed
        'depth_images': ' ',        # need to be complete, for example... '/camera/camera/aligned_depth_to_color/image_raw'
        'joint_states': '/joint_states', 
        'imu': '/camera/camera/imu'  # IMU topic corrected to the proper path
    }

    # this is for RGBD topic
    # path_to_RGBDmsg = '/home/samuel/dev/ros2_ws/install/realsense2_camera_msgs/share/realsense2_camera_msgs/msg/RGBD.msg'
    path_to_RGBDmsg = '/home/samuel/dev/ros2_ws/src/realsense-ros/realsense2_camera_msgs/msg/RGBD.msg'
    path_to_RGBDmsg = '/home/samuel/ros2_realsenseWS/install/realsense2_camera_msgs/share/realsense2_camera_msgs/msg/RGBD.msg'
    typestore = load_custom_types(path_to_RGBDmsg)  # Load RGBD message types

    try:
        with Reader(rosbag_path) as reader:
            available_topics = list_topics(reader)
            print(available_topics)
                
            print("\nReading and extracting data from the ROSBAG...\n")
            rgb_path, depth_path, joint_data_path, imu_data_path = create_output_directories(output_dir, available_topics, topics)
            rgb_timestamps, depth_timestamps = extract_and_save_data(reader, typestore, rgb_path, depth_path, joint_data_path, imu_data_path, available_topics, topics)
                
            if len(rgb_timestamps) > 0:
                associations = find_closest_timestamps(rgb_timestamps, depth_timestamps)
                create_associations_file(output_dir, associations)

            print(f"\nProcessing completed. Total RGB images: {len(rgb_timestamps)}. Total depth images: {len(depth_timestamps)}")

    except FileNotFoundError:
        print(f"Error: The ROSBAG file at {rosbag_path} was not found.")
    except Exception as e:
        print(f"An unexpected error occurred: {str(e)}")

if __name__ == "__main__":
    rosbag_path = "/home/samuel/dev/environment_modeling/ROSBAGS/jarvis_mockup_3"
    output_dir = "/home/samuel/dev/environment_modeling/ROSBAGS/jarvis_mockup_3_data"
    main(rosbag_path, output_dir)

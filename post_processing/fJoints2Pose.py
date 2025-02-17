import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import numpy as np
import os
import argparse

class RobotPosePublisher(Node):
    def __init__(self, joint_positions_path, pose_file_path):
        super().__init__('robot_pose_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Paths for joint positions file and pose output
        self.joint_positions_path = joint_positions_path
        self.pose_file_path = pose_file_path

        # Check if the pose file already exists. If it does, remove it.
        if os.path.exists(self.pose_file_path):
            self.get_logger().info(f"Pose file {self.pose_file_path} exists. Replacing it.")
            os.remove(self.pose_file_path)

        # Open files for reading joint data and saving pose data
        self.joint_file = open(self.joint_positions_path, 'r')
        self.pose_file = open(self.pose_file_path, 'a')

        # Read the header to get the joint names and order
        self.joint_header = self.read_joint_header()

        # Add header to pose file only once
        self.header_written = False

        # Publisher for joint commands
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_trajectory_controller/commands',
            10
        )

        # Read all joint data lines at the start
        self.joint_data_lines = self.joint_file.readlines()
        self.joint_index = 0

        # initialize the timestamp for the last command
        self.last_command_time = None
        
        # Timer to run periodically
        #self.timer = self.create_timer(0.004, self.publish_pose_from_joint_data)        # 4ms between commands
        self.timer = self.create_timer(0.05, self.publish_pose_from_joint_data)        # 100ms between commands


    def read_joint_header(self):
        header_line = self.joint_file.readline().strip()
        header_values = header_line.split()
        if header_values[0] == "#timestamp":
            joint_names = header_values[1:]
            self.get_logger().info(f"Detected joint header: {joint_names}")
            return joint_names
        else:
            raise ValueError("Invalid header format. The header should start with '#timestamp'.")

    def publish_pose_from_joint_data(self):
        current_time = self.get_clock().now()
        #if self.last_command_time is not None:
        #    time_diff = (current_time - self.last_command_time).nanoseconds / 1e6
        #    self.get_logger().info(f"Time since last command: {time_diff:.3f} ms")
        self.last_command_time = current_time
        
        if self.joint_index < len(self.joint_data_lines):
            joint_line = self.joint_data_lines[self.joint_index].strip()
            if joint_line.startswith('#'):
                self.joint_index += 1
                return

            joint_values = joint_line.split()
            timestamp = float(joint_values[0])
            joint_angles = list(map(float, joint_values[1:]))

            joint_log = ", ".join([f"{joint_name}: {angle}" for joint_name, angle in zip(self.joint_header, joint_angles)])
            self.get_logger().info(f"Publishing joint values at timestamp {timestamp:.9f}: {joint_log}")

            ordered_joint_angles = self.reorder_joint_angles(joint_angles)
            self.publish_joint_commands(ordered_joint_angles)
            self.get_logger().info(f"Retrieving pose at timestamp {timestamp:.9f}")
            self.query_pose(timestamp)
            self.joint_index += 1
        else:
            self.get_logger().info("All joint data processed. Shutting down.")
            rclpy.shutdown()

    def reorder_joint_angles(self, joint_angles):
        #joint_order = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
        joint_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_dict = dict(zip(self.joint_header, joint_angles))
        ordered_angles = [joint_dict[joint] for joint in joint_order]
        self.get_logger().info(f"Ordered joint angles: {ordered_angles}")
        return ordered_angles

    def publish_joint_commands(self, joint_angles):
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published joint command: {msg.data}")

    def query_pose(self, timestamp):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            rotation = trans.transform.rotation
            translation = trans.transform.translation
            pose_data = f"{timestamp:.9f} {rotation.x} {rotation.y} {rotation.z} {rotation.w} "
            pose_data += f"{1000*translation.x} {1000*translation.y} {1000*translation.z}\n"

            if not self.header_written:
                self.pose_file.write("#timestamp qx qy qz qw x y z[mm]\n")
                self.header_written = True
            self.pose_file.write(pose_data)
            self.pose_file.flush()

            self.get_logger().info(f"Pose of tool0 w.r.t base_link:")
            self.get_logger().info(f"Translation: ({translation.x}, {translation.y}, {translation.z})")
            self.get_logger().info(f"Quaternion: ({rotation.x}, {rotation.y}, {rotation.z}, {rotation.w})")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Error getting transform: {e}")

    def __del__(self):
        self.joint_file.close()
        self.pose_file.close()


def main(args=None):
    parser = argparse.ArgumentParser(description='Robot Pose Publisher')
    parser.add_argument('--input', type=str, required=True, help='Folder containing joint data')
    args = parser.parse_args()
    
    rclpy.init(args=None)
    dataset_path = os.path.join('/home/samuel/dev/environment_modeling/ROSBAGS', args.input)
    joint_positions_path = os.path.join(dataset_path, 'joint_data/joint_positions.txt')
    pose_file_path = os.path.join(dataset_path, 'flange_poses.txt')
    
    robot_pose_publisher = RobotPosePublisher(joint_positions_path, pose_file_path)
    rclpy.spin(robot_pose_publisher)
    robot_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

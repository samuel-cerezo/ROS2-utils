import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time
import numpy as np

class RobotPosePublisher(Node):
    def __init__(self, joint_positions_path, pose_file_path):
        super().__init__('robot_pose_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Paths for joint positions file and pose output
        self.joint_positions_path = joint_positions_path
        self.pose_file_path = pose_file_path

        # Open files for reading joint data and saving pose data
        self.joint_file = open(self.joint_positions_path, 'r')
        self.pose_file = open(self.pose_file_path, 'a')

        # Add header to pose file only once
        self.header_written = False

        # Publisher for joint commands
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_trajectory_controller/commands',
            10
        )

        # Publish and record poses at regular intervals
        self.timer = self.create_timer(0.1, self.publish_pose_from_joint_data)

        # Read all joint data lines at the start
        self.joint_data_lines = self.joint_file.readlines()
        self.joint_index = 0

    def publish_pose_from_joint_data(self):
        if self.joint_index < len(self.joint_data_lines):
            joint_line = self.joint_data_lines[self.joint_index].strip()
            # Skip comments (lines starting with '#')
            if joint_line.startswith('#'):
                self.joint_index += 1
                return
            
            joint_values = joint_line.split()
            timestamp = float(joint_values[0])  # First value is the timestamp
            joint_angles = list(map(float, joint_values[1:]))  # The rest are joint angles

            # Log joint values and timestamp
            self.get_logger().info(f"Publishing joint values: {joint_angles} at timestamp {timestamp}")

            # Publish the joint values to control the robot's motion
            self.publish_joint_commands(joint_angles)

            # Get the transformation matrix for this timestamp using tool0 and base_link
            self.get_logger().info(f"Retrieving pose at timestamp {timestamp}")
            self.query_pose(timestamp)

            self.joint_index += 1  # Move to the next joint data line
        else:
            # Once all joint data has been processed, shutdown the node
            self.get_logger().info("All joint data processed. Shutting down.")
            rclpy.shutdown()

    def publish_joint_commands(self, joint_angles):
        """Publishes joint commands to the robot to move it."""
        msg = Float64MultiArray()
        
        # You can modify this to publish your desired joint values. For now, we publish the angles read from the file.
        msg.data = joint_angles

        # Publish the joint command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published joint command: {msg.data}")

    def query_pose(self, timestamp):
        try:
            # Get the current transform between base_link and tool0
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())

            # Extract rotation (quaternion) and translation (vector)
            rotation = trans.transform.rotation
            translation = trans.transform.translation

            # Convert quaternion to a rotation matrix manually
            rotation_matrix = self.quaternion_to_matrix(rotation)

            # Format the pose data as translation + quaternion
            pose_data = f"{timestamp:.9f} "
            pose_data += f"{translation.x} {translation.y} {translation.z} "  # Translation vector (t)
            pose_data += f"{rotation.x} {rotation.y} {rotation.z} {rotation.w}\n"  # Quaternion

            # Write the header only once at the beginning
            if not self.header_written:
                self.pose_file.write("# timestamp translation_x translation_y translation_z quaternion_x quaternion_y quaternion_z quaternion_w\n")
                self.header_written = True

            # Write the pose data to the file
            self.pose_file.write(pose_data)
            self.pose_file.flush()  # Ensure the data is written immediately

            # Log the pose (Translation + Quaternion)
            self.get_logger().info(f"Pose of tool0 w.r.t base_link:")
            self.get_logger().info(f"Translation: ({translation.x}, {translation.y}, {translation.z})")
            self.get_logger().info(f"Quaternion: ({rotation.x}, {rotation.y}, {rotation.z}, {rotation.w})")

            # Log the rotation matrix
            self.get_logger().info(f"Rotation Matrix (3x3): \n{rotation_matrix}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Error getting transform: {e}")

    def quaternion_to_matrix(self, quat):
        """Convert a quaternion to a 3x3 rotation matrix."""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w

        # Calculate the rotation matrix from the quaternion
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])

        return R

    def __del__(self):
        # Close the files when the node is destroyed
        self.joint_file.close()
        self.pose_file.close()

def main(args=None):
    rclpy.init(args=args)

    # Define the paths for joint positions and pose data
    joint_positions_path = '/home/samuel/dev/environment_modeling/ROSBAGS/iisy_random_motion_data/joint_data/joint_positions.txt'
    pose_file_path = '/home/samuel/dev/environment_modeling/ROSBAGS/iisy_random_motion_data/pose_data.txt'

    # Initialize the RobotPosePublisher with the file paths
    robot_pose_publisher = RobotPosePublisher(joint_positions_path, pose_file_path)

    rclpy.spin(robot_pose_publisher)
    robot_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

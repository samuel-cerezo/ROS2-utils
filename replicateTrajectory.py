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
    def __init__(self, joint_positions_path):
        super().__init__('robot_pose_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Paths for joint positions file and pose output
        self.joint_positions_path = joint_positions_path

        # Open files for reading joint data and saving pose data
        self.joint_file = open(self.joint_positions_path, 'r')

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

        self.count = 0

        # Read all joint data lines at the start
        self.joint_data_lines = self.joint_file.readlines()
        self.joint_index = 0

        # Timer to run periodically
        self.timer = self.create_timer(0.004, self.publish_pose_from_joint_data)

    def read_joint_header(self):
        """Reads the joint header to determine the joint names and order."""
        header_line = self.joint_file.readline().strip()  # Read the first line (header)
        
        # Split the line and remove the timestamp
        header_values = header_line.split()
        if header_values[0] == "#timestamp":
            joint_names = header_values[1:]  # Everything after '#timestamp' is the joint names
            return joint_names
        else:
            raise ValueError("Invalid header format. The header should start with '#timestamp'.")

    def stop_execution(self):
        """Detiene el nodo de manera explícita."""
        self.get_logger().info("Deteniendo ejecución del nodo...")
        self.timer.cancel()  # Cancelar el temporizador si sigue activo
        self.destroy_node()  # Destruir el nodo
        rclpy.shutdown()  # Finalizar rclpy
    
    def publish_pose_from_joint_data(self):
        if self.joint_index < len(self.joint_data_lines):
            joint_line = self.joint_data_lines[self.joint_index].strip()
            
            # Skip comments (lines starting with '#')
            if joint_line.startswith('#'):
                self.joint_index += 1
                return

            joint_values = joint_line.split()
            joint_angles = list(map(float, joint_values[1:]))  # The rest are joint angles
            # Publish the joint values to control the robot's motion
            self.publish_joint_commands(joint_angles)

            if self.count<3000:
                # Increment joint index to move to the next set of joint values
                self.joint_index = 1
                #print(self.count)
                self.count +=1
            else:
                self.joint_index += 1

        else:
            # Once all joint data has been processed, shutdown the node
            self.get_logger().info("All joint data processed. Shutting down.")
            self.stop_execution()

    def reorder_joint_angles(self, joint_angles):
        """Reorders the joint angles to match the order joint1, joint2, ..., joint6."""
        joint_order = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']

        # Create a dictionary with joint names as keys and their respective angles as values
        joint_dict = dict(zip(self.joint_header, joint_angles))

        # Reorder the joint angles based on the correct order
        ordered_angles = [joint_dict[joint] for joint in joint_order]

        self.get_logger().info(f"Ordered joint angles: {ordered_angles}")
        return ordered_angles

    def publish_joint_commands(self, joint_angles):
        """Publishes joint commands to the robot to move it."""
        msg = Float64MultiArray()
        
        # Publish the joint angles in the correct order
        msg.data = joint_angles

        # Publish the joint command
        self.command_publisher.publish(msg)
        print(msg.data)

    def __del__(self):
        # Close the files when the node is destroyed
        self.joint_file.close()


def main(args=None):
    rclpy.init(args=args)

    # Parse the command line arguments
    parser = argparse.ArgumentParser(description="Robot Pose Publisher")
    parser.add_argument('dataset_name', type=str, help="The name of the dataset folder")
    args = parser.parse_args()

    # Define the paths for joint positions and pose data
    #rosbag_folder = '/home/samuel/dev/environment_modeling/ROSBAGS/'
    rosbag_folder = os.path.expandvars("$HOME/dev/environment_modeling/ROSBAGS/")

    dataset_path = os.path.join(rosbag_folder, args.dataset_name)
    joint_positions_path = os.path.join(dataset_path, 'joint_data', 'joint_positions.txt')

    # Initialize the RobotPosePublisher
    robot_pose_publisher = RobotPosePublisher(joint_positions_path)
    try:
        rclpy.spin(robot_pose_publisher)  # Ejecutar el nodo
    except KeyboardInterrupt:
        robot_pose_publisher.get_logger().info("Interrupción manual. Cerrando nodo.")
    finally:
        # Asegurarnos de que se cierre correctamente
        print("Cerrando nodo...")
        robot_pose_publisher.stop_execution()  # Llamamos directamente a la detención
        print("Nodo cerrado correctamente.")

if __name__ == '__main__':
    main()
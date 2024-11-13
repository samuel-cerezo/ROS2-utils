import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class DriverTestFloatArray(Node):
    def __init__(self, filepath):
        super().__init__('driver_test_float_array')
        self.dt = 50  # Time interval in milliseconds
        self.command_publisher = self.create_publisher(Float64MultiArray, '/joint_trajectory_controller/commands', 10)

        # Load joint position data from the file
        self.joint_data = self.load_joint_data(filepath)
        self.data_index = 0

        # Set up the timer to publish data
        self.timer = self.create_timer(self.dt / 1000.0, self.timer_callback)

    def load_joint_data(self, filepath):
        joint_data = []
        with open(filepath, 'r') as file:
            for line in file:
                if line.startswith('#') or not line.strip():
                    continue
                parts = line.split()
                timestamp = float(parts[0])
                joint_positions = [float(value) for value in parts[1:]]
                joint_data.append({"timestamp": timestamp, "positions": joint_positions})
        return joint_data

    def timer_callback(self):
        # Publish the next set of joint data
        if self.data_index < len(self.joint_data):
            msg = Float64MultiArray()
            joint_positions = self.joint_data[self.data_index]["positions"]
            msg.data = joint_positions

            # Publish the message
            self.command_publisher.publish(msg)
            self.get_logger().info(f'Published joint positions: {joint_positions}')

            # Increment the index for the next data set
            self.data_index += 1
        else:
            # Stop the timer when all data has been published
            self.get_logger().info("All joint data has been published.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)

    # Variable for the file path
    filepath = '/home/samuel/dev/environment_modeling/ROSBAGS/iisy_random_motion_data/joint_data/joint_positions.txt'
    
    node = DriverTestFloatArray(filepath)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import csv
import keyboard  # Install this with `pip install keyboard`
from rclpy.qos import qos_profile_default
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.pose_callback,
            qos_profile=QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        depth=10
    ))
        self.subscription  # Prevent unused variable warning
        self.pole_position = None  # To store the coordinates of "pole_position"
        self.csv_file = 'coordinates.csv'

        # Initialize the CSV file with headers
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)

        self.get_logger().info("Node initialized. Press 'p' to save coordinates.")

    def pose_callback(self, msg):
        # Search for the rigid body named "pole_position"
        for pose in msg.poses:
            if pose.name == "pole_position":
                self.pole_position = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                break

    def save_coordinates(self):
        if self.pole_position:
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.pole_position)
            self.get_logger().info(f"Saved coordinates: {self.pole_position}")
        else:
            self.get_logger().warn("No data for 'pole_position' received yet.")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Non-blocking spin
            if keyboard.is_pressed('p'):  # Detect if 'p' is pressed
                node.save_coordinates()
                time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

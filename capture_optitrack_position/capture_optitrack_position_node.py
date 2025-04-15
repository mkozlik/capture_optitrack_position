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
        self.csv_file_obstacle = '/root/ros2_ws/src/icuas25_competition/coordinates.csv'
        self.csv_file_aruco = '/root/ros2_ws/src/icuas25_competition/coordinates_aruco.csv'

        # Initialize the CSV file with headers
        with open(self.csv_file_obstacle, 'w', newline='') as f:
            writer_obstacle = csv.writer(f)

        # Initialize the CSV file with headers
        with open(self.csv_file_aruco, 'w', newline='') as f:
            writer_aruco = csv.writer(f)


        self.get_logger().info("Node initialized. Press 'p' to save obstacle coordinates.")
        self.get_logger().info("Node initialized. Press 'o' to save aruco marker coordinates.")

    def pose_callback(self, msg):
        # Search for the rigid body named "pole_position"
        for pose in msg.poses:
            if pose.name == "pole_position":
                self.pole_position = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                self.pole_position = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                                      pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
                break

    def save_pole_coordinates(self):
        if self.pole_position:
            with open(self.csv_file_obstacle, 'a', newline='') as f:
                writer_obstacle = csv.writer(f)
                writer_obstacle.writerow(self.pole_position)
            self.get_logger().info(f"Saved coordinates: {self.pole_position}")
        else:
            self.get_logger().warn("No data for 'pole_position' received yet.")

    def save_aruco_coordinates(self):
        if self.pole_position:
            with open(self.csv_file_aruco, 'a', newline='') as f:
                writer_aruco = csv.writer(f)
                writer_aruco.writerow(self.pole_position)
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
                node.save_pole_coordinates()
                time.sleep(1)
            if keyboard.is_pressed('o'):  # Detect if 'o' is pressed
                node.save_aruco_coordinates()
                time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

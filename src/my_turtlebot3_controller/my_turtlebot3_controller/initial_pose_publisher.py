# src/my_turtlebot3_controller/my_turtlebot3_controller/initial_pose_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # This publisher sends the initial pose to AMCL
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # --- Define your robot's starting pose here ---
        # These should match the values you want from your parameter file.
        x = 0.0021
        y = -0.0147
        yaw_degrees = 0.0

        # Wait for AMCL to be ready
        self.get_logger().info('Waiting for Nav2 to initialize...')
        time.sleep(10.0) # Wait 10 seconds

        # Create the message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y

        yaw_rad = math.radians(yaw_degrees)
        pose_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # Publish the message
        self.get_logger().info(f"Publishing initial pose: X={x}, Y={y}")
        self.publisher.publish(pose_msg)
        self.get_logger().info("Initial pose published. Shutting down.")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    # The node does its work and then we can let it shut down.
    # A short spin allows the message to be sent.
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

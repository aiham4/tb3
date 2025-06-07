# src/my_turtlebot3_controller/my_turtlebot3_controller/mission_control_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time
import math

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        # --- State Machine ---
        self.mission_phase = "IDLE"  # IDLE, NAV_TO_PICKUP, AT_PICKUP, NAV_TO_DROPOFF, AT_DROPOFF

        # --- Mission Queue & Current Mission ---
        self.order_queue = []
        self.current_order = None

        # --- Publishers and Subscribers ---
        self.goal_publisher = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)
        self.order_subscriber = self.create_subscription(String, '/new_delivery_order', self.new_order_callback, 10)
        self.nav_status_subscriber = self.create_subscription(String, '/navigation_executor_status', self.nav_status_callback, 10)

        # --- Main Control Loop ---
        self.control_timer = self.create_timer(1.0, self.control_cycle)

        self.get_logger().info("Mission Control initialized. Waiting for delivery orders.")

    def new_order_callback(self, msg):
        """Adds a new order to the queue."""
        self.get_logger().info("New delivery order received!")
        order = json.loads(msg.data)
        self.order_queue.append(order)

    def control_cycle(self):
        """The main state machine loop."""
        self.get_logger().info(f"Control cycle. Current phase: {self.mission_phase}")

        if self.mission_phase == "IDLE":
            if self.order_queue:
                self.current_order = self.order_queue.pop(0) # Get the next order
                self.get_logger().info(f"Processing order {self.current_order['order_id']}: {self.current_order['pickup_location']} -> {self.current_order['dropoff_location']}")

                # Dispatch to pickup location
                coords = self.current_order['pickup_coords']
                self.send_goal(coords['x'], coords['y'], coords['yaw'])
                self.mission_phase = "NAV_TO_PICKUP"

        # Note: The rest of the state transitions happen in nav_status_callback

    def nav_status_callback(self, msg: String):
        """Handles status updates from the navigation executor."""
        nav_status = msg.data
        self.get_logger().info(f"Received Nav Status: '{nav_status}' in phase '{self.mission_phase}'")

        if nav_status == "SUCCEEDED_AT_POSE":
            if self.mission_phase == "NAV_TO_PICKUP":
                self.get_logger().info("Arrived at pickup location. Simulating pickup...")
                self.mission_phase = "AT_PICKUP"
                time.sleep(5) # Simulate a 5-second pickup

                # Dispatch to dropoff location
                self.get_logger().info("Pickup complete. Navigating to dropoff...")
                coords = self.current_order['dropoff_coords']
                self.send_goal(coords['x'], coords['y'], coords['yaw'])
                self.mission_phase = "NAV_TO_DROPOFF"

            elif self.mission_phase == "NAV_TO_DROPOFF":
                self.get_logger().info("Arrived at dropoff location. Simulating dropoff...")
                self.mission_phase = "AT_DROPOFF"
                time.sleep(5) # Simulate a 5-second dropoff

                self.get_logger().info(f"Order {self.current_order['order_id']} complete!")
                self.current_order = None
                self.mission_phase = "IDLE" # Ready for the next order

        elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION"]:
            self.get_logger().error(f"Navigation failed for order {self.current_order['order_id']}. Re-queueing order and resetting.")
            # For simplicity, we add the failed order back to the front of the queue
            if self.current_order:
                self.order_queue.insert(0, self.current_order)
            self.current_order = None
            self.mission_phase = "IDLE"

    def send_goal(self, x, y, yaw_degrees):
        """Constructs and publishes a PoseStamped navigation goal."""
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)

        # This is the direct math to convert yaw to a quaternion
        yaw_rad = math.radians(float(yaw_degrees))
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.goal_publisher.publish(goal_pose)
        self.get_logger().info(f"Goal dispatched to NavExecutor: X={x}, Y={y}, Yaw={yaw_degrees}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# src/lab_turtlebot3/lab_turtlebot3/mission_control_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import json
import time
import math
import os

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        # --- State Machine & Task Queues ---
        self.mission_phase = "IDLE"  # Added new phases: NAV_TO_CHARGE, CHARGING
        self.current_order = None
        self.order_queue = []
        self.priority_queue = []
        self.wait_timer = None

        # --- Battery State ---
        self.battery_level = 100.0
        self.low_battery_threshold = 30.0 # Go charge when battery is at or below this %

        self.load_locations()
        self.load_route()

        # --- Publishers and Subscribers ---
        self.goal_publisher = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)
        self.charge_publisher = self.create_publisher(Empty, '/charge_robot', 10)
        self.priority_order_subscriber = self.create_subscription(String, '/priority_delivery_order', self.priority_order_callback, 10)
        self.nav_status_subscriber = self.create_subscription(String, 'navigation_executor_status', self.nav_status_callback, 10)
        self.battery_subscriber = self.create_subscription(Float32, '/battery_level', self.battery_level_callback, 10)

        self.control_timer = self.create_timer(1.0, self.control_cycle)
        self.get_logger().info("Mission Control initialized. Loaded {} standard tasks.".format(len(self.task_queue)))

    def load_locations(self):
        """Loads delivery location coordinates."""
        self.locations = {
            'station_a': {'x': 4.73394, 'y': 0.17560, 'yaw': 90.0},
            'station_b': {'x': 4.73394, 'y': -5.32599, 'yaw': -90.0},
            'station_c': {'x': 2.46012, 'y': 0.20725, 'yaw': 180.0},
            'station_d': {'x': 2.50568, 'y': -5.33227, 'yaw': 0.0},
            'station_e': {'x': -2.24485, 'y': 0.17683, 'yaw': 0.0},
            'station_f': {'x': -3.72440, 'y': 0.14240, 'yaw': 0.0},
            'station_g': {'x': -3.72784, 'y': -1.97177, 'yaw': 0.0},
            'Origin': {'x': 0.00105, 'y': -0.016816, 'yaw': 0.0},
            'charging_1': {'x': 1.04122, 'y': 0.17214, 'yaw': 90.0}
        }

    def load_route(self):
        # ... (this function is unchanged)
        package_share_directory = get_package_share_directory('my_turtlebot3_controller')
        route_file_path = os.path.join(package_share_directory, 'config', 'delivery_route.json')
        try:
            with open(route_file_path, 'r') as f:
                self.task_queue = json.load(f)
            self.get_logger().info(f"Successfully loaded route from {route_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load delivery route: {e}")


    def priority_order_callback(self, msg):
        # ... (this function is unchanged)
        self.get_logger().info("PRIORITY order received!")
        try:
            order = json.loads(msg.data)
            self.priority_queue.insert(0, order)
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON in priority order message.")

    def battery_level_callback(self, msg):
        """Updates the current battery level."""
        self.battery_level = msg.data

    def control_cycle(self):
        """The main state machine loop to check for and start new missions."""
        if self.mission_phase != "IDLE":
            return # Robot is busy

        # 1. HIGHEST PRIORITY: Check battery level.
        if self.battery_level <= self.low_battery_threshold:
            self.get_logger().warn("LOW BATTERY DETECTED! Overriding all tasks to go charge.")
            charge_coords = self.locations['charging_1']
            self.send_goal(charge_coords['x'], charge_coords['y'], charge_coords['yaw'])
            self.mission_phase = "NAV_TO_CHARGE"
            return # Don't process any other tasks

        # 2. Second Priority: Handle priority orders.
        if self.priority_queue:
            self.current_order = self.priority_queue.pop(0)
            self.get_logger().info(f"Accepting PRIORITY order {self.current_order.get('order_id', 'N/A')}")
        # 3. Lowest Priority: Handle standard route orders.
        elif self.task_queue:
            self.current_order = self.task_queue.pop(0)
            self.get_logger().info(f"Accepting standard route order {self.current_order.get('order_id', 'N/A')}")
        else:
            return # No tasks to perform

        if self.current_order:
            # ... (The logic to start a standard NAV_TO_PICKUP mission is unchanged)
            pickup_loc = self.current_order.get('pickup_location')
            if pickup_loc and pickup_loc in self.locations:
                self.get_logger().info(f"Processing: {pickup_loc} -> {self.current_order.get('dropoff_location')}")
                coords = self.locations[pickup_loc]
                self.send_goal(coords['x'], coords['y'], coords['yaw'])
                self.mission_phase = "NAV_TO_PICKUP"
            else:
                self.get_logger().error(f"Invalid or missing pickup location '{pickup_loc}' in order. Skipping.")
                self.mission_phase = "IDLE"

    def nav_status_callback(self, msg: String):
        """Handles status updates from the navigation executor to advance the state machine."""
        nav_status = msg.data
        if self.mission_phase in ["IDLE", "AT_PICKUP", "CHARGING"]:
            return

        if nav_status == "SUCCEEDED_AT_POSE":
            if self.mission_phase == "NAV_TO_PICKUP":
                self.mission_phase = "AT_PICKUP"
                self.get_logger().info("Arrived at pickup. Simulating 5-second pickup...")
                self.wait_timer = self.create_timer(5.0, self.pickup_complete_callback)
            elif self.mission_phase == "NAV_TO_DROPOFF":
                self.get_logger().info(f"Order {self.current_order.get('order_id', 'N/A')} complete!")
                self.current_order = None
                self.mission_phase = "IDLE"
            # Handle arrival at charging station
            elif self.mission_phase == "NAV_TO_CHARGE":
                self.mission_phase = "CHARGING"
                self.get_logger().info("Arrived at charging station. Charging for 15 seconds...")
                self.wait_timer = self.create_timer(15.0, self.charging_complete_callback)

        elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION"]:
            self.get_logger().error(f"Navigation failed for order/task. Resetting.")
            if self.mission_phase == "NAV_TO_PICKUP" or self.mission_phase == "NAV_TO_DROPOFF":
                 # Re-queue the failed delivery order
                 if self.current_order:
                    self.task_queue.insert(0, self.current_order)
            self.current_order = None
            self.mission_phase = "IDLE"

    def pickup_complete_callback(self):
        # ... (this function is unchanged)
        self.wait_timer.cancel()
        dropoff_loc = self.current_order.get('dropoff_location')
        if dropoff_loc and dropoff_loc in self.locations:
            self.get_logger().info("Pickup complete. Navigating to dropoff...")
            coords = self.locations[dropoff_loc]
            self.send_goal(coords['x'], coords['y'], coords['yaw'])
            self.mission_phase = "NAV_TO_DROPOFF"
        else:
            self.get_logger().error(f"Invalid or missing dropoff location '{dropoff_loc}' in order. Ending task.")
            self.mission_phase = "IDLE"

    def charging_complete_callback(self):
        """Called by the timer after waiting at the charging station."""
        self.wait_timer.cancel()
        self.get_logger().info("Charging complete.")
        # Send a message to the battery node to reset its level to 100%
        self.charge_publisher.publish(Empty())
        self.mission_phase = "IDLE"

    def send_goal(self, x, y, yaw_degrees):
        # ... (this function is unchanged)
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        yaw_rad = math.radians(float(yaw_degrees))
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self.goal_publisher.publish(goal_pose)
        self.get_logger().info(f"Dispatching goal to NavExecutor: X={x}, Y={y}, Yaw={yaw_degrees}")

def main(args=None):
    # ... (this function is unchanged)
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

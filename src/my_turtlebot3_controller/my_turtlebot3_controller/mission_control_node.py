# src/my_turtlebot3_controller/my_turtlebot3_controller/mission_control_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
        self.mission_phase = "IDLE"
        self.current_order = None
        self.priority_queue = []
        self.task_queue = []

        # --- Load Delivery Locations and Route ---
        self.load_locations()
        self.load_route()

        # --- Publishers and Subscribers ---
        self.goal_publisher = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)
        self.priority_order_subscriber = self.create_subscription(String, '/priority_delivery_order', self.priority_order_callback, 10)
        self.nav_status_subscriber = self.create_subscription(String, 'navigation_executor_status', self.nav_status_callback, 10)

        # --- Main Control Loop ---
        self.control_timer = self.create_timer(1.0, self.control_cycle)
        self.get_logger().info("Mission Control initialized. Loaded {} standard tasks.".format(len(self.task_queue)))

    def load_locations(self):
        """Loads delivery location coordinates from a config file."""
        self.locations = {
            'station_a': {'x': 2.55279, 'y': 1.06474, 'yaw': 90.0},
            'station_b': {'x': 1.46613, 'y': 1.03531, 'yaw': -90.0},
            'station_c': {'x': 0.39297, 'y': 1.06064, 'yaw': 180.0},
            'station_d': {'x': 2.05827, 'y': 2.40405, 'yaw': 0.0},
            'station_e': {'x': 3.83550, 'y': -0.08136, 'yaw': 0.0},
            'station_f': {'x': -0.35381, 'y': 0.54885, 'yaw': 0.0},
            'station_g': {'x': 1.500010848, 'y': -0.613825202, 'yaw': 0.0},
            'Origin': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        }

    def load_route(self):
        """Loads the standard delivery route from a JSON file."""
        package_share_directory = get_package_share_directory('my_turtlebot3_controller')
        route_file_path = os.path.join(package_share_directory, 'config', 'delivery_route.json')
        try:
            with open(route_file_path, 'r') as f:
                self.task_queue = json.load(f)
            self.get_logger().info(f"Successfully loaded route from {route_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load delivery route: {e}")

    def priority_order_callback(self, msg):
        """Adds a high-priority order to the front of the priority queue."""
        self.get_logger().info("PRIORITY order received!")
        try:
            order = json.loads(msg.data)
            self.priority_queue.insert(0, order)
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON in priority order message.")

    def control_cycle(self):
        """The main state machine loop to check for and start new missions."""
        if self.mission_phase != "IDLE":
            return # Don't start a new mission if we are already busy

        # Prioritize priority queue over the standard task queue
        if self.priority_queue:
            self.current_order = self.priority_queue.pop(0)
            self.get_logger().info(f"Accepting PRIORITY order {self.current_order.get('order_id', 'N/A')}")
        elif self.task_queue:
            self.current_order = self.task_queue.pop(0)
            self.get_logger().info(f"Accepting standard route order {self.current_order.get('order_id', 'N/A')}")
        else:
            return # No tasks to perform
            
        if self.current_order:
            pickup_loc = self.current_order.get('pickup_location')
            if pickup_loc and pickup_loc in self.locations:
                self.get_logger().info(f"Processing: {pickup_loc} -> {self.current_order.get('dropoff_location')}")
                coords = self.locations[pickup_loc]
                self.send_goal(coords['x'], coords['y'], coords['yaw'])
                self.mission_phase = "NAV_TO_PICKUP"
            else:
                self.get_logger().error(f"Invalid or missing pickup location '{pickup_loc}' in order. Skipping.")
                self.mission_phase = "IDLE" # Reset to be able to take next task
    
    def nav_status_callback(self, msg: String):
        """Handles status updates from the navigation executor to advance the state machine."""
        nav_status = msg.data
        if self.mission_phase in ["IDLE", "AT_PICKUP"]:
            return # Ignore status updates when not actively navigating for a mission

        if nav_status == "SUCCEEDED_AT_POSE":
            if self.mission_phase == "NAV_TO_PICKUP":
                self.mission_phase = "AT_PICKUP"
                self.get_logger().info("Arrived at pickup. Simulating 5-second pickup...")
                time.sleep(5)
                
                dropoff_loc = self.current_order.get('dropoff_location')
                if dropoff_loc and dropoff_loc in self.locations:
                    self.get_logger().info("Pickup complete. Navigating to dropoff...")
                    coords = self.locations[dropoff_loc]
                    self.send_goal(coords['x'], coords['y'], coords['yaw'])
                    self.mission_phase = "NAV_TO_DROPOFF"
                else:
                    self.get_logger().error(f"Invalid or missing dropoff location '{dropoff_loc}' in order. Ending task.")
                    self.mission_phase = "IDLE"
                
            elif self.mission_phase == "NAV_TO_DROPOFF":
                self.get_logger().info(f"Order {self.current_order.get('order_id', 'N/A')} complete!")
                self.current_order = None
                self.mission_phase = "IDLE"
        
        elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION"]:
            self.get_logger().error(f"Navigation failed for order {self.current_order.get('order_id', 'N/A')}. Resetting.")
            self.current_order = None
            self.mission_phase = "IDLE"

    def send_goal(self, x, y, yaw_degrees):
        """Constructs and publishes a PoseStamped navigation goal without external dependencies."""
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
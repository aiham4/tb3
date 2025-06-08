# src/my_turtlebot3_controller/my_turtlebot3_controller/order_dispatch_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json # We will use JSON to pass order data easily

class OrderDispatchNode(Node):
    """
    Simulates a dispatch system creating new delivery orders.
    An order consists of a pickup and a drop-off location.
    """
    def __init__(self):
        super().__init__('order_dispatch_node')

        # --- Parameters ---
        self.declare_parameter('dispatch_interval_sec', 15.0)
        interval = self.get_parameter('dispatch_interval_sec').get_parameter_value().double_value

        # --- Pre-defined locations for demonstration ---
        # In a real system, these might come from a database or another service.
        self.locations = {
            'Warehouse A': {'x': 2.55279, 'y': 1.06474, 'yaw': 90.0},
            'Station B':   {'x': 1.46613, 'y': 1.03531, 'yaw': -90.0},
            'Hub C':       {'x': 0.39297, 'y': 1.06064, 'yaw': 180.0},
            'Dock D':      {'x': 2.05827, 'y': 2.40405, 'yaw': 0.0}
        }
        self.location_names = list(self.locations.keys())

        # --- ROS Communications ---
        self.order_publisher = self.create_publisher(String, '/new_delivery_order', 10)

        # Timer to periodically create a new order
        self.dispatch_timer = self.create_timer(interval, self.dispatch_new_order)
        self.get_logger().info("Order Dispatch Node started. Will dispatch a new order every {} seconds.".format(interval))

    def dispatch_new_order(self):
        """Creates and publishes a new random delivery order."""

        # Ensure pickup and dropoff are not the same
        pickup_name, dropoff_name = random.sample(self.location_names, 2)

        pickup_coords = self.locations[pickup_name]
        dropoff_coords = self.locations[dropoff_name]

        order_data = {
            'order_id': 'Order' + str(random.randint(1000, 9999)),
            'pickup_location': pickup_name,
            'pickup_coords': pickup_coords,
            'dropoff_location': dropoff_name,
            'dropoff_coords': dropoff_coords
        }

        # Publish the order as a JSON string
        order_json = json.dumps(order_data)
        msg = String()
        msg.data = order_json
        self.order_publisher.publish(msg)

        self.get_logger().info(f"--- NEW ORDER DISPATCHED ---")
        self.get_logger().info(f"  ID: {order_data['order_id']}")
        self.get_logger().info(f"  Pickup: {pickup_name} at {pickup_coords}")
        self.get_logger().info(f"  Dropoff: {dropoff_name} at {dropoff_coords}")

def main(args=None):
    rclpy.init(args=args)
    node = OrderDispatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

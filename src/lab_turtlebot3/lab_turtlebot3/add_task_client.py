# src/lab_turtlebot3/lab_turtlebot3/add_task_client.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class AddTaskClient(Node):
    def __init__(self):
        super().__init__('add_task_client')
        
        # --- Parameters for the new task ---
        self.declare_parameter('pickup', 'station_a')
        self.declare_parameter('dropoff', 'station_e')

        pickup_name = self.get_parameter('pickup').get_parameter_value().string_value
        dropoff_name = self.get_parameter('dropoff').get_parameter_value().string_value
        
        # --- Publisher to the priority topic ---
        self.priority_publisher = self.create_publisher(String, '/priority_delivery_order', 10)
        
        # Give publisher a moment to connect before sending
        time.sleep(1)
        
        order_data = {
            'order_id': 'PRIORITY_MANUAL',
            'pickup_location': pickup_name,
            'dropoff_location': dropoff_name
        }
        
        msg = String()
        msg.data = json.dumps(order_data)
        self.priority_publisher.publish(msg)
        self.get_logger().info(f"Sent priority task: {pickup_name} -> {dropoff_name}")

def main(args=None):
    rclpy.init(args=args)
    client_node = AddTaskClient()
    # The node sends the message and can then be shut down
    rclpy.spin_once(client_node, timeout_sec=1)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

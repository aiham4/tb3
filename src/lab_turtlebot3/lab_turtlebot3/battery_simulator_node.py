# src/lab_turtlebot3/lab_turtlebot3/battery_simulator_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty

class BatterySimulatorNode(Node):
    """
    Simulates a robot battery that depletes over time and can be recharged.
    """
    def __init__(self):
        super().__init__('battery_simulator_node')

        # Start with a full battery
        self.battery_level = 100.0

        # --- Publishers and Subscribers ---
        self.battery_level_publisher = self.create_publisher(Float32, '/battery_level', 10)
        self.charge_subscriber = self.create_subscription(Empty, '/charge_robot', self.charge_battery_callback, 10)

        # --- Timers ---
        # Timer to decrease battery level every second
        self.depletion_timer = self.create_timer(0.4, self.deplete_battery)
        # Timer to publish the current battery level every 2 seconds
        self.publish_timer = self.create_timer(2.0, self.publish_battery_level)

        self.get_logger().info("Battery Simulator started. Initial level: 100%")

    def deplete_battery(self):
        """Simulates battery usage over time."""
        if self.battery_level > 0:
            self.battery_level -= 0.5 # Decrease by 0.5% every second
            self.battery_level = max(0.0, self.battery_level)

    def publish_battery_level(self):
        """Publishes the current battery level."""
        msg = Float32()
        msg.data = self.battery_level
        self.battery_level_publisher.publish(msg)
        self.get_logger().info(f"Current battery level: {self.battery_level:.1f}%")

    def charge_battery_callback(self, msg):
        """Callback to reset the battery to full."""
        self.get_logger().info("Charging command received! Resetting battery to 100%.")
        self.battery_level = 100.0
        # Publish the new full level immediately
        self.publish_battery_level()

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

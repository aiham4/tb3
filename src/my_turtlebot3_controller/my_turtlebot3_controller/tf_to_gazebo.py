import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion

class TfToGazeboNode(Node):
    def __init__(self):
        super().__init__('tf_to_gazebo_bridge')

        # The name of the robot model in Gazebo (must match the <model name='...'> in the SDF)
        self.declare_parameter('gazebo_model_name', 'robot1')
        self.model_name = self.get_parameter('gazebo_model_name').get_parameter_value().string_value

        # The frames we want to listen to for the PHYSICAL robot
        # We want the transform from the map frame to the robot's base_footprint
        self.declare_parameter('parent_frame', 'map')
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value

        self.declare_parameter('child_frame', 'base_footprint')
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        # TF listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Client to call the Gazebo service to set the model's state
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo\'s /set_entity_state service not available, waiting again...')

        # Timer to periodically check for TF and update Gazebo
        self.timer = self.create_timer(0.05, self.on_timer) # 20 Hz update rate

    def on_timer(self):
        try:
            # Look up the transform from the map to the physical robot's base_footprint
            t = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.parent_frame} to {self.child_frame}: {ex}')
            return

        # Create the request for the Gazebo service
        req = SetEntityState.Request()
        req.state.name = self.model_name
        req.state.pose.position = Point(
            x=t.transform.translation.x,
            y=t.transform.translation.y,
            z=t.transform.translation.z
        )
        req.state.pose.orientation = Quaternion(
            x=t.transform.rotation.x,
            y=t.transform.rotation.y,
            z=t.transform.rotation.z,
            w=t.transform.rotation.w
        )

        # Asynchronously call the service to update the twin's pose in Gazebo
        self.set_state_client.call_async(req)
        # self.get_logger().info(f"Updated Gazebo model '{self.model_name}' pose.")

def main(args=None):
    rclpy.init(args=args)
    node = TfToGazeboNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

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

        # This should match the <model name='...'> in your edited SDF file.
        # For the setup we designed, this is 'robot1'.
        self.model_name = 'robot1'

        # The frames we want to listen to for the PHYSICAL robot
        self.parent_frame = 'map'
        self.child_frame = 'base_footprint'

        # TF listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- THIS IS THE CORRECTED LINE ---
        # The service to set a model's state in Gazebo is '/gazebo/set_entity_state'
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo service /gazebo/set_entity_state not available, waiting...')

        # Timer to periodically check for TF and update Gazebo
        self.timer = self.create_timer(0.05, self.on_timer) # 20 Hz update rate
        self.get_logger().info('TF-to-Gazebo bridge started.')
        self.get_logger().info(f"Will synchronize physical robot's '{self.child_frame}' frame to Gazebo model '{self.model_name}'.")

    def on_timer(self):
        try:
            # Look up the latest transform from the map to the physical robot's base_footprint
            t = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not get transform from "{self.parent_frame}" to "{self.child_frame}": {ex}',
                 throttle_duration_sec=1.0)
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
        # We don't need to set velocity, so we leave those fields empty

        # Asynchronously call the service to update the twin's pose in Gazebo
        self.set_state_client.call_async(req)

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

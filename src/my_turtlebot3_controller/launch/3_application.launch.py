from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_turtlebot3_controller', executable='mission_control_node', output='screen'),
        Node(package='my_turtlebot3_controller', executable='navigation_executor_node', output='screen')
    ])

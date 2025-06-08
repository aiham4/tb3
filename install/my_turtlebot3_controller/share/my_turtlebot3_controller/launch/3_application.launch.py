# launch/3_application.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    start_order_dispatch_cmd = Node(
        package='my_turtlebot3_controller',
        executable='order_dispatch_node',
        output='screen'
    )

    start_mission_control_cmd = Node(
        package='my_turtlebot3_controller',
        executable='mission_control_node',
        output='screen'
    )

    start_navigation_executor_cmd = Node(
        package='my_turtlebot3_controller',
        executable='navigation_executor_node',
        output='screen'
    )

    return LaunchDescription([
        start_order_dispatch_cmd,
        start_mission_control_cmd,
        start_navigation_executor_cmd
    ])

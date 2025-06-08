# /home/ayham/Courses/CBL/ub/projects/src/my_turtlebot3_controller/launch/start_complete_system.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define paths to necessary packages
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    my_controller_dir = get_package_share_directory('my_turtlebot3_controller')

    # Define paths to configuration files
    map_yaml_file = LaunchConfiguration('map',
                                    default=os.path.join(
                                        my_controller_dir, 'config', 'map.yaml'))

    nav2_params_file = LaunchConfiguration('params_file',
                                           default=os.path.join(
                                               my_controller_dir, 'config', 'tuned_nav_params.yaml'))

    rviz_config_file = LaunchConfiguration('rviz_config_file',
                                           default=os.path.join(
                                               my_controller_dir, 'config', 'nav_view.rviz'))

    # This is the core Nav2 simulation launch command. It starts Gazebo, the robot, Nav2, and RViz.
    start_tb3_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'true',
            'rviz_config_file': rviz_config_file,
        }.items(),
    )

    # --- Add Your Custom Application Nodes ---

    # The Mission Controller (The Brain)
    start_mission_control_node_cmd = Node(
        package='my_turtlebot3_controller',
        executable='mission_control_node',
        name='mission_control_node',
        output='screen'
    )

    # The Navigator
    start_navigation_executor_cmd = Node(
        package='my_turtlebot3_controller',
        executable='navigation_executor_node',
        name='navigation_executor_node',
        output='screen'
    )

    # The Order Dispatcher
    start_order_dispatch_cmd = Node(
        package='my_turtlebot3_controller',
        executable='order_dispatch_node',
        name='order_dispatch_node',
        output='screen'
    )

    # Create the final launch description
    ld = LaunchDescription()

    # Add the main simulation launch action
    ld.add_action(start_tb3_simulation_cmd)

    # Add your custom application nodes to the launch
    ld.add_action(start_mission_control_node_cmd)
    ld.add_action(start_navigation_executor_cmd)
    ld.add_action(start_order_dispatch_cmd)

    return ld

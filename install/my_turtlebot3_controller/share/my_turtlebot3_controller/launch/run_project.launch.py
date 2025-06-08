import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # --- Define Paths ---
    pkg_my_controller = get_package_share_directory('my_turtlebot3_controller')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # --- Define Configuration Files ---
    map_file_path = os.path.join(pkg_my_controller, 'config', 'map.yaml')
    params_file_path = os.path.join(pkg_my_controller, 'config', 'tuned_nav_params.yaml')
    rviz_config_path = os.path.join(pkg_my_controller, 'config', 'nav_view.rviz')

    # --- Declare Actions ---

    # Action 1: Start Gazebo and the Robot using the official, stable launch file
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Action 2: Start the Nav2 Stack using the official bringup, but passing our safe params
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': 'True',
            'params_file': params_file_path
        }.items(),
    )

    # Action 3: Start RViz with our custom configuration
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Action 4: Start your Application Nodes after a delay to ensure Nav2 is ready
    delayed_app_nodes = TimerAction(
        period=8.0, # Wait 8 seconds for Nav2 to fully initialize
        actions=[
            GroupAction(actions=[
                Node(package='my_turtlebot3_controller', executable='order_dispatch_node', name='order_dispatch_node', output='screen'),
                Node(package='my_turtlebot3_controller', executable='mission_control_node', name='mission_control_node', output='screen'),
                Node(package='my_turtlebot3_controller', executable='navigation_executor_node', name='navigation_executor_node', output='screen')
            ])
        ]
    )

    # --- Assemble the Launch ---
    return LaunchDescription([
        # Set use_sim_time for all nodes in this launch context
        SetParameter(name='use_sim_time', value=True),

        start_gazebo_cmd,
        start_nav2_cmd,
        start_rviz_cmd,
        delayed_app_nodes
    ])

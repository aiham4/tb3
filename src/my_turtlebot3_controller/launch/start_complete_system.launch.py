import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Define Paths ---
    my_controller_dir = get_package_share_directory('my_turtlebot3_controller')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    # --- Configs ---
    map_file = os.path.join(my_controller_dir, 'config', 'map.yaml')
    rviz_config_file = os.path.join(my_controller_dir, 'config', 'nav_view.rviz')

    # --- Actions ---

    # 1. Start Gazebo (Known-Good)
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py'))
    )

    # 2. Start Nav2 and RViz (Known-Good command, no custom params)
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_nav2_dir, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file,
            'use_rviz': 'True',
            'rviz_config_file': rviz_config_file
        }.items()
    )

    # 3. Start the Initial Pose Publisher (after a delay)
    start_initial_pose_pub_cmd = TimerAction(
        period=10.0,
        actions=[Node(package='my_turtlebot3_controller', executable='initial_pose_publisher', output='screen')]
    )

    # 4. Start your Application Nodes (after a longer delay)
    start_app_nodes_cmd = TimerAction(
        period=20.0,
        actions=[
            GroupAction(actions=[
                Node(package='my_turtlebot3_controller', executable='mission_control_node', output='screen'),
                Node(package='my_turtlebot3_controller', executable='navigation_executor_node', output='screen')
            ])
        ]
    )

    return LaunchDescription([
        start_gazebo_cmd,
        start_nav2_cmd,
        start_initial_pose_pub_cmd,
        start_app_nodes_cmd
    ])

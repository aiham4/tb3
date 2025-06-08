import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # --- Paths ---
    pkg_my_controller = get_package_share_directory('my_turtlebot3_controller')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # --- Config Files ---
    map_file_path = os.path.join(pkg_my_controller, 'config', 'map.yaml')
    params_file_path = os.path.join(pkg_my_controller, 'config', 'final_params.yaml')
    rviz_config_path = os.path.join(pkg_my_controller, 'config', 'nav_view.rviz')

    # --- Actions ---

    # 1. LAUNCH GAZEBO & ROBOT (using the stable official launch file)
    start_gazebo_and_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 2. LAUNCH NAV2 (explicitly launching each node for stability)
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[params_file_path], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[params_file_path], output='screen'),
        Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', parameters=[params_file_path], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[{'yaml_filename': map_file_path}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[params_file_path], output='screen'),
    ]

    # 3. LAUNCH LIFECYCLE MANAGER (with a delay to prevent race conditions)
    delayed_lifecycle_manager = TimerAction(
        period=7.0, # Wait 7 seconds for Gazebo and nodes to be ready
        actions=[
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': ['planner_server', 'controller_server', 'recoveries_server', 'bt_navigator', 'map_server', 'amcl']}]
            )
        ]
    )

    # 4. LAUNCH RViz
    start_rviz_cmd = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen')

    # 5. LAUNCH YOUR APPLICATION NODES (after Nav2 is stable)
    delayed_app_nodes = TimerAction(
        period=10.0, # Wait 10 seconds total
        actions=[
            GroupAction(actions=[
                Node(package='my_turtlebot3_controller', executable='mission_control_node', output='screen'),
                Node(package='my_turtlebot3_controller', executable='navigation_executor_node', output='screen')
            ])
        ]
    )

    # --- Assemble the Launch ---
    ld = LaunchDescription()
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(start_gazebo_and_robot_cmd)
    for node in nav2_nodes:
        ld.add_action(node)
    ld.add_action(delayed_lifecycle_manager)
    ld.add_action(start_rviz_cmd)
    ld.add_action(delayed_app_nodes)

    return ld

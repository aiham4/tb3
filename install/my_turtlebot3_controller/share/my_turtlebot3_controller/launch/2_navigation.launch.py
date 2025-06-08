# launch/2_navigation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    pkg_my_controller = get_package_share_directory('my_turtlebot3_controller')

    # Use our new, definitive parameter file
    params_file_path = os.path.join(pkg_my_controller, 'config', 'final_nav2_params.yaml')
    map_file_path = os.path.join(pkg_my_controller, 'config', 'map.yaml')

    # Define all the Nav2 nodes
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[params_file_path]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[params_file_path]),
        Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server', output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[params_file_path]),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', parameters=[{'yaml_filename': map_file_path}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[params_file_path])
    ]

    # Define the lifecycle manager, which will be started after a delay
    delayed_lifecycle_manager = TimerAction(
        period=5.0, # Wait 5 seconds
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True},
                            {'node_names': ['planner_server', 'controller_server', 'recoveries_server', 'bt_navigator', 'map_server', 'amcl']}]
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(SetParameter(name='use_sim_time', value=True))

    for node in nav2_nodes:
        ld.add_action(node)

    ld.add_action(delayed_lifecycle_manager)

    return ld

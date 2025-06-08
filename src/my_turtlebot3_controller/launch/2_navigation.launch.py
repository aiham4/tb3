import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    pkg_my_controller = get_package_share_directory('my_turtlebot3_controller')
    params_file_path = os.path.join(pkg_my_controller, 'config', 'final_nav2_params.yaml')
    map_file_path = os.path.join(pkg_my_controller, 'config', 'map.yaml')

    nav2_nodes = [
        Node(package='nav2_controller',executable='controller_server',name='controller_server',parameters=[params_file_path],output='screen'),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params_file_path],output='screen'),
        Node(package='nav2_recoveries',executable='recoveries_server',name='recoveries_server',parameters=[{'use_sim_time': True}],output='screen'),
        Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',parameters=[params_file_path],output='screen'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'yaml_filename': map_file_path}],output='screen'),
        Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[params_file_path],output='screen'),
    ]

    delayed_lifecycle_manager = TimerAction(
        period=5.0,
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

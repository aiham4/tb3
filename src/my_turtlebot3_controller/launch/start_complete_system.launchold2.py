# /home/ayham/Courses/CBL/ub/projects/src/my_turtlebot3_controller/launch/start_complete_system.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    my_controller_dir = get_package_share_directory('my_turtlebot3_controller')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')

    map_file = LaunchConfiguration('map', default=os.path.join(my_controller_dir, 'config', 'map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(my_controller_dir, 'config', 'tuned_nav_params.yaml'))

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py'))
    )

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_nav2_dir, 'launch', 'navigation2.launch.py')),
        launch_arguments={'use_sim_time': 'True', 'map': map_file, 'params_file': params_file}.items()
    )

    start_app_nodes_cmd = GroupAction(
        actions=[
            Node(package='my_turtlebot3_controller', executable='mission_control_node', output='screen'),
            Node(package='my_turtlebot3_controller', executable='navigation_executor_node', output='screen')
        ]
    )

    return LaunchDescription([start_gazebo_cmd, start_nav2_cmd, start_app_nodes_cmd])

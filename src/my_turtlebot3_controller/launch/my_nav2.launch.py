import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your package
    my_tb3_controller_dir = get_package_share_directory('my_turtlebot3_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # --- Declare launch arguments ---

    # Set use_sim_time to True, as we are using a Gazebo simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Point to your local map file
    map_dir = LaunchConfiguration('map',
        default=os.path.join(my_tb3_controller_dir, 'config', 'map.yaml'))

    # Point to your local, tuned parameter file
    param_dir = LaunchConfiguration('params_file',
        default=os.path.join(my_tb3_controller_dir, 'config', 'tuned_nav_params.yaml'))

    # Point to your custom RViz config file
    rviz_config_dir = LaunchConfiguration('rviz_config',
        default=os.path.join(my_tb3_controller_dir, 'config', 'nav_view.rviz'))

    # --- Define actions ---

    # Action to launch the main Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir  # Corrected argument name from 'params' to 'params_file'
        }.items(),
    )

    # Action to launch RViz2 with the custom config
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # You can keep these declarations if you want to override them from the command line
        DeclareLaunchArgument('map', default_value=map_dir),
        DeclareLaunchArgument('params_file', default_value=param_dir),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config_dir),

        # Add the actions to the launch description
        nav2_bringup_cmd,
        rviz_cmd
    ])

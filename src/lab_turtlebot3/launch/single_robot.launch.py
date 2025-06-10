import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Define Paths ---
    my_controller_dir = get_package_share_directory('lab_turtlebot3')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Using the standard Nav2 bringup launch for better namespace support
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # --- Configs ---
    namespace = 'robot1'
    map_file = os.path.join(my_controller_dir, 'config', 'map1.yaml')
    # Make sure this points to your corrected, reusable parameter file
    params_file = os.path.join(my_controller_dir, 'config', 'default_nav2_params.yaml')
    rviz_config_file = os.path.join(my_controller_dir, 'config', 'nav_view.rviz')

    # Path to the robot's SDF model which includes the plugins
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger_robot1', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # --- Actions ---

    # 1. Start Gazebo with your world (robot is NOT in the world file anymore)
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(my_controller_dir, 'launch', 'nsmap_world.launch.py'))
    )

    # 2. Add Robot State Publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content,
            'frame_prefix': [namespace, '/']
        }]
    )

    # 3. Spawn the robot into Gazebo
    start_spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace,
        arguments=[
            '-topic', 'robot_description',
            '-entity', namespace,
            '-robot_namespace', namespace, # Tells plugins to use this namespace
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.01'
        ],
        output='screen'
    )

    # 4. Start Nav2 for the namespaced robot
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'True',
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'True',
            'autostart': 'True',
            'use_rviz': 'True', # We can let the Nav2 launch bring up RViz
            'rviz_config_file': rviz_config_file
        }.items()
    )

    # 5. Start your application nodes, now with a namespace
    start_app_nodes_cmd = TimerAction(
        period=21.0,
        actions=[   '''
            GroupAction(actions=[
                Node(package='my_turtlebot3_controller', executable='mission_control_node', namespace=namespace, output='screen'),
                Node(package='my_turtlebot3_controller', executable='navigation_executor_node', namespace=namespace, output='screen'),
                Node(package='my_turtlebot3_controller', executable='battery_simulator_node', namespace=namespace, output='screen'),
                Node(package='my_turtlebot3_controller', executable='initial_pose_publisher', namespace=namespace, output='screen')
            ])
            '''
        ]
    )

    return LaunchDescription([
        start_gazebo_cmd,
        start_robot_state_publisher_cmd,
        start_spawn_entity_cmd,
        start_nav2_cmd,
        start_app_nodes_cmd
    ])

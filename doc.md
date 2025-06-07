cd to projects
rm -rf install build log
colcon build
source /opt/ros/foxy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger # source and declare model in every terminal!
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch my_turtlebot3_controller my_nav2.launch.py 
# 2d pose estimate
ros2 run my_turtlebot3_controller order_dispatch_node
ros2 run my_turtlebot3_controller mission_control_node 
ros2 run my_turtlebot3_controller navigation_executor_node 


# current functionality
move to predetermined points at an interval

# issues:
rviz does not always work
turtlebot stops after some time



## ignore for now:
#ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false 
#rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
  # In a new terminal on your laptop (sourced)
# Save it into your package's src/maps directory for version control
#ros2 run nav2_map_server map_saver_cli -f ~/your_ros2_ws/src/my_smart_city_pkg/maps/#lab_map --ros-args -p use_sim_time:=false


#ros2 launch nav2_bringup bringup_launch.py \
 #   use_sim_time:=false \
  #  autostart:=true \
   # map:=/home/anas/your_ros2_ws/src/my_smart_city_pkg/maps/lab_map_from_cli.yaml \
   # params_file:=/home/anas/your_ros2_ws/src/my_smart_city_pkg/config/#nav2_real_robot_params.yaml

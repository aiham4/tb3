# tb3
# steps
`cd to projects`
`rm -rf install build log`
`colcon build`
`source /opt/ros/foxy/setup.bash`
`source install/setup.bash`
`export TURTLEBOT3_MODEL=burger` 
## source and declare model in every terminal!
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
`ros2 launch my_turtlebot3_controller my_nav2.launch.py `
then do the 2d pose estimate
`ros2 run my_turtlebot3_controller order_dispatch_node`
`ros2 run my_turtlebot3_controller mission_control_node `
`ros2 run my_turtlebot3_controller navigation_executor_node `


# current functionality
move to predetermined points at an interval

# issues:
rviz does not always work
turtlebot stops after some time

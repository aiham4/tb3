import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ayham/Courses/CBL/ub/projects/install/my_turtlebot3_controller'

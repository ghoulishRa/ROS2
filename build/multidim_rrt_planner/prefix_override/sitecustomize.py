import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/israamaciaas/ros2_ws/install/multidim_rrt_planner'

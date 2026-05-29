import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vicen/ISDEFE/TurtleBot_DualSim/ros2_ws/install/dual_sim_bridge'

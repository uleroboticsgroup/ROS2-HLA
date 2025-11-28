import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vicen/ISDEFE/ros2_hla_ws/install/ROS2HLA_Bridge'

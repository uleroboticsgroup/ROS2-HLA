import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vicen/ISDEFE/GZ_HLA_ws/install/bridge_hla2ros2'

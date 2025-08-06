import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/addinnedu/ROS2_Jazzy_Study/install/my_demo_pkg'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jewoo/ROS2_Jazzy_Study/install/send_goal'

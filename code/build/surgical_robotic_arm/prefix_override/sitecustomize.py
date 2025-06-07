import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ashish/My_Lab/ROS/surgical_robotic_arm/install/surgical_robotic_arm'

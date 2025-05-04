import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ayushi/Collective_Robotics/A2/ros_work/install/collective_robotics'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/MAS/MAS_Program2024/MAS-Program/Collective_Robotics/ros_work/install/robile_reach_goal'

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = ["robot_0", "robot_1", "robot_2", "robot_3", "robot_4"]

    return LaunchDescription([
        Node(
            package='collective_robotics',
            executable='multi_bot_task2_wait', 
            name=f'{robot}_controller_wait',
            output='screen',
            parameters=[{'robot_name': robot}]
        ) for robot in robots
    ])

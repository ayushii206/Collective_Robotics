from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    robots = ["robot_0", "robot_1", "robot_2", "robot_3", "robot_4"]

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='cave_multi')
    stage = LaunchConfiguration('stage', default='True')
    rviz = LaunchConfiguration('rviz', default='False')
    namespace = LaunchConfiguration('namespace', default='')

    declare_world = DeclareLaunchArgument('world', default_value='cave_multi')
    declare_stage = DeclareLaunchArgument('stage', default_value='True')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='False')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='')

    stage_ros2_dir = get_package_share_directory('stage_ros2')
    launch_dir = os.path.join(stage_ros2_dir, 'launch')

    return LaunchDescription([
        declare_world,
        declare_stage,
        declare_rviz,
        declare_namespace,

        # Include Stage simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'stage.launch.py')),
            launch_arguments={
                'world': world,
                'use_sim_time': use_sim_time,
                'namespace': namespace,
            }.items(),
        ),

        # Optionally include RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'config': world,
                'use_sim_time': use_sim_time,
                'namespace': namespace,
            }.items(),
        ),

        # Launch your 5 robot controllers
        *[
            Node(
                package='collective_robotics',
                executable='multi_bot_task3_aggregate',
                name=f'{robot}_swarm_aggregator',
                output='screen',
                parameters=[{'robot_name': robot}]
            ) for robot in robots
        ]
    ])

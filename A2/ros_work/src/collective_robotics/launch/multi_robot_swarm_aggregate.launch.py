from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def add_robot_nodes(context, *args, **kwargs):
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    robot_nodes = []

    for i in range(robot_count):
        robot_name = f"robot_{i}"
        robot_nodes.append(
            Node(
                package='collective_robotics',
                executable='multi_bot_task3_aggregate',  
                name=f'{robot_name}_swarm_aggregator',
                output='screen',
                parameters=[
                    {'robot_name': robot_name},
                    {'robot_count': robot_count}
                ]
            )
        )
    return robot_nodes

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='cave_multi')
    stage = LaunchConfiguration('stage', default='True')
    rviz = LaunchConfiguration('rviz', default='False')
    namespace = LaunchConfiguration('namespace', default='')
    robot_count = LaunchConfiguration('robot_count', default='5')

    declare_world = DeclareLaunchArgument('world', default_value='cave_multi')
    declare_stage = DeclareLaunchArgument('stage', default_value='True')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='False')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='')
    declare_robot_count = DeclareLaunchArgument('robot_count', default_value='5')

    stage_ros2_dir = get_package_share_directory('stage_ros2')
    launch_dir = os.path.join(stage_ros2_dir, 'launch')

    return LaunchDescription([
        declare_world,
        declare_stage,
        declare_rviz,
        declare_namespace,
        declare_robot_count,

        # Stage simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'stage.launch.py')),
            launch_arguments={
                'world': world,
                'use_sim_time': use_sim_time,
                'namespace': namespace,
            }.items(),
        ),

        # RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(rviz),
            launch_arguments={
                'config': world,
                'use_sim_time': use_sim_time,
                'namespace': namespace,
            }.items(),
        ),

        # Dynamically generated robot nodes
        OpaqueFunction(function=add_robot_nodes),
    ])

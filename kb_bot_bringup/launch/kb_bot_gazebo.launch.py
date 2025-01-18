import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    description_pkg = 'kb_bot_description'
    bringup_pkg = 'kb_bot_bringup'

    xacro_file_path = os.path.join(
        FindPackageShare(package=description_pkg).find(description_pkg),
        'urdf',
        'kb_bot.urdf.xacro'
    )

    gazebo_launch_file_path = os.path.join(
        FindPackageShare(package='gazebo_ros').find('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    rviz_config_file_path = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'rviz',
        'rviz_config2.rviz'
    )

    world_path = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'worlds',
        'turtlebot3_world'
    )

    robot_description_config = Command(['xacro ', xacro_file_path])

    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file_path),
            launch_arguments={
                'world': world_path,
            }.items(),
    )

    gazebo_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
        '-topic', '/robot_description',
        '-entity', 'kb_bot'
        ]
    )

    robot_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config, 'use_sim_time': use_sim_time}],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{"robot_description": robot_description_config, 'use_sim_time': use_sim_time}],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file_path],
        condition = IfCondition(use_rviz)
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name = 'use_sim_time',
                                        default_value='true',
                                        description='Use sim time if true'
                                        ))
    ld.add_action(DeclareLaunchArgument(name = 'use_rviz',
                                        default_value='true',
                                        description='Uses rviz2 if true'
                                        ))
    
    ld.add_action(robot_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(gazebo_launch)
    ld.add_action(gazebo_spawn_entity)
    ld.add_action(rviz2_node)
        
    return ld
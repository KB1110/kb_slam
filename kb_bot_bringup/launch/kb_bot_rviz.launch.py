import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joint_gui = LaunchConfiguration('joint_gui')

    description_pkg = 'kb_bot_description'
    bringup_pkg = 'kb_bot_bringup'

    xacro_file_path = os.path.join(
        FindPackageShare(package=description_pkg).find(description_pkg),
        'urdf',
        'kb_bot.urdf.xacro'
    )

    rviz_config_file_path = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'rviz',
        'rviz_config.rviz'
    )

    robot_description_config = Command(['xacro ', xacro_file_path])

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
        condition = UnlessCondition(joint_gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition = IfCondition(joint_gui)
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_file_path]
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name = 'joint_gui', 
                                        default_value='true',
                                        description="This is flag for joint_state_publisher_gui"
                                        ))
    ld.add_action(DeclareLaunchArgument(name = 'use_sim_time',
                                        default_value='false',
                                        description='Use sim time if true'
                                        ))
    
    ld.add_action(robot_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz2_node)
        
    return ld
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
        FindPackageShare(package='ros_gz_sim').find('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    rviz_config_file_path = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'rviz',
        'rviz_config2.rviz'
    )

    world_path = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'worlds',
        'depot_world.sdf'
    )

    bridge_params = os.path.join(
        FindPackageShare(package=bringup_pkg).find(bringup_pkg),
        'params',
        'kb_bot_gz_bridge.yaml'
    )

    robot_description_config = Command(['xacro ', xacro_file_path])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_file_path
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world_path], 
        'on_exit_shutdown': 'true'}.items()
    )

    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "kb_bot",
            '-topic', '/robot_description',
            '-x', "0.0",
            '-y', "0.0",
            '-z', '1.0'
        ],
        output='screen',
    )

    start_gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Use this when you are using the camera
    # start_gazebo_ros_image_bridge = Node(
        # package='ros_gz_image',
        # executable='image_bridge',
        # arguments=['/camera/image_raw'],
        # output='screen',
    # )

    robot_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config, 'use_sim_time': use_sim_time}],
        output="screen"
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
    ld.add_action(gazebo_launch)
    ld.add_action(gazebo_spawn_entity)
    ld.add_action(start_gazebo_ros_bridge)
    # ld.add_action(start_gazebo_ros_image_bridge)
    ld.add_action(rviz2_node)
        
    return ld
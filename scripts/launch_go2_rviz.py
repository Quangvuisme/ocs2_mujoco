#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the directory path - scripts are in /scripts/, robots are in /robots/
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(scripts_dir)  # Parent of scripts/
    robot_dir = os.path.join(workspace_dir, 'robots', 'go2')
    urdf_file = os.path.join(robot_dir, 'go2_urdf', 'urdf', 'go2.urdf')
    mesh_dir = os.path.join(robot_dir, 'go2_urdf', 'dae')
    
    # Read URDF file and replace relative paths with absolute paths
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Replace relative mesh paths with absolute paths
    robot_description = robot_description.replace('filename="../dae/', f'filename="file://{mesh_dir}/')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 Node with snap fix
    rviz_config_file = os.path.join(scripts_dir, 'rviz', 'go2.rviz')
    
    # Fix for snap rviz2 conflict
    rviz_env = os.environ.copy()
    rviz_env['LD_PRELOAD'] = ''
    
    from launch.conditions import IfCondition
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        additional_env=rviz_env,
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

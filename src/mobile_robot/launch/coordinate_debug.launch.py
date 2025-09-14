#!/usr/bin/env python3
"""
Debug launch file for testing coordinate frame alignment fixes
This launches minimal nodes for testing without the full simulation
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_mobile_robot = get_package_share_directory('mobile_robot')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # EKF configuration files
    ekf_local_yaml = os.path.join(pkg_mobile_robot, 'parameters', 'ekf_local.yaml')
    navsat_yaml = os.path.join(pkg_mobile_robot, 'parameters', 'navsat.yaml')
    ekf_global_yaml = os.path.join(pkg_mobile_robot, 'parameters', 'ekf_global.yaml')
    
    # Static transforms for coordinate frame alignment
    tf_map_to_utm = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_utm',
        arguments=[
            '-332254.927093',
            '-4128604.136462',
            '0',
            '0', '0', '0',
            'map',
            'utm'
        ],
        output='screen',
    )
    
    # Orientation correction transform
    tf_map_orientation_correction = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_orientation_correction',
        arguments=[
            '0', '0', '0',
            '0', '0', '2.8',  # Northwestern orientation offset
            'map_raw',
            'map'
        ],
        output='screen',
    )
    
    # EKF local (wheel + IMU)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/filtered_local')],
    )
    
    # NavSat transform
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        remappings=[
            ('/imu', '/imu/with_cov'),
            ('/imu/data', '/imu/with_cov'),
            ('/gps/fix', '/gps/fix'),
            ('/odometry/filtered', '/odometry/filtered_local'),
            ('/odometry/gps', '/gps/odom'),
        ],
        parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
    )
    
    # EKF global (GPS + local EKF)
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/filtered_map')],
    )
    
    # Debug and validation nodes
    frame_debug = Node(
        package='mobile_robot',
        executable='frame_alignment_debug.py',
        name='frame_alignment_debug',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    coordinate_validation = Node(
        package='mobile_robot',
        executable='coordinate_validation_test.py',
        name='coordinate_validation_test',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    wheel_diagnostic = Node(
        package='mobile_robot',
        executable='wheel_odom_diagnostic.py',
        name='wheel_odom_diagnostic',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # TF tree visualization
    tf_tree = Node(
        package='tf2_tools',
        executable='view_frames',
        name='view_frames',
        output='screen',
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        tf_map_to_utm,
        tf_map_orientation_correction,
        ekf_local,
        navsat,
        ekf_global,
        frame_debug,
        coordinate_validation,
        wheel_diagnostic,
        # tf_tree,  # Uncomment to generate TF tree visualization
    ])
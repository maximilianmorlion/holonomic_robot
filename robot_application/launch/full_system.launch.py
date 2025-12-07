#!/usr/bin/env python3
"""Launch all application nodes for full autonomous operation."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='patrol',
        description='Mission type: patrol, pick_place, or teleop'
    )
    
    # Include Nav2 bringup (assuming it's in robot_bringup)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('holonomic_robot_bringup'),
                'launch',
                'navigation.launch.py'
            ])
        ])
    )
    
    # Include hardware bridge (STM32 communication)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_hardware_cpp'),
                'launch',
                'stm32_bridge.launch.py'
            ])
        ])
    )
    
    # Servo controller node
    servo_controller_node = Node(
        package='robot_actuators',
        executable='servo_controller',
        name='servo_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_actuators'),
                'config',
                'servo_controller.yaml'
            ])
        ]
    )
    
    # Pump controller node
    pump_controller_node = Node(
        package='robot_actuators',
        executable='pump_controller',
        name='pump_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_actuators'),
                'config',
                'pump_controller.yaml'
            ])
        ]
    )
    
    return LaunchDescription([
        mission_type_arg,
        nav2_launch,
        hardware_launch,
        servo_controller_node,
        pump_controller_node
        # Mission-specific nodes would be launched separately
    ])

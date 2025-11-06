#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Gazebo-ROS bridges for nala2_boat model.
    Bridges thrust commands and angular velocities for left and right propellers.
    """
    
    # Declare launch arguments
    model_name = LaunchConfiguration('model_name', default='nala2_boat')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'model_name',
            default_value='nala2_boat',
            description='Name of the boat model in Gazebo'
        ),
        
        # bow left engine thrust command bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bow_left_thrust_bridge',
            arguments=[
                '/model/nala2_boat/joint/bow_left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            output='screen'
        ),
        
        # bow right engine thrust command bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bow_right_thrust_bridge',
            arguments=[
                '/model/nala2_boat/joint/bow_right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            output='screen'
        ),

        # Stern left engine thrust command bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='stern_left_thrust_bridge',
            arguments=[
                '/model/nala2_boat/joint/stern_left_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            output='screen'
        ),

        # Stern right engine thrust command bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='stern_right_thrust_bridge',
            arguments=[
                '/model/nala2_boat/joint/stern_right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            output='screen'
        ),
        
        # Stern left engine rotate angle bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='stern_left_rotate_bridge',
            arguments=[
                '/model/nala2_boat/joint/stern_left_chasis_engine_joint/rotate_angle@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            output='screen'
        ),

        # Stern right engine rotate angle bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='stern_right_rotate_bridge',
            arguments=[
                '/model/nala2_boat/joint/stern_right_chasis_engine_joint/rotate_angle@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            output='screen'
        ),
    ])

import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_directory('asv_model'), 'urdf', 'nala.urdf.xacro')
    asv_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': asv_description}]
    )

    rviz_config_path = os.path.join(get_package_share_directory('rviz_plugin'), 'rviz', 'manta.rviz')
    rviz_plugin_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_plugin_node,
    ])

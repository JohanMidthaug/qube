import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to xacro file
    xacro_file = os.path.join(get_package_share_directory('qube_description'), 'urdf', 'qube.urdf.xacro')

    # Process the xacro file
    joint_description_content = xacro.process_file(xacro_file).toxml()
    
    
    return LaunchDescription([
        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory("qube_description"), 
                'config', 
                'qube_config.rviz')]),
              
        # Start Robot State Publisher
        Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher_description',
           output='screen',
           parameters=[{'robot_description': joint_description_content}]),
    ])


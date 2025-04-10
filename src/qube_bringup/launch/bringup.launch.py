import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    baud_rate_arg = DeclareLaunchArgument("baud_rate", default_value="115200", description="Serial baud rate")
    device_arg = DeclareLaunchArgument("device", default_value="/dev/ttyACM0", description="USB device")
    simulation_arg = DeclareLaunchArgument("simulation", default_value="true", description="Simulation mode on/off")

    # Path to xacro file
    xacro_file = os.path.join(
        get_package_share_directory('qube_bringup'),
        'urdf',
        'controlled_qube.urdf.xacro'
    )

    # Include the qube_driver launch file
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('qube_driver'), 'launch', 'qube_driver.launch.py')
        )
    )

    # Define a launch_setup function, which packs inn all the necessary procedures to run neatly,
    # and is also able to be passed arguments to at runtime
    def launch_setup(context, *args, **kwargs):
        # Evaluate launch configurations at runtime
        baud_rate = LaunchConfiguration('baud_rate').perform(context)
        device = LaunchConfiguration('device').perform(context)
        simulation = LaunchConfiguration('simulation').perform(context)

        # Process xacro file with runtime arguments
        joint_description_content = xacro.process_file(
            xacro_file,
            mappings={
                "baud_rate": baud_rate,
                "device": device,
                "simulation": simulation
            }
        ).toxml()

        return [
            # Include driver, through the launch file we created
            qube_driver_launch,

            # Start QUBE controller node
            Node(
                package='qube_controller',
                executable='qube_controller_node',
                name='qube_controller',
                parameters=[{'robot_description': joint_description_content}]
            ),

            # Start RViz, with our configurations
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(
                    get_package_share_directory("qube_description"),
                    'config',
                    'qube_config.rviz')]
            ),

            # Start Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher_bringup',
                output='screen',
                parameters=[{'robot_description': joint_description_content}]
            ),
        ]

    return LaunchDescription([
        baud_rate_arg,
        device_arg,
        simulation_arg,
        # Use opaque function, so that the launch doesn't proceed until all launch arguments have been resolved
        OpaqueFunction(function=launch_setup)
    ])

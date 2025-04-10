
# Mini-project in AIS2105 Mechatronics & robotics.

This repository is a complete ros2 project which does the following:

- Control a Quanser Qube with ROS2
- Has a working simulator
- A Launch file to switch between the physical qube and the simulator
- Visualization through Rviz
- Control angle thorugh terminal and/or GUI

# Documentation of project

This project is split into clearly defined packages, each handling a specific part of the project.

- **qube_description**
qube_description handles the geomertical description of the physical qube.This package contains two URDF-files, called `qube.macro.xacro`, which contains a macro which describes the physical qube. The other URDF-file is `qube.urdf.xacro`, which contains a scene of the qube. The reason for splitting up into two URDF-files is reusability. This package also contains a launch file, `view_qube_launch.py`.

- **qube_driver**
The qube_driver contains the communication interface with the physical qube.
The qube_driver package is documented here: https://github.com/adamleon/qube_driver/tree/main

- **qube_bringup**
This package includes launch and config files to bridge to whole project together. In this package, there is a new URDF-file, `controlled_qube.urdf.xacro`, which is the same as `qube.urdf.xacro`, but it also includes the `qube_driver.ros2_control.xacro file`. The `controlled_qube.urdf.xacro` makes it possible to define three macro arguments, `baud_rate`, `device` & `simulation`. There is also a launch file called `bringup.launch.py`, which starts up `qube_driver.launch.py`, rvizz and robot state controller.

- **qube_controller**
The qube_controller includes a node that subscribes to `/joint_states`, and gets position and velocity to the qube. Thereafter there is a PID controller that regulates and outputs a velocity signal. This velocity signal is a Float64MultiArray which gets sent to `/velocity_controller/commands`.

# How to run
To run this project, git pull or download the zipped version of the project. Then colcon build, source install, and run the command: `ros2 launch qube_bringup bringup.launch.py`. If you want to change the startup parameters, add `simulation="false"`.

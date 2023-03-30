# Kuka_LBR_iiwa
This repository contains the code and files for the Kuka LBR iiwa robotic arm. The robotic arm's forward kinematics, inverse kinematics, and workspace plotting are implemented in Python. The package "kuka_cobot" contains the teleop script used to control the robotic arm in the Gazebo environment.

# Code Files
1. KUKA_FK.py: This file outputs the general transformation matrices and prints the final transformation matrices for four poses used for FK validation.
2. KUKA_7X7_jacobian.py: This file prints out the 7X7 General Jacobian Matrix and 7X7 Jacobian for the pose [90, 0, 0, -90, 0 , 90, 0] before drawing the circle for IK validation.
3. KUKA_IK.py: This file gives the 6X6 Jacobian matrix for the initial pose [90, 0, 0, -90, 0 , 90, 0] before drawing and gives the circle of 10 cm drawn on XY plane as the output, and hence validates the IK of the KUKA cobot.
4. workspace_plot.py: This file gives the workspace plot of the KUKA Cobot. It takes time to plot as we are plotting more than 2000 points.

# Package
The package "kuka_cobot" contains the teleop script used to control the robotic arm in the Gazebo environment. 
The package has been built successfully and contains the teleop_KUKA.py teleop script.

# Launch Files
template__kuka_launch.launch: This launch file is used to launch the Kuka cobot in the Gazebo environment. 
Simultaneously, it launches Rviz to visualize the camera feed. A custom world has been created to mimic the factory floor.

# CAD Model
The CAD Model folder contains the assembly and parts of the Kuka LBR iiwa robotic arm.

# Demonstration Videos
1. Gazebo_and_Rviz_successful_launch.mp4: This video shows the successful launch of the submitted package in the Gazebo environment. 
Rviz has been launched, and the camera feed is being streamed.
2. Pick_and_Place_operation.mp4: This video shows the robotic arm being controlled using teleop. 
The arm goes to the pick place and then takes the arm to the drop place. The pick and drop places are confirmed by referring to the camera feed from Rviz.

# Report
The report contains details about the forward kinematics and inverse kinematics, along with the validation of both.

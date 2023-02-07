# Kuka_LBR_iiwa

1. There are four python files in the code foler:

	1. KUKA_FK.py : It will output the all the general transformation matrices and also prints the final transformation matrices for four poses we used to FK validation.

	2. KUKA_7X7_jacobian.py : It will print out the 7X7 General Jacobian Matrix as well as 7X7 Jacobian for the pose [90, 0, 0, -90, 0 , 90, 0] before drawing the circle        for IK validation.

	3. KUKA_IK.py : It will give the 6X6 Jacobian matrix for the initial pose [90, 0, 0, -90, 0 , 90, 0] before drawing and also gives the circle of 10 cm drawn on XY plane      as the output, and hence validates the IK of the KUKA cobot.

	4. workspace_plot.py: This scripts gives the workspace plot of the KUKA Cobot, but this takes time to plot as we are plotting more than 2000 points. We added the output      plot of this program in the report.

2. In folder named "Project_workspace_package" you can find the project package "kuka_cobot" in src folder.

3. The package is built successfully and contains teleop_KUKA.py teleop script in the project2_ws_achavan1_krishnah ->src ->kuka_cobot->src folder (Using python3). 
   The instructions to use the teleop will be displayed once you launch the teleop.
   
4. Launch "template__kuka_launch.launch" to launch the kuka cobot in the Gazebo environment, simultaneously launch Rviz to visualise the camera feed. We created our         own world, to mimic the factory floor.

5. Final demonstration videos of tasks:

	1. Gazebo_and_Rviz_successful_launch.mp4 : We launched the submitted package in the Gazebo, then launched Rviz and streamed the camera feed. This video shows the          successful launch of the submitted package.
		
          LINK: https://drive.google.com/file/d/1t82u_cCCnZdAm0vJG0dwsVCfcxsx7XGa/view?usp=share_link

	2. Pick_and_Place_operation.mp4: We controlled the cobot arm using teleop and showed that the arm goes to the pick place and then took the arm to the drop place. The      pick and drop places confirmed by refering the camera feed from Rviz.
	   
	   LINK : https://drive.google.com/file/d/1kvMZsTBaxkdASuH5KDzIhSfBdr_wc2f9/view?usp=share_link
     
6. The details about the forward kinematics and inverse kinematics along with the validation of the both can be found in the report.

7. In the CAD Model folder, we submitted the assembly and the parts of the cobot.

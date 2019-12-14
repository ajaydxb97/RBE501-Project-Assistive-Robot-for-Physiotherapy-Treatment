# RBE501 Project: Assistive Robot System for Upper Limb Physiotherapy Treatment of Stroke Patients
This is a course project for Fall 2019 RBE501 Robot Dynamics at Worcester Polytechnic Institute

The goal of this project is to develop a robot system that assists stroke patients in performing physiotherapy exercises for the arm by guiding them back to the the desired exercise trajectory when they divert. We plan to build this system in simulation using the Asynchronous Multi-Body Framework (AMBF). We model a human arm and use the 7-degree of freedom (DOF) KUKA as the assistive robot.

## Experiment of Applying Disturbances to Kuka Moving Along a Trajectory
In this experiment the Kuka arm moves across the desired trajectory of a line segment along the y-axis. We apply disturbances to the arm using the mouse. As we can see the Kuka continues to move along the desired trajectory despite experiencing diverting forces. A human arm model should have been performing the application of diverting forces, but we were facing a lot of complicated issues when we tried attaching the human arm to the Kuka. As a result, we decided to work on it at a later time as part of future work.

<img src="https://github.com/ajaydxb97/RBE501-Project-Assistive-Robot-for-Physiotherapy-Treatment/blob/master/Images/y-axis_test.gif" align="middle" width=50% height=50%>

## Final Human Arm Model
The model of the final human arm created using Blender and spawned in AMBF is shown below:
<img src="https://github.com/ajaydxb97/RBE501-Project-Assistive-Robot-for-Physiotherapy-Treatment/blob/master/Images/Human_arm_final.png" width=30% height=30%>

## Final Kuka Arm Model
The model of the final Kuka arm modified by adding a cylindrical extension using Blender and spawned in AMBF is shown below:

<img src="https://github.com/ajaydxb97/RBE501-Project-Assistive-Robot-for-Physiotherapy-Treatment/blob/master/Images/kuka_model.png" width=20% height=20%>

## Trajectories Used
We generated a line segment trajectory for the Kuka to follow along the z-axis and y-axis as the figures below show:

<img src="https://github.com/ajaydxb97/RBE501-Project-Assistive-Robot-for-Physiotherapy-Treatment/blob/master/Images/kuka_z.png" width=30% height=30%>              <img src="https://github.com/ajaydxb97/RBE501-Project-Assistive-Robot-for-Physiotherapy-Treatment/blob/master/Images/kuka_y.png" width=30% height=30%>

## Inverse Dynamics Control of the Kuka Arm
We implemented task space inverse dynamics control of the Kuka arm using the following flow.
<img src="https://github.com/ajaydxb97/RBE501-Project-Assistive-Robot-for-Physiotherapy-Treatment/blob/master/Images/kuka_control.png" width=30% height=30%>

## Future Work
We faced many issues when we tried to attach the human arm to the Kuka. A relativly unstable combination of the two was achieved but no trajectory traversal could be performed using it. We tried changing the collision parameters, mesh type etc. but didn't overcome the problems. This is an opportunity for future work and whoever wants to continue this project.

<img src="https://github.com/ajaydxb97/RBE501-Project-Assistive-Robot-for-Physiotherapy-Treatment/blob/master/Images/Gripper_attachment.png" width=20% height=20%>

## Acknowledgment
We thank Professor Gregory S. Fischer for his advice and members of the WPI AIM lab for their support without which we would not have been able to complete this project. 

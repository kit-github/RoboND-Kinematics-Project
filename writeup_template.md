## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Foward Kinematics Example 1 ![forward kinematics](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/forward_kinematics.png)
Foward Kinematics Example 2![forward kinematics2](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/forward_kinematics_2.png)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i)   | theta(i)
---   | ---        | ---    | ---    | ---
0->1  | 0          | 0      | 0.75   | q1
1->2  | -pi/2      | 0.35   | 0      | q2 -pi/2 
2->3  | 0          | 1.25   | 0      | q3
3->4  | -pi/2      |-0.054  | 1.50   | q4
4->5  | pi/2       | 0      | 0      | q5
5->6  | -pi/2      | 0      | 0      | q6
6->EE | 0          | 0      | 0.303  | 0

Individual transformations:
T0_1 = Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])

T1_2 =
Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])


T2_3 =
Matrix([
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,    0],
[      0,        0, 0,    1]])


T3_4=
Matrix([
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]])

T4_5=
Matrix([
[cos(q5), -sin(q5),  0, 0],
[      0,        0, -1, 0],
[sin(q5),  cos(q5),  0, 0],
[      0,        0,  0, 1]])

T5_6=
Matrix([
[ cos(q6), -sin(q6), 0, 0],
[       0,        0, 1, 0],
[-sin(q6), -cos(q6), 0, 0],
[       0,        0, 0, 1]])

T6_G =
Matrix([
[1, 0, 0,     0],
[0, 1, 0,     0],
[0, 0, 1, 0.303],
[0, 0, 0,     1]])

Values for zero configuration that is {q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}
('T0_1 = ', Matrix([
[1.0,   0,   0,    0],
[  0, 1.0,   0,    0],
[  0,   0, 1.0, 0.75],
[  0,   0,   0,  1.0]]))

('T0_2 = ', Matrix([
[  0, 1.0,   0, 0.35],
[  0,   0, 1.0,    0],
[1.0,   0,   0, 0.75],
[  0,   0,   0,  1.0]]))

('T0_3 = ', Matrix([
[  0, 1.0,   0, 0.35],
[  0,   0, 1.0,    0],
[1.0,   0,   0,  2.0],
[  0,   0,   0,  1.0]]))

('T0_4 = ', Matrix([
[  0,    0, 1.0,  1.85],
[  0, -1.0,   0,     0],
[1.0,    0,   0, 1.946],
[  0,    0,   0,   1.0]]))

('T0_5 = ', Matrix([
[  0, 1.0,   0,  1.85],
[  0,   0, 1.0,     0],
[1.0,   0,   0, 1.946],
[  0,   0,   0,   1.0]]))

('T0_6 = ', Matrix([
[  0,    0, 1.0,  1.85],
[  0, -1.0,   0,     0],
[1.0,    0,   0, 1.946],
[  0,    0,   0,   1.0]]))

('T0_G = ', Matrix([
[  0,    0, 1.0, 2.153],
[  0, -1.0,   0,     0],
[1.0,    0,   0, 1.946],
[  0,    0,   0,   1.0]]))

T0_total is with the rotational correction for the grip 
('T0_total = ', Matrix([
[1.0,   0,   0, 2.153],
[  0, 1.0,   0,     0],
[  0,   0, 1.0, 1.946],
[  0,   0,   0,   1.0]]))


Generalized homogenous transform between base_link and gripper link using only end-effector pose is of the form
T = [R_t      | P_vector]
    [0_vector | 1       ]
Px, Py, Pz = [-1.3863, 0.02074, 0.90986]
orientation_euler = [0.01735, -0.2179, 0.9025, 0.371016]
[row, pitch, yaw] = [-0.39818863537626115, -0.19423603941828343, 2.400827937066692]

Rotation Matrix from end effector
Matrix([
[-0.130379773257434, 0.677285271388215, -0.724075808107087,  -1.38628179843151],
[-0.406207902327869, 0.629711423593225,  0.662162112388339, 0.0207428060324226],
[ 0.904431453904459, 0.380457861210391,  0.193016996742467,  0.909881507038079],
[                 0,                 0,                  0,                1.0]])


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


since the last 3 joints are revolute and their axes intersect at a single point we have a case of spherical wrist with joint_5 being the wrist center. Spherical wrist allows us to kinematically decouple Inverse kinematics into inverse position and inverse orientation. That is instead of solving 12 non-linear equations, it is now possible to independently solve 2 simpler problems -- 1. cartesian coordiates of the wrist center  2. composition of the rrotations to orient the end effector


1. Compute the end-effector pose with respect to the base_link that is R_rpy (using the correctional rotational matrix)
   R_rpy = rot(z,yaw) * rot(y, pitch) * rot(x, roll) * R_corr.

   We get the yaw, pitch and roll from the euler_from_quaternions() function

2. Get the nx, ny, nz from the R_rpy. 
3. px, py, pz is obtained from the gripper position.
4. using this info and substituting for d6 and l





And here's where you can draw out and show your math for the derivation of your theta angles. 
Figure and Derivation ![drawing](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/notes_derivation.png))

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]



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

Foward Kinematics Example 1 (recreate the gripper position/pose given in forward kinematics figure as a test FK is working)  ![forward kinematics](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/forward_kinematics.png)
Forward Kinematics Example 2 ![forward kinematics2](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/forward_kinematics_2.png)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

##### DH Parameters
To get the DH parameters. 

1. Label all the joints
2. labels all links starting from fixed base link as 0.
3. Draw lines through joints and define joint axes
4. Assign z-axes for each frame to point along the joint axis.
5. Identify common normal between Z_{i-1} and Z_{i}
Allocate the X axis based on the DH parameter rules. Please see figure for more details.

DH parameters and axis allocation ![DH parameters](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/dh_parameter_hand.png)

DH parameters table derivation ![DH parameters](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/dh_parameter_table_hand.png)


Links | alpha(i-1) | a(i-1) | d(i)   | theta(i)
---   | ---        | ---    | ---    | ---
0->1  | 0          | 0      | 0.75   | q1
1->2  | -pi/2      | 0.35   | 0      | q2 -pi/2 
2->3  | 0          | 1.25   | 0      | q3
3->4  | -pi/2      |-0.054  | 1.50   | q4
4->5  | pi/2       | 0      | 0      | q5
5->6  | -pi/2      | 0      | 0      | q6
6->EE | 0          | 0      | 0.303  | 0

###### Individual transformations:
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

##### Values for transform matrixes from base frame for the zero configuration: {q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}
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

T0_total is the total transformation matrix with the rotational correction for the grip: 

('T0_total = ', Matrix([
[1.0,   0,   0, 2.153],
[  0, 1.0,   0,     0],
[  0,   0, 1.0, 1.946],
[  0,   0,   0,   1.0]]))


Generalized homogenous transform between base_link and gripper link using only end-effector pose is of the form
T = Matrix([
[R_t      | P_vector],
[0_vector | 1       ],
])

##### End-effector position and pose
P_vector = Px, Py, Pz = [-1.3863, 0.02074, 0.90986]

orientation_euler = [0.01735, -0.2179, 0.9025, 0.371016]

[roll, pitch, yaw] = [-0.39818863537626115, -0.19423603941828343, 2.400827937066692]

##### Rotation Matrix from end effector:

Matrix([
[-0.130379773257434, 0.677285271388215, -0.724075808107087,  -1.38628179843151],
[-0.406207902327869, 0.629711423593225,  0.662162112388339, 0.0207428060324226],
[ 0.904431453904459, 0.380457861210391,  0.193016996742467,  0.909881507038079],
[                 0,                 0,                  0,                1.0]])


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Since the last 3 joints are revolute and their axes intersect at a single point, we have a case of spherical wrist with joint_5 being the wrist center. This allows us to kinematically decouple Inverse kinematics into inverse position and inverse orientation. That is instead of solving 12 non-linear equations, it is now possible to independently solve 2 simpler problems -- 1. cartesian coordiates of the wrist center  2. composition of the rrotations to orient the end effector

##### Computing wrist center from end-effector pose and location 
1. Compute the end-effector pose with respect to the base_link that is R_rpy (using the correctional rotational matrix)
   R_rpy = rot(z,yaw) * rot(y, pitch) * rot(x, roll) * R_corr. roll, pitch, yaw are known from the end-effector pose. 
2. Get the nx, ny, nz component of the z-axis of the end effector from the R_rpy. R_rpy[:,2]
3. px, py, pz is obtained from the gripper position which is known. 
4. Using this information and substituting d6 and l we can get the wx, wy, wz.

#####  Computing theta1, theta2, theta3 from wrist center 
Once we get the wrist center wx, wy, wz we can compute the joint angles theta1, theta2, theta3. There is no clear strategy but we can use trignometry to project the 3D configuration in different subspace x-z, x-y and y-z space. For solving this problem we used the following method. Please see figure below for more details. 
Figure for angle derivation ![forward kinematics](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/hand_figure.png)

The wrist center and the end-effector is wrt to the base frame '0' that is 0_r_{WC/0} and 0_r_{EE/0} respectively. Since the joint1 only rotates around the z axis, theta1 only effects the end effector x and y coordinates. We subtract the x and z shift to get the x, y, z location of wrist center with respect to joint_2 and joint_3. Once this is done, we get the schematic shown in figure (taken from course). 

##### Supporting figures 

Schematic for angle computation ![schematic angle](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/angles_figure_from_course.png)

Derivation Figure 1 ![hand_drawing1](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/angle_derivation_hand.png))

##### Derivation 
wx, wy, wz are coordinates with respect to base link.
Once we have the sides A, B and C it is easy to compute the angle theta2, theta3. 
C = 1.25 which is the link length from DH parameter table between joint 2 and 3. 
A = 1.500971 which is equal to sqrt(a3^2+d4^2), where a3 and d4 are from the DH parameter
wc_xy_joint2 = sqrt((wc_x^2 + wc_y^2)) - a1 , where a1 = 0.35 from DH parameter 
wc_z_joint2 = wz - d1 , where d1 = 0.75
B = sqrt(wc_xy_joint2^2 + wc_z_joint2^2), where wc_xy_joint2, wc_z_joint2 is the distance of wc in xy and z axis

Once all the sides are known, angle_a, angle_b and angle_c can be computed using cosine law. 
angle_a = cos_angle(side_a, side_b, side_c)
angle_b = cos_angle(side_b, side_a, side_c)
angle_c = cos_angle(side_c, side_a, side_b)

Once these angles are known. 
#####  theta1
theta1 = atan2(wx,wy)

#####  theta2 
theta2 is the angle wrt to the y axis and is given by 
theta2 = pi/2 - angle_a - atan2(wc_z_joint2, wc_xy_joint2). 

#####  theta3
theta3 =  pi/2 - (angle_b - angle_3_4), where angle_3_4 is given by  angle_3_4 = atan2(a3,d4). a3=-0.054 and d4=1.50
Here we subtract angle_3_4 because in zeroth configuration we have that angle. 



#####  Computing theta4, theta5, theta6 from R3_6 
 1. We first compute R3_6. Once theta1, theta2, theta3 is known, we can compute R0_3 and multiply its inv(R0_3) with R_rpy to get R3_6. 
 2. Compute the euler angles corresponding to the matrix R3_6. This can be done by realizing that the pose is a composition of the following R3_6 = rot_x(-pi/2) * rot_z(q4) * rot_y(q5) * rot_x(pi/2) * rot_z(q6) * R_corr


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

##### Run1 
Screen shot run1 (in between) ![hand_drawing1](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/drop1.png))
Screen shot run1 (target drop) ![hand_drawing1](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/drop1_final.png))

##### Run2 
Screen shot run2 (in between) ![hand_drawing1](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/drop2.png))
Screen shot run2 (target drop) ![hand_drawing1](https://github.com/kit-github/RoboND-Kinematics-Project/blob/master/data/writeup/drop2_final.png))

Did some optimization. Took part of the computation that are common and static out of the for loop to speed computation. 
The code for computing Inverse Kinematics is still somewhat slow, even after those optimizations. Further factoring the code may improve speed. 

The DH parameters and visualization was conceptually not easy. Had to revisit to get a better understanding. 

Did unit test the Inverse Kinematics part using the IK_Debug.py and made sure it works on the test cases. 


And just for fun, another example image:
![alt text][image3]



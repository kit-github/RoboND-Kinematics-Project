#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from tf import transformations
#from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
import ipdb

np.set_printoptions(precision=4)

def get_mat(alpha0_, a0_, d1_, q1_):
    """
    Function to create Rotation Matrix using DH convention

    Parameters
    -----------
    alpha0_: The rotation of link i-1 around axis X_i-1
    a0_: Distance from Zi-1 to Zi along X_i-1
    d1_: Signed distance from Xi-1 to Xi measure along Zi
    q1_: Rotation along new Zi axis (angle between Xi-1 and Xi)

    Return
    --------
    """
    mat = Matrix([
        [cos(q1_),                -sin(q1_),            0,          a0_],
        [sin(q1_)*cos(alpha0_),   cos(q1_)*cos(alpha0_), -sin(alpha0_), -sin(alpha0_)*d1_],
        [sin(q1_)*sin(alpha0_), cos(q1_)*sin(alpha0_),   cos(alpha0_), cos(alpha0_)*d1_],
        [0,                          0,               0,              1]
        ])
    return mat


def rot_x(q):
    R_x = Matrix([
        [1, 0, 0 ],
        [0,cos(q), -sin(q)],
        [0,sin(q), cos(q)]
        ])

    return R_x

  # y is special case since z and x's order changes
def rot_y(q):
    R_y = Matrix([
        [cos(q), 0, sin(q)],
        [0,1,0],
        [-sin(q), 0, cos(q)]
    ])

    return R_y

def rot_z(q):
    R_z = Matrix([
        [cos(q),-sin(q),0],
        [sin(q),cos(q),0],
        [0,0,1]
    ])

    return R_z


def inverse_kinematics(gt_gripper_position, gt_gripper_orientation_quaternion, forward_transforms):

    gripper_angles = transformations.euler_from_quaternion(gt_gripper_orientation_quaternion)
    print ("euler angles for gripper using ROS simulation: {}".format(gripper_angles))

    # inverse kinematics code
    # get the angles from the ground truth gripper position and rotation

    roll, pitch, yaw = gripper_angles
    # corrective angle for gripper frame is R_corr
    R_corr = simplify(rot_z(pi)*rot_y(-pi/2))
    Rrpy= rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
    nx, ny, nz = Rrpy[:,2]
    px, py, pz = gt_gripper_position # can also be T_total_val[1:4,4]

    d6_val = 0; l_val = 0.303
    wx = px - (d6_val + l_val)*nx
    wy = py - (d6_val + l_val)*ny
    wz = pz - (d6_val + l_val)*nz

    # rotation around the z0 axis
    rtd = 180.0/pi
    theta1 = atan2(wy,wx)*rtd

    # calculate the sides A, B, C
    side_a = 1.500971
    side_c = 1.25
    # side_b is computation of wc coordinate from joint 1. so subtract joint_0 (x and z shifts)
    wc_z_joint2 = wz - 0.75
    wc_xy_joint2 = sqrt(wx*wx + wy*wy) - 0.35
    side_b = sqrt(pow(wc_xy_joint2, 2) + pow(wc_z_joint2, 2))


    def cos_angle(opp_side, side1, side2):
       return  acos( (side1 * side1 + side2 * side2 - opp_side * opp_side)  / (2 * side1 * side2))

    def get_rot_mat(T0):
        return T0[0:3,0:3]

    def get_euler_angles_from_rot_mat_x_y_z(rot_mat):
        """
        sequence of extrinsic x, y and z is same as intrinsic with order z, y, x
        """
        alpha = atan2(rot_mat[1,0],rot_mat[0,0]) # rotation about z''
        gamma  = atan2(rot_mat[2,1], rot_mat[2,2]) # rotation about x
        beta = atan2(-rot_mat[2,0], sqrt(rot_mat[0,0]*rot_mat[0,0] + rot_mat[1,0]*rot_mat[1,0])) # rot about y
        return alpha, beta, gamma

    def get_euler_angles_from_rot_mat_z_y_z(rot_3_6):
        """
        if extrinsic rotation with rotation about the common origin o5 and
        z axis, y axis and z axis was peformed, discounting the axis shifts
        in the coordinate frames.

        rot_3_6 = simplify(rot_z(q4)*rot_y(q5)*rot_z(q6))

        rot_3_6 = [
        [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), sin(q5)*cos(q4)],
        [ sin(q4)*cos(q5)*cos(q6) + sin(q6)*cos(q4), -sin(q4)*sin(q6)*cos(q5) + cos(q4)*cos(q6), sin(q4)*sin(q5)],
        [                          -sin(q5)*cos(q6),                            sin(q5)*sin(q6),         cos(q5)]
        ]
        """
        theta6 = atan2(-rot_3_6[2,1], rot_3_6[2,0])
        theta4 = atan2(rot_3_6[1,2], rot_3_6[0,2])

    angle_a = cos_angle(side_a, side_b, side_c)
    angle_b = cos_angle(side_b, side_a, side_c)
    angle_c = cos_angle(side_c, side_a, side_b)

    theta2 = pi/2 - angle_a - atan2(wc_z_joint2, wc_xy_joint2)
    angle_3_4 = atan2(-0.054,1.50)
    # not sure how theta3 is pi/2 - angle_b
    theta3 = pi/2 - (angle_b - angle_3_4)

    T0_1 = forward_transforms['T0_1']
    T1_2 = forward_transforms['T1_2']
    T2_3 = forward_transforms['T2_3']
    q1,q2,q3 = forward_transforms['q_symbols']
    R0_3 = get_rot_mat(T0_1) * get_rot_mat(T1_2) * get_rot_mat(T2_3)
    R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

    R3_6 = R0_3.inv("LU") * Rrpy

    # Euler angles for rotation matrix
    # simplify(rot_x(-pi/2)*rot_z(q4)*rot_y(q5)*rot_x(pi/2)*rot_z(q6)*R_corr)
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
    ipdb.set_trace()
    return theta1, theta2, theta3, theta4, theta5, theta6

def main():

    alpha0, alpha1, alpha2, alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
    a0,a1,a2,a3,a4,a5,a6  = symbols('a0:7') # distance along x-1
    d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')  # offset (along z axis or y-axis or both?)
    q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8') # rotation along new z axis

    #
    #
    # Create Modified DH parameters
    #
    s = {alpha0:0,      a0: 0,     d1: 0.75,
         alpha1:-pi/2,  a1: 0.35 , d2: 0,          q2: q2-pi/2,
         alpha2:0,      a2:1.25,   d3: 0,
         alpha3:-pi/2,  a3:-0.054, d4: 1.50,
         alpha4: pi/2,  a4:0,      d5:0,
         alpha5:-pi/2,  a5:0,      d6:0,
         alpha6:0,      a6:0,      d7:.303, q7:0,
         }

    #
    # Define Modified DH Transformation matrix
    #
    #

    if 0:
      T0_1 = Matrix([
            [cos(q1),                -sin(q1),            0,          a0],
            [sin(q1)*cos(alpha0),   cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
            [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),   cos(alpha0), cos(alpha0)*d1],
            [0,                          0,               0,              1]
      ])

    T0_1 = get_mat(alpha0, a0, d1, q1)
    T0_1 = T0_1.subs(s)

    T1_2 = get_mat(alpha1, a1, d2, q2)
    T1_2 = T1_2.subs(s)

    T2_3 = get_mat(alpha2, a2, d3, q3)
    T2_3 = T2_3.subs(s)

    T3_4 = get_mat(alpha3, a3, d4, q4)
    T3_4 = T3_4.subs(s)

    T4_5 = get_mat(alpha4, a4, d5, q5)
    T4_5 = T4_5.subs(s)

    T5_6 = get_mat(alpha5, a5, d6, q6)
    T5_6 = T5_6.subs(s)

    T6_G = get_mat(alpha6, a6, d7, q7)
    T6_G = T6_G.subs(s)


    # composition of homogenous transforms
    do_simple = False
    fn_apply = simplify if do_simple == True else lambda(x):x

    ipdb.set_trace()
    T0_2 = fn_apply(T0_1 * T1_2)
    T0_3 = fn_apply(T0_2 * T2_3)
    T0_4 = fn_apply(T0_3 * T3_4)
    T0_5 = fn_apply(T0_4 * T4_5)
    T0_6 = fn_apply(T0_5 * T5_6)
    T0_G = fn_apply(T0_6 * T6_G)

    ang1 = pi; ang2 = -pi/2
    R_G_corrected_3x3 = simplify(rot_z(ang1)*rot_y(ang2))
    R_G_corrected = R_G_corrected_3x3.row_join(Matrix([[0],[0],[0]]))
    R_G_corrected = R_G_corrected.col_join(Matrix([[0,0,0,1]]))

    # Numerically evaluate transforms
    print("T0_1 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
    print("T0_2 = ", T0_2.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
    print("T0_3 = ", T0_3.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
    print("T0_4 = ", T0_4.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
    print("T0_5 = ", T0_5.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
    print("T0_6 = ", T0_6.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
    print("T0_G = ", T0_G.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))

    T_total = T0_G * R_G_corrected
    T_total_val = T_total.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0})
    print("T0_total = ", T_total_val)

    gt_gripper_position=[2.51286, 0, 1.94653]
    gt_gripper_orientation_quaternion = [0, -0.00014835, 0, 1]
    forward_transforms = {'T0_1': T0_1,
                          'T1_2': T1_2,
                          'T2_3': T2_3,
                          'q_symbols':(q1,q2,q3)}


    # ground truth position for the gripper using rviz
    gt_rot_mat = transformations.quaternion_matrix(gt_gripper_orientation_quaternion)
    print("ground truth poisiton: {} rot_mat:{} = ".format(gt_gripper_position, gt_rot_mat))
    ipdb.set_trace()
    thetas = inverse_kinematics(gt_gripper_position, gt_gripper_orientation_quaternion, forward_transforms)

    #print ("wrist centers: wx:{} wy:{} wz:{}".format(wx,wy,wz))268

    return T_total, T0_G

if __name__ == '__main__':
    main()

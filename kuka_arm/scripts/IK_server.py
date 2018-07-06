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
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2


def get_rot_mat(T0):
    return T0[0:3,0:3]


def cos_angle(opp_side, side1, side2):
    return  acos( (side1 * side1 + side2 * side2 - opp_side * opp_side)  / (2 * side1 * side2))


def get_euler_angles_from_rot_mat_x_y_z(rot_mat):
    """
    sequence of extrinsic x, y and z is same as intrinsic with order z, y, x
    """
    alpha = atan2(rot_mat[1,0],rot_mat[0,0]) # rotation about z''
    gamma  = atan2(rot_mat[2,1], rot_mat[2,2]) # rotation about x
    beta = atan2(-rot_mat[2,0], sqrt(rot_mat[0,0]*rot_mat[0,0] + rot_mat[1,0]*rot_mat[1,0])) # rot about y
    return alpha, beta, gamma


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


def inverse_kinematics(gripper_position, gripper_angles, aux_variables):

    roll, pitch, yaw = gripper_angles
    Rrpy = aux_variables['Rrpy']
    r,p,y = aux_variables['rpy_symbols']
    Rrpy = Rrpy.subs({'r':roll, 'p':pitch, 'y':yaw})

    # calculation to get the wrist center
    nx, ny, nz = R_rpy[:,2]
    px, py, pz = gripper_position # can also be T_total_val[1:4,4]
    d6_val = 0; l_val = 0.303
    wx = px - (d6_val + l_val)*nx
    wy = py - (d6_val + l_val)*ny
    wz = pz - (d6_val + l_val)*nz

    # calculate the sides A, B, C
    side_a = 1.501
    side_c = 1.25
    # side_b is computation of wc coordinate from joint 1. so subtract joint_0 (x and z shifts)
    wc_z_joint2 = (wz - 0.75)
    r_xy = sqrt(wx*wx + wy*wy) - 0.35
    side_b = sqrt(r_xy**2 + wc_z_joint2**2)


    angle_a = cos_angle(side_a, side_b, side_c)
    angle_b = cos_angle(side_b, side_a, side_c)
    angle_c = cos_angle(side_c, side_a, side_b)

    # computes theta 1 to 3
    angle_3_4 = atan2(-0.054,1.50)
    theta1 = atan2(wy,wx)
    theta2 = pi/2 - angle_a - atan2(wc_z_joint2, r_xy)
    theta3 = pi/2 - (angle_b - angle_3_4) # is this correct?

    # get the rotation from base joint 0 to joint 3
    R0_3 = aux_variables['R0_3']
    q1, q2, q3 = aux_variables['q_symbols']

    R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
    R3_6 = R0_3.T * R_rpy

    # Euler angles for rotation matrix
    # simplify(rot_x(-pi/2)*rot_z(q4)*rot_y(q5)*rot_x(pi/2)*rot_z(q6)*R_corr)
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
    wc = [wx,wy,wz]
    return thetas, wc

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        # rotation along x-1 axis

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

        if print_individual_mat:
            print('T0_1 Symbolic Matrix: {}'.format(T0_1))
            #print('T0_1 Matrix: {}'.format(T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0})))

        if 0:
            T0_4 = simplify(T0_1 * T1_2 * T2_3 * T3_4)
            print('T0_4 Matrix: {}'.format(T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0})))

        # composition of homogenous transforms
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        if 1:
            print('T0_G_eval:{}'.format(T0_G.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5:0, q6=0})))
            print('T0_G:{}'.format(simplify(T0_G))
            
        r, p, y = symbols('r p y')
        R_x = rot_x(r)
        R_y = rot_y(p)
        R_z = rot_z(y)
        # this is the correction term
        R_corr = R_z.subs(y,pi) *R_y.subs(p,-pi/2)
        Rrpy = R_z * R_y * R_x * R_corr

        R0_3 = get_rot_mat(T0_1) * get_rot_mat(T1_2) * get_rot_mat(T2_3)

        aux_variables = {'R0_3': R0_3,
                         'Rrpy': Rrpy,
                         'rpy_symbols': (r,p,y),
                         'q_symbols':(q1,q2,q3)}

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            gripper_position = [px, py, pz]
            gripper_angles = [roll, pitch, yaw]

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            print ('Processing IK request:{}'.format(x))
            thetas, wc = inverse_kinematics(gripper_position, gripper_angles, aux_variables)
            [theta1, theta2, theta3, theta4, theta5, theta6] = thetas
            [wx,wy,wz] = wc
            
            #
            # Calculate joint angles using Geometric IK method
            #
            #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

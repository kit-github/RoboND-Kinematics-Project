from sympy import *
from time import time
from mpmath import radians
import tf
from tf import transformations
import ipdb
import numpy as np

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}



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



def cos_angle(opp_side, side1, side2):
    return  acos( (side1 * side1 + side2 * side2 - opp_side * opp_side)  / (2 * side1 * side2))

def get_rot_mat(T0):
    return T0[0:3,0:3]


def get_euler_angles_from_rot_mat_x_y_z(rot_mat):
    """
    Perhaps wrong
    sequence of extrinsic x, y and z is same as intrinsic with order z, y, x
    """
    alpha = atan2(rot_mat[1,0],rot_mat[0,0]) # rotation about z''
    gamma  = atan2(rot_mat[2,1], rot_mat[2,2]) # rotation about x
    beta = atan2(-rot_mat[2,0], sqrt(rot_mat[0,0]*rot_mat[0,0] + rot_mat[1,0]*rot_mat[1,0])) # rot about y
    return alpha, beta, gamma


def inverse_kinematics(gripper_position, gripper_angles, aux_variables, test_case):
    # ground truth position for the gripper using rviz

    roll, pitch, yaw = gripper_angles
    R_EE = aux_variables['R_EE']
    r,p,y = aux_variables['rpy_symbols']
    R_EE = R_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

    # calculation to get the wrist center
    nx, ny, nz = R_EE[:,2]
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
    R3_6 = R0_3.T * R_EE

    # Euler angles for rotation matrix
    # simplify(rot_x(-pi/2)*rot_z(q4)*rot_y(q5)*rot_x(pi/2)*rot_z(q6)*R_corr)
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    # simplify
    # R3_4(y) * R4_5(z) * rotx(pi/2) * R5_6(z)
    ipdb.set_trace()
    simplify(rot_y(theta4)* rot_z(theta5)*rot_x(pi/2)*rot_z(q6))

    thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
    wc = [wx,wy,wz]

    print ('theta:{}'.format(thetas))
    print ('gt_theta:{}'.format(test_case[2]))
    print ('wc:{}'.format(wc))
    print ('gt_wc:{}'.format(test_case[1]))
    return thetas, wc


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##
    ## Insert IK code here!

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


    # composition of homogenous transforms
    do_simple = False
    fn_apply = simplify if do_simple == True else lambda(x):x

    if 0:
        T0_2 = fn_apply(T0_1 * T1_2)
        T0_3 = fn_apply(T0_2 * T2_3)
        T0_4 = fn_apply(T0_3 * T3_4)
        T0_5 = fn_apply(T0_4 * T4_5)
        T0_6 = fn_apply(T0_5 * T5_6)
        T0_G = fn_apply(T0_6 * T6_G)

    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

    ang1 = pi; ang2 = -pi/2
    R_G_corrected_3x3 = simplify(rot_z(ang1)*rot_y(ang2))
    R_G_corrected = R_G_corrected_3x3.row_join(Matrix([[0],[0],[0]]))
    R_G_corrected = R_G_corrected.col_join(Matrix([[0,0,0,1]]))

    # Numerically evaluate transforms
    if 0:
        print("T0_1 = ", T0_1.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        print("T0_2 = ", T0_2.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        print("T0_3 = ", T0_3.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        print("T0_4 = ", T0_4.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        print("T0_5 = ", T0_5.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        print("T0_6 = ", T0_6.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        print("T0_G = ", T0_G.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        T_total = T0_G * R_G_corrected


    r, p, y = symbols('r p y')
    R_x = rot_x(r)
    R_y = rot_y(p)
    R_z = rot_z(y)
    R_corr = R_z.subs(y,pi) *R_y.subs(p,-pi/2)
    R_EE = R_z * R_y * R_x * R_corr
    R0_3 = get_rot_mat(T0_1) * get_rot_mat(T1_2) * get_rot_mat(T2_3)


    aux_variables = {'R0_3': R0_3,
                     'R_EE': R_EE,
                     'q_symbols':(q1,q2,q3),
                     'rpy_symbols':(r,p,y)}

    if 0:
        gripper_position=[2.51286, 0, 1.94653]
        gt_gripper_orientation_quaternion = [0, -0.00014835, 0, 1]
        thetas, wc = inverse_kinematics(gripper_position, gt_gripper_orientation_quaternion, aux_variables)
    else:
        position_ = [position.x, position.y, position.z]
        orientation_ = [orientation.x, orientation.y, orientation.z, orientation.w ]
        (row, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_)
        gripper_angles = [row, pitch, yaw]
        thetas, wc = inverse_kinematics(position_, gripper_angles, aux_variables, test_case)

    theta1, theta2, theta3, theta4, theta5, theta6 = thetas


    #ipdb.set_trace()


    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    FK = T0_G.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
    print("T0_G = {}".format(FK))
    your_wc = wc
    your_ee = FK[0:3,3]

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    #your_wc = [wc[0],wc[1],wc[2]] # <--- Load your calculated WC values in this array
    #your_ee = my_ee # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])

    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        print ("your ee:{}".format(your_ee))
        print ("test ee:{}".format(test_case[0][0]))

        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])

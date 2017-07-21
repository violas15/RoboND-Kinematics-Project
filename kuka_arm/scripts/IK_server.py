#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
from math import *
from sympy import *
import random

# Define DH param symbols

alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
a0,a1,a2,a3,a4,a5,a6 = symbols('a1:8')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')

# Joint angle symbols
q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8') #theta_i
alpha,beta,gamma = symbols('alpha,beta,gamma')


# Modified DH params
dhParams = {alpha0:     0, a0:      0, d1:  .75, q1:      q1,
            alpha1: -pi/2, a1:    .35, d2:    0, q2: q2-pi/2,
            alpha2:     0, a2:   1.25, d3:    0, q3:      q3,
            alpha3: -pi/2, a3:  -.054, d4:  1.5, q4:      q4,
            alpha4:  pi/2, a4:      0, d5:    0, q5:      q5,
            alpha5: -pi/2, a5:      0, d6:    0, q6:      q6,
            alpha6:     0, a6:      0, d7: .303, q7:       0}


# Create individual transformation matrices
T0_1 = Matrix([
    [cos(q1),                          -sin(q1),               0,                  a0],
    [sin(q1)*cos(alpha0),   cos(q1)*cos(alpha0),    -sin(alpha0),   -sin(alpha0) * d1],
    [sin(q1)*sin(alpha0),   cos(q1)*sin(alpha0),     cos(alpha0),    cos(alpha0) * d1],
    [                  0,                     0,               0,                   1]])
T0_1 = T0_1.subs(dhParams)


T1_2 = Matrix([
    [cos(q2),                          -sin(q2),               0,                  a1],
    [sin(q2)*cos(alpha1),   cos(q2)*cos(alpha1),    -sin(alpha1),   -sin(alpha1) * d2],
    [sin(q2)*sin(alpha1),   cos(q2)*sin(alpha1),     cos(alpha1),    cos(alpha1) * d2],
    [                  0,                     0,               0,                   1]])
T1_2 = T1_2.subs(dhParams)


T2_3 = Matrix([
    [cos(q3),                          -sin(q3),               0,                  a2],
    [sin(q3)*cos(alpha2),   cos(q3)*cos(alpha2),    -sin(alpha2),   -sin(alpha2) * d3],
    [sin(q3)*sin(alpha2),   cos(q3)*sin(alpha2),     cos(alpha2),    cos(alpha2) * d3],
    [                  0,                     0,               0,                   1]])
T2_3 = T2_3.subs(dhParams)


T3_4 = Matrix([
    [cos(q4),                          -sin(q4),               0,                  a3],
    [sin(q4)*cos(alpha3),   cos(q4)*cos(alpha3),    -sin(alpha3),   -sin(alpha3) * d4],
    [sin(q4)*sin(alpha3),   cos(q4)*sin(alpha3),     cos(alpha3),    cos(alpha3) * d4],
    [                  0,                     0,               0,                   1]])
T3_4 = T3_4.subs(dhParams)


T4_5 = Matrix([
    [cos(q5),                          -sin(q5),               0,                  a4],
    [sin(q5)*cos(alpha4),   cos(q5)*cos(alpha4),    -sin(alpha4),   -sin(alpha4) * d5],
    [sin(q5)*sin(alpha4),   cos(q5)*sin(alpha4),     cos(alpha4),    cos(alpha4) * d5],
    [                  0,                     0,               0,                   1]])
T4_5 = T4_5.subs(dhParams)

T5_6 = Matrix([
    [cos(q6),                          -sin(q6),               0,                  a5],
    [sin(q6)*cos(alpha5),   cos(q6)*cos(alpha5),    -sin(alpha5),   -sin(alpha5) * d6],
    [sin(q6)*sin(alpha5),   cos(q6)*sin(alpha5),     cos(alpha5),    cos(alpha5) * d6],
    [                  0,                     0,               0,                   1]])
T5_6 = T5_6.subs(dhParams)


T6_G = Matrix([
    [cos(q7),                          -sin(q7),               0,                  a6],
    [sin(q7)*cos(alpha6),   cos(q7)*cos(alpha6),    -sin(alpha6),   -sin(alpha6) * d7],
    [sin(q7)*sin(alpha6),   cos(q7)*sin(alpha6),     cos(alpha6),    cos(alpha6) * d7],
    [                  0,                     0,               0,                   1]])
T6_G = T6_G.subs(dhParams)



# rotation matricies for gripper orientation correction
R_y = Matrix([
  [ cos(-pi/2),        0,  sin(-pi/2),  0],
  [          0,        1,        0,     0],
  [-sin(-pi/2),        0,  cos(-pi/2),  0],
  [          0,        0,           0,  1]])

R_z = Matrix([
    [ cos(pi), -sin(pi),        0,      0],
    [ sin(pi),  cos(pi),        0,      0],
    [ 0,              0,        1,      0],
    [ 0,              0,        0,      1]])

R_corr = simplify(R_z * R_y)

##Composition of transformations
print("Calculating Matricies...")

T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)
T_total = simplify(T0_G * R_corr)
T3_6 = simplify(T3_4*T4_5*T5_6)
T0_3inv = simplify(T0_1 * T1_2 * T2_3).inv()


Rrpy =Matrix([
    [cos(alpha)*cos(beta),  cos(alpha)*sin(beta)*sin(gamma)- sin(alpha)*cos(gamma),  cos(alpha)*sin(beta)*cos(gamma)+ sin(alpha)*sin(gamma),  0],
    [sin(alpha)*cos(beta),  sin(alpha)*sin(beta)*sin(gamma)+ cos(alpha)*cos(gamma),  sin(alpha)*sin(beta)*cos(gamma)- cos(alpha)*sin(gamma),   0],
    [          -sin(beta),                                    cos(beta)*sin(gamma),                                   cos(beta)*cos(gamma),   0],
    [0,0,0,1]])

RrpyCorrected = Rrpy*R_corr;


def handle_calculate_IK(req):
    
    if len(req.poses) < 1:
        print ("No valid poses received")
        return -1
    else:
        print("entering into handle ik!!")
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            rospy.loginfo(x)
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
           
            # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            wristDisplacement = dhParams.get(d7)
            alpha1 = yaw  #z
            beta1 = pitch   #y
            gamma1 = roll    #x
            
            wx = px - (dhParams.get(d6)+wristDisplacement)*RrpyCorrected[0,2].evalf(subs={alpha:alpha1,beta:beta1,gamma:gamma1})#((cos(alpha1)*sin(beta1)*cos(gamma1) + sin(alpha1)*sin(gamma1)))
            wy = py - (dhParams.get(d6)+wristDisplacement)*RrpyCorrected[1,2].evalf(subs={alpha:alpha1,beta:beta1,gamma:gamma1})#((sin(alpha1)*sin(beta1)*cos(gamma1)-cos(alpha1)*sin(gamma1)))
            wz = pz - (dhParams.get(d6)+wristDisplacement)*RrpyCorrected[2,2].evalf(subs={alpha:alpha1,beta:beta1,gamma:gamma1})#((cos(beta1)*cos(gamma1)))

            endEffPos = Matrix([px, 
                                py,
                                pz,
                                1]) #column Matrix of the end effector position

            link1 = dhParams.get(d1)
            link2 = dhParams.get(a2)
            link3 = sqrt(dhParams.get(d4)**2 + dhParams.get(a3)**2)
            th3Offset = abs(atan(dhParams.get(a3)/dhParams.get(d4)))
            a1offset = dhParams.get(a1)

            th1,th2,th3 = symbols('th1,th2, th3', real=True)
            theta1 = atan2(wy,wx)
            eq1 = link2*sin(th2) + link3*sin(pi/2+th2+th3+th3Offset) - sqrt(wx**2 + wy**2) +a1offset
            eq2 = link2*cos(th2) + link3*cos(pi/2+th2+th3+th3Offset) - wz + link1

            print("solving equation for th2 and th3")
            startt1 = 0
            startt2 = 0
            tryCount = 0
            while(1):
                try:
                    print("count: %d",tryCount)
                    angles = nsolve([eq1,eq2],[th2,th3],[startt1,startt2])
                    for i in range(2):
                        if(angles[i] >2*pi or angles[i]< -2*pi):
                            raise ValueError
                    break
                except e:
                    print("failed... retrying")
                    startt1 = random.randrange(-3.14,3.14,.1)
                    startt1 = random.randrange(-3.14,3.14,.1)
                    startt1 = random.randrange(-3.14,3.14,.1)
            
            print("solved")
            
            theta2 = angles[0]
            theta3 = angles[1]


            ######### Orientation Calculations
            actualRpy = RrpyCorrected.evalf(subs={alpha:alpha1, beta:beta1, gamma:gamma1})

            # print(actualRpy)
            actualT0_3inv = (T0_3inv.evalf(subs={q1:theta1, q2:theta2, q3:theta3}))
            # print(actualT0_3inv)

            orientationRHS = actualT0_3inv * actualRpy
            print(orientationRHS)
            equations = []
            print(T3_6.shape)
            print(orientationRHS.shape)
            for row in range(3):    
                for col in range(3):
                    equations.append(Eq(T3_6[row,col],orientationRHS[row,col]))
            print(equations)
            print("solving equations for theta 4-6")
            startt1 = random.random()*6.28 - 3.14
            startt2 = random.random()*6.28 - 3.14
            startt3 = random.random()*6.28 - 3.14
            cont = true
            while(1):
                try:
                    print("count: %d",tryCount)
                    angles = nsolve(equations,[q4,q5,q6],[startt1,startt2,startt3])
                    for i in range(3):
                        if(angles[i] >2*pi or angles[i]< -2*pi):
                            raise ValueError
                    break
                except ValueError:
                    print("failed... retrying")
                    startt1 = random.random()*6.28 - 3.14
                    startt2 = random.random()*6.28 - 3.14
                    startt3 = random.random()*6.28 - 3.14
                    


            # theta1 = -0.65
            # theta2 = 0.45
            # theta3 = -0.36
            theta4 = angles[0]
            theta5 = angles[1]
            theta6 = angles[2]
            print("theta 1 %f\n",theta1)
            print("theta 2 %f\n",theta2)
            print("theta 3 %f\n",theta3)
            print("theta 4 %f\n",theta4)
            print("theta 5 %f\n",theta5)
            print("theta 6 %f\n",theta6)
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

        # rospy.logfatal("length of Joint Trajectory List: %s" % len(joint_trajectory_list))


        #rospy.logfatal("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print ("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()

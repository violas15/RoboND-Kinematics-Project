from sympy import *
from time import time
from mpmath import radians
import tf
import random

'''
Format of test case is [ [[EE position],[EE orientation as rpy]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generate test cases can be added to the test_cases dictionary
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
              4:[[[2.153,0.0,1.946],
                  [0,0,0,1]],
                  [1.85,0,1.946],
                  [0,0,0,0,0,0]],
              5:[]}


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
    ## Insert IK code here starting at: Define DH parameter symbols
               # Define DH param symbols
           
            # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
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
                if(angles[i] >pi or angles[i]< -pi):
                    raise ValueError
            break
        except ValueError:
            print("failed... retrying")
            startt1 = random.random()*6.28 - 3.14
            startt2 = random.random()*6.28 - 3.14
            startt3 = random.random()*6.28 - 3.14
    
    print("solved")
    
    theta2 = angles[0]
    theta3 = angles[1]


    ######### Orientation Calculations
    actualRpy = RrpyCorrected.evalf(subs={alpha:alpha1, beta:beta1, gamma:gamma1})

    actualT0_3inv = (T0_3inv.evalf(subs={q1:theta1, q2:theta2, q3:theta3}))

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
                if(angles[i] >pi or angles[i]< -pi):
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
    
    # Populate response for the IK request
    # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    #joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, 0, 0]
# joint_trajectory_list.append(joint_trajectory_point)

# rospy.logfatal("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

    ## Ending at: Populate response for the IK request
    ########################################################################################
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]
    jointAngles = {q1: theta1, q2:theta2, q3:theta3,q4:theta4,q5:theta5, q6:theta6}
    eeMatrix = T_total.evalf(subs={q1: theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
    wcMatrix = T0_4.evalf(subs={q1: theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
    your_ee = [eeMatrix[0,3], eeMatrix[1,3], eeMatrix[2,3]]
    your_wc = [wcMatrix[0,3], wcMatrix[1,3], wcMatrix[2,3]]
    #print(your_ee)
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    #your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    #your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
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
    print("Theta 1: %04.8f" % theta1)
    print("Theta 2: %04.8f" % theta2)
    print("Theta 3: %04.8f" % theta3)
    print("Theta 4: %04.8f" % theta4)
    print("Theta 5: %04.8f" % theta5)
    print("Theta 6: %04.8f" % theta6)
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple posisiotns. It is best to add your forward kinmeatics to \
           \nlook at the confirm wether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
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
    test_case_number = 4

    test_code(test_cases[test_case_number])
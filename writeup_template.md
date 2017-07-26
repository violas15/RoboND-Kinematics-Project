## Project: Kinematics Pick & Place


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
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[dhAnalysis]: ./images/DHParameterGraph.jpg
[theta13]: ./images/Theta1-3.jpg
[dhSetup]: ./images/dhsetup.png
[posKin]: ./images/posKinematics.jpg
[bin]: ./images/10binpicks.jpg
[drop]: ./images/drop.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The analysis of forward kinematics began by sketching out the Kuka KR210 and assigning values based off the modified DH parameter rules as seen below.
![alt text][dhAnalysis]

Points 4,5,6 were chosen to be coincident to allow for easier calculation of the wrist center and end effector. The a_i-1 parameters represent the distance from the DH origin 0_i-1 to 0_i for each dh origin rotated so the z axis is aligned with the axis of rotation of the corresponding joint. The d_i parameters represent the distance along the x axis from O_i to 0_i+1. Alpha_i is the twist angle between consecutive Z axes. Theta_i is the variable parameter corresponding to the current angle of the joint.

i   | Alpha_i-1 | a_i-1 | d_i | theta_i
--- | ---       | ---   | --- |
1   |0          | 0     | .75 |theta_1
2   |-90        | .35   | 0   |theta_2 - 90
3   |0          | 1.25  | 0   |theta_3
4   | -90       | -0.054| 1.5 |theta_4
5   | 90        |0      | 0   |theta_5
6   | -90       | 0     | 0   |theta6
G   | 0         |0      |.303 |0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
In IK_server.py each individual transformation matrix was formed following the equation below.
![alt text][dhSetup]

Transformation T0_1 through T6_G were generated and then composed with matrix multiplication to determine the overall transform from the base_link to the gripper_link. 

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

There are 2 parts of the Inverse Kinematics analysis, the Position Kinematics and the Orientation Kinematics.

#####Position Kinematics
In order to determine the position kinematics it was first necessary to determine the wrist center location. This was achieved using a transformation starting at the end effector location and position.
The transformation from gripper_link to wrist_center was based off of the orientation of the gripper. A rotation is applied to the end effector and then the final grip offset of .303m along the new z axis was subtracted. This resulted in a location for the wrist center based on the the grippers desired position and pose.
Once the wrist center location was calculated trigonometry was used to determine the effects of each joint angle on the position.
![alt text][posKin]
Based of of this image the following equation were generated

`theta1 = tan^-1(y/x)`

`0 = a2 * sin(theta2) + sqrt(d4**2 + a3**2) * sin(90 + theta2 + theta3) -sqrt(x**2 + y**2) + a1`

`0 = a2 * cos(theta2) + sqrt(d4**2 + a3**2) * cos(90 + theta2 + theta3) -z +d1`

Combining these with sympy resulted in valid theta1, theta2 and theta3 angles. Multiple guess points were used using sympy's nsolve function to ensure that the angles are within the correct range.

![alt text][image2]

#####Orientation Kinematics
Once theta1 through 3 are calculated it is then possible to figure out theta4 through6. Since the final orientation of the of the gripper must be the same as the complete rotation from base to gripper link we can say that `R0_6 = Rrpy`
If we premultiply both side by inv(R0_3) we end up with `R3_6 = inv(R0_3) * Rrpy`
Plugging in theta1 through 3 results in the right side having no variables resulting in 9 equations that can be used to solve for theta4 through 6. An additional correction rotation must also be applied due to differences in the urdf and the dh parameter model. 


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
The IK_server.py implementation followed the above analysis very closely. All of the math was the same and sympy was used to solve all equations listed. The IK_server.py was modified from the original file so the overall symbolic matricies were calculated once at the beginning in order to improve the speed of the inverse kinematics calculation. That being said the inverse kinematics was still rather slow taking around half a minute to calculate the correct angles for each path. In a realtime system this would be much too slow for any kind of reactions to events. It was also important to check the bounds of the angles returned from the sympy nsolve function in order to keep them within the physical limitations of the robot. No checks were made to choose the best set of angles, only that they were possible. With further improvement the inverse kinematics angles could be used to optimize for speed or minimum power. Overall this project was successful and it was generally able to pick and place 10/10 of the rods and deposit it into the bin as seen below.

![alt text][bin]
![alt text][drop]
![alt text][image3]



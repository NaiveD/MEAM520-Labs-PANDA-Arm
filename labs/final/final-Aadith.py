import sys
import numpy as np
from copy import deepcopy

import rospy
import roslib

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# The library you implemented over the course of this semester!
from lib.calculateFK import FK
from lib.calcJacobian import calcJacobian
from lib.solveIK import IK
from lib.rrt import rrt
from lib.loadmap import loadmap
from math import pi
import copy
BLOCK_HEIGHT = 0.05 #50 mm blocks
TABLE_HEIGHT = 0 #will be updated in code : We need to hard code this based on actual obervation

def read_camera(data): #Gives a list of transforms for the top face of each static blocks only
    detector = ObjectDetector();
    ground = ["tag0"];
    static_blocks = ["tag1", "tag2", "tag3","tag4","tag5", "tag6"]  #The list all tags read in the static block side
    blocks = [];                                                    #final set of static blocks

    base_tag0 = np.array([[1,  0,  0 , -0.5],                       #Transformation of tag 0 with respect to robot base
                         [ 0,  1,  0,   0  ],
                         [ 0,  0,  1,   0  ],
                         [ 0,  0,  0, 1   ]])

    for (name,pose) in data:
        if name in ground:                                          #Find the transform for Tag0 wrt Camera
            tag0 = pose;
            break;

    for (name,pose) in data:
        if name in static_blocks:                                   #For each tag 6 or below
            block_cam  = pose;
            block = np.matmul(np.matmul(base_tag0,np.linalg.inv(tag0)),pose); #Find the transform wrt to Robot frame : T^Robo_Tag0 * T^Tag0_Cam * T ^Cam_block = T^Robo_block
            #print("check :\n", block)
            if(np.array_equal(np.round(block[:,2]),np.round(base_tag0[:,2])) and block[0,3] > 0.3 and block [1,3] >0 and block[2,3] > 0.2): #Get only the blocks tags that are the top face of each block
                blocks += [block]                                    #The above limits need tuning and may not be having any effect ***     #add it to the list of blocks

    return blocks



#This function takes a block transform as input, and picks up that block and takes it to the center position
def grab_static_block(block, arm):
    #A seed for the IK - from a position nearby the blocks
    q_static_block_point = np.array([ 0.19597102,  0.02002129  ,0.17654651 ,-2.11382587  ,0.03078337 , 2.10129901 ,1.11344131]);

    limits_last_joint = [-2.8973,2.8973]
    fk = FK()
    ik = IK()
    jp,target = fk.forward(arm.neutral_position());

    #Calculate the angle about Z axis that the block is oriented in : We use this to turn the last joint of the Robot to match the block
    ##   Needs more debugging ##
    # angle = get_rotation_about_z(block)%(pi/2);
    # print("block: ", block)
    # print("angle of block : ", math.degrees(angle))
    # if(-angle+pi/4 > limits_last_joint[1]):          ##This needs  correction! Not turning in some cases***
    #     angle = (pi/2-(-angle+pi/4));
    # else:
    # angle = -angle + pi/4
    # print("angle of turn : ", math.degrees(angle-pi/4))

    ## ---               --- ##


    # target[:,3] = block[:,3] # Position of the block
    # target[2,3] += 0.1 # move exactly above the block and position ourselves.
    target1 = copy.deepcopy(block)
    target2 = copy.deepcopy(block) 
    target3 = copy.deepcopy(block)
    target4 = copy.deepcopy(block)
    print("block = ", block)
    print("target1 = ", target1)
    
    target1[:, 2] = -block[:, 2]
    print("block = ", block)
    print("target1 = ", target1)
    target2[:, 2] = -block[:, 2]
    target3[:, 2] = -block[:, 2]
    target4[:, 2] = -block[:, 2]

    target1[:, 0] = -block[:, 0]
    target2[:, 1] = -block[:, 1]
    target3[:, 0] = block[:, 1]
    target3[:, 1] = block[:, 0]
    target4[:, 0] = -block[:, 1]
    target4[:, 1] = -block[:, 0]

    targets = [target1, target2, target3, target4]

    # Find IK for required target (currently, right above the required block)
    success = False;
    # while not success:    
    #     q_goal,success,xx = ik.inverse(target, q_static_block_point);
    # print(q_goal);

    for t in targets:
        q_goal,success,xx = ik.inverse(t, q_static_block_point)

        if success == True:
            print("succeed")
            break
        print("fail")

    #Move to target
    arm.open_gripper();
    arm.safe_move_to_position(q_goal)

    #move down to grasp the block
    target[2,3] -= 0.125
    success = False;
    while not success:
        q_goal,success,xx = ik.inverse(target, q_static_block_point);

    # Rotate the last joint by this angle to grasp the block on it's face and not on it's edge
    q_goal[-1] = angle

    arm.safe_move_to_position(q_goal)
    arm.close_gripper();                    #hold the block
    arm.safe_move_to_position(arm.neutral_position())
    #wait at neutral position with the block held

#This function drops a block at a given location on the field
# Target - the location (not orientation) that the block needs to be dropped
#stack_no : What block is it on this target location's stack (first block, or second one, or third..)
def drop_static_block(target, arm,stack_no = 1):
    fk = FK()
    ik = IK()
    q_static_block_point = np.array([ -0.19597102,  0.02002129  ,0.17654651 ,-2.11382587  ,0.03078337 , 2.10129901 ,1.11344131]);
    #jp,target = fk.forward(target);

    target[2,3] += (stack_no)*BLOCK_HEIGHT #This goes one block above the stack as to not knock it down
    #print("in drop static block")
    #print("stack height : ", target[2,3])
    success = False;
    while not success:
        #print("in loop")
        q_goal1,success,xx = ik.inverse(target, q_static_block_point);
    print(q_goal1);

    #move to a point above the block drop location
    arm.safe_move_to_position(q_goal1)

    #print("moved to target")
    #move to grasp the block
    target[2,3] -= BLOCK_HEIGHT
    success = False;
    while not success:
        q_goal,success,xx = ik.inverse(target, q_static_block_point);

    arm.safe_move_to_position(q_goal)
    arm.open_gripper();
    arm.safe_move_to_position(q_goal1)
    arm.safe_move_to_position(arm.neutral_position())
    print("Done moving")

import math
#find the angle at which a given rotation matrix has rot
def get_rotation_about_z(rot):
    return math.atan2(rot[1,0],rot[0,0]);




if __name__ == "__main__":

    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    #arm.safe_move_to_position(arm.neutral_position()) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!


    fk = FK();
    ik = IK();

    print("reading detector")

    #This function reads and identifies all the static blocks in the field by their top tag
    static_blocks = read_camera(detector.get_detections());
    #The stack number of this particular block
    stack = 1;
    print("Neutral Position : ",arm.neutral_position())
    #The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] -= BLOCK_HEIGHT/2

    #print(drop_target)

    #pick each block and place it, stacking them one on top of the other
    for block in static_blocks:
        drop_target_copy = copy.deepcopy(drop_target)
        grab_static_block(block,arm); #Grab the block from the table
        drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
        stack = stack+1;
        angle = get_rotation_about_z(block);
        print("Angle of block ", math.degrees(angle))
        print("angle % 90 ", math.degrees(angle%(pi/2)))

        pass;

    # STUDENT CODE HERE

    # Detect some tags...


    for (name, pose) in detector.get_detections():
        #print(detector.get_detections())
        print(name,'\n',pose)
        pass;
    # Move around...
    #arm.safe_move_to_position(arm.neutral_position() + .1)

    #q_stack_table_point = [-0.1933028,  -0.0421463,  -0.18381685, -1.92314552, -0.00777114,  1.83777088, 0.38233042] # [0.5,-0.2,0.4,1]






    # END STUDENT CODE

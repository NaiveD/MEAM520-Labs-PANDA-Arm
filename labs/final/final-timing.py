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
#from lib.calcJacobian import FK
from lib.solveIK import IK
from lib.rrt import rrt
from lib.loadmap import loadmap
from math import pi
import copy
import time
import timeit


################################### HELPER FUNCTION ENVIRONMENT #############################################
fk = FK();
ik = IK();

BLOCK_HEIGHT = 0.05 #50 mm blocks
TABLE_HEIGHT = 0.25 # Actually the height of table + one block : obtained from eperical obervation from one of the blocks height
MAX_STACK_HEIGHT = 1
team = "blue"




def read_camera(data): #Gives a list of transforms for the top face of each static blocks only
    detector = ObjectDetector();
    ground = ["tag0"];
    static_blocks = ["tag1", "tag2", "tag3","tag4","tag5", "tag6"]  #The list all tags read in the static block side
    blocks = [];
    white_blocks = [];
    if team == "blue" :
        sign = 1;
    else:
        sign = -1;                                               #final set of static blocks

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
            if(np.array_equal(np.round(block[:,2]),np.round(base_tag0[:,2])) and block[0,3] > 0.3 and block [1,3]*sign >0 and block[2,3] > 0.2): #Get only the blocks tags that are the top face of each block
                #if name == "tag6":
                #    white_blocks += [block]
                #else:
                blocks += [block]                                    #The above limits need tuning and may not be having any effect ***     #add it to the list of blocks
    # blocks = detector.get_static_blocks()
    return blocks+white_blocks


 ########################################################### DYNAMIC BLOCKS ###########################################################################
def has_gripped_block(arm):

    arm.exec_gripper_cmd(0.03, 100)
    position = arm.get_gripper_state();
    pos = position["position"]
    print("position : ", pos[0] + pos[1] )
    if pos[0] + pos[1] < 0.03 :
        arm.exec_gripper_cmd(0.8, 100)
        return False;
    return True;

def read_dynamic_camera(data): #Gives a list of transforms for the top face of each static blocks only
    detector = ObjectDetector();
    ground = ["tag0"];
    static_blocks = ["tag7", "tag8", "tag9","tag10","tag11", "tag12"]  #The list all tags read in the static block side
    closest_block = []
    distance = 2;
    if team == "blue" :
        sign = 1;
    else:
        sign = -1;                                               #final set of static blocks

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
            if(np.array_equal(np.round(block[:,2]),np.round(base_tag0[:,2]))): #Get only the blocks tags that are the top face of each block
                if(np.abs(block[1,3]) < distance):
                    distance = np.abs(block[1,3]);
                    closest_block = block;                             #The above limits need tuning and may not be having any effect ***     #add it to the list of blocks

    return closest_block

#def scoop_dynamic_block(block, arm):
#    arm.safe_move_to_position(arm.neutral_position())
#    q_dynamic_block_point = np.array([-2.8,1.5,1.7,-1,-.7,1.5,-0.7])
#
#    jp,target = fk.forward(q_dynamic_block_point);


def grab_dynamic_block(arm):

    #A seed for the IK - from a position nearby the blocks
    # #q_dynamic_block_point = np.array(	[-1.25429965 , 0.57334985 ,-0.34829473, -1.2432462  , 0.19061448 , 1.78492184, -0.87674544])	#[ -1.5,  0.7  ,0 ,-1 ,0,1.7, -pi/4]);
    # q_dynamic_block_point = np.array(	[1.25429965 , 0.57334985 ,-0.34829473, -1.2432462  , 0.19061448 , 1.78492184, -0.87674544])	#[ -1.5,  0.7  ,0 ,-1 ,0,1.7, -pi/4]);
    # q_dynamic_block_point = np.array([8.982745203667264e-06, -0.7850058882600521, -2.8116932599075994e-05, -2.356013974984109, -8.1156550955086e-06, 1.5700130978418718, 0.785002584889833])
    # limits_last_joint = [-2.8973,2.8973]
    if team == "blue":
        q_dynamic_block_point = np.array(	[-1.25429965 , 0.57334985 ,-0.34829473, -1.2432462  , 0.19061448 , 1.78492184, -0.87674544])	# BLUE [ -1.5,  0.7  ,0 ,-1 ,0,1.7, -pi/4]);
    else:
        q_dynamic_block_point = np.array([ 1.18381931 , 0.64162709 , 0.48129071, -1.16685028, -0.28514617 , 1.74460097, -0.65467289]);  # RED[ 1.5,  0.7  ,0 ,-1 ,0,1.7, -pi/4])


    jp,target = fk.forward(q_dynamic_block_point);

    #target[1,3] += 0.03
    arm.safe_move_to_position(q_dynamic_block_point)
    # # ---  Move above block --- ##

    #Move to above target
    arm.open_gripper();

    target[2,3] -= 0.11

    success = False;
    while not success:
        q_goal,success,xx = ik.inverse(target, q_dynamic_block_point);

    print(q_goal);
    #move down to catch the block
    arm.safe_move_to_position(q_goal)

    #close gripper
    #has_block = has_gripped_block(arm)
    start_time = timeit.default_timer()

    count = 0;
    sleep_list = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1]
    success = has_gripped_block(arm)
    while not success:
        time.sleep(sleep_list[count%7])
        count += 1;
        end_time = timeit.default_timer()
        success = has_gripped_block(arm)
        print("in grab_dynamic_block, elasped time = ", end_time - start_time)
        if ((end_time - start_time) > 30):
            break

    # if(count >= 6):
    #     grab_dynamic_block(arm);
    #arm.exec_gripper_cmd(arm._gripper.MIN_WIDTH, 50 )
    #arm.close_gripper();
    #move to neutral position
    arm.safe_move_to_position(arm.neutral_position())
    return success


 ############################################################END DYNAMIC #################################################################################


#This function takes a block transform as input, and picks up that block and takes it to the center position
def grab_static_block(block, arm):
    #A seed for the IK - from a position nearby the blocks
    q_static_block_point = np.array([ 0.19597102,  0.02002129  ,0.17654651 ,-2.11382587  ,0.03078337 , 2.10129901 ,1.11344131]);

    limits_last_joint = [-2.8973,2.8973]

    jp,target = fk.forward(arm.neutral_position());

    #Calculate the angle about Z axis that the block is oriented in : We use this to turn the last joint of the Robot to match the block
    ##   Needs more debugging ##
    #angle = get_rotation_about_z(block)%(pi/2);
    #print("angle of block : ", math.degrees(angle))
    #if(angle+pi/4 > limits_last_joint[1]):          ##This needs  correction! Not turning in some cases***
    #    angle = pi/4 - (pi/2-(angle)); #*******
    #else:
    #    angle = angle - pi/4 #****
    #print("angle of turn : ", math.degrees(angle-pi/4))


    # # ---  Move above block --- ##

    target[:,3] =  block[:,3] #TABLE_HEIGHT
    target[2,3] = TABLE_HEIGHT - BLOCK_HEIGHT/3;
    target[2,3] += 0.1 # move exactly above the block and position ourselves.

    # Find IK for required target (currently, right above the required block)
    success = False;
    while not success:
        q_goal1,success,xx = ik.inverse(target, q_static_block_point);
    print(q_goal1);

    #Move to above target
    arm.open_gripper();
    arm.safe_move_to_position(q_goal1)


    ## ---  ANGLE CODE   --- ###

    target1 = copy.deepcopy(block)
    target2 = copy.deepcopy(block)
    target3 = copy.deepcopy(block)
    target4 = copy.deepcopy(block)
    #print("block = ", block)
    #print("target1 = ", target1)

    target1[:, 2] = -block[:, 2]
    #print("block = ", block)
    #print("target1 = ", target1)
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

    # Finds the minimum angle that the EE needs to rotate
    y = np.array([0,-1,0,0])
    angles = np.zeros(4)
    for i in range(4):
        t = targets[i]
        # Dot product between approximate direction of end-effector y and target y
        angles[i] = (math.acos(np.dot(y, t[:,1])/np.linalg.norm(t[:,1])))

    #Sorts targets from smallest to largest angle
    targetSorted = []
    for j in range(4):
        ind = np.argmin(angles)
        targetSorted.append(targets[ind])
        angles[ind] = 10

    # Find the config q[7] of the gripper to match any of the axes
    success = False;
    for t in targetSorted:
        q_goal_angle,success,xx = ik.inverse(t, q_static_block_point)

        if success == True:
            print("succeed")
            break
        print("fail")

    ## ---               --- ##

    ## Rotate last joint to match the block angle above the target
    q_goal1[-1] = q_goal_angle[-1];
    arm.safe_move_to_position(q_goal1)

    #Find IK to move down to grasp the block
    target[2,3] -= 0.1
    success = False;
    while not success:
        q_goal,success,xx = ik.inverse(target, q_static_block_point);

    # Ensure the gripper is rotated to match the face of a block
    q_goal[-1] = q_goal_angle[-1];

    #move down to catch the block
    arm.safe_move_to_position(q_goal)

    #close gripper
    arm.exec_gripper_cmd(arm._gripper.MIN_WIDTH, 50 )
    #arm.close_gripper();white_blocks
    #move to neutral position
    arm.safe_move_to_position(q_goal1)
    #arm.safe_move_to_position(arm.neutral_position())
    print(arm.get_gripper_state())

#This function drops a block at a given location on the field
# Target - the location (not orientation) that the block needs to be dropped
#stack_no : What block is it on this target location's stack (first block, or second one, or third..)
def drop_static_block(target, arm,stack_no = 1):
    global MAX_STACK_HEIGHT

    if(stack_no > MAX_STACK_HEIGHT):
        MAX_STACK_HEIGHT = stack_no;

    q_static_block_point = np.array([ -0.19597102,  0.02002129  ,0.17654651 ,-2.11382587  ,0.03078337 , 2.10129901 ,1.11344131]);
    #jp,target = fk.forward(target);
    target_height = target[2,3];
    target[2,3] += (MAX_STACK_HEIGHT)*BLOCK_HEIGHT #This goes one block above the stack as to not knock it down
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
    target[2,3] = target_height+ (stack_no-1)*BLOCK_HEIGHT -0.01
    success = False;
    while not success:
        q_goal,success,xx = ik.inverse(target, q_static_block_point);

    arm.safe_move_to_position(q_goal)
    arm.open_gripper();
    arm.safe_move_to_position(q_goal1)
    #arm.safe_move_to_position(arm.neutral_position())
    print("Done moving")

import math
#find the angle at which a given rotation matrix has rot
def get_rotation_about_z(rot):
    return math.atan2(rot[1,0],rot[0,0]);




############################ TESTING ENVIRONMENTS ################################


def test_1(detector, arm): #Pick and place 1 block
    static_blocks = read_camera(detector.get_detections());

    #The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] = TABLE_HEIGHT

    #pick one block and place it
    block = static_blocks[0]
    drop_target_copy = copy.deepcopy(drop_target)
    grab_static_block(block,arm); #Grab the block from the table
    drop_static_block(drop_target_copy,arm); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other

def test_3(detector, arm): #Stacks all blocks using table height **
    static_blocks = read_camera(detector.get_detections());


    print("Neutral Position : ",arm.neutral_position())
    ##The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] = TABLE_HEIGHT


    ##The stack number of this particular block
    stack = 1;
    ##pick each block and place it, stacking them one on top of the other
    for block in static_blocks:
        drop_target_copy = copy.deepcopy(drop_target)
        grab_static_block(block,arm); #Grab the block from the table
        drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
        stack = stack+1;


def test_2(detector, arm): #Use their own Z estimate and see how bad it is
    print("Executing Test 2 : Completely using the Z estimates from the camera")

    ##This function reads and identifies all the static blocks in the field by their top tag
    static_blocks = read_camera(detector.get_detections());


    print("Neutral Position : ",arm.neutral_position())
    ##The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    drop_target[1,3] = -drop_target[1,3]
    #drop_target[2,3] = TABLE_HEIGHT


    ##The stack number of this particular block
    stack = 1;
    ##pick each block and place it, stacking them one on top of the other
    for block in static_blocks:
        drop_target_copy = copy.deepcopy(drop_target)
        grab_static_block(block,arm); #Grab the block from the table
        drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
        stack = stack+1;
        pass;

def test_4(detector, arm): #stack 2 stacks of 2
    print("Executing Test 4 : Stacking 2 blocks of 2, using table height")

    ##This function reads and identifies all the static blocks in the field by their top tag
    static_blocks = read_camera(detector.get_detections());


    print("Neutral Position : ",arm.neutral_position())
    ##The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] = TABLE_HEIGHT


    ##The stack number of this particular block
    stack = 1;
    ##pick each block and place it, stacking them one on top of the other
    for block in static_blocks:
        if stack > 2 :
            break;
        drop_target_copy = copy.deepcopy(drop_target)
        grab_static_block(block,arm); #Grab the block from the table
        drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
        stack = stack+1;

    stack = 1; #reset stack height
    static_blocks = static_blocks[2:];#read_camera(detector.get_detections()); #reread camera for blocks
    ##The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] = TABLE_HEIGHT
    for block in static_blocks:
        drop_target_copy = copy.deepcopy(drop_target)
        grab_static_block(block,arm); #Grab the block from the table
        drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
        stack = stack+1;

def test_5(detector, arm):
    """
        1. Grab 2 dynamic blocks in 1 min.
        2. Stack the 4 static blocks.
    """
    static_blocks = read_camera(detector.get_detections());
    dynamic_block = read_dynamic_camera(detector.get_detections());


    stack = 1;
    #The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    print("table height: ",drop_target[2,3])
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] -= BLOCK_HEIGHT/2

    print(drop_target)

    start_time = timeit.default_timer()
    while True:
        drop_target_copy = copy.deepcopy(drop_target)
        success = grab_dynamic_block(arm) #Grab the block from the table
        if success:
            drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
            stack += 1;
        end_time = timeit.default_timer()
        print("elapsed time: ", end_time - start_time)
        if (end_time - start_time > 60):
            break
    
    arm.safe_move_to_position(arm.neutral_position())
    
    # Stack the 4 static blocks
    stack = 1; #reset stack height
    static_blocks = static_blocks[2:];#read_camera(detector.get_detections()); #reread camera for blocks
    ##The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] = TABLE_HEIGHT
    for block in static_blocks:
        drop_target_copy = copy.deepcopy(drop_target)
        grab_static_block(block,arm); #Grab the block from the table
        drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
        stack = stack+1;
    

    


def dynamic_test(detector,arm):
     #This function reads and identifies all the static blocks in the field by their top tag
    static_blocks = read_camera(detector.get_detections());
    dynamic_block = read_dynamic_camera(detector.get_detections());


    stack = 1;
    #The location where we want to drop the box - here I have just mirrored the drop location of the first static block on the drop side, about the robot
    _,drop_target =fk.forward(arm.neutral_position());
    drop_target[:,3] = static_blocks[0][:,3];
    print("table height: ",drop_target[2,3])
    drop_target[1,3] = -drop_target[1,3]
    drop_target[2,3] -= BLOCK_HEIGHT/2

    print(drop_target)

    while True:
        drop_target_copy = copy.deepcopy(drop_target)
        grab_dynamic_block(arm); #Grab the block from the table
        drop_static_block(drop_target_copy,arm,stack_no = stack); #drop the block on to the mirror location on the other side of the first block, and stack the rest on top of each other
        stack += 1;



def save_readings():
    pass;
    #save camera readings on a file




############################################################################### MAIN #############################################################################

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

    print("MAX_STACK_HEIGHT = ", MAX_STACK_HEIGHT)
    MAX_STACK_HEIGHT = 1;

    #test_4(detector, arm);
    #test_3(detector,arm);
    # dynamic_test(detector,arm)
    test_5(detector,arm)





    #arm.safe_move_to_position(arm.neutral_position() + .1)

    #q_stack_table_point = [-0.1933028,  -0.0421463,  -0.18381685, -1.92314552, -0.00777114,  1.83777088, 0.38233042] # [0.5,-0.2,0.4,1]


    # END STUDENT CODE

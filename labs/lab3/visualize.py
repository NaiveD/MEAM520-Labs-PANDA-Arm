import sys
from math import pi, sin, cos
import numpy as np
from time import perf_counter

import rospy
import roslib
import tf
import geometry_msgs.msg

from core.interfaces import ArmController

from lib.solveIK import IK

rospy.init_node("visualizer")

# Using your solution code
ik = IK()

#########################
##  RViz Communication ##
#########################

tf_broad  = tf.TransformBroadcaster()

# Broadcasts a frame using the transform from given frame to world frame
def show_pose(H,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(H),
        tf.transformations.quaternion_from_matrix(H),
        rospy.Time.now(),
        frame,
        "world"
    )

#############################
##  Transformation Helpers ##
#############################

def trans(d):
    """
    Compute pure translation homogenous transformation
    """
    return np.array([
        [ 1, 0, 0, d[0] ],
        [ 0, 1, 0, d[1] ],
        [ 0, 0, 1, d[2] ],
        [ 0, 0, 0, 1    ],
    ])

def roll(a):
    """
    Compute homogenous transformation for rotation around x axis by angle a
    """
    return np.array([
        [ 1,     0,       0,  0 ],
        [ 0, cos(a), -sin(a), 0 ],
        [ 0, sin(a),  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def pitch(a):
    """
    Compute homogenous transformation for rotation around y axis by angle a
    """
    return np.array([
        [ cos(a), 0, -sin(a), 0 ],
        [      0, 1,       0, 0 ],
        [ sin(a), 0,  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def yaw(a):
    """
    Compute homogenous transformation for rotation around z axis by angle a
    """
    return np.array([
        [ cos(a), -sin(a), 0, 0 ],
        [ sin(a),  cos(a), 0, 0 ],
        [      0,       0, 1, 0 ],
        [      0,       0, 0, 1 ],
    ])

def transform(d,rpy):
    """
    Helper function to compute a homogenous transform of a translation by d and
    rotation corresponding to roll-pitch-yaw euler angles
    """
    return trans(d) @ roll(rpy[0]) @ pitch(rpy[1]) @ yaw(rpy[2])

#################
##  IK Targets ##
#################

# TODO: Try testing your own targets!

# Note: below we are using some helper functions which make it easier to generate
# valid transformation matrices from a translation vector and Euler angles, or a
# sequence of successive rotations around z, y, and x. You are free to use these
# to generate your own tests, or directly write out transforms you wish to test.

targets = [
    np.array([
        [0, 0,1,0.5],
        [-1,0,0,0],
        [0,-1,0,0.9],
        [0,0,0, 1]]),
    np.array([
        [1,0,0,0.5],
        [0,-1,0,0],
        [0,0,-1,0.5],
        [0,0,0, 1]]),
    np.array([
        [0,1,0,0.5],
        [1,0,0,0],
        [0,0,-1,0.3],
        [0,0,0, 1],
    ]),
    transform( np.array([-.2, -.3, .5]), np.array([0,pi,pi])            ),
    transform( np.array([-.2, .3, .5]),  np.array([pi/6,5/6*pi,7/6*pi]) ),
    transform( np.array([.5, 0, .5]),    np.array([0,pi,pi])            ),
    transform( np.array([.7, 0, .5]),    np.array([0,pi,pi])            ),
    transform( np.array([.2, .6, 0.5]),  np.array([0,pi,pi])            ),
    transform( np.array([.2, .6, 0.5]),  np.array([0,pi,pi-pi/2])       ),
    transform( np.array([.2, -.6, 0.5]), np.array([0,pi-pi/2,pi])       ),
    transform( np.array([.2, -.6, 0.5]), np.array([pi/4,pi-pi/2,pi])    ),
    transform( np.array([.5, 0, 0.2]),   np.array([0,pi-pi/2,pi])       ),
    transform( np.array([.4, 0, 0.2]),   np.array([pi/2,pi-pi/2,pi])    ),
    transform( np.array([.4, 0, 0]),     np.array([pi/2,pi-pi/2,pi])    ),
]

# targets2 = [
#     transform( np.array([0.1, 0.3, 0.5]), np.array([pi/4,1/7*pi,0])        ),
#     transform( np.array([0.321, -0.23, 0.23]),  np.array([pi/2,2/3*pi,pi/6]) ),
#     transform( np.array([0.123, 0.221, 0.35]),    np.array([0,0,pi/2])     ),
#     transform( np.array([-0.3, -0.12, 0.34]),    np.array([0,pi/8,pi])     ),
#     transform( np.array([0.114, 0.514, 0.8]),  np.array([0,0,0])           ),
#     transform( np.array([0.233, -0.302, 0.7]),  np.array([0,pi,pi/2])      ),
#     transform( np.array([-0.121, -0.6, 0.25]), np.array([0,0,7/6*pi])      ),
#     transform( np.array([0.12, -0.36, 0.42]), np.array([pi/3,pi/4,pi/5])   ),
#     transform( np.array([0.35, 0.132, 0.22]),   np.array([0,pi-pi/2,pi/5]) ),
#     transform( np.array([0.44, -0.24, 0.23]),   np.array([pi/5,pi/2,-pi/3])),
#     transform( np.array([0.24, 0.6, 0.6]),     np.array([0,pi*3/2,pi/4])   ),    
# ]

####################
## Test Execution ##
####################

np.set_printoptions(suppress=True)

if __name__ == "__main__":

    arm = ArmController()

    # Iterates through the given targets, using your IK solution
    # Try editing the targets list above to do more testing!
    for i, target in enumerate(targets):
        print("Target " + str(i) + " located at:")
        print(target)
        print("Solving... ")
        show_pose(target,"target")

        seed = arm.neutral_position() # use neutral configuration as seed

        start = perf_counter()
        q, success, rollout = ik.inverse(target, seed)
        stop = perf_counter()
        dt = stop - start

        if success:
            print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt,it=len(rollout)))
            arm.move_to_position(q)
        else:
            print('IK Failed for this target using this seed.')


        if i < len(targets) - 1:
            input("Press Enter to move to next target...")


    # dt_list = []
    # iter_list = []
    # success_cnt = 0
    # # For Performance Statistics:
    # for i, target in enumerate(targets):
    #     # print("Target " + str(i) + " located at:")
    #     # print(target)
    #     print("Target " + str(i) + ": ")

    #     # seed = arm.neutral_position() # use neutral configuration as seed
    #     seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    #     start = perf_counter()
    #     q, success, rollout = ik.inverse(target, seed)
    #     stop = perf_counter()
    #     dt = stop - start

    #     if success:
    #         print("Solution found in {time:2.2f} seconds ({it} iterations).".format(time=dt,it=len(rollout)))
    #         success_cnt += 1
    #         dt_list.append(dt)
    #         iter_list.append(len(rollout))

    #     else:
    #         print('IK Failed for this target using this seed.')

    #     print("\n")

    #     # if i < len(targets) - 1:
    #     #     input("Press Enter to move to next target...")
    
    # time_arr = np.array(dt_list)
    # iter_arr = np.array(iter_list)
    # success_rate = success_cnt / len(targets)

    # print(np.mean(time_arr), np.median(time_arr), np.max(time_arr))
    # print(np.mean(iter_arr), np.median(iter_arr), np.max(iter_arr))
    # print(success_rate)
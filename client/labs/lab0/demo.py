"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
sys.path.insert(1, '../../') # necessary to find core files!

import rospy
import numpy as np

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()

arm.open_gripper()

arm.untuck() # move to a neutral configuration

arm.close_gripper()

# move to a new position
q = np.array([0,0,0,0,0,0,0]) # TODO: try changing this!
arm.move_to_position(q, timeout=15)

arm.open_gripper()

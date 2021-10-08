"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
import rospy
import numpy as np
from math import pi

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(.2)

arm.close_gripper()

q = arm.neutral_position()
arm.move_to_position(q)
arm.open_gripper()

# Joint angles in radians (7-DoF)
# q = np.array([0,-1 ,0,-2,0,1,1]) # TODO: try changing this!
# q = np.array([1.57, -1.3, 0.3, 1.2, -2.3, 2, -0.3]) # Configuration 1
# q = np.array([0.3, -2.3, 0.1, -0.2, -0.3, 0.7, 0]) # Configuration 2
# q = np.array([0.89, -0.64, 0.13, -1, 0, 2, -3]) # Configuration 3
# q = np.array([0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7]) # Configuration 4
# q = np.array([0, 1.2, 0, 1.1, 0, 2, 0]) # Configuration 5

# Zero Configuration
# q = np.array([0, 0, 0, 0, 0, 0, 0])

# Configuration in Lab 1 instructions
q = np.array([0, 0, 0, -pi/2, 0, pi/2, pi/4])
arm.move_to_position(q)

# Test for joint 1 to see the relationship with theta_0
q = np.array([pi/4, 0, 0, -pi/2, 0, pi/2, pi/4])
arm.move_to_position(q)

arm.close_gripper()

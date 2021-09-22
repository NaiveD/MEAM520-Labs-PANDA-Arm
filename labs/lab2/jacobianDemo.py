import sys
import numpy as np
import rospy
import matplotlib.pyplot as plt

from core.interfaces import ArmController

# Include IK and FK
from lib.IK_velocity import IK_velocity
from lib.calculateFK import FK

rospy.init_node("jacobianDemo")

class JacobianDemo():
    """
    Demo class for testing Jacobian contains callback function and state history
    """
    joint_list = [] # list of joint states
    continue_command = True # When to stop commanding arm
    start_time = 0 # start time
    dt = 0.03 # constant for how to turn positions into times

    def straight_line_callback(self, state):
        """
        Draw a straight line and record the joint states
        """
        if self. continue_command:
            q = state['position']
            self.joint_list.append(q)

            # STUDENT CODE GOES HERE, calculate dq
            dq = np.zeros(7)

            arm.exec_position_cmd(q + self.dt * dq)

    def circle_callback(self, state):
        """
        Draw a circle and record the joint states
        """
        if self. continue_command:
            current_time = rospy.get_time()
            q = state['position']
            self.joint_list.append(q)

            # STUDENT CODE GOES HERE, calculate dq
            dq = np.zeros(7)

            arm.exec_position_cmd(q + self.dt * dq)


    def custom_callback(self, state):
        """
        Custom Jacobian test for students
        """
        if self. continue_command:
            current_time = rospy.get_time()
            q = state['position']
            self.joint_list.append(q)

            # STUDENT CODE GOES HERE, calculate dq
            dq = np.zeros(7)

            arm.exec_position_cmd(q + self.dt * dq)


####################
## Test Execution ##
####################

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage:\n\tpython jacobianDemo.py line\n\tpython jacobianDemo.py circle\n\tpython jacobianDemo.py custom"
              "\n\tpython jacobianDemo.py reset")
        exit()

    JD = JacobianDemo()
    if sys.argv[1] == 'line':
        arm = ArmController(on_state_callback=JD.straight_line_callback)
        input("Press Enter to stop")
    elif sys.argv[1] == 'circle':
        JD.start_time=rospy.get_time()
        arm = ArmController(on_state_callback=JD.circle_callback)
        input("Press Enter to stop")
    elif sys.argv[1] == 'custom':
        JD.start_time=rospy.get_time()
        arm = ArmController(on_state_callback=JD.custom_callback)
        input("Press Enter to stop")
    elif sys.argv[1] == 'reset':
        arm = ArmController()
        arm.move_to_position(arm.neutral_position())
    else:
        print("invalid option")

    JD.continue_command = False


    if sys.argv[1] != 'reset':
        # STUDENT CODE GOES HERE, calculate dq
        # Use JD.joint_list to plot end effector position and make other plots to show IK velocity works
        fk = FK()

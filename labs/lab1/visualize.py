import sys
from math import pi
import numpy as np

import rospy
import roslib
import tf
import geometry_msgs.msg


from core.interfaces import ArmController

from calculateFK import FK
from PlanarIK import PlanarIK

rospy.init_node("visualizer")

fk = FK()
ik = PlanarIK()

#################
##  IK Targets ##
#################

# TODO: Try testing your own targets!

targets = [
    {
        'o': np.array([.3,.5]),
        'theta': 0
    },
    {
        'o': np.array([.3,.2]),
        'theta': -pi/6
    },
    {
        'o': np.array([.3,.7]),
        'theta': pi/6
    }
]

#########################
##  RViz Communication ##
#########################

tf_broad  = tf.TransformBroadcaster()
point_pubs = [
    rospy.Publisher('/vis/joint'+str(i), geometry_msgs.msg.PointStamped, queue_size=10)
    for i in range(7)
]

# Publishes the position of a given joint on the corresponding topic
def show_joint_position(joints,i):
    msg = geometry_msgs.msg.PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    msg.point.x = joints[i,0]
    msg.point.y = joints[i,1]
    msg.point.z = joints[i,2]
    point_pubs[i].publish(msg)

# Broadcasts a T0e as the transform from given frame to world frame
def show_pose(T0e,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(T0e),
        tf.transformations.quaternion_from_matrix(T0e),
        rospy.Time.now(),
        frame,
        "world"
    )

# Uses the above methods to visualize the full results of your FK
def show_all_FK(state):
    q = np.zeros(8)
    q[0:7] = state['position']
    joints, T0e = fk.forward(q)
    show_pose(T0e,"endeffector")
    for i in range(7):
        show_joint_position(joints,i)

# visualize the chosen IK target
def show_target(target):
    x = target['o'][0]
    z = target['o'][1]
    theta = target['theta']
    T0_target = tf.transformations.translation_matrix(np.array([x,0,z])) @ tf.transformations.euler_matrix(0,theta,0)
    show_pose(T0_target,"target")

#####################
##  Test Execution ##
#####################

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("usage:\n\tpython visualize.py FK\npython visualize.py IK")
        exit()

    arm = ArmController(on_state_callback=show_all_FK)

    if sys.argv[1] == 'FK':

        # TODO: try different configurations!
        # pick a random configuration near the neutral configuration
        q = arm.neutral_position() + 2 * (np.random.rand(7) - .5)
        arm.move_to_position(q)

    elif sys.argv[1] == 'IK':

        # Iterates through the given targets, using your IK solution
        # Try editing the targets list above to do more testing!
        for i, target in enumerate(targets):
            print("Moving to target " + str(i) + "...")
            show_target(target)
            solutions = ik.panda_ik(target)
            q = solutions[0,:]
            arm.move_to_position(q)
            if i < len(targets) - 1:
                input("Press Enter to move to next target...")
    else:
        print("invalid option")
        exit()

import numpy as np

import rospy
import roslib
import tf
import geometry_msgs.msg

from calculateFK import FK
from core.interfaces import ArmController


rospy.init_node("visualizer")

fk = FK()
tf_broad  = tf.TransformBroadcaster()
point_pubs = [
    rospy.Publisher('/vis/joint'+str(i), geometry_msgs.msg.PointStamped, queue_size=10)
    for i in range(7)
]

def show_joint_position(joints,i):
    msg = geometry_msgs.msg.PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    msg.point.x = joints[i,0]/1e3
    msg.point.y = joints[i,1]/1e3
    msg.point.z = joints[i,2]/1e3
    point_pubs[i].publish(msg)

def show_pose(T0e,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(T0e) / 1e3,
        tf.transformations.quaternion_from_matrix(T0e),
        rospy.Time.now(),
        frame,
        "world"
    )

def show_all_FK(state):

    q = np.zeros(8)
    q[0:7] = state['position']

    joints, T0e = fk.forward(q)

    # visualize FK
    show_pose(T0e,"endeffector")
    for i in range(7):
        show_joint_position(joints,i)


if __name__ == "__main__":

    arm = ArmController(on_state_callback=show_all_FK)

    # visualize IK targets
    for i in range(3):
        # TODO: load targets from file?
        target = np.identity(4)
        target[0:3,3] = np.array([500,200 * i,500 + 150 * i])
        show_pose(target,"target" + str(i+1))

    arm.move_to_position(arm.neutral_position() + 2 * (np.random.rand(7) - .5))

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyRequest
from math import sin, cos, pi
import numpy as np
from time import sleep
import os

import tf

rospy.init_node('block_spawner')


# from gazebo_ros.gazebo_interface import set_model_configuration_client
#
# set = False
# while not set:
#     set = set_model_configuration_client(
#         'panda','robot_description',
#         ['panda_joint' + str(i+1) for i in range(7)],
#         [ 0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785],
#         'gazebo'
#         )
#

from gazebo_msgs.srv import SpawnModel
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

count = 0
def place(x,y,z,type):
    global count
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    r = np.random.randint(0,3) * 2 * np.pi / 4
    p = np.random.randint(0,3) * 2 * np.pi / 4
    y = np.random.rand(1) * 2 * pi
    q = tf.transformations.quaternion_from_euler(r,p,y)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    count = count + 1
    success = False
    while not success and not rospy.is_shutdown():
        try:
            success = spawn_model_client(
                    model_name='cube'+str(count)+'_'+type,
                    model_xml=open(os.path.expanduser('~/meam520_ws/src/meam520_labs/ros/meam520_labs/urdf/cube.xacro'), 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=pose,
                    reference_frame='world')
        except:
            print('Waiting to spawn cubes...')
            sleep(1)


def noise(radius):
    return radius * (np.random.rand(1) - .5)


for i in [-1,1]:
    for j in [-1,1]:
        x = .562 + 3*.0254 * i
        y = 1.147 + 3*.0254 * j
        place(x + noise(.01) ,y + noise(.01),.23,'static')
        place(x + noise(.01) ,-y + noise(.01),.23,'static')

n = 8
r = .2
for i in range(n):
    place((r + noise(r/5)) * cos(2*pi/n * (i + noise(pi/n))),(r + noise(r/5)) * sin(2*pi/n * (i + noise(pi/n))),.23,'dynamic')



# pause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
# pause_physics_client(EmptyRequest())

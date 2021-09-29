from lib.calculateFK import FK
from core.interfaces import ArmController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
 ]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# TODO: update this with real results
# ax.scatter(1,1,1) # plot the point (1,1,1)

joint0_list = np.linspace(limits[0]['lower'], limits[0]['upper'], 5)
joint1_list = np.linspace(limits[1]['lower'], limits[1]['upper'], 5)
joint2_list = np.linspace(limits[2]['lower'], limits[2]['upper'], 5)
joint3_list = np.linspace(limits[3]['lower'], limits[3]['upper'], 5)
joint4_list = np.linspace(limits[4]['lower'], limits[4]['upper'], 5)
joint5_list = np.linspace(limits[5]['lower'], limits[5]['upper'], 5)
joint6_list = np.linspace(limits[6]['lower'], limits[6]['upper'], 5)

# for i in range(0, len(joint0_list)):
#     # q = np.array([joint0_list[i], joint1_list[i], joint2_list[i], joint3_list[i], joint4_list[i], joint5_list[i], joint6_list[i],])
#     q1 = np.array([joint0_list[i], joint1_list[0], joint2_list[0], joint3_list[0], joint4_list[0], joint5_list[0], joint6_list[0]])
#     q2 = np.array([joint0_list[i], joint1_list[-1], joint2_list[-1], joint3_list[-1], joint4_list[-1], joint5_list[-1], joint6_list[-1]])
#     joint_positions, T0e = fk.forward(q1)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])
#     joint_positions, T0e = fk.forward(q2)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])

# for i in range(0, len(joint1_list)):
#     # q = np.array([joint0_list[i], joint1_list[i], joint2_list[i], joint3_list[i], joint4_list[i], joint5_list[i], joint6_list[i],])
#     q1 = np.array([joint0_list[0], joint1_list[i], joint2_list[0], joint3_list[0], joint4_list[0], joint5_list[0], joint6_list[0]])
#     q2 = np.array([joint0_list[-1], joint1_list[i], joint2_list[-1], joint3_list[-1], joint4_list[-1], joint5_list[-1], joint6_list[-1]])
#     joint_positions, T0e = fk.forward(q1)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])
#     joint_positions, T0e = fk.forward(q2)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])

# for i in range(0, len(joint2_list)):
#     # q = np.array([joint0_list[i], joint1_list[i], joint2_list[i], joint3_list[i], joint4_list[i], joint5_list[i], joint6_list[i],])
#     q1 = np.array([joint0_list[0], joint1_list[0], joint2_list[i], joint3_list[0], joint4_list[0], joint5_list[0], joint6_list[0]])
#     q2 = np.array([joint0_list[-1], joint1_list[-1], joint2_list[i], joint3_list[-1], joint4_list[-1], joint5_list[-1], joint6_list[-1]])
#     joint_positions, T0e = fk.forward(q1)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])
#     joint_positions, T0e = fk.forward(q2)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])

# for i in range(0, len(joint3_list)):
#     # q = np.array([joint0_list[i], joint1_list[i], joint2_list[i], joint3_list[i], joint4_list[i], joint5_list[i], joint6_list[i],])
#     q1 = np.array([joint0_list[0], joint1_list[0], joint2_list[0], joint3_list[i], joint4_list[0], joint5_list[0], joint6_list[0]])
#     q2 = np.array([joint0_list[-1], joint1_list[-1], joint2_list[-1], joint3_list[i], joint4_list[-1], joint5_list[-1], joint6_list[-1]])
#     joint_positions, T0e = fk.forward(q1)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])
#     joint_positions, T0e = fk.forward(q2)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])

# for i in range(0, len(joint4_list)):
#     # q = np.array([joint0_list[i], joint1_list[i], joint2_list[i], joint3_list[i], joint4_list[i], joint5_list[i], joint6_list[i],])
#     q1 = np.array([joint0_list[0], joint1_list[0], joint2_list[0], joint3_list[0], joint4_list[i], joint5_list[0], joint6_list[0]])
#     q2 = np.array([joint0_list[-1], joint1_list[-1], joint2_list[-1], joint3_list[-1], joint4_list[i], joint5_list[-1], joint6_list[-1]])
#     joint_positions, T0e = fk.forward(q1)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])
#     joint_positions, T0e = fk.forward(q2)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])

# for i in range(0, len(joint5_list)):
#     # q = np.array([joint0_list[i], joint1_list[i], joint2_list[i], joint3_list[i], joint4_list[i], joint5_list[i], joint6_list[i],])
#     q1 = np.array([joint0_list[0], joint1_list[0], joint2_list[0], joint3_list[0], joint4_list[0], joint5_list[i], joint6_list[0]])
#     q2 = np.array([joint0_list[-1], joint1_list[-1], joint2_list[-1], joint3_list[-1], joint4_list[-1], joint5_list[i], joint6_list[-1]])
#     joint_positions, T0e = fk.forward(q1)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])
#     joint_positions, T0e = fk.forward(q2)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])

# for i in range(0, len(joint6_list)):
#     # q = np.array([joint0_list[i], joint1_list[i], joint2_list[i], joint3_list[i], joint4_list[i], joint5_list[i], joint6_list[i],])
#     q1 = np.array([joint0_list[0], joint1_list[0], joint2_list[0], joint3_list[0], joint4_list[0], joint5_list[0], joint6_list[i]])
#     q2 = np.array([joint0_list[-1], joint1_list[-1], joint2_list[-1], joint3_list[-1], joint4_list[-1], joint5_list[-1], joint6_list[i]])
#     joint_positions, T0e = fk.forward(q1)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])
#     joint_positions, T0e = fk.forward(q2)
#     ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])

for j0 in range(0, len(joint0_list)):
    for j1 in range(0, len(joint1_list)):
        for j2 in range(0, len(joint2_list)):
            for j3 in range(0, len(joint3_list)):
                for j4 in range(0, len(joint4_list)):
                    for j5 in range(0, len(joint5_list)):
                        for j6 in range(0, len(joint6_list)):
                            q = np.array([joint0_list[j0], joint1_list[j1], joint2_list[j2], joint3_list[j3], joint4_list[j4], joint5_list[j5], joint6_list[j6]])
                            joint_positions, T0e = fk.forward(q)
                            ax.scatter(joint_positions[6, :][0], joint_positions[6, :][1], joint_positions[6, :][2])


plt.show()

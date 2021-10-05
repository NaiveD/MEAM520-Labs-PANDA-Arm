import math

import numpy as np
from math import pi
import functools as func


class FK():

    def __init__(self):
        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass

    @staticmethod
    def zRotTransformation(a, x, y, z):
        '''
            Generate transformation matrix for homogenous z-axis rotations with y and z translation
            Inputs:
            a - angle in radians
            y - translation
            z - translation
            Outputs:
            (4x4) transformation matrix
        '''
        return np.matrix(
            [[math.cos(a), -math.sin(a), 0, x],
            [math.sin(a), math.cos(a), 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]])

    @staticmethod
    def yRotTransformation(a, x, y, z):
        '''
            Generate transformation matrix for homogenous y-axis rotations with y and z translation
            Inputs:
            a - angle in radians
            y - translation
            z - translation
            Outputs:
            (4x4) transformation matrix
        '''
        return np.matrix(
            [[math.cos(a), 0, math.sin(a), x],
            [0, 1, 0, y],
            [-math.sin(a), 0, math.cos(a), z],
            [0, 0, 0, 1]])

    @staticmethod
    def xRotTransformation(a, x, y, z):
        '''
            Generate transformation matrix for homogenous y-axis rotations with y and z translation
            Inputs:
            a - angle in radians
            y - translation
            z - translation
            Outputs:
            (4x4) transformation matrix
        '''
        return np.matrix(
            [[1, 0, 0, x],
            [0, math.cos(a), -math.sin(a), y],
            [0, math.sin(a), math.cos(a), z],
            [0, 0, 0, 1]])

    # feel free to define additional helper methods to modularize your solution
    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 7 x 3 matrix, where each row corresponds to a rotational joint of the robot
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """
        # z direction
        l = np.zeros(8)
        l[0] = .141
        l[1] = .192
        l[2] = .195
        l[3] = .121
        l[4] = .083
        l[5] = 0
        l[6] = -.051
        l[7] = -.159
        lx = np.zeros(8)
        lx[3] = .082
        lx[4] = .125
        lx[5] = .259
        lx[6] = .088
        # Joint 6 y-axis offset
        ly = .015

        # Transformation Matrices
        T_B1 = self.zRotTransformation(q[0], 0, 0, l[0])
        T_12 = self.yRotTransformation(q[1], 0, 0, l[1])
        T_23 = self.zRotTransformation(q[2], 0, 0, l[2])
        T_34 = self.yRotTransformation(-(q[3]+(pi/2)), lx[3], 0, l[3])
        T_45 = self.xRotTransformation(q[4], lx[4], 0, l[4])
        T_56 = self.yRotTransformation((-q[5]+(pi/2)), lx[5], ly, l[5])
        T_67 = self.zRotTransformation((-q[6]+(pi/4)), lx[6], -ly, l[6])
        T_7E = self.yRotTransformation(pi, 0, 0, l[7])
        T_7E2 = self.zRotTransformation(pi, 0, 0, 0)

        # Finding each joint position
        jointPositions = np.zeros((7, 3))
        pos_vec = np.array([0, 0, 0, 1])

        #Joint 1 Position
        pos = np.matmul(T_B1, pos_vec)
        jointPositions[0, :] = pos[0, 0:3]

        # Joint 2 Position
        pos = np.matmul(np.matmul(T_B1, T_12), pos_vec)
        jointPositions[1, :] = pos[0, 0:3]

        # Joint 3 Position
        pos = np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), pos_vec)
        jointPositions[2, :] = pos[0, 0:3]

        # Joint 4 Position
        pos = np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), pos_vec)
        jointPositions[3, :] = pos[0, 0:3]

        # Joint 5 Position
        pos = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), pos_vec)
        jointPositions[4, :] = pos[0, 0:3]

        # Joint 6 Position
        pos = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56), pos_vec)
        jointPositions[5, :] = pos[0, 0:3]

        # Joint 7 Position
        pos = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56), T_67), pos_vec)
        jointPositions[6, :] = pos[0, 0:3]

        T0e = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56), T_67), T_7E),T_7E2)

        # Your code ends here

        return jointPositions, T0e




if __name__ == "__main__":
    fk = FK()

    # matches figure in the handout
    q = np.array([0, 0, 0, -pi / 2, 0, pi / 2, pi / 4])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)

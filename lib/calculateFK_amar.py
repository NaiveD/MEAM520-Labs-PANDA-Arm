import math

import numpy as np
from math import pi


class FK():

    def __init__(self):
        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass
    # feel free to define additional helper methods to modularize your solution
    def zRotTransformation(self, a, y, z):
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
            [math.cos(a), -math.sin(a), 0, 0],
            [math.sin(a), math.cos(a), 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        )

    def yRotTransformation(self, a, y, z):
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
            [math.cos(a), 0, math.sin(a), 0],
            [0, 1, 0, y],
            [-math.sin(a), 0, math.cos(a), z],
            [0, 0, 0, 1]
        )

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
        # Link lengths
        l = np.zeros(8)
        l[0] = .141
        l[1] = .192
        l[2] = .195
        l[3] = math.sqrt((.121 ** 2) + (.082 ** 2))
        l[4] = math.sqrt((.125 ** 2) + (.083 ** 2))
        l[5] = .259
        l[6] = math.sqrt((.088 ** 2) + (.051 ** 2))
        l[7] = .159
        # Joint 6 y-axis offset
        ly = .015

        # Transformation Matrices
        T_B1 = zRotTransformation(q[0], 0, 0, l[0])
        T_12 = yRotTransformation(q[1], 0, 0, l[1])
        T_23 = zRotTransformation(q[2], 0, 0, l[2])
        T_34 = yRotTransformation(q[3], 0, 0, l[3])
        T_45 = zRotTransformation(q[4], 0, 0, l[4])
        T_56 = yRotTransformation(q[5], 0, ly, l[5])
        T_67 = zRotTransformation(q[6], 0, -ly, l[6])
        T_7E = np.matrix(
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, l[7]],
            [0, 0, 0, 1]
        )

        #TODO Tranformation multiplication and calculating joint positions
        jointPositions = np.zeros((7, 3))
        T0e = np.identity(4)

        # Your code ends here

        return jointPositions, T0e




if __name__ == "__main__":
    fk = FK()

    # matches figure in the handout
    q = np.array([0, 0, 0, -pi / 2, 0, pi / 2, pi / 4])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)

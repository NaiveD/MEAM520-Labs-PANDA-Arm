import numpy as np
import math
from math import pi
# from lib.calculateFK import FK


# def zRotTransformation(a, x, y, z):
#     '''
#         Generate transformation matrix for homogenous z-axis rotations with y and z translation
#         Inputs:
#         a - angle in radians
#         y - translation
#         z - translation
#         Outputs:
#         (4x4) transformation matrix
#     '''
#     return np.matrix(
#         [[math.cos(a), -math.sin(a), 0, x],
#          [math.sin(a), math.cos(a), 0, y],
#          [0, 0, 1, z],
#          [0, 0, 0, 1]])


# def yRotTransformation(a, x, y, z):
#     '''
#         Generate transformation matrix for homogenous y-axis rotations with y and z translation
#         Inputs:
#         a - angle in radians
#         y - translation
#         z - translation
#         Outputs:
#         (4x4) transformation matrix
#     '''
#     return np.matrix(
#         [[math.cos(a), 0, math.sin(a), x],
#          [0, 1, 0, y],
#          [-math.sin(a), 0, math.cos(a), z],
#          [0, 0, 0, 1]])


# def xRotTransformation(a, x, y, z):
#     '''
#         Generate transformation matrix for homogenous y-axis rotations with y and z translation
#         Inputs:
#         a - angle in radians
#         y - translation
#         z - translation
#         Outputs:
#         (4x4) transformation matrix
#     '''
#     return np.matrix(
#         [[1, 0, 0, x],
#          [0, math.cos(a), -math.sin(a), y],
#          [0, math.sin(a), math.cos(a), z],
#          [0, 0, 0, 1]])


def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q: 0 x 7 configuration vector (of joint angles) [q0,q1,q2,q3,q4,q5,q6]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    # ================================== Old =================================
    # # z direction
    # l = np.zeros(8)
    # l[0] = .141
    # l[1] = .192
    # l[2] = .195
    # l[3] = .121
    # l[4] = .083
    # l[5] = 0
    # l[6] = -.051
    # l[7] = -.159
    # lx = np.zeros(8)
    # lx[3] = .082
    # lx[4] = .125
    # lx[5] = .259
    # lx[6] = .088
    # # Joint 6 y-axis offset
    # ly = .015

    # # Transformation Matrices
    # T_B1 = zRotTransformation(q[0], 0, 0, l[0])
    # T_12 = yRotTransformation(q[1], 0, 0, l[1])
    # T_23 = zRotTransformation(q[2], 0, 0, l[2])
    # T_34 = yRotTransformation(-(q[3] + (pi / 2)), lx[3], 0, l[3])
    # T_45 = xRotTransformation(q[4], lx[4], 0, l[4])
    # T_56 = yRotTransformation((-q[5] + (pi / 2)), lx[5], ly, l[5])
    # T_67 = zRotTransformation((-q[6] + (pi / 4)), lx[6], -ly, l[6])
    # # Two tranformation matrices to rotate the coordinate frame around the y and z axis to match desired end-effector frame
    # T_7E = yRotTransformation(pi, 0, 0, l[7])
    # T_7E2 = zRotTransformation(pi, 0, 0, 0)


    # # Find every transfer function at each joint with respect to base frame
    # T = []
    # # Joint 1
    # T.append(T_B1)
    # # Joint 2
    # T.append(np.matmul(T_B1, T_12))
    # # Joint 3
    # T.append(np.matmul(np.matmul(T_B1, T_12), T_23))
    # # Joint 4
    # T.append(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34))
    # # Joint 5
    # T.append(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45))
    # # Joint 6
    # T.append(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56))
    # # Joint 7 Position
    # T.append(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56), T_67))
    # # End Effector
    # T.append(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56), T_67),T_7E), T_7E2))
    # ================================== Old =================================

    # ================================== New =================================
    T_B1 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, 0],
                     [np.sin(q[0]), np.cos(q[0]), 0, 0],
                     [0, 0, 1, 0.141],
                     [0, 0, 0, 1]])
    T_12 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0, 0],
                     [0, 0, 1, 0],
                     [-np.sin(q[1]), -np.cos(q[1]), 0, 0.192],
                     [0, 0, 0, 1]])
    T_23 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0, 0],
                     [0, 0, -1, -0.195],
                     [np.sin(q[2]), np.cos(q[2]), 0, 0],
                     [0, 0, 0, 1]])
    T_34 = np.array([[np.cos(q[3]+pi/2), -np.sin(q[3]+pi/2), 0, 0.082],
                     [0, 0, -1, 0],
                     [np.sin(q[3]+pi/2), np.cos(q[3]+pi/2), 0, 0.121],
                     [0, 0, 0, 1]])
    T_45 = np.array([[0, 0, 1, 0.125],
                     [np.sin(q[4]), np.cos(q[4]), 0, 0.083],
                     [-np.cos(q[4]), np.sin(q[4]), 0, 0],
                     [0, 0, 0, 1]])
    T_56 = np.array([[0, 0, -1, 0.015],
                     [np.sin(q[5]-pi/2), np.cos(q[5]-pi/2), 0, 0],
                     [np.cos(q[5]-pi/2), -np.sin(q[5]-pi/2), 0, 0.259],
                     [0, 0, 0, 1]])
    T_67 = np.array([[np.cos(q[6]-pi/4), -np.sin(q[6]-pi/4), 0, 0.088],
                     [0, 0, -1, -0.051],
                     [np.sin(q[6]-pi/4), np.cos(q[6]-pi/4), 0, 0.015],
                     [0, 0, 0, 1]])
    T_7E = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0.159],
                     [0, 0, 0, 1]])
    
    T = []
    # Joint 1
    T.append(T_B1)
    # Joint 2
    T.append(np.matmul(T_B1, T_12))
    # Joint 3
    T.append(np.matmul(np.matmul(T_B1, T_12), T_23))
    # Joint 4
    T.append(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34))
    # Joint 5
    T.append(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45))
    # Joint 6
    T.append(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56))
    # Joint 7 Position
    T.append(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56), T_67))
    # End Effector
    T.append(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T_B1, T_12), T_23), T_34), T_45), T_56), T_67), T_7E))
    # ================================== New =================================

    # Axis of rotation
    x = np.array([1, 0, 0])
    y = np.array([0, 1, 0])
    z = np.array([0, 0, 1])
    # axis = [z, y, z, -y, x, -y, -z]
    axis = [z, z, z, z, z, z, z]
    # print(T[7][0:3, 3])
    # print(T[6][0:3, 3])
    # print(np.subtract(T[7][0:3, 3], T[6][0:3, 3]))

    # Calculate Jacobian
    J = np.zeros((6, 7))
    for i in range(7):
        R = T[i][:3, :3]
        o = np.subtract(T[7][0:3, 3], T[i][0:3, 3])

        j_vec = np.cross(np.matmul(R, np.array([0, 0, 1])), o)
        J[0:3, i] = j_vec
        
        # Calculates Angular Velocity
        J[3:6, i] = np.matmul(R, axis[i])

    return J


if __name__ == '__main__':
    q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    
# [[ 0.     0.189 -0.     0.127 -0.     0.21   0.   ]
#  [ 0.554  0.     0.554  0.     0.21   0.     0.   ]
#  [ 0.    -0.554  0.     0.472  0.     0.088  0.   ]
#  [ 0.     0.     0.     0.     1.     0.     0.   ]
#  [ 0.     1.     0.    -1.     0.    -1.     0.   ]
#  [ 1.     0.     1.     0.     0.     0.    -1.   ]]
    
    print("q = ", q)
    print("J = ", np.round(calcJacobian(q),3))

    q = np.array([0, 0, 0, 0, 0, 0, 0])

# [[ 0.     0.49   0.    -0.174  0.     0.21   0.   ]
#  [ 0.087  0.     0.087  0.     0.088  0.     0.   ]
#  [ 0.    -0.087  0.     0.005  0.     0.088  0.   ]
#  [ 0.     0.     0.     0.     0.     0.     0.   ]
#  [ 0.     1.     0.    -1.     0.    -1.     0.   ]
#  [ 1.     0.     1.     0.     1.     0.    -1.   ]]

    print("q = ", q)
    print("J = ", np.round(calcJacobian(q),3))



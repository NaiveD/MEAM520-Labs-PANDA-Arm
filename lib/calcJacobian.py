import numpy as np
from lib.calculateFK import FK

def calcJacobian(q, joint):
    """
    Calculate the Jacobian of a particular joint of the  robot in a given configuration
    :param q: 0 x 7 vector of joint inputs [q0, q1,q2,q3,q4,q5,q6]
    :param joint: scalar in [0,7] representing which joint we care about
    :return: J - 6 x 7 matrix representing the Jacobian
    """
    J = np.zeros((6,7))
    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q,7),3))
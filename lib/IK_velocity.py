import numpy as np
from lib.calcJacobian import calcJacobian


def IK_velocity(q_in, v_in, omega_in):
    """
    :param q: 0 x 7 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 0 x 7 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error. If v
         and omega have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """
    #Calculate Jacobian
    J = calcJacobian(q_in.flatten())

    #Calculate xi
    xi = np.concatenate((v_in, omega_in), axis=0)

    #Remove unconstrained rows
    ind = ~np.isnan(xi)
    xi = xi[ind]
    J = J[ind, :]

    #Solve for q dot
    dq = np.linalg.lstsq(J, xi, rcond=None)[0]

    return dq

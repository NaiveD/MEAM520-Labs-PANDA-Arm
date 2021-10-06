import numpy as np
from math import pi

from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.IK_velocity import IK_velocity

class IK():

    def __init__(self):
        fk = FK()

    def inverse(self, target, seed, linear_tol=1e-6, angular_tol=1e-3):
        """
        INPUT:
        target - 4x4 numpy array representing the desired transformation from
        end effector to world
        seed - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], which
        is the "initial guess" from which to proceed with optimization
        linear_tol - the maximum distance between the target end effector origin
        and actual end effector origin for a solution to be considered successful
        linear_tol - the maximum angle of rotation between the target end
        effector frame and actual end effector frame for a solution to be
        considered successful

        OUTPUTS:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6], giving the
        solution if success is True or the closest guess if success is False.
        success - True if the IK algorithm successfully found a configuration
        which achieves the target within the given tolerance. Otherwise False
        """

        # Your code starts here

        q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
        success = False

        # Your code ends here

        return q, success

    # feel free to define additional helper methods to modularize your solution

if __name__ == "__main__":

    ik = IK()

    # matches figure in the handout
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    target = np.array([
        [1,0,0,.5],
        [0,1,0,.5],
        [0,0,1,.5],
        [0,0,0, 1],
    ])

    q, success = ik.inverse(target, seed)

    if success:
        print("Solution found,\nq=",q)
    else:
        print("Solution not found, best guess is\nq =",q)

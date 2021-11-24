import sys
import rospy
import numpy as np
from math import pi
from time import perf_counter

from core.interfaces import ArmController
from    lib.rrt import rrt
from lib.astar import Astar as astar

from lib.loadmap import loadmap
from copy import deepcopy


starts = [np.array([0, -1, 0, -2, 0, 1.57, 0]),
        #   np.array([-1.2, 0.4, 0.7, -1.5, -0.2, 1.8, 0.707]), 
          np.array([-0.16344, -0.84826,  0.12434, -2.61019,  0.09496,  1.76492,  0.68605]),
          np.array([-0.39976,  0.04211, -0.39125, -1.96422,  0.01768,  2.0031 , -0.0127]),
          np.array([-0.16344, -0.84826,  0.12434, -2.61019,  0.09496,  1.76492,  0.68605]), 
          np.array([0, 0, 0, -pi / 2, 0, pi / 2, pi / 4])]
goals = [np.array([-1.2, 1.57, 1.57, -2.07, -1.57, 1.57, 0.7]),
        #  np.array([1.2, 0.4, -0.7, -1.5, 0.2, 1.8, 0.707]), 
         np.array([ -0.5767, -0.0745,  1.2743, -0.0819, -1.0496,  2.2222, -0.8858]),
         np.array([ 0.4272 , 0.04166 , 0.36336 ,-1.96423 ,-0.0163   ,2.00312 , 1.5825]), 
         np.array([ -0.5767, -0.0745,  1.2743, -0.0819, -1.0496,  2.2222, -0.8858]), 
         np.array([ 0.40586,  0.01584,  0.38218, -1.86523, -0.0062,   1.87991,  1.57528])]
mapNames = ["map1",
            "map2",
            "map3",
            "map4",
            "map2"]

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("usage:\n\tpython planning_demo.py 1 rrt \n\tpython planning_demo.py 2 rrt\n\tpython planning_demo.py 3 astar ...")
        exit()

    rospy.init_node('RRT')

    arm = ArmController()
    index = int(sys.argv[1])-1
    print("Running test "+sys.argv[1])
    print("Moving to Start Position")
    # arm.move_to_position(np.array( [-0.41987,  0.07102, -0.38701 ,-1.41488,  0.02689,  1.4807 , -0.01818])) # for map 3
    arm.move_to_position(starts[index])
    map_struct = loadmap("../../maps/"+mapNames[index] + ".txt")
    print("Map = "+ mapNames[index])

    print("Starting to plan")
    start = perf_counter()
    if str(sys.argv[2]) == 'astar':
        print('Planning with A*')
        path = astar(deepcopy(map_struct), deepcopy(starts[index]), deepcopy(goals[index]))
    elif str(sys.argv[2]) == 'rrt':
        print('Planning with rrt')
        path = rrt(deepcopy(map_struct), deepcopy(starts[index]), deepcopy(goals[index]))
    else:
        print('Undefinded planning method')
    stop = perf_counter()
    dt = stop - start
    print("Planning took {time:2.2f} sec. Path is.".format(time=dt))
    print(np.round(path,4))
    input("Press Enter to Send Path to Arm")

    for joint_set in path:
        arm.move_to_position(joint_set)
    print("Trajectory Complete!")

import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy

def scalebox():
    """ 
    Array of how much we should scale each box depending on the link to take the volume of each link into account.
    """
    pass

def sampleRandom(lowerLim, upperLim):
    """ 
    Sample a random point in the workspace, and use IK to convert it into the configuration space

    Or just

    Sample a random point in the configuration space
    """
    
    sample = []
    for i in range(len(lowerLim)):
        sample.append(lowerLim[i] + (upperLim[i] - lowerLim[i]) * np.random.random_sample())
    return np.array(sample)

def getincrementPoint():
    """

    """
    pass

def checkCollision():
    """
    TODO Amar
    """
    pass


def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """

    # initialize path
    path = []

    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    obstacles = map.obstacles

    terminate = False
    while not terminate:
        # Randomly sample a point in the configuration space
        newSample = sampleRandom(lowerLim, upperLim)






    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

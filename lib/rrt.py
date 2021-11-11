import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy

class Node:
    def __init__(self, data):
        self.data = data
        self.parent = None
        self.child = None

class Tree:
    def __init__(self, root):
        self.root = root
        self.vertices = [root]
    
    def addNode(self, NewPoint):
        NewPoint.parent = self.getNearestNode(NewPoint)
        self.vertices.append(NewPoint)

    def getNearestNode(self, NewPoint):
        NearestNode = self.vertices[0]
        dist = getDistance(self.vertices[0], NewPoint)
        for eachNode in self.vertices:
            if getDistance(eachNode, NewPoint) < dist:
                NearestNode = eachNode
                dist = getDistance(eachNode, NewPoint)
        return NearestNode

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

def checkPathCollision(EndPoint1, EndPoint2):
    """
    TODO Keyan
    Returns true if there are no collisions between the path and the obstacles and it's ok.
    Returns false if there are collisions between the path and the obstacles and it's not ok.
    """
    collide_ok = True
    
    dq = 0.01 # The minimum step (needs to be tuned by testing)
    dist = np.linalg.norm(EndPoint1-EndPoint2) # The distance between the two end points of the path
    num_points = int(dist/dq)
    # print("dist = ", dist)
    # print("dq = ", dq)
    # print("num_points = ", num_points)
    t = 1 / num_points
    
    for i in range(num_points+1):
        inter_point = EndPoint1 + i * t * (EndPoint2 - EndPoint1)
        if (not checkPointCollision(inter_point)):
            collide_ok = False
            return collide_ok
        # else:
        #     print("%d-th intermediate point, collision ok!" % i)
    
    return collide_ok
    
def checkPointCollision(Point):
    """
    TODO Amar
    """
    return True

def retrievePath(startTree, endTree):
    pass

def getDistance(Point1, Point2):
    """
    Get the distance between two points. Used for finding the closest point to the new point in the tree.
    """
    q1 = Point1.data
    q2 = Point2.data

    dist = np.linalg.norm(q1-q2)
    return dist

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
    startNode = Node(start)
    goalNode = Node(goal)
    startTree = Tree(startNode)
    goalTree = Tree(goalNode)
    while not terminate:
        # Randomly sample a new point in the configuration space
        # Note: Point is just the joint angles data, Node is in the tree
        NewPoint = sampleRandom(lowerLim, upperLim)
        NewNode = Node(NewPoint)
        print(NewPoint)

        # If this NewPoint is not in the free configuration space, then sample a new point
        if (not checkPointCollision(NewPoint)):
            continue

        # If this NewPoint is in the free configuration space
        # Draw a line between the nearest point in the starting tree and the NewPoint
        ST_NearestPoint = startTree.getNearestNode(NewNode).data # Get the nearest point in the starting tree
        GT_NearestPoint = goalTree.getNearestNode(NewNode).data # Get the nearest point in the goal tree

        # If there are no collisions then add the new point to the starting tree
        if (checkPathCollision(ST_NearestPoint, NewPoint)):
            startTree.addNode(NewNode)
        if (checkPathCollision(GT_NearestPoint, NewPoint)):
            goalTree.addNode(NewNode)
            terminate = True

    # Retrieve the path from the startTree and the goalTree
    path = retrievePath(startTree, goalTree)

    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

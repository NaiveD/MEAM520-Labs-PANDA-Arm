import numpy as np
import random
from lib.detectCollision import detectCollision
from lib.calculateFK import FK
from lib.loadmap import loadmap
from copy import deepcopy

class Node:
    def __init__(self, data):
        self.data = data
        self.parent = None

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

def scalebox(obstacle):
    """
    Returns a scaled up box for the obstacle
    :param obstacle: [0x6] array based on the obstacle struct
    :return: [0x6] list of the box parameters
    """
    # Increase the box by this size
    scale = .1
    box = []
    for i in range(6):
        # Make the box slightly larger than the obstacle to account for arm volume
        if i < 3:
            box.append((obstacle[i] - scale))
        else:
            box.append((obstacle[i] + scale))
    return box

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

def checkPathCollision(EndPoint1, EndPoint2, obstacles):
    """
    TODO Keyan
    Returns false if there are collisions between the path and the obstacles and it's not ok.
    Returns true if there are no collisions between the path and the obstacles and it's ok.
    """
    collide = False
    
    dq = 0.1 # The minimum step (needs to be tuned by testing)
    dist = np.linalg.norm(EndPoint1-EndPoint2) # The distance between the two end points of the path
    num_points = int(dist/dq)
    t = 1 / num_points
    
    for i in range(num_points+1):
        inter_point = EndPoint1 + i * t * (EndPoint2 - EndPoint1)
        if (checkPointCollision(inter_point, obstacles)):
            collide = True
            return collide
        # else:
        #     print("%d-th intermediate point, collision ok!" % i)
    
    return collide

def getJointPos(Point):
    """
    Helper function to get all the points needed for collision checking
    :param Point: [0x7] joint configuration
    :return: 9x3 matrix of all the points in order from 0 to end-effector
    """
    fk = FK()
    JP, EE = fk.forward(Point)
    points = np.zeros((10,3))
    points[1:8, :] = JP
    points[9,:] = EE.T[0,0:3]
    return points

def checkPointCollision(Point, obstacles):
    """
    Checks if a the link collides with a object
    :param Point: [0x7] joint configuration
    :param obstacles: nx6 matrix of the obstacles
    :return: True if the arm collides with an object
    """
    # Get all the points for the links
    points = getJointPos(Point)

    # Initialize Collision
    collision = False

    # Check collision for each object
    for i in range(obstacles.shape[0]):
        box = scalebox(obstacles[i, :])
        # If there is a collision then return true
        if detectCollision(points[0:8, :], points[1:9, :], box):
            collision = True
            break

    return collision

def retrievePath(startTree, goalTree):
    path = []
    ST_new_Node = startTree.vertices.pop()
    path.append(ST_new_Node.data)
    ST_parent_node = ST_new_Node.parent

    while (ST_parent_node != startTree.root):
        path.insert(0, ST_parent_node.data)
        ST_parent_node = ST_parent_node.parent
    path.insert(0, startTree.root.data)

    GT_new_Node = goalTree.vertices.pop()
    GT_parent_node = GT_new_Node.parent
    while (GT_parent_node != goalTree.root):
        path.append(GT_parent_node.data)
        GT_parent_node = GT_parent_node.parent
    path.append(goalTree.root.data)

    return path


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
        addST = False
        addGT = False

        # If this NewPoint is not in the free configuration space, then sample a new point
        if (checkPointCollision(NewPoint, obstacles)):
            print("NewPoint has collision, sample next.")
            continue
            
        print("NewPoint: ", NewPoint)

        # If this NewPoint is in the free configuration space
        # Draw a line between the nearest point in the starting tree and the NewPoint
        ST_NearestPoint = startTree.getNearestNode(NewNode).data # Get the nearest point in the starting tree
        GT_NearestPoint = goalTree.getNearestNode(NewNode).data # Get the nearest point in the goal tree

        # If there are no collisions then add the new point to the starting tree
        if (not checkPathCollision(ST_NearestPoint, NewPoint, obstacles)):
            startTree.addNode(NewNode)
            addST = True
            print("New Point add to start tree.")

        if (not checkPathCollision(GT_NearestPoint, NewPoint, obstacles)):
            goalTree.addNode(NewNode)
            addGT = True
            print("New Point add to goal tree.")
        
        if (addST and addGT):
            terminate = True
            

    # Retrieve the path from the startTree and the goalTree
    path = retrievePath(startTree, goalTree)
    print("path = ", path)

    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

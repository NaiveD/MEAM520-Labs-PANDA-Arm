#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Writen Shane Rozen-Levy 10/3/20
"""

import numpy as np
from collections import namedtuple


def loadmap(filename):
    """
    :param filename: string with the location of the map file
    :return: map struct with obstacles element
             map.obstacles [Nx6] array of the obstacle boundaries
    """
    obstacles = []
    with open(filename, 'r') as reader:
        # Read file line by line
        line = reader.readline()
        while line != '':  # The EOF char is an empty string
            line = reader.readline()
            # Check to see if first character is b for boundary or block
            if len(line) > 0 and line[0] == 'b':
                words = line.split()
                # Check if block our boundary
                if words[0] == "block":
                    # Append to obstacles array or set to obstacles array
                    if len(obstacles) == 0:
                        obstacles = np.array([[float(words[i]) for i in range(1, len(words))]])
                    else:
                        obstacles = np.append(obstacles, np.array([[float(words[i]) for i in range(1, len(words))]]), axis=0)
    # Returns the map in a struct
    MyStruct = namedtuple("map", "obstacles")
    return MyStruct(obstacles = obstacles)

# # For testing
# if __name__ == "__main__":
#     map = loadmap("../maps/map1.txt")
#     print(map)
#     print(map.obstacles)
#     # map1.txt
#     # [[ 0.15     -0.3       0.496825  0.45      0.3       0.503175]]
#     # map2.txt 
#     # [[ 0.18     0.19665 -0.25     0.69     0.20335  0.65   ]
#     # [ 0.18    -0.20335 -0.25     0.69    -0.19665  0.65   ]]
#     print(map.obstacles.shape)

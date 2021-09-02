# This is a script for plotting the pose results
# from a raw poses txt file.

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.path as mpath
from matplotlib import collections as mc
import sys


def processPoseLine(line):
    """"Process a text line the pose format
        # timestamp x y z roll pitch yaw id
        input: line
        output: float x, float y, string id"""

    spaceCount = 0
    x = ""
    y = ""
    id = ""

    for char in line: 
        if char == ",":
            char = "."
        if char == " ":
            spaceCount += 1
        elif spaceCount == 1:
            x += char
        elif spaceCount == 2: 
            y += char
        elif spaceCount == 7 and char != "\n": 
            id += char

    fx = float(x)
    fy = float(y)

    return fx, fy, id

def processConstraintLine(line):
    """"Process a text line with the constraint format
        # idFrom idTo0 idTo1 ...
        input: line 
        output: string idFrom, list IdTo"""

    spaceCount = 0
    idFrom = ""
    idTo = ""

    for char in line: 
        if spaceCount > 0 and char != "\n": 
            idTo += char
        if char == " ":
            spaceCount += 1
        elif spaceCount == 0:
            idFrom += char
    
    idToList = list(idTo.split(" "))
    idToList = [x for x in idToList if x] # erase possible empty elements 

    return idFrom, idToList

def processLandmarkLine(line):
    """"Process a text line the landmark format
        # x y z roll pitch yaw id
        input: line
        output: float x, float y, string id"""

    spaceCount = 0
    x = ""
    y = ""
    id = ""

    for char in line: 
        if char == ",":
            char = "."
        if char == " ":
            spaceCount += 1
        elif spaceCount == 0:
            x += char
        elif spaceCount == 1: 
            y += char
        elif spaceCount == 6 and char != "\n": 
            id += char

    fx = float(x)
    fy = float(y)

    return fx, fy, id

def updatePlotLimits(x, y, minXLim, maxXLim, minYLim, maxYLim):
    """"Update plot limits.
        input: x, y
        output: float x, float y, updated spaceCount"""

    fx = float(x)
    fy = float(y)

    # Update plot limits
    if fx < minXLim:
        minXLim = fx   
    if fx > maxXLim:
        maxXLim = fx   
    if fy < minYLim:
        minYLim = fy   
    if fy > maxYLim:
        maxYLim = fy 
    
    return minXLim, maxXLim, minYLim, maxYLim

def main():
    
    rawPoses = sys.argv[1]
    # rawPoses = 'rawposes.txt' 
    
    fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot

    minXLim = 0
    maxXLim = 1
    minYLim = 0
    maxYLim = 1
    radius = 0.05 

    # list of x and y of the trajectory
    trajX = []
    trajY = []

    # dictionary with the robot poses 
    nodePoses = {}
    # dictionary with the landmark poses
    lmPoses = {}

    # Get the trajectory
    with open(rawPoses, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline() # get rid off the comment
        line = reader.readline()
        
        # Process the nodes and trajectory
        while line != '\n':  # Until the first empty line          
            fx, fy, id = processPoseLine(line)
            minXLim, maxXLim, minYLim, maxYLim = updatePlotLimits(fx, fy, minXLim, maxXLim, minYLim, maxYLim)                      

            # Save the trajectory to plot it after
            trajX.append(fx)
            trajY.append(fy)
            nodePoses.update({id: (fx, fy)})

            # Plot the nodes
            circle = plt.Circle((fx, fy), radius = radius, fc = 'r', lw = radius/10, zorder = 2)        
            # circle = plt.Circle((fx, fy), radius = radius, ec = 'w', fc = 'r', lw = radius/10, zorder = 2)        
            ax.add_patch(circle)

            line = reader.readline()
        
        # Process the constraints
        idFrom = ""
        idTo = []
        line = reader.readline() # get rid off the comment
        line = reader.readline() 
        while line != '\n':  # The EOF char is an empty string          
            idFrom, idTo = processConstraintLine(line)
            poseFrom = nodePoses[idFrom]
            for id in idTo:
                step = abs(int(id) - int(idFrom))
                if step > 1:
                    poseTo = nodePoses[id]
                    ax.plot([poseFrom[0], poseTo[0]], [poseFrom[1], poseTo[1]], zorder=3, color='purple', linewidth=0.5)

            line = reader.readline()

        # Process the landmarks
        line = reader.readline() # get rid off the comment
        line = reader.readline() 
        while line != '\n':  # Until the first empty line          
            fx, fy, id = processLandmarkLine(line)
            minXLim, maxXLim, minYLim, maxYLim = updatePlotLimits(fx, fy, minXLim, maxXLim, minYLim, maxYLim)                      

            # Plot the nodes
            ax.plot(fx, fy, 'r*', zorder=3)
            lmPoses.update({id: (fx, fy)})

            line = reader.readline()

        # Process the landmark constraints
        idFrom = ""
        idTo = []
        line = reader.readline() # get rid off the comment
        line = reader.readline() 
        while line != '':  # The EOF char is an empty string     
            idFrom, idTo = processConstraintLine(line)
            poseFrom = lmPoses[idFrom]
            for id in idTo:
                poseTo = nodePoses[id]
                ax.plot([poseFrom[0], poseTo[0]], [poseFrom[1], poseTo[1]], '--', zorder=1, linewidth=0.5)

            line = reader.readline()

    # plot the trajectory
    ax.plot(trajX, trajY, zorder=1, color='black')
  
    ax.set_xlim((min(minXLim, minYLim) - 2*radius, max(maxXLim,maxYLim) + 2*radius))
    ax.set_ylim((min(minXLim, minYLim) - 2*radius, max(maxXLim,maxYLim) + 2*radius))
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title('Modular framework result')

    # fig.savefig(rawPoses[:-4]+'.png')
    plt.savefig(rawPoses[:-4]+'.eps', format='eps')



if __name__ == "__main__":
    main()

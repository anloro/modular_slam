# This is a script for plotting the pose results
# from a raw poses txt file.

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.path as mpath
from matplotlib import collections as mc
import sys


def processLine(line):
    """"Process a text line with rtabmap RGBDid format.
        input: line, int spaceCount
        output: float x, float y, updated spaceCount"""

    spaceCount = 0
    x = ""
    y = ""
    for char in line: 
        if char == ",":
            char = "."
        if char == " ":
            spaceCount += 1
        elif spaceCount == 1:
            x += char
        elif spaceCount == 2: 
            y += char
        elif spaceCount == 5: 
            theta = 0
            # theta += char

    fx = float(x)
    fy = float(y)

    return fx, fy

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
    countLines = 0

    # Get the trajectory
    with open(rawPoses, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        line = reader.readline() # we don't want the first line
        # list of lines
        trajX = []
        trajY = []
        while line != '':  # The EOF char is an empty string          
            fx, fy = processLine(line)
            minXLim, maxXLim, minYLim, maxYLim = updatePlotLimits(fx, fy, minXLim, maxXLim, minYLim, maxYLim)                      

            if countLines != 0:
                trajX.append(fx)
                trajY.append(fy)

            line = reader.readline()
            countLines += 1

    # lc = mc.LineCollection(lines, color='gray', linewidths=2)
    # ax.add_collection(lc)
    ax.plot(trajX, trajY, zorder=1, color='black')

    # Get nodes
    with open(rawPoses, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        line = reader.readline() # we don't want the first line
        while line != '':  # The EOF char is an empty string
            fx, fy = processLine(line)
            minXLim, maxXLim, minYLim, maxYLim = updatePlotLimits(fx, fy, minXLim, maxXLim, minYLim, maxYLim)          
              
            circle = plt.Circle((fx, fy), radius = radius, ec = 'w', fc = 'r', lw = radius/10, zorder = 2)        
            ax.add_patch(circle)

            line = reader.readline()
            countLines += 1
  
    ax.set_xlim((min(minXLim, minYLim) - 2*radius, max(maxXLim,maxYLim) + 2*radius))
    ax.set_ylim((min(minXLim, minYLim) - 2*radius, max(maxXLim,maxYLim) + 2*radius))
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title('RTAB-Map result')

    # fig.savefig(rawPoses[:-4]+'.png')
    plt.savefig(rawPoses[:-4]+'.eps', format='eps')



if __name__ == "__main__":
    main()

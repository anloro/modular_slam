# This is a script for plotting the pose results
# from a raw poses txt file.

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.path as mpath
import sys


def main():
    
    rawPoses = sys.argv[1]
    # rawPoses = 'rawposes.txt' 
    
    fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot

    minXLim = 0
    maxXLim = 1
    minYLim = 0
    maxYLim = 1
    radius = 0.1 
    countLines = 0

    with open(rawPoses, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        line = reader.readline() # we don't want the first line
        while line != '':  # The EOF char is an empty string
            countSpaces = 0
            x = ""
            y = ""
            theta = ""
            for char in line: 
                if char == " ":
                    countSpaces += 1
                if countSpaces == 1:
                    x += char
                if countSpaces == 2: 
                    y += char
                if countSpaces == 5: 
                    theta = 0
                    # theta += char
            
            fx = float(x)
            fy = float(y)
            ftheta = float(theta) -1.5708
            # Update plot limits
            if fx < minXLim:
                minXLim = fx   
            if fx > maxXLim:
                maxXLim = fx   
            if fy < minYLim:
                minYLim = fy   
            if fy > maxYLim:
                maxYLim = fy           

            if countLines != 0:
                arrow = mpatches.Arrow(lastX, lastY, dx = fx-lastX, dy = fy-lastY, width=0.1, color='gray')
                ax.add_patch(arrow)
            
            circle = plt.Circle((fx, fy), radius, color='r')
            triangle = mpatches.RegularPolygon((fx, fy), numVertices = 3, radius = radius, orientation = ftheta)           
            ax.add_patch(triangle)
            
            lastX = fx
            lastY = fy

            line = reader.readline()
            countLines += 1


    # (or if you have an existing figure)
    # fig = plt.gcf()
    # ax = fig.gca()
    ax.set_xlim((min(minXLim, minYLim) - 2*radius, max(maxXLim,maxYLim) + 2*radius))
    ax.set_ylim((min(minXLim, minYLim) - 2*radius, max(maxXLim,maxYLim) + 2*radius))
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title('RTAB-Map result')

    # fig.savefig('rawposes.png')
    fig.savefig(rawPoses[:-4]+'.png')



if __name__ == "__main__":
    main()

# This is a script for plotting the pose results
# from a raw poses txt file.

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.path as mpath
from matplotlib import collections as mc
import argparse


class ModularSlamGraph:
    """Contains the information of the graph."""
    
    def __init__(self):
        # dictionary with the robot poses 
        self.nodePoses = {}
        # list of constraints between nodes
        self.nodeConstraints = []
        # dictionary with the landmark poses
        self.lmPoses = {}
        # list of constraints between nodes and landmarks
        self.landmarkConstraints = []
        # lists with the trajectory
        self.trajX = []
        self.trajY = []

class ModularSlamGraphReader:
    """Gets the path to a raw graph and reads its content."""

    def __init__(self, graphPath: str):
        self.reader = open(graphPath, 'r')
        self.graph = ModularSlamGraph()
        self.processFile()
        self.reader.close()

    def processPoseLine(self, line):
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

    def processConstraintLine(self, line):
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

    def processLandmarkLine(self, line):
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

    def processNodes(self):
        line = self.reader.readline() # get rid off the comment
        line = self.reader.readline()
        
        # Process the nodes and trajectory
        while line != '\n':  # Until the first empty line          
            fx, fy, id = self.processPoseLine(line)
            self.graph.trajX.append(fx)
            self.graph.trajY.append(fy)
            self.graph.nodePoses.update({id: (fx, fy)})
            line = self.reader.readline()
    
    def processNodeConstraints(self):
        # Process the constraints
        idFrom = ""
        idTo = []
        line = self.reader.readline() # get rid off the comment
        line = self.reader.readline() 
        while line != '\n':  # The EOF char is an empty string          
            idFrom, idTo = self.processConstraintLine(line)
            self.graph.nodeConstraints.append((idFrom, idTo))
            line = self.reader.readline()
    
    def processLandmarks(self):
        # Process the landmarks
        line = self.reader.readline() # get rid off the comment
        line = self.reader.readline() 
        while line != '\n':  # Until the first empty line          
            fx, fy, id = self.processLandmarkLine(line)
            self.graph.lmPoses.update({id: (fx, fy)})
            line = self.reader.readline()

    def processLandmarkConstraints(self):
        # Process the landmark constraints
        idFrom = ""
        idTo = []
        line = self.reader.readline() # get rid off the comment
        line = self.reader.readline() 
        while line != '':  # The EOF char is an empty string     
            idFrom, idTo = self.processConstraintLine(line)
            self.graph.landmarkConstraints.append((idFrom, idTo))
            line = self.reader.readline()

    def processFile(self):
        """Process a graph file sequentially."""
        self.processNodes()
        self.processNodeConstraints()
        self.processLandmarks()
        self.processLandmarkConstraints()

class ModularSlamGraphPlotter:
    """Plot a graph"""
    def __init__(self, graph=ModularSlamGraph(), title="", path=""):
        # Plot parameters
        self.minXLim = 0
        self.maxXLim = 1
        self.minYLim = 0
        self.maxYLim = 1
        self.radius = 0.05
        self.title = title
        self.trajectoryFormat = 'midnightblue'
        self.trajectoryName = ""
        self.trajecotryWidth = 1.5
        self.landmarkColor = 'royalblue'
        self.nodeColor = 'royalblue'
        # Graph to plot 
        self.graph = graph
        self.path = path

        fig, self.ax = plt.subplots() # note we must use plt.subplots, not plt.subplot


    def updatePlotLimits(self, x, y):
        """"Update plot limits."""

        fx = float(x)
        fy = float(y)

        # Update plot limits
        if fx < self.minXLim:
            self.minXLim = fx   
        if fx > self.maxXLim:
            self.maxXLim = fx   
        if fy < self.minYLim:
            self.minYLim = fy   
        if fy > self.maxYLim:
            self.maxYLim = fy 

    def plotNodes(self):
        """Plot the nodes."""
        # Get the whole trajectory
        poses = list(self.graph.nodePoses.values())
        for pose in poses:
            x = pose[0]
            y = pose[1]
            self.updatePlotLimits(x, y)
            # Plot the individual nodes
            circle = plt.Circle((x, y), radius = self.radius, fc = self.nodeColor, lw = self.radius/10, zorder = 3)        
            self.ax.add_patch(circle)
    
    def plotTrajectory(self):
        """Plot the trajectory."""
        # plot the trajectory
        trajX = self.graph.trajX
        trajY = self.graph.trajY
        self.updatePlotLimits(min(trajX), min(trajY))
        self.updatePlotLimits(max(trajX), max(trajY))
        if self.trajectoryFormat:
            self.ax.plot(trajX, trajY, self.trajectoryFormat, lw = self.trajecotryWidth, zorder=1, label=self.trajectoryName)
        else:
            self.ax.plot(trajX, trajY, lw = self.trajecotryWidth, zorder=1, label=self.trajectoryName)

        
    
    def plotNodeConstraints(self):
        """Plot constraints between nodes (only the loop closures)."""
        constraints = self.graph.nodeConstraints
        for constraint in constraints:
            idFrom = constraint[0]
            idTo = constraint[1] #idTo is a list
            poseFrom = self.graph.nodePoses[idFrom]
            for id in idTo:
                step = abs(int(id) - int(idFrom))
                # Only plot loop closures
                if step > 1:
                    poseTo = self.graph.nodePoses[id]
                    self.ax.plot([poseFrom[0], poseTo[0]], [poseFrom[1], poseTo[1]], zorder=2, color='red', linewidth=0.5)

    def plotLandmarks(self):
        """Plot the landmarks."""
        poses = list(self.graph.lmPoses.values())
        for pose in poses:
            x = pose[0]
            y = pose[1]
            self.updatePlotLimits(x, y)                      
            # Plot the landmark poses
            if self.landmarkColor:
                self.ax.plot(x, y, '*', color=self.landmarkColor, zorder=3)
            else:
                self.ax.plot(x, y, '*', zorder=3)

    def plotLandmarkConstraints(self):
        """Plot the constraints between nodes and landmarks"""
        constraints = self.graph.landmarkConstraints
        for constraint in constraints:
            idFrom = constraint[0]
            idTo = constraint[1] #idTo is a list
            poseFrom = self.graph.lmPoses[idFrom]
            for id in idTo:
                poseTo = self.graph.nodePoses[id]
                self.ax.plot([poseFrom[0], poseTo[0]], [poseFrom[1], poseTo[1]], '-', zorder=0, linewidth=0.3)
    
    def configurePlot(self):
        mn = min(self.minXLim, self.minYLim)
        mx = max(self.maxXLim,self.maxYLim)
        print((mn,mx))
        offset = 1
        self.ax.set_xlim((mn - offset, mx + offset))
        self.ax.set_ylim((mn - offset, mx + offset))
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title(self.title)
        if self.trajectoryName:
            plt.legend(loc='best')

    def plot(self):
        self.plotNodes()
        self.plotTrajectory()
        self.plotNodeConstraints()
        self.plotLandmarks()
        self.plotLandmarkConstraints()
        self.configurePlot()

    def addGraph(self, graph):
        self.graph = graph
        self.plot()

    def addTrajectory(self, graph=ModularSlamGraph(), format='', lw='1.5', trajectoryName=""):
        self.graph = graph
        self.trajectoryFormat = format
        self.trajectoryName = trajectoryName
        self.trajecotryWidth = lw
        self.plotTrajectory()
        self.configurePlot()

    def addLandmarks(self, graph=ModularSlamGraph(), color=''):
        self.graph = graph
        self.landmarkColor = color
        self.plotLandmarks()
        self.configurePlot()

    def savePlot(self):
        plt.savefig(self.path +'.eps', format='eps')



def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("poses", nargs='+',
                        help="plot the poses in the specified .txt file using the modular slam framework format")
    parser.add_argument("-t", "--title",
                        help="set the title of the plot")
    parser.add_argument("-gt", "--groundTruth",
                        help="add the ground truth in the specified .txt file to the plot")

    args = parser.parse_args()

    # Get the title
    if args.title:
        title = args.title
    else:
        title = "Modular framework result"

    # Process multiple graphs and make a trajectory comparison 
    if len(args.poses) > 1:
        title = "Comparison of trajectories"
        print(title) 

        plotter = ModularSlamGraphPlotter(title = title)
        # legend = ["experiment 1", "experiment 2", "experiment 3"]
        legend = ["RTAB-Map", "experiment 1", "experiment 2"]
        i = 1
        for poses in args.poses:
            readPath = poses
            reader = ModularSlamGraphReader(readPath)
            graph = reader.graph
            # trajectoryName = "experiment " + str(i)
            trajectoryName = legend[i-1]
            plotter.addTrajectory(graph, trajectoryName=trajectoryName)
            i = i + 1
        
        pos = readPath.rfind('/')
        name = "trajectoryComparison"
        if(pos > -1):
            folder = readPath[:pos]
            writePath = folder + name
        else:
            writePath = name
        plotter.path = writePath
    
    # Get a single graph and plot everything
    else:
        readPath = args.poses[0]

        reader = ModularSlamGraphReader(readPath)
        graph = reader.graph

        writePath = readPath[:-4]

        plotter = ModularSlamGraphPlotter(graph, title, writePath)
        # plotter.trajectoryName = title
        plotter.plot()

    if args.groundTruth:
        reader = ModularSlamGraphReader(args.groundTruth)
        gtGraph = reader.graph
        plotter.addTrajectory(gtGraph, 'k--', 1, "ground truth")
        plotter.addLandmarks(gtGraph, 'k')

    plotter.savePlot()

if __name__ == "__main__":
    main()

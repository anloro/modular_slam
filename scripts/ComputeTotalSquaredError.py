# This is a script for plotting the pose results
# from a raw poses txt file.

import sys
import numpy as np

def processPoseLine(line):
    """"Process a text line the pose format
        # timestamp x y z roll pitch yaw id
        input: line
        output: float x, float y, string id, float timeStamp"""

    spaceCount = 0
    x = ""
    y = ""
    id = ""
    timeStamp = ""

    for char in line: 
        if char == ",":
            char = "."
        if char == " ":
            spaceCount += 1
        elif spaceCount == 0:
            timeStamp += char
        elif spaceCount == 1:
            x += char
        elif spaceCount == 2: 
            y += char
        elif spaceCount == 7 and char != "\n": 
            id += char

    fx = float(x)
    fy = float(y)
    timeStamp = float(timeStamp)

    return fx, fy, id, timeStamp

def main():
    
    groundTruth = sys.argv[1]
    queryPoses = sys.argv[2]
    # groundTruth = "/media/angel/8408bac5-959f-44d6-adcd-ef40b74f0fc5/master_thesis/02_results_single_loop_w4/poses_gt.txt"
    # queryPoses = "/media/angel/8408bac5-959f-44d6-adcd-ef40b74f0fc5/master_thesis/02_results_single_loop_w4/poses_rtabmap.txt"

    xListGt = np.array([])
    yListGt = np.array([])
    xListQuery = np.array([])
    yListQuery = np.array([])

    nodePosesGt = {}
    nodeStampsGt = {}
    nodeStampsQuery = {}
    nodePosesQuery = {}

    with open(groundTruth, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        line = reader.readline()
        while line != '\n':  # The end of the poses is an empty line
            fx, fy, id, timeStamp = processPoseLine(line)
            
            nodePosesGt.update({id: (fx, fy)})
            nodeStampsGt.update({id: timeStamp})

            line = reader.readline()


    with open(queryPoses, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        line = reader.readline()
        while line != '\n':  # The end of the poses is an empty line
            fx, fy, id, timeStamp = processPoseLine(line)

            nodePosesQuery.update({id: (fx, fy)})
            nodeStampsQuery.update({id: timeStamp})

            line = reader.readline()

    # Get the closest nodes using the timeStamp
    for query in nodeStampsQuery.values():
        timeDif = np.array(list(nodeStampsGt.values())) - query
        argMin = np.argmin(abs(timeDif))
        # Check for a time difference of less than 1 sec
        if np.amin(abs(timeDif)) < 1:
            # Get the ID of the current compared query
            queryPos = list(nodeStampsQuery.values()).index(query)
            queryId = list(nodeStampsQuery.keys())[queryPos]
            # Get its value and put it in an array for compare with gt
            fx = nodePosesQuery[queryId][0]
            fy = nodePosesQuery[queryId][1]
            xListQuery = np.append(xListQuery, fx)
            yListQuery = np.append(yListQuery, fy)            
            # Do the same for the Gt
            gtId = list(nodeStampsGt.keys())[argMin]
            fx = nodePosesGt[gtId][0]
            fy = nodePosesGt[gtId][1]
            xListGt = np.append(xListGt, fx)
            yListGt = np.append(yListGt, fy) 


    xError = xListGt - xListQuery
    xErrorSqr = [number ** 2 for number in xError]
    yError = yListGt - yListQuery
    yErrorSqr = [number ** 2 for number in yError]

    totalError = np.sqrt(xErrorSqr + yErrorSqr)

    # print("The total accumulated Error is: " + str(np.sum(totalError)))
    print("The mean is: " + str(np.mean(totalError)))
    print("The median is: " + str(np.median(totalError)))
    print("The standard deviation is: " + str(np.std(totalError)))




if __name__ == "__main__":
    main()

# This is a script for plotting the pose results
# from a raw poses txt file.

import sys
import numpy as np
from scipy import interpolate

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

    nodePosesGt = {} # {id: (x,y)}
    nodeStampsGt = {} # {id: timeStamp}
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
    # for query in nodeStampsQuery.values():
    #     timeDif = np.array(list(nodeStampsGt.values())) - query
    #     argMin = np.argmin(abs(timeDif))
    #     # Check for a time difference of less than 1 sec
    #     if np.amin(abs(timeDif)) < 1:
    #         # Get the ID of the current compared query
    #         queryPos = list(nodeStampsQuery.values()).index(query)
    #         queryId = list(nodeStampsQuery.keys())[queryPos]
    #         # Get its value and put it in an array for compare with gt
    #         fx = nodePosesQuery[queryId][0]
    #         fy = nodePosesQuery[queryId][1]
    #         xListQuery = np.append(xListQuery, fx)
    #         yListQuery = np.append(yListQuery, fy)            
    #         # Do the same for the Gt
    #         gtId = list(nodeStampsGt.keys())[argMin]
    #         fx = nodePosesGt[gtId][0]
    #         fy = nodePosesGt[gtId][1]
    #         xListGt = np.append(xListGt, fx)
    #         yListGt = np.append(yListGt, fy) 

    # xError = xListGt - xListQuery
    # xErrorSqr = [number ** 2 for number in xError]
    # yError = yListGt - yListQuery
    # yErrorSqr = [number ** 2 for number in yError]

    # transAbsError = np.sqrt(xErrorSqr + yErrorSqr)

    # # print("The total accumulated Error is: " + str(np.sum(transAbsError)))
    # print("The mean is: " + str(np.mean(transAbsError)))
    # print("The median is: " + str(np.median(transAbsError)))
    # print("The standard deviation is: " + str(np.std(transAbsError)))

    # SquaredtransAbsError = [number ** 2 for number in transAbsError]
    # print("The ATE (RMSE) is: " + str(np.sqrt(np.mean(SquaredtransAbsError))))

    xListGt = []
    yListGt = []
    xListGt = [pose[0] for pose in list(nodePosesGt.values())]
    yListGt = [pose[1] for pose in list(nodePosesGt.values())]
    xListQuery = []
    yListQuery = []
    xListQuery = np.array([pose[0] for pose in list(nodePosesQuery.values())])
    yListQuery = np.array([pose[1] for pose in list(nodePosesQuery.values())])
    # Try interpolating
    timeStampListGt = list(nodeStampsGt.values())
    tckX = interpolate.splrep(timeStampListGt, xListGt, s=0)
    tckY = interpolate.splrep(timeStampListGt, yListGt, s=0)
    
    timeStampListQuery = list(nodeStampsQuery.values())
    xnew = interpolate.splev(timeStampListQuery, tckX, der=0)
    ynew = interpolate.splev(timeStampListQuery, tckY, der=0)

    # Get the absolute translational error
    xError = xnew - xListQuery
    xErrorSqr = [number ** 2 for number in xError]
    yError = ynew - yListQuery
    yErrorSqr = [number ** 2 for number in yError]

    transAbsError = np.sqrt(xErrorSqr + yErrorSqr)

    # Get the relative translational error
    dxGt = xnew[:-1] - xnew[1:]
    dyGt = ynew[:-1] - ynew[1:]
    dxQuery = xListQuery[:-1] - xListQuery[1:]
    dyQuery = yListQuery[:-1] - yListQuery[1:]

    xError = dxGt - dxQuery
    xErrorSqr = [number ** 2 for number in xError]
    yError = dyGt - dyQuery
    yErrorSqr = [number ** 2 for number in yError]

    transRelError = np.sqrt(xErrorSqr + yErrorSqr)

    print("The mean is: " + str(np.mean(transAbsError)))
    print("The median is: " + str(np.median(transAbsError)))
    print("The standard deviation is: " + str(np.std(transAbsError)))

    SquaredTransAbsError = [number ** 2 for number in transAbsError]
    SquaredTransRelError = [number ** 2 for number in transRelError]
    print("The ATE (RMSE) is: " + str(np.sqrt(np.mean(SquaredTransAbsError))))
    print("The RPE (RMSE) is: " + str(np.sqrt(np.mean(SquaredTransRelError))))
    

if __name__ == "__main__":
    main()

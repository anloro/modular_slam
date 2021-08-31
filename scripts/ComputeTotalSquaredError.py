# This is a script for plotting the pose results
# from a raw poses txt file.

import sys
import numpy as np

def main():
    
    groundTruth = sys.argv[1]
    queryPoses = sys.argv[2]

    xListGt = np.array([])
    yListGt = np.array([])
    xListQuery = np.array([])
    yListQuery = np.array([])

    with open(groundTruth, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        while line != '':  # The EOF char is an empty string
            countSpaces = 0
            x = ""
            y = ""
            for char in line: 
                if char == " ":
                    countSpaces += 1
                if countSpaces == 1:
                    x += char
                if countSpaces == 2: 
                    y += char
            
            xListGt = np.append(xListGt, float(x))
            yListGt = np.append(yListGt, float(y))

            line = reader.readline()


    with open(queryPoses, 'r') as reader:
        # Read and print the entire file line by line
        line = reader.readline()
        while line != '':  # The EOF char is an empty string
            countSpaces = 0
            x = ""
            y = ""
            for char in line: 
                if char == " ":
                    countSpaces += 1
                if countSpaces == 1:
                    x += char
                if countSpaces == 2: 
                    y += char
            
            xListQuery = np.append(xListQuery, float(x))
            yListQuery = np.append(yListQuery, float(y))

            line = reader.readline()

    xError = xListGt - xListQuery
    xErrorSqr = [number ** 2 for number in xError]
    yError = yListGt - yListQuery
    yErrorSqr = [number ** 2 for number in yError]

    totalError = np.sqrt(xErrorSqr + yErrorSqr)

    print("The total Error is: " + str(np.sum(totalError)))
    print("The mean is: " + str(np.mean(totalError)))
    print("The median is: " + str(np.median(totalError)))
    print("The standard deviation is: " + str(np.std(totalError)))




if __name__ == "__main__":
    main()

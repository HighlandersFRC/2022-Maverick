import numpy as np
import math
def getCircleOfBestFitCenter(xList, yList):
    inputList = []
    dList = []
    for i in range(0, len(xList)):
        inputList.append([xList[i], yList[i], 1])
        dList.append([(xList[i] ** 2) + (yList[i] ** 2)])
    
    inputMatrix = np.array(inputList)
    dMatrix = np.array(dList)

    outputMatrix = (np.linalg.pinv(inputMatrix)) @ dMatrix

    xCenter = outputMatrix[0]/2
    yCenter = outputMatrix[1]/2

    # rSquared = outputMatrix[2] - (xCenter ** 2) - (yCenter ** 2)
    rSquared = math.sqrt(((4 * outputMatrix[2]) + (outputMatrix[0] ** 2) + (outputMatrix[1] ** 2)))/2
    centers = [[xCenter, yCenter, rSquared]]

    return centers

xList = [205.26037743043992, 203.23717997069133, 201.3400034980003, 202.16121837617698]

yList = [95.39624792384349, 62.42071196788877, 84.03825895054133, 73.17519781157725]

circle = getCircleOfBestFitCenter(xList, yList)

print(circle)

# print("X: " + str(convertedCoordinates[0]) + " Y: " + str(convertedCoordinates[1]) + " Z: " + str(convertedCoordinates[2]))

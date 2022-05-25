import numpy as np


def rewardFunction(testParameters, positionHeading):

    # convert pos/heading list to numpy array
    posHeadingArray = np.array(positionHeading)

    # split into heading and position
    pos = posHeadingArray[:,0:3]
    heading = posHeadingArray[:,3:6]

    pitchMean, pitchStd = np.abs(np.mean(heading[:, 0])), np.std(heading[:,0])
    rollMean, rollStd = np.abs(np.mean(heading[:, 1])), np.std(heading[:,1])

    # handling of when the robot is oriented between pi and -pi, to avoid inflated loss
    if np.min(heading[:,2]) < -np.pi/2 and np.max(heading[:,2]) > np.pi/2:
        heading[:,2] = np.where(heading[:,2] > 0, heading[:,2], np.pi + (np.pi+heading[:,2]))

    yawStd = np.std(heading[:,2])
    heightStd = np.std(pos[:,2])

    # calulate values for direction loss
    startPos = pos[0,0:2]
    endPos = pos[-1,0:2]
    # startheading changed to allow for future calculations
    startHeading = -(heading[0,2]+2*np.pi)

    # calulation of movementVector and safe normalization
    movementVec = endPos-startPos
    nomrMovement = movementVec / (np.linalg.norm(movementVec)+0.00001)

    # create vector from headning
    headingVec = np.array([np.sin(startHeading), np.cos(startHeading)])

    # movement Distance = norm a movement vector
    distanceNew = np.linalg.norm(movementVec)

    # calculate correlation of movement vecotr and heading vector [-1,1]
    vectorCorrelation = np.linalg.norm(nomrMovement - headingVec)-1
    # scale to give less reward for sideways movement
    vectorCorrelation =vectorCorrelation*(vectorCorrelation**2)

    # retrive metrics from parameters
    slipping            = testParameters[1]

    # weight metrics and calculate fitnessvalue

    slipping = slipping * 1
    pitchMean       *=1
    rollMean        *=1
    pitchStd        *=1
    rollStd         *=1
    yawStd          *=1
    yawStd          *=1
    heightStd       *=10

    stability = pitchMean + rollMean + pitchStd + rollStd + yawStd + heightStd
    stability *= 2

    distanceFitness = distanceNew*vectorCorrelation*10

    fitnessValue = distanceFitness - stability

    fitnessDict = {"distFitness" : distanceFitness, "StabilityLoss": stability, "pitchMean": pitchMean, "pitchStd": pitchStd, "rollMean" : rollMean, "rollStd" : rollStd, "heightStd" : heightStd, "yawStd" : yawStd}


    return fitnessValue, fitnessDict
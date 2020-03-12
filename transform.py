"""
Some functions for transforming between different coordinate
"""
import numpy as np

def calcAbsPosUseRelaPosWRTRob0(posRob0, relaPos, xTrue, numRob):
    # Calculate the world-frame position of all robots by using the relative states with respect to robot 0
    xEsti = np.zeros((3, numRob))
    for i in range(numRob):
        # xj = R*x0j+x0
        xEsti[0,i] = relaPos[0,0,i]*np.cos(xTrue[2,0]) - relaPos[1,0,i]*np.sin(xTrue[2,0])
        xEsti[1,i] = relaPos[0,0,i]*np.sin(xTrue[2,0]) + relaPos[1,0,i]*np.cos(xTrue[2,0])
        xEsti[2,i] = relaPos[2,0,i]
        xEsti[:,i] = xEsti[:,i] + posRob0
    return xEsti

def calcRelaState(xTrue, numRob):
    # Calculate the relative states by using the position in world-frame
    xRelaGT = np.zeros((3, numRob)) # ground truth
    x0 = xTrue[0,0]
    y0 = xTrue[1,0]
    yaw0 = xTrue[2,0]
    for i in range(numRob):
        # xj = R*x0j+x0
        x0i = xTrue[0, i] - x0
        y0i = xTrue[1, i] - y0
        yaw0i = xTrue[2, i] - yaw0
        xRelaGT[0,i] = x0i*np.cos(yaw0)  + y0i*np.sin(yaw0)
        xRelaGT[1,i] = -x0i*np.sin(yaw0) + y0i*np.cos(yaw0)
        xRelaGT[2,i] = yaw0i
    return xRelaGT
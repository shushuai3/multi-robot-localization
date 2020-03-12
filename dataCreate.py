"""
Create input data such as velocities, yaw rates, distances...
"""
import numpy as np

class dataCreate:

    def __init__(self, numRob, border, maxVel, dt, devInput, devObser):
        self.velocity = np.zeros((3, numRob))
        self.numRob = numRob
        self.border = border
        self.maxVel = maxVel
        self.dt = dt
        self.devInput = devInput
        self.devObser = devObser

        # variables to be used in PID formation control
        self.intErrX = 0
        self.oldErrX = 0
        self.intErrY = 0
        self.oldErrY = 0

    def calcInput_PotentialField(self, step, xTrue):
    # Calculate control inputs [vx, vy, yaw_rate]' of all robots using Potential Field method
    # such that all robots fly randomly within the border and avoid each other
        if (step % 500 == 0):
            # Create random [vx, vy, yaw_rate]' that sustain for 5 seconds
            # The range is [-1, 1]m and [-1, 1]rad/s
            self.velocity[0:2, :] = np.random.uniform(0, self.maxVel*2, (2, self.numRob)) - self.maxVel
            self.velocity[2, :] = np.random.uniform(-1, 1, (1, self.numRob))
        # Reverse the velocity when reaching the border
        for i in range(self.numRob):
            if self.border["xmax"]-xTrue[0,i]<1.0:
                self.velocity[0, i] = -abs(self.velocity[0, i])
            if xTrue[0,i]-self.border["xmin"]<1.0:
                self.velocity[0, i] = abs(self.velocity[0, i])
            if self.border["ymax"]-xTrue[1,i]<1.0:
                self.velocity[1, i] = -abs(self.velocity[1, i])
            if xTrue[1,i]-self.border["ymin"]<1.0:
                self.velocity[1, i] = abs(self.velocity[1, i])
        # Collision avoidance
        velocity_avoidance = np.zeros((3, self.numRob))
        for i in range(self.numRob):
            [vx, vy] = [0, 0]
            posI = [xTrue[0,i], xTrue[1,i]]
            for j in range(self.numRob):
                if j!= i:
                    posJ = [xTrue[0,j], xTrue[1,j]]
                    distIJ = np.sqrt((posI[0]-posJ[0])**2+(posI[1]-posJ[1])**2)
                    if distIJ < 1:
                        vx = vx + 1/(posI[0]-posJ[0])
                        vy = vy + 1/(posI[1]-posJ[1])
            velocity_avoidance[0:2,i] = 0.15*np.clip(np.array([vx, vy]).T, -10, 10)
        velocity_temp = self.velocity+velocity_avoidance
        velocity_output = self.velocity+velocity_avoidance
        # Rotate the velocity_temp from earth-frame to body frame
        velocity_output[0,:] =  velocity_temp[0,:] * np.cos(xTrue[2,:]) + velocity_temp[1,:] * np.sin(xTrue[2,:])
        velocity_output[1,:] = -velocity_temp[0,:] * np.sin(xTrue[2,:]) + velocity_temp[1,:] * np.cos(xTrue[2,:])
        return velocity_output

    def calcInput_FlyIn1m(self, step):
    # Calculate control inputs [vx, vy, yaw_rate]' of all robots such that
    # all robots fly randomly within 1m range
        if (step % 100) == 0:
            if (step % 200) == 0:
                self.velocity = -self.velocity
            else:
                self.velocity[0:2,:] = np.random.uniform(0, self.maxVel*2, (2, self.numRob)) - self.maxVel
                self.velocity[2,:] = np.random.uniform(0, 1, (1, self.numRob)) - 0.5
        return self.velocity

    def calcInput_Formation01(self, step, relativeState):
        # Robot 0 keeps [-2m, -2m] WRT robot 1 after 40s, while other robots keep flyIn1m flight 
        if (step % 100) == 0:
            if (step % 200) == 0:
                self.velocity = -self.velocity
            else:
                self.velocity[0:2,:] = np.random.uniform(0, self.maxVel*2, (2, self.numRob)) - self.maxVel
                self.velocity[2,:] = np.random.uniform(0, 1, (1, self.numRob)) - 0.5
        if step > 4000:
            self.velocity[2,:] = np.zeros((1, self.numRob))
            self.velocity[0, 0], self.velocity[1, 0] = self.pidControl(relativeState[0, 0, 1], relativeState[1, 0, 1])
            self.velocity[2, 0] = 0
        return self.velocity

    def calcInput_FlyIn1mRob1NoVel(self, step):
        # After 40s, robot 1 is not moving, i.e., zero velocity of robot 1
        if (step % 100) == 0:
            if (step % 200) == 0:
                self.velocity = -self.velocity
            else:
                self.velocity[0:2,:] = np.random.uniform(0, self.maxVel*2, (2, self.numRob)) - self.maxVel
                self.velocity[2,:] = np.random.uniform(0, 1, (1, self.numRob)) - 0.5
        if step > 4000:
            self.velocity[0, 1] = 0
            self.velocity[1, 1] = 0
        return self.velocity

    def pidControl(self, relaX01, relaY01):
        [kp, kd, ki] = [1.4, 0.001, 0.0001]
        ErrX = relaX01 - 2
        self.intErrX = self.intErrX + ErrX
        self.intErrX = np.clip(-10, 10, self.intErrX)
        ctrlX = kp*ErrX + kd*(ErrX-self.oldErrX) + ki*self.intErrX
        self.oldErrX = ErrX
        ErrY = relaY01 - 2
        self.intErrY = self.intErrY + ErrY
        self.intErrY = np.clip(-10, 10, self.intErrY)
        ctrlY = kp*ErrY + kd*(ErrY-self.oldErrY) + ki*self.intErrY
        self.oldErrY = ErrY
        return ctrlX, ctrlY

    def motion_model(self, x, u):
        # Robot model for state prediction
        xPred = np.zeros((3, self.numRob))
        for i in range(self.numRob):
            # X_{k+1} = X_k + Ve * dt; e means earth, b means body
            # Ve = [[c(psi), -s(psi)],[s(psi),c(psi)]] * Vb
            F = np.array([[1.0, 0, 0],
                        [0, 1.0, 0],
                        [0, 0, 1.0]])
            B = np.array([[np.cos(x[2, i]), -np.sin(x[2, i]), 0],
                        [np.sin(x[2, i]),  np.cos(x[2, i]), 0],
                        [0.0, 0.0, 1]])*self.dt
            xPred[:,i] = F@x[:,i] + B@u[:,i]
        return xPred

    def update(self, xTrue, u):
        # Calculate the updated groundTruth(xTrue), noised observation(zNoise), and noised input(uNoise)
        xTrue = self.motion_model(xTrue, u)
        zTrue = np.zeros((self.numRob, self.numRob)) # distances
        for i in range(self.numRob):
            for j in range(self.numRob):
                dx = xTrue[0, i] - xTrue[0, j]
                dy = xTrue[1, i] - xTrue[1, j]
                zTrue[i, j] = np.sqrt(dx**2 + dy**2)
        randNxN = np.random.randn(self.numRob, self.numRob) # standard normal distribution.
        np.fill_diagonal(randNxN, 0) # self distance is zero
        zNois = zTrue + randNxN * self.devObser # add noise
        rand3xN = np.random.randn(3, self.numRob)
        uNois = u + rand3xN * self.devInput # add noise
        return xTrue, zNois, uNois

import csv
import statistics
class realData:
# Get inputs and observations from real-world dataset
    def __init__(self, fileName, numRob):
        self.fileName = fileName
        self.numRob = numRob

    def readDataTolist(self):
        uList = []
        zList = []
        GtList = []
        with open(self.fileName) as csvfile:
            reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
            for row in reader: # each row is a list
                if row[self.numRob*9-8]!=0: # starts from the line at which the last robot begins working
                    uList.append(row[1:4]+row[10:13]+row[19:22])
                    zList.append(row[5:8]+row[14:17]+row[23:26])
                    GtList.append(row[46:67])
            print("Reading the csv file is done!")
            simTime = round(len(zList)/100)-2 # make sure not exceed the range
        # Data preprocessing: average filter to reject the outliers
        tmpArray = np.array(zList)/1000
        for i in range(len(tmpArray[1,:])):
            uwbNoOutlier = self.columnSmoothFilter(tmpArray[:,i])
            uwbBias = 0.048*uwbNoOutlier + 0.6508
            tmpArray[:,i] = uwbNoOutlier - uwbBias
        zList = tmpArray.tolist()
        return uList, zList, GtList, simTime

    def calcInputDataset(self, uRow, zRow, GtRow):
        uNois = np.array(uRow).reshape(3,3).T
        zNois = np.array(zRow).reshape(3,3).T
        posXY = np.array(GtRow[0:1]+GtRow[2:3]+GtRow[7:8]+GtRow[9:10]+GtRow[14:15]+GtRow[16:17]).reshape(3,2).T
        posXY[1,:] = -posXY[1,:] # optiTrack gives negative y position
        attQuat = np.array(GtRow[3:7]+GtRow[10:14]+GtRow[17:21]).reshape(3,4)
        yaw = []
        for i in range(self.numRob):
            q = attQuat[i,:] / np.linalg.norm(attQuat[i,:])
            yaw.append(np.arctan2( -2*(q[1]*q[3]-q[0]*q[2]), q[0]**2-q[1]**2-q[2]**2+q[3]**2) )
        xTrue = np.vstack((posXY,-np.array(yaw))) # clockwise is positive in optiTrack
        return xTrue, zNois, uNois

    def columnSmoothFilter(self, colum):
        columnNoOutlier = []
        for i in range(len(colum)):
            if i < 4:
                columnNoOutlier.append(colum[i])
            else:
                averValue = statistics.mean(colum[i-3:i])
                if np.abs(averValue - colum[i]) > 0.4:
                    # colum[i] = colum[i-1] # comment for the noise dataset
                    columnNoOutlier.append(colum[i])
                else:
                    columnNoOutlier.append(colum[i])
        return np.array(columnNoOutlier)
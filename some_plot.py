"""
File for figure plots, such as estimation error of 50 tests, and tests with different inputs.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from dataCreate import dataCreate
from relativeEKF import EKFonSimData
import transform

# Simulation settings
show_animation = True # True: animation; Flase: figure
np.random.seed(19910620) # seed the random number generator for reproducibility
border = {"xmin":-4, "xmax":4, "ymin":-4, "ymax":4, "zmin":0, "zmax":4}
numRob = 4 # number of robots
dt = 0.01 # time interval [s]
simTime = 70.0 # simulation time [s]
maxVel = 1 # maximum velocity [m/s]
devInput = np.array([[0.25, 0.25, 0.01]]).T # input deviation in simulation, Vx[m/s], Vy[m/s], yawRate[rad/s]
devObser = 0.1 # observation deviation of distance[m]
ekfStride = 1 # update interval of EKF is simStride*0.01[s]    

# figure X-Y-Yaw error of 50 random tests
testNum = 50
relaX01err = [[] for i in range(testNum)]
relaY01err = [[] for i in range(testNum)]
relaYaw01err = [[] for i in range(testNum)]
for i in range(testNum):
    print("Test number: {}".format(i))
    xTrue = np.random.uniform(-3, 3, (3, numRob))
    relativeState = np.zeros((3, numRob, numRob))
    data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
    relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
    step = 0
    while simTime >= dt*step:
        step += 1
        u = data.calcInput_FlyIn1m(step)
        xTrue, zNois, uNois = data.update(xTrue, u)   
        if step % ekfStride == 0:
            relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
        xEsti = relativeState[:,0,:]
        xTrueRL = transform.calcRelaState(xTrue, numRob)
        relaX01err[i].append(xEsti[0,1]-xTrueRL[0,1])
        relaY01err[i].append(xEsti[1,1]-xTrueRL[1,1])
        relaYaw01err[i].append(xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14))
timePlot = np.arange(0, len(relaX01err[0]))/100
x01array = np.array(relaX01err)
y01array = np.array(relaY01err)
yaw01array = np.array(relaYaw01err)
f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
plt.margins(x=0)
for i in range(testNum):
    ax1.plot(timePlot, x01array[i, :])
    ax2.plot(timePlot, y01array[i, :])
    ax3.plot(timePlot, yaw01array[i, :])
ax1.set_ylabel(r"Err $x_{ij}$ (m)", fontsize=12)
ax2.set_ylabel(r"Err $y_{ij}$ (m)", fontsize=12)
ax3.set_ylabel(r"Err $\mathrm{\psi}_{ij}$ (rad)", fontsize=12)
ax3.set_xlabel("Time (s)", fontsize=12)
f.subplots_adjust(hspace=0)
plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
plt.show()

# Figure with unobservable control inputs
iterNum = 50
relaX01errRandInput = [[] for i in range(iterNum)]
relaY01errRandInput = [[] for i in range(iterNum)]
relaYaw01errRandInput = [[] for i in range(iterNum)]
for i in range(iterNum):
    print("Test number: {}".format(i))
    xTrue = np.random.uniform(-3, 3, (3, numRob))
    relativeState = np.zeros((3, numRob, numRob))
    data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
    relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
    step = 0
    while simTime >= dt*step:
        step += 1
        u = data.calcInput_FlyIn1m(step)
        xTrue, zNois, uNois = data.update(xTrue, u)   
        if step % ekfStride == 0:
            relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
        xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
        if step>5000:
            # relaX01errRandInput[i].append(np.abs(xEsti[0,1]-xTrue[0,1]))
            # relaY01errRandInput[i].append(np.abs(xEsti[1,1]-xTrue[1,1]))
            # relaYaw01errRandInput[i].append(np.abs(xEsti[2,1]-xTrue[2,1] - 3.14*round((xEsti[2,1]-xTrue[2,1])/3.14)))
            xEsti = relativeState[:,0,:]
            xTrueRL = transform.calcRelaState(xTrue, numRob)
            relaX01errRandInput[i].append(np.abs(xEsti[0,1]-xTrueRL[0,1]))
            relaY01errRandInput[i].append(np.abs(xEsti[1,1]-xTrueRL[1,1]))
            relaYaw01errRandInput[i].append(np.abs( xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14) ))

relaX01errFormationInput = [[] for i in range(iterNum)]
relaY01errFormationInput = [[] for i in range(iterNum)]
relaYaw01errFormationInput = [[] for i in range(iterNum)]
for i in range(iterNum):
    print("Test number: {}".format(i))
    xTrue = np.random.uniform(-3, 3, (3, numRob))
    relativeState = np.zeros((3, numRob, numRob))
    data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
    relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
    step = 0
    while simTime >= dt*step:
        step += 1
        u = data.calcInput_Formation01(step, relativeState)
        xTrue, zNois, uNois = data.update(xTrue, u)   
        if step % ekfStride == 0:
            relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
        xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
        if step>5000:
            # relaX01errFormationInput[i].append(np.abs(xEsti[0,1]-xTrue[0,1]))
            # relaY01errFormationInput[i].append(np.abs(xEsti[1,1]-xTrue[1,1]))
            # relaYaw01errFormationInput[i].append(np.abs(xEsti[2,1]-xTrue[2,1] - 3.14*round((xEsti[2,1]-xTrue[2,1])/3.14)))
            xEsti = relativeState[:,0,:]
            xTrueRL = transform.calcRelaState(xTrue, numRob)
            relaX01errFormationInput[i].append(np.abs(xEsti[0,1]-xTrueRL[0,1]))
            relaY01errFormationInput[i].append(np.abs(xEsti[1,1]-xTrueRL[1,1]))
            relaYaw01errFormationInput[i].append(np.abs(xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14)))                

relaX01errV1zeroInput = [[] for i in range(iterNum)]
relaY01errV1zeroInput = [[] for i in range(iterNum)]
relaYaw01errV1zeroInput = [[] for i in range(iterNum)]
for i in range(iterNum):
    print("Test number: {}".format(i))
    xTrue = np.random.uniform(-3, 3, (3, numRob))
    relativeState = np.zeros((3, numRob, numRob))
    data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
    relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
    step = 0
    while simTime >= dt*step:
        step += 1
        u = data.calcInput_FlyIn1mRob1NoVel(step) 
        xTrue, zNois, uNois = data.update(xTrue, u)   
        if step % ekfStride == 0:
            relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
        xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
        if step>5000:
            # relaX01errV1zeroInput[i].append(np.abs(xEsti[0,1]-xTrue[0,1]))
            # relaY01errV1zeroInput[i].append(np.abs(xEsti[1,1]-xTrue[1,1]))
            # relaYaw01errV1zeroInput[i].append(np.abs(xEsti[2,1]-xTrue[2,1] - 3.14*round((xEsti[2,1]-xTrue[2,1])/3.14)))
            xEsti = relativeState[:,0,:]
            xTrueRL = transform.calcRelaState(xTrue, numRob)
            relaX01errV1zeroInput[i].append(np.abs(xEsti[0,1]-xTrueRL[0,1]))
            relaY01errV1zeroInput[i].append(np.abs(xEsti[1,1]-xTrueRL[1,1]))
            relaYaw01errV1zeroInput[i].append(np.abs(xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14)))

xRand = np.array(relaX01errRandInput).mean(axis=1) # [20][3000]->[20]
yRand = np.array(relaY01errRandInput).mean(axis=1)
yawRand = np.array(relaYaw01errRandInput).mean(axis=1)
xForm = np.array(relaX01errFormationInput).mean(axis=1) # [20][3000]->[20]
yForm = np.array(relaY01errFormationInput).mean(axis=1)
yawForm = np.array(relaYaw01errFormationInput).mean(axis=1)
xV1Zero = np.array(relaX01errV1zeroInput).mean(axis=1) # [20][3000]->[20]
yV1Zero = np.array(relaY01errV1zeroInput).mean(axis=1)
yawV1Zero = np.array(relaYaw01errV1zeroInput).mean(axis=1)

xBox = [xRand, xForm, xV1Zero]
yBox = [yRand, yForm, yV1Zero]
yawBox = [yawRand, yawForm, yawV1Zero]

fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(5, 2))
fig.tight_layout(pad=1.0) # spacing between subplots
bplot1 = axes[0].boxplot(xBox, vert=True, patch_artist=True, widths=0.4)
bplot2 = axes[1].boxplot(yBox, vert=True, patch_artist=True, widths=0.4)
bplot3 = axes[2].boxplot(yawBox, vert=True, patch_artist=True, widths=0.4)
colors = ['pink', 'lightblue', 'lightgreen']
for bplot in (bplot1, bplot2, bplot3):
    for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)
for ax in axes:
    ax.yaxis.grid(True)
    ax.yaxis.labelpad = 2 # position of y-label
    ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False) # remove xTick
axes[0].set_ylabel(r'MAE of $x_{ij}$ estimation (m)', fontsize=12)
axes[0].set_ylim(0,0.25)
axes[1].set_ylabel(r'MAE of $y_{ij}$ estimation (m)', fontsize=12)
axes[1].set_ylim(0,0.5)
axes[2].set_ylabel(r'MAE of $\psi_{ij}$ estimation (rad)', fontsize=12)
axes[2].set_ylim(0,0.4)
plt.legend([bplot1["boxes"][0], bplot1["boxes"][1], bplot1["boxes"][2]], ['Random', 'Formation', '$v_j=0$'], fontsize=11) # bbox_to_anchor=(0.8, 1)
plt.show()

# Figure of estimation on real-world data
from dataCreate import realData
from relativeEKF import EKFonRealData

numRob = 3 # number of robots
relativeState = np.zeros((3, numRob, numRob))
dataFromReal = realData("./dataset/dat01.csv", numRob)
uList, zList, GtList, simTime = dataFromReal.readDataTolist()
relaEKFonRealData = EKFonRealData(10, 1, 0.25, 0.1, 0.1, numRob)
pos01ekfVSgt = [[] for i in range(6)] # x, xGT, y, yGT, z, zGT
step = 0
while simTime >= dt*step:
    step += 1
    xTrue, zNois, uNois = dataFromReal.calcInputDataset(uList[step], zList[step], GtList[step])
    relativeState = relaEKFonRealData.EKF(uNois, zNois, relativeState)
    xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
    pos01ekfVSgt[0].append(xEsti[0,1])
    pos01ekfVSgt[1].append(xTrue[0,1])
    pos01ekfVSgt[2].append(xEsti[1,1])
    pos01ekfVSgt[3].append(xTrue[1,1])
    pos01ekfVSgt[4].append(xEsti[2,1])
    pos01ekfVSgt[5].append(xTrue[2,1])
pos01array = np.array(pos01ekfVSgt)
pos01array = pos01array[:,1800:9200] # clip
timePlot = np.arange(0, len(pos01array[0]))/100

relativeState = np.zeros((3, numRob, numRob))
dataFromReal = realData("./dataset/dat02.csv", numRob)
uList, zList, GtList, simTime = dataFromReal.readDataTolist()
relaEKFonRealData = EKFonRealData(10, 1, 0.25, 0.1, 0.1, numRob)
pos01ekfVSgt = [[] for i in range(6)] # x, xGT, y, yGT, z, zGT
step = 0
while simTime >= dt*step:
    step += 1
    xTrue, zNois, uNois = dataFromReal.calcInputDataset(uList[step], zList[step], GtList[step])
    relativeState = relaEKFonRealData.EKF(uNois, zNois, relativeState)
    xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
    pos01ekfVSgt[0].append(xEsti[0,1])
    pos01ekfVSgt[1].append(xTrue[0,1])
    pos01ekfVSgt[2].append(xEsti[1,1])
    pos01ekfVSgt[3].append(xTrue[1,1])
    pos01ekfVSgt[4].append(xEsti[2,1])
    pos01ekfVSgt[5].append(xTrue[2,1])
pos01arrayRob4 = np.array(pos01ekfVSgt)
pos01arrayRob4 = pos01arrayRob4[:,2300:] # clip
timePlotRob4 = np.arange(0, len(pos01arrayRob4[0]))/100

f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
plt.margins(x=0)
ax1.plot(timePlot, pos01array[0,:])
ax1.plot(timePlot, pos01array[1,:])
ax1.plot(timePlotRob4, pos01arrayRob4[0,:])
ax1.plot(timePlotRob4, pos01arrayRob4[1,:])
ax1.set_ylabel("x (m)")
ax1.grid(True)
ax2.plot(timePlot, pos01array[2,:])
ax2.plot(timePlot, pos01array[3,:])
ax2.plot(timePlotRob4, pos01arrayRob4[2,:])
ax2.plot(timePlotRob4, pos01arrayRob4[3,:])
ax2.set_ylabel("y (m)")
ax2.grid(True)
ax3.plot(timePlot, pos01array[4,:], label='Relative EKF in 3 robots')
ax3.plot(timePlot, pos01array[5,:], label='Ground-truth in 3 robots')
ax3.plot(timePlotRob4, pos01arrayRob4[4,:], label='Relative EKF in 4 robots')
ax3.plot(timePlotRob4, pos01arrayRob4[5,:], label='Ground-truth in 4 robots')
ax3.set_ylabel(r"$\mathrm{\psi (rad)}$")
ax3.set_xlabel("Time (s)")
ax3.grid(True)
ax3.legend(loc='upper center', bbox_to_anchor=(0.8, 0.8))
# Fine-tune figure; make subplots close to each other and hide x ticks for
# all but bottom plot.
f.subplots_adjust(hspace=0)
plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
plt.margins(x=0)
plt.show()

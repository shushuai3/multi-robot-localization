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
# testNum = 50
# relaX01err = [[] for i in range(testNum)]
# relaY01err = [[] for i in range(testNum)]
# relaYaw01err = [[] for i in range(testNum)]
# for i in range(testNum):
#     print("Test number: {}".format(i))
#     xTrue = np.random.uniform(-3, 3, (3, numRob))
#     relativeState = np.zeros((3, numRob, numRob))
#     data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
#     relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
#     step = 0
#     while simTime >= dt*step:
#         step += 1
#         u = data.calcInput_FlyIn1m(step)
#         xTrue, zNois, uNois = data.update(xTrue, u)   
#         if step % ekfStride == 0:
#             relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
#         xEsti = relativeState[:,0,:]
#         xTrueRL = transform.calcRelaState(xTrue, numRob)
#         relaX01err[i].append(xEsti[0,1]-xTrueRL[0,1])
#         relaY01err[i].append(xEsti[1,1]-xTrueRL[1,1])
#         relaYaw01err[i].append(xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14))
# timePlot = np.arange(0, len(relaX01err[0]))/100
# x01array = np.array(relaX01err)
# y01array = np.array(relaY01err)
# yaw01array = np.array(relaYaw01err)
# f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
# plt.margins(x=0)
# for i in range(testNum):
#     ax1.plot(timePlot, x01array[i, :])
#     ax2.plot(timePlot, y01array[i, :])
#     ax3.plot(timePlot, yaw01array[i, :])
# ax1.set_ylabel(r"Err $x_{ij}$ (m)", fontsize=12)
# ax2.set_ylabel(r"Err $y_{ij}$ (m)", fontsize=12)
# ax3.set_ylabel(r"Err $\mathrm{\psi}_{ij}$ (rad)", fontsize=12)
# ax3.set_xlabel("Time (s)", fontsize=12)
# f.subplots_adjust(hspace=0)
# plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
# plt.show()

# Figure with unobservable control inputs
# iterNum = 50
# relaX01errRandInput = [[] for i in range(iterNum)]
# relaY01errRandInput = [[] for i in range(iterNum)]
# relaYaw01errRandInput = [[] for i in range(iterNum)]
# for i in range(iterNum):
#     print("Test number: {}".format(i))
#     xTrue = np.random.uniform(-3, 3, (3, numRob))
#     relativeState = np.zeros((3, numRob, numRob))
#     data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
#     relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
#     step = 0
#     while simTime >= dt*step:
#         step += 1
#         u = data.calcInput_FlyIn1m(step)
#         xTrue, zNois, uNois = data.update(xTrue, u)   
#         if step % ekfStride == 0:
#             relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
#         xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
#         if step>5000:
#             # relaX01errRandInput[i].append(np.abs(xEsti[0,1]-xTrue[0,1]))
#             # relaY01errRandInput[i].append(np.abs(xEsti[1,1]-xTrue[1,1]))
#             # relaYaw01errRandInput[i].append(np.abs(xEsti[2,1]-xTrue[2,1] - 3.14*round((xEsti[2,1]-xTrue[2,1])/3.14)))
#             xEsti = relativeState[:,0,:]
#             xTrueRL = transform.calcRelaState(xTrue, numRob)
#             relaX01errRandInput[i].append(np.abs(xEsti[0,1]-xTrueRL[0,1]))
#             relaY01errRandInput[i].append(np.abs(xEsti[1,1]-xTrueRL[1,1]))
#             relaYaw01errRandInput[i].append(np.abs( xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14) ))

# relaX01errFormationInput = [[] for i in range(iterNum)]
# relaY01errFormationInput = [[] for i in range(iterNum)]
# relaYaw01errFormationInput = [[] for i in range(iterNum)]
# for i in range(iterNum):
#     print("Test number: {}".format(i))
#     xTrue = np.random.uniform(-3, 3, (3, numRob))
#     relativeState = np.zeros((3, numRob, numRob))
#     data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
#     relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
#     step = 0
#     while simTime >= dt*step:
#         step += 1
#         u = data.calcInput_Formation01(step, relativeState)
#         xTrue, zNois, uNois = data.update(xTrue, u)   
#         if step % ekfStride == 0:
#             relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
#         xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
#         if step>5000:
#             # relaX01errFormationInput[i].append(np.abs(xEsti[0,1]-xTrue[0,1]))
#             # relaY01errFormationInput[i].append(np.abs(xEsti[1,1]-xTrue[1,1]))
#             # relaYaw01errFormationInput[i].append(np.abs(xEsti[2,1]-xTrue[2,1] - 3.14*round((xEsti[2,1]-xTrue[2,1])/3.14)))
#             xEsti = relativeState[:,0,:]
#             xTrueRL = transform.calcRelaState(xTrue, numRob)
#             relaX01errFormationInput[i].append(np.abs(xEsti[0,1]-xTrueRL[0,1]))
#             relaY01errFormationInput[i].append(np.abs(xEsti[1,1]-xTrueRL[1,1]))
#             relaYaw01errFormationInput[i].append(np.abs(xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14)))                

# relaX01errV1zeroInput = [[] for i in range(iterNum)]
# relaY01errV1zeroInput = [[] for i in range(iterNum)]
# relaYaw01errV1zeroInput = [[] for i in range(iterNum)]
# for i in range(iterNum):
#     print("Test number: {}".format(i))
#     xTrue = np.random.uniform(-3, 3, (3, numRob))
#     relativeState = np.zeros((3, numRob, numRob))
#     data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
#     relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
#     step = 0
#     while simTime >= dt*step:
#         step += 1
#         u = data.calcInput_FlyIn1mRob1NoVel(step) 
#         xTrue, zNois, uNois = data.update(xTrue, u)   
#         if step % ekfStride == 0:
#             relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
#         xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
#         if step>5000:
#             # relaX01errV1zeroInput[i].append(np.abs(xEsti[0,1]-xTrue[0,1]))
#             # relaY01errV1zeroInput[i].append(np.abs(xEsti[1,1]-xTrue[1,1]))
#             # relaYaw01errV1zeroInput[i].append(np.abs(xEsti[2,1]-xTrue[2,1] - 3.14*round((xEsti[2,1]-xTrue[2,1])/3.14)))
#             xEsti = relativeState[:,0,:]
#             xTrueRL = transform.calcRelaState(xTrue, numRob)
#             relaX01errV1zeroInput[i].append(np.abs(xEsti[0,1]-xTrueRL[0,1]))
#             relaY01errV1zeroInput[i].append(np.abs(xEsti[1,1]-xTrueRL[1,1]))
#             relaYaw01errV1zeroInput[i].append(np.abs(xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14)))

# xRand = np.array(relaX01errRandInput).mean(axis=1) # [20][3000]->[20]
# yRand = np.array(relaY01errRandInput).mean(axis=1)
# yawRand = np.array(relaYaw01errRandInput).mean(axis=1)
# xForm = np.array(relaX01errFormationInput).mean(axis=1) # [20][3000]->[20]
# yForm = np.array(relaY01errFormationInput).mean(axis=1)
# yawForm = np.array(relaYaw01errFormationInput).mean(axis=1)
# xV1Zero = np.array(relaX01errV1zeroInput).mean(axis=1) # [20][3000]->[20]
# yV1Zero = np.array(relaY01errV1zeroInput).mean(axis=1)
# yawV1Zero = np.array(relaYaw01errV1zeroInput).mean(axis=1)

# xBox = [xRand, xForm, xV1Zero]
# yBox = [yRand, yForm, yV1Zero]
# yawBox = [yawRand, yawForm, yawV1Zero]

# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(5, 2))
# fig.tight_layout(pad=1.0) # spacing between subplots
# bplot1 = axes[0].boxplot(xBox, vert=True, patch_artist=True, widths=0.4)
# bplot2 = axes[1].boxplot(yBox, vert=True, patch_artist=True, widths=0.4)
# bplot3 = axes[2].boxplot(yawBox, vert=True, patch_artist=True, widths=0.4)
# colors = ['pink', 'lightblue', 'lightgreen']
# for bplot in (bplot1, bplot2, bplot3):
#     for patch, color in zip(bplot['boxes'], colors):
#         patch.set_facecolor(color)
# for ax in axes:
#     ax.yaxis.grid(True)
#     ax.yaxis.labelpad = 2 # position of y-label
#     ax.tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False) # remove xTick
# axes[0].set_ylabel(r'MAE of $x_{ij}$ estimation (m)', fontsize=12)
# axes[0].set_ylim(0,0.25)
# axes[1].set_ylabel(r'MAE of $y_{ij}$ estimation (m)', fontsize=12)
# axes[1].set_ylim(0,0.5)
# axes[2].set_ylabel(r'MAE of $\psi_{ij}$ estimation (rad)', fontsize=12)
# axes[2].set_ylim(0,0.4)
# plt.legend([bplot1["boxes"][0], bplot1["boxes"][1], bplot1["boxes"][2]], ['Random', 'Formation', '$v_j=0$'], fontsize=11) # bbox_to_anchor=(0.8, 1)
# plt.show()

# Figure of estimation on real-world data
# from dataCreate import realData
# from relativeEKF import EKFonRealData

# numRob = 3 # number of robots
# relativeState = np.zeros((3, numRob, numRob))
# dataFromReal = realData("./dataset/dat01.csv", numRob)
# uList, zList, GtList, simTime = dataFromReal.readDataTolist()
# relaEKFonRealData = EKFonRealData(10, 1, 0.25, 0.1, 0.1, numRob)
# pos01ekfVSgt = [[] for i in range(6)] # x, xGT, y, yGT, z, zGT
# step = 0
# while simTime >= dt*step:
#     step += 1
#     xTrue, zNois, uNois = dataFromReal.calcInputDataset(uList[step], zList[step], GtList[step])
#     relativeState = relaEKFonRealData.EKF(uNois, zNois, relativeState)
#     xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
#     pos01ekfVSgt[0].append(xEsti[0,1])
#     pos01ekfVSgt[1].append(xTrue[0,1])
#     pos01ekfVSgt[2].append(xEsti[1,1])
#     pos01ekfVSgt[3].append(xTrue[1,1])
#     pos01ekfVSgt[4].append(xEsti[2,1])
#     pos01ekfVSgt[5].append(xTrue[2,1])
# pos01array = np.array(pos01ekfVSgt)
# pos01array = pos01array[:,1800:9200] # clip
# timePlot = np.arange(0, len(pos01array[0]))/100

# relativeState = np.zeros((3, numRob, numRob))
# dataFromReal = realData("./dataset/dat02.csv", numRob)
# uList, zList, GtList, simTime = dataFromReal.readDataTolist()
# relaEKFonRealData = EKFonRealData(10, 1, 0.25, 0.1, 0.1, numRob)
# pos01ekfVSgt = [[] for i in range(6)] # x, xGT, y, yGT, z, zGT
# step = 0
# while simTime >= dt*step:
#     step += 1
#     xTrue, zNois, uNois = dataFromReal.calcInputDataset(uList[step], zList[step], GtList[step])
#     relativeState = relaEKFonRealData.EKF(uNois, zNois, relativeState)
#     xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
#     pos01ekfVSgt[0].append(xEsti[0,1])
#     pos01ekfVSgt[1].append(xTrue[0,1])
#     pos01ekfVSgt[2].append(xEsti[1,1])
#     pos01ekfVSgt[3].append(xTrue[1,1])
#     pos01ekfVSgt[4].append(xEsti[2,1])
#     pos01ekfVSgt[5].append(xTrue[2,1])
# pos01arrayRob4 = np.array(pos01ekfVSgt)
# pos01arrayRob4 = pos01arrayRob4[:,2300:] # clip
# timePlotRob4 = np.arange(0, len(pos01arrayRob4[0]))/100

# f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
# plt.margins(x=0)
# ax1.plot(timePlot, pos01array[0,:])
# ax1.plot(timePlot, pos01array[1,:])
# ax1.plot(timePlotRob4, pos01arrayRob4[0,:])
# ax1.plot(timePlotRob4, pos01arrayRob4[1,:])
# ax1.set_ylabel("x (m)")
# ax1.grid(True)
# ax2.plot(timePlot, pos01array[2,:])
# ax2.plot(timePlot, pos01array[3,:])
# ax2.plot(timePlotRob4, pos01arrayRob4[2,:])
# ax2.plot(timePlotRob4, pos01arrayRob4[3,:])
# ax2.set_ylabel("y (m)")
# ax2.grid(True)
# ax3.plot(timePlot, pos01array[4,:], label='Relative EKF in 3 robots')
# ax3.plot(timePlot, pos01array[5,:], label='Ground-truth in 3 robots')
# ax3.plot(timePlotRob4, pos01arrayRob4[4,:], label='Relative EKF in 4 robots')
# ax3.plot(timePlotRob4, pos01arrayRob4[5,:], label='Ground-truth in 4 robots')
# ax3.set_ylabel(r"$\mathrm{\psi (rad)}$")
# ax3.set_xlabel("Time (s)")
# ax3.grid(True)
# ax3.legend(loc='upper center', bbox_to_anchor=(0.8, 0.8))
# # Fine-tune figure; make subplots close to each other and hide x ticks for
# # all but bottom plot.
# f.subplots_adjust(hspace=0)
# plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
# plt.margins(x=0)
# plt.show()

# Figure of estimation drift with formation flight and different noises
def ekf_one_dataset(q_vel, r_dist, iterNum):
    # default value 0.25 and 0.1
    devInput = np.array([[q_vel, q_vel, 0.01]]).T # input deviation in simulation, Vx[m/s], Vy[m/s], yawRate[rad/s]
    devObser = r_dist # observation deviation of distance[m]
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
            if step>4000:
                u = np.zeros((3, numRob)) #### hovering test
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

    xForm = np.array(relaX01errFormationInput).mean(axis=1)
    mean_xForm = xForm.mean()
    yForm = np.array(relaY01errFormationInput).mean(axis=1)
    mean_yForm = yForm.mean()
    print(mean_xForm, mean_yForm)
    return [mean_xForm, mean_yForm]

# xErr_grid = np.zeros((10, 10))
# yErr_grid = np.zeros((10, 10))
# for i in range(10):
#     for j in range(10):
#         print('grid_x: {}, grid_y: {}'.format(i, j))
#         mean = ekf_one_dataset(0.05*(i+1), 0.05*(j+1), 30)
#         xErr_grid[i, j] = mean[0]
#         yErr_grid[i, j] = mean[1]
# mat_xErr_grid = np.matrix(xErr_grid)
# mat_yErr_grid = np.matrix(yErr_grid)
# with open('mat_xErr_grid.txt', 'w') as f:
#     for line in mat_xErr_grid:
#         np.savetxt(f, line, fmt='%.5f')
# with open('mat_yErr_grid.txt', 'w') as f:
#     for line in mat_yErr_grid:
#         np.savetxt(f, line, fmt='%.5f') #remember to delete the last blank line

# with open('mat_xErr_grid_formation.txt', 'r') as f:
#     lines = f.readlines()
#     lines_string_list = [line.strip() for line in lines if len(line.strip().split()[1:]) != 0]
# grid_list = [list(map(float, line_string.split())) for line_string in lines_string_list]
# xErr_formation = np.array(grid_list)
# with open('mat_yErr_grid_formation.txt', 'r') as f:
#     lines = f.readlines()
#     lines_string_list = [line.strip() for line in lines if len(line.strip().split()[1:]) != 0]
# grid_list = [list(map(float, line_string.split())) for line_string in lines_string_list]
# yErr_formation = np.array(grid_list)
# with open('mat_xErr_grid_hover.txt', 'r') as f:
#     lines = f.readlines()
#     lines_string_list = [line.strip() for line in lines if len(line.strip().split()[1:]) != 0]
# grid_list = [list(map(float, line_string.split())) for line_string in lines_string_list]
# xErr_hover = np.array(grid_list)
# with open('mat_yErr_grid_hover.txt', 'r') as f:
#     lines = f.readlines()
#     lines_string_list = [line.strip() for line in lines if len(line.strip().split()[1:]) != 0]
# grid_list = [list(map(float, line_string.split())) for line_string in lines_string_list]
# yErr_hover = np.array(grid_list)

# start, stop, n_values = 0.05, 0.5, 10
# x_vals = np.linspace(start, stop, n_values)
# y_vals = np.linspace(start, stop, n_values)
# X, Y = np.meshgrid(x_vals, y_vals)
# fig, (ax1, ax2, ax3, ax4) = plt.subplots(figsize=(8, 3), ncols=4)

# MAE_x_formation = ax1.contourf(X, Y, xErr_formation, vmin=0, vmax=1.4, cmap=plt.cm.get_cmap("coolwarm"))
# ax1.set_title('Formation: MAE X (m)')
# MAE_x_hover = ax2.contourf(X, Y, xErr_hover, vmin=0, vmax=1.4, cmap=plt.cm.get_cmap("coolwarm"))
# ax2.set_title('Hover: MAE X (m)')
# MAE_y_formation = ax3.contourf(X, Y, yErr_formation, vmin=0, vmax=1.4, cmap=plt.cm.get_cmap("coolwarm"))
# ax3.set_title('Formation: MAE Y (m)')
# MAE_y_hover = ax4.contourf(X, Y, yErr_hover, vmin=0, vmax=1.4, cmap=plt.cm.get_cmap("coolwarm"))
# fig.colorbar(MAE_y_hover) ### fig.colorbar(MAE_y_formation, ax=ax3)
# ax4.set_title('Hover: MAE Y (m)')
# ax1.set_ylabel('Ranging deviation (m)')
# fig.text(0.5, 0.04, 'Velocity deviation (m/s)', ha='center')
# plt.show()

# Figure of 2D drift trace
def ekf_2D_drift(q_vel, r_dist, iterNum, ctrl=0):
    # default value 0.25 and 0.1
    devInput = np.array([[q_vel, q_vel, 0.01]]).T # input deviation in simulation, Vx[m/s], Vy[m/s], yawRate[rad/s]
    devObser = r_dist # observation deviation of distance[m]
    relaX01err = []
    relaY01err = []
    relaYaw01err = []
    for i in range(iterNum):
        print("Test number: {}".format(i))
        xTrue = np.random.uniform(-3, 3, (3, numRob))
        relativeState = np.zeros((3, numRob, numRob))
        data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
        relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
        step = 0
        while simTime >= dt*step:
            step += 1
            u = data.calcInput_Formation01(step, relativeState) # !!!!!! change this function inside, the reference needs to be constant
            if step>6000:
                if ctrl == 0:
                    u = np.zeros((3, numRob)) #### hovering test
                elif ctrl == 1:
                    u = data.calcInput_Formation01(step, relativeState)
            xTrue, zNois, uNois = data.update(xTrue, u)   
            if step % ekfStride == 0:
                relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
            xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
            if step>6000:
                # relaX01errFormationInput[i].append(np.abs(xEsti[0,1]-xTrue[0,1]))
                # relaY01errFormationInput[i].append(np.abs(xEsti[1,1]-xTrue[1,1]))
                # relaYaw01errFormationInput[i].append(np.abs(xEsti[2,1]-xTrue[2,1] - 3.14*round((xEsti[2,1]-xTrue[2,1])/3.14)))
                xEsti = relativeState[:,0,:]
                xTrueRL = transform.calcRelaState(xTrue, numRob)
                relaX01err.append(xEsti[0,1])
                relaY01err.append(xEsti[1,1])
                relaYaw01err.append(xEsti[2,1]-xTrueRL[2,1] - 3.14*round((xEsti[2,1]-xTrueRL[2,1])/3.14))                

    xDrift = np.array(relaX01err)
    yDrift = np.array(relaY01err)
    yawDrift = np.array(relaYaw01err)
    return xDrift, yDrift, yawDrift

# hover_ctrl_by_real_position = 0
# hover_ctrl_by_esti_position = 1
# iterNum = 5
# simTime = 100
# fig, axs = plt.subplots(2, 4, figsize=(15, 5))
# xy_lim = [[0, 3.5], [0, 3.5]] # [[xmin, xmax], [ymin, ymax]]
# x_data, y_data, yaw_data = ekf_2D_drift(0.25, 0.1, iterNum, hover_ctrl_by_real_position)
# axs[0][0].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# x_data, y_data, yaw_data = ekf_2D_drift(0.25, 0.4, iterNum, hover_ctrl_by_real_position)
# axs[0][1].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# x_data, y_data, yaw_data = ekf_2D_drift(0.5, 0.1, iterNum, hover_ctrl_by_real_position)
# axs[0][2].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# x_data, y_data, yaw_data = ekf_2D_drift(0.5, 0.4, iterNum, hover_ctrl_by_real_position)
# axs[0][3].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# x_data, y_data, yaw_data = ekf_2D_drift(0.25, 0.1, iterNum, hover_ctrl_by_esti_position)
# axs[1][0].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# x_data, y_data, yaw_data = ekf_2D_drift(0.25, 0.4, iterNum, hover_ctrl_by_esti_position)
# axs[1][1].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# x_data, y_data, yaw_data = ekf_2D_drift(0.5, 0.1, iterNum, hover_ctrl_by_esti_position)
# axs[1][2].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# x_data, y_data, yaw_data = ekf_2D_drift(0.5, 0.4, iterNum, hover_ctrl_by_esti_position)
# im = axs[1][3].hist2d(x_data, y_data, range=xy_lim, bins=(50, 50))
# cax = plt.axes([0.91, 0.15, 0.005, 0.73]) # [left, bottom, width, height]
# fig.colorbar(im[-1], ax=axs.ravel(), cax=cax)
# axs[0][0].set_ylabel('Hover by ground-truth')
# axs[1][0].set_ylabel('Hover by estimation')
# axs[1][0].set_xlabel('v_noise = 0.25m/s, d_noise = 0.1m')
# axs[1][1].set_xlabel('v_noise = 0.25m/s, d_noise = 0.4m')
# axs[1][2].set_xlabel('v_noise = 0.5m/s, d_noise = 0.1m')
# axs[1][3].set_xlabel('v_noise = 0.5m/s, d_noise = 0.4m')
# # add a big axis, hide frame, hide tick and tick label of the big axis
# fig.add_subplot(111, frameon=False)
# plt.tick_params(labelcolor='none', top=False, bottom=False, left=False, right=False)
# plt.ylabel("relative estimation Y (m)", labelpad=10)
# plt.xlabel("relative estimation X (m)", labelpad=20)
# plt.gcf().subplots_adjust(bottom=0.15)
# plt.show()

# Figure of PID control VS NDI control
# first run PID, then NDI, then plot
# xTrue = np.random.uniform(-3, 3, (3, numRob))
# relativeState = np.zeros((3, numRob, numRob))
# data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
# relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)
# uNois = np.zeros((3, numRob))
# delayInput = [np.zeros((3, numRob)) for i in range(7)]
# delayIndex = 0
# xEsti = relativeState[:,0,:] # relative states in robot0's body-frame
# xTrueRL = transform.calcRelaState(xTrue, numRob) # groundTruth relative states
# dataForPlot = []
# simTime = 120
# step = 0
# while simTime >= dt*step:
#     step += 1
#     # u = data.calcInput_Formation01(step, relativeState) ### PID
#     u, refer = data.calcInput_NDI(step, relativeState, uNois) ### NDI
#     delayIndex = delayIndex + 1
#     if delayIndex == 7:
#         delayIndex = 0
#     delayInput[delayIndex] = u
#     u_delay = delayInput[delayIndex+1 if delayIndex<6 else 1]
#     xTrue, zNois, uNois = data.update(xTrue, u_delay)   
#     if step % ekfStride == 0:
#         relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
#     if step>5000:
#         xEsti = relativeState[:,0,:]
#         xTrueRL = transform.calcRelaState(xTrue, numRob)
#         # dataForPlot.append([xTrueRL[0,1], xTrueRL[1,1]]) ### PID
#         dataForPlot.append([xTrueRL[0,1], xTrueRL[1,1], refer[0,0], refer[0,1]]) ### NDI
# dataForPlot = np.array(dataForPlot)

# mat_data = np.matrix(dataForPlot)
# with open('mat_NDI.txt', 'w') as f:
#     for line in mat_data:
#         np.savetxt(f, line, fmt='%.5f')

with open('mat_PID.txt', 'r') as f:
    lines = f.readlines()
    lines_string_list = [line.strip() for line in lines if len(line.strip().split()[1:]) != 0]
PID_list = [list(map(float, line_string.split())) for line_string in lines_string_list]
xy_PID = np.array(PID_list)
with open('mat_NDI.txt', 'r') as f:
    lines = f.readlines()
    lines_string_list = [line.strip() for line in lines if len(line.strip().split()[1:]) != 0]
NDI_list = [list(map(float, line_string.split())) for line_string in lines_string_list]
xy_NDI = np.array(NDI_list)

x_axis_value = np.arange(0, len(xy_PID[:, 0]))/100
f, (ax1, ax2) = plt.subplots(2, figsize=(12, 4), sharex=True)
plt.margins(x=0)
ax1.plot(x_axis_value, xy_PID[:, 0])
ax1.plot(x_axis_value, xy_NDI[:, 0])
ax1.plot(x_axis_value, xy_NDI[:, 2])
ax1.set_ylabel("x (m)")
ax1.grid(True)
ax2.plot(x_axis_value, xy_PID[:, 1], label='PID control')
ax2.plot(x_axis_value, xy_NDI[:, 1], label='NDI control')
ax2.plot(x_axis_value, xy_NDI[:, 3], label='reference')
ax2.set_ylabel("y (m)")
ax2.grid(True)
ax2.set_xlabel("Time (s)")
ax2.legend()
# Fine-tune figure; make subplots close to each other and hide x ticks for
# all but bottom plot.
f.subplots_adjust(hspace=0)
plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
plt.margins(x=0)
plt.show()

# dataForPlotArray = dataForPlot.T
# timePlot = np.arange(0, len(dataForPlotArray[0]))/100
# f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
# plt.margins(x=0)
# ax1.plot(timePlot, dataForPlotArray[0,:])
# ax1.plot(timePlot, dataForPlotArray[1,:])
# ax1.set_ylabel(r"$x_{ij}$ (m)", fontsize=12)
# ax1.grid(True)
# ax2.plot(timePlot, dataForPlotArray[2,:])
# ax2.plot(timePlot, dataForPlotArray[3,:])
# ax2.set_ylabel(r"$y_{ij}$ (m)", fontsize=12)
# ax2.grid(True)
# ax3.plot(timePlot, dataForPlotArray[4,:], label='Relative EKF')
# ax3.plot(timePlot, dataForPlotArray[5,:], label='Ground-truth')
# ax3.set_ylabel(r"$\mathrm{\psi_{ij}}$ (rad)", fontsize=12)
# ax3.set_xlabel("Time (s)", fontsize=12)
# ax3.grid(True)
# ax3.legend(loc='upper center', bbox_to_anchor=(0.8, 0.6), shadow=True, ncol=1, fontsize=12)
# # Fine-tune figure; make subplots close to each other and hide x ticks for all but bottom plot.
# f.subplots_adjust(hspace=0)
# plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
# plt.show()
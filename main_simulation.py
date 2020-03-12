"""
Project: Simulation of a swarm of robots with relative localization
Author: Shushuai Li, MAVLab, TUDelft
Reference: arxiv link

This file: main file to show animation or figure plot of the relative position and yaw
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

# Variables being updated in simulation
xTrue = np.random.uniform(-3, 3, (3, numRob)) # random initial groundTruth of state [x, y, yaw]' of numRob robots
relativeState = np.zeros((3, numRob, numRob)) # [x_ij, y_ij, yaw_ij]' of the second robot in the first robot's view
data = dataCreate(numRob, border, maxVel, dt, devInput, devObser)
relativeEKF = EKFonSimData(10, 0.1, 0.25, 0.4, 0.1, numRob)

def animate(step):
    global xTrue, relativeState
    u = data.calcInput_FlyIn1m(step)
    # u = data.calcInput_PotentialField(step, xTrue)
    # u = data.calcInput_Formation01(step, relativeState)
    xTrue, zNois, uNois = data.update(xTrue, u)   
    if step % ekfStride == 0:
        relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)
    xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
    pointsTrue.set_data(xTrue[0, :], xTrue[1, :]) # plot groundTruth points
    pointsEsti.set_data(xEsti[0, :], xEsti[1, :]) # plot estimated points
    pointsTrueHead.set_data(xTrue[0, :]+0.07*np.cos(xTrue[2, :]), xTrue[1, :]+0.07*np.sin(xTrue[2, :])) # heading
    pointsEstiHead.set_data(xEsti[0, :]+0.07*np.cos(xEsti[2, :]), xEsti[1, :]+0.07*np.sin(xEsti[2, :])) # heading
    circle.center = (xTrue[0, 0], xTrue[1, 0])
    circle.radius = zNois[0, 1] # plot a circle to show the distance between robot 0 and robot 1
    time_text.set_text("t={:.2f}s".format(step * dt))
    return pointsTrue, pointsEsti, circle, pointsTrueHead, pointsEstiHead, time_text

if show_animation:
    # Set up an animation
    fig = plt.figure()
    ax  = fig.add_subplot(111, aspect='equal')
    ax.set(xlim=(border["xmin"], border["xmax"]), ylim=(border["ymin"], border["ymax"]))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    title = ax.set_title('Simulated swarm')
    pointsTrue,  = ax.plot([], [], linestyle="", marker="o", color="b", label="GroundTruth")
    pointsEsti,  = ax.plot([], [], linestyle="", marker="o", color="r", label="Relative EKF")
    pointsTrueHead,  = ax.plot([], [], linestyle="", marker=".", color="g")
    pointsEstiHead,  = ax.plot([], [], linestyle="", marker=".", color="g")
    ax.legend()
    circle = plt.Circle((0, 0), 0.2, color='black', fill=False)
    ax.add_patch(circle)
    time_text = ax.text(0.01, 0.97, '', transform=ax.transAxes)
    time_text.set_text('')
    ani = animation.FuncAnimation(fig, animate, frames=None, interval=10, blit=True)
    #ani.save('particle_box.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
    plt.show()   
else:
    # Simulation using while-loop; figure of X-Y-Yaw errors
    # xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob) # absolute states of all robots in world-frame
    # dataForPlot = np.array([xEsti[0,1], xTrue[0,1], xEsti[1,1], xTrue[1,1], xEsti[2,1], xTrue[2,1]]) # x, xGT, y, yGT, yaw, yawGT
    xEsti = relativeState[:,0,:] # relative states in robot0's body-frame
    xTrueRL = transform.calcRelaState(xTrue, numRob) # groundTruth relative states
    dataForPlot = np.array([xEsti[0,1], xTrueRL[0,1], xEsti[1,1], xTrueRL[1,1], xEsti[2,1], xTrueRL[2,1]]) # x, xGT, y, yGT, yaw, yawGT    
    step = 0
    while simTime >= dt*step:
        step += 1
        u = data.calcInput_FlyIn1m(step)
        # u = data.calcInput_PotentialField(step, xTrue)
        # u = data.calcInput_Formation01(step, relativeState)
        # u = data.calcInput_FlyIn1mRob1NoVel(step)
        xTrue, zNois, uNois = data.update(xTrue, u)
        if step % ekfStride == 0:
            relativeState = relativeEKF.EKF(uNois, zNois, relativeState, ekfStride)

        # xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
        # dataForPlot = np.vstack([dataForPlot, np.array([xEsti[0,1], xTrue[0,1], xEsti[1,1], xTrue[1,1], xEsti[2,1], xTrue[2,1]])])
        xEsti = relativeState[:,0,:]
        xTrueRL = transform.calcRelaState(xTrue, numRob)
        dataForPlot = np.vstack([dataForPlot, np.array([xEsti[0,1], xTrueRL[0,1], xEsti[1,1], xTrueRL[1,1], xEsti[2,1], xTrueRL[2,1]])]) 

    dataForPlotArray = dataForPlot.T
    timePlot = np.arange(0, len(dataForPlotArray[0]))/100
    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
    plt.margins(x=0)
    ax1.plot(timePlot, dataForPlotArray[0,:])
    ax1.plot(timePlot, dataForPlotArray[1,:])
    ax1.set_ylabel(r"$x_{ij}$ (m)", fontsize=12)
    ax1.grid(True)
    ax2.plot(timePlot, dataForPlotArray[2,:])
    ax2.plot(timePlot, dataForPlotArray[3,:])
    ax2.set_ylabel(r"$y_{ij}$ (m)", fontsize=12)
    ax2.grid(True)
    ax3.plot(timePlot, dataForPlotArray[4,:], label='Relative EKF')
    ax3.plot(timePlot, dataForPlotArray[5,:], label='Ground-truth')
    ax3.set_ylabel(r"$\mathrm{\psi_{ij}}$ (rad)", fontsize=12)
    ax3.set_xlabel("Time (s)", fontsize=12)
    ax3.grid(True)
    ax3.legend(loc='upper center', bbox_to_anchor=(0.8, 0.6), shadow=True, ncol=1, fontsize=12)
    # Fine-tune figure; make subplots close to each other and hide x ticks for all but bottom plot.
    f.subplots_adjust(hspace=0)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    plt.show()
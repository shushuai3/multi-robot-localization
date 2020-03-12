"""
This file: animation or figure plot of the relative position and yaw based on real-world data
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from dataCreate import realData
from relativeEKF import EKFonRealData
import transform

# system settings
show_animation = True
border = {"xmin":-4, "xmax":4, "ymin":-4, "ymax":4, "zmin":0, "zmax":4}
numRob = 3 # number of robots
dt = 0.01 # time interval [s]

relativeState = np.zeros((3, numRob, numRob))
dataFromReal = realData("./dataset/dat02.csv", numRob)
uList, zList, GtList, simTime = dataFromReal.readDataTolist()
relaEKFonRealData = EKFonRealData(10, 1, 0.25, 0.1, 0.1, numRob)

def animate(step):
    global relativeState
    xTrue, zNois, uNois = dataFromReal.calcInputDataset(uList[step], zList[step], GtList[step])
    relativeState = relaEKFonRealData.EKF(uNois, zNois, relativeState)
    xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
    pointsTrue.set_data(xTrue[0, :], xTrue[1, :])
    pointsEsti.set_data(xEsti[0, :], xEsti[1, :])
    pointsTrueHead.set_data(xTrue[0, :]+0.07*np.cos(xTrue[2, :]), xTrue[1, :]+0.07*np.sin(xTrue[2, :]))
    pointsEstiHead.set_data(xEsti[0, :]+0.07*np.cos(xEsti[2, :]), xEsti[1, :]+0.07*np.sin(xEsti[2, :]))   
    circle.center = (xTrue[0, 0], xTrue[1, 0])
    circle.radius = zNois[0, 1]
    time_text.set_text("t={:.2f}s".format(step * dt))
    return pointsTrue, pointsEsti, circle, pointsTrueHead, pointsEstiHead, time_text

if show_animation:
    # set up figure and animation
    fig = plt.figure()
    ax  = fig.add_subplot(111, aspect='equal')
    ax.set(xlim=(border["xmin"], border["xmax"]), ylim=(border["ymin"], border["ymax"]))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    title = ax.set_title('Real-world swarm')
    pointsTrue,  = ax.plot([], [], linestyle="", marker="o", color="b", label="Ground-truth")
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
    pos01ekfVSgt = [[] for i in range(6)] # x, xGT, y, yGT, z, zGT
    step = 0
    while simTime >= dt*step:
        step += 1
        xTrue, zNois, uNois = dataFromReal.calcInputDataset(uList[step], zList[step], GtList[step])
        relativeState = relaEKFonRealData.EKF(uNois, zNois, relativeState)
        xEsti = transform.calcAbsPosUseRelaPosWRTRob0(xTrue[:,0], relativeState, xTrue, numRob)
        cmpWhichRob = 1 # show position of cmpWhichRob calculated by rob0 and relativeState
        pos01ekfVSgt[0].append(xEsti[0,cmpWhichRob])
        pos01ekfVSgt[1].append(xTrue[0,cmpWhichRob])
        pos01ekfVSgt[2].append(xEsti[1,cmpWhichRob])
        pos01ekfVSgt[3].append(xTrue[1,cmpWhichRob])
        pos01ekfVSgt[4].append(xEsti[2,cmpWhichRob])
        pos01ekfVSgt[5].append(xTrue[2,cmpWhichRob])
    pos01array = np.array(pos01ekfVSgt)
    timePlot = np.arange(0, len(pos01ekfVSgt[0]))/100
    f, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
    plt.margins(x=0)
    ax1.plot(timePlot, pos01array[0,:])
    ax1.plot(timePlot, pos01array[1,:])
    ax1.set_ylabel("x (m)")
    ax1.grid(True)
    ax2.plot(timePlot, pos01array[2,:])
    ax2.plot(timePlot, pos01array[3,:])
    ax2.set_ylabel("y (m)")
    ax2.grid(True)
    ax3.plot(timePlot, pos01array[4,:], label='relative EKF')
    ax3.plot(timePlot, pos01array[5,:], label='ground-truth')
    ax3.set_ylabel(r"$\mathrm{\psi (rad)}$")
    ax3.set_xlabel("Time (sec)")
    ax3.grid(True)
    ax3.legend(loc='upper center', bbox_to_anchor=(0.8, 0.6), shadow=True, ncol=1)
    # Fine-tune figure; make subplots close to each other and hide x ticks for all but bottom plot.
    f.subplots_adjust(hspace=0)
    plt.setp([a.get_xticklabels() for a in f.axes[:-1]], visible=False)
    plt.show()
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation
from   mpl_toolkits.mplot3d import Axes3D

numRobots = 10
pos3d = np.random.uniform(0, 1, (numRobots, 3)) # numRobots*3

def update_graph(num):
    global pos3d
    d_pos = 0.01 * np.random.uniform(0, 1, (numRobots, 3)) - 0.005
    pos3d = pos3d + d_pos
    graph.set_data(pos3d[:, 0], pos3d[:, 1])
    graph.set_3d_properties(pos3d[:, 2])
    title.set_text('3D Test, time={}s'.format(num))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
title = ax.set_title('3D Test')
graph, = ax.plot(pos3d[:, 0], pos3d[:, 1], pos3d[:, 2], linestyle="", marker="o")

# Delay 10ms between two frames
ani = matplotlib.animation.FuncAnimation(fig, update_graph, None, interval=10)
plt.show()

# -- another 3D animation -- #
# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation
# from   mpl_toolkits.mplot3d import Axes3D

# numRobots = 10
# pos3d = np.random.uniform(0, 1, (numRobots, 3)) # numRobots*3

# def update_graph(num):
#     global pos3d
#     d_pos = 0.01 * np.random.uniform(0, 1, (numRobots, 3)) - 0.005
#     pos3d = pos3d + d_pos
#     graph._offsets3d = (pos3d[:, 0], pos3d[:, 1], pos3d[:, 2])
#     title.set_text('3D Test, time={}s'.format(num))

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# title = ax.set_title('3D Test')
# graph = ax.scatter(pos3d[:, 0], pos3d[:, 1], pos3d[:, 2])

# # Delay 10ms between two frames
# ani = matplotlib.animation.FuncAnimation(fig, update_graph, None, interval=10)
# plt.show()

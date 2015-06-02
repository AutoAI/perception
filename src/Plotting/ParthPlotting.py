import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def drawLine(startX, startY, startZ, endX, endY, endZ):
    ax.plot([startX, endX], [startY, endY],zs=[startZ, endZ])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

drawLine(1, 1, 1, 2, 1, 3)
drawLine(0, 0, 0, 3, 3, 3)

plt.show()
Axes3D.plot()

#!/usr/bin/env python

from numpy import *
from matplotlib import pyplot as plt
import matplotlib
import mpl_toolkits.mplot3d.axes3d as p3
from fractions import *
from matplotlib import *
from scipy.interpolate import *
from scipy.spatial import Delaunay

import math
import random

# make a list for each point of all its neighbors
def find_neighbors(tess, points):

	neighbors = []

	for point in range(points.shape[0]):
		neighbors[point] = []

	for simplex in tess.simplices:
		neighbors[simplex[0]].add([simplex[1],simplex[2]])
		neighbors[simplex[1]].add([simplex[2],simplex[0]])
		neighbors[simplex[2]].add([simplex[0],simplex[1]])

	return neighbors

def f(x, y):
	d = math.sqrt(x**2 + y**2)
	return math.cos(d)*math.exp(-d/4)

def dist2(t1, t2):
	return (t1[0]-t2[0])**2 + (t1[1]-t2[1])**2

def dist(t1, t2):
	return math.sqrt(dist2(t1, t2))

def imgToCoord(x, m, d):
	return x*2*m/d-m

def coordToImg(x, m, d):
	return (x+m)*d/(2*m)

def drawLine(startX, startY, startZ, endX, endY, endZ):
	ax.plot([startX, endX], [startY, endY],zs=[startZ, endZ])

# Setting up the input
n = 80
m = 6.28
# n random x values between -m and m
x = [random.random()*2*m-m for _ in range(0, n)]
# n random y values between -m and m
y = [random.random()*2*m-m for _ in range(0, n)]
# z = f(x, y)
z = []
for i in range(0, n):
	z.append(f(x[i], y[i]))

p = []
for i in range(0,n):
	p.append([x[i], y[i]])

# dat triangulation tho
tri = Delaunay(p)

# Plot it up
fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
for simplex in tri.vertices:
	drawLine(x[simplex[0]], y[simplex[0]], z[simplex[0]], x[simplex[1]], y[simplex[1]], z[simplex[1]])
	drawLine(x[simplex[1]], y[simplex[1]], z[simplex[1]], x[simplex[2]], y[simplex[2]], z[simplex[2]])
	drawLine(x[simplex[2]], y[simplex[2]], z[simplex[2]], x[simplex[0]], y[simplex[0]], z[simplex[0]])
plt.show()
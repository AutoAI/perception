#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from scipy.spatial import Delaunay
from scipy.spatial import KDTree

import math
import random

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
	return int((x+m)*d/(2*m))

def drawLine(startX, startY, startZ, endX, endY, endZ, c):
	ax.plot([startX, endX], [startY, endY],zs=[startZ, endZ], color=c)

# Setting up the input
n = 10
m = 4.71
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

# Dat triangulation tho
tri = Delaunay(p)

# Plot it up
plot_points = False
plot_triangulation = False

fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
if(plot_points):
	for i in range(0,n):
		ax.scatter(x, y, z, c = 'k')
if(plot_triangulation):
	for simplex in tri.vertices:
		drawLine(x[simplex[0]], y[simplex[0]], z[simplex[0]], x[simplex[1]], y[simplex[1]], z[simplex[1]], 'g')
		drawLine(x[simplex[1]], y[simplex[1]], z[simplex[1]], x[simplex[2]], y[simplex[2]], z[simplex[2]], 'g')
		drawLine(x[simplex[2]], y[simplex[2]], z[simplex[2]], x[simplex[0]], y[simplex[0]], z[simplex[0]], 'g')
plt.show()
#!/usr/bin/env python

from numpy import *
from matplotlib import pyplot as p
import matplotlib
import mpl_toolkits.mplot3d.axes3d as p3
from fractions import *
from matplotlib import *
from scipy.interpolate import *

import math
import random

def dist2(t1, t2):
	return (t1[0]-t2[0])**2 + (t1[1]-t2[1])**2

def dist(t1, t2):
	return math.sqrt(dist2(t1, t2))

def imgToCoord(x, m, d):
	return x*2*m/d-m

def coordToImg(x, m, d):
	return (x+m)*d/(2*m)

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

# Setting up the image
# image is d-by-d pixels
d = 30
img1 = InterpolatedImage(d, n, 2)
for i in range(0, n):
	img1.processPointCircle(coordToImg(x[i], m, d), coordToImg(y[i], m, d), coordToImg(z[i], m, d))

# Making the plot
fig = p.figure()
ax = p3.Axes3D(fig)
# ax.scatter(x,y,z) # use this to plot input
img1.plotAll(ax)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
p.show()

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

class InterpolatedImage:
	def __init__(self, s, n, r):
		# dimension of image
		self.s = s
		# number of points that are going to be processed
		self.n = n
		# inradius of the square around which points will be drawn
		self.r = r
		# values that will be stored from points
		self.img = numpy.zeros((s, s, n))
		# number of values that have been stored at each index
		self.lengths = numpy.zeros((s, s), numpy.int32)

	def processPointSquare(self, x, y, z):
		for i in range(int(max(round(x)-self.r,0)), int(min(round(x)+self.r, self.s))):
			for j in range(int(max(round(y)-self.r, 0)), int(min(round(y)+self.r, self.s))):
				self.img[i][j][self.lengths[i][j]] = z
				self.lengths[i][j] += 1

	def processPointCircle(self, x, y, z):
		for i in range(int(max(round(x)-self.r,0)), int(min(round(x)+self.r, self.s))):
			print "r:", self.r
			print "dx: ", (i-x)**2
			for j in range(int(max(round(y)-math.sqrt(self.r**2 - (i-x)**2), 0)), int(min(round(y)+math.sqrt(self.r**2 - (i-x)**2), self.s))):
				self.img[i][j][self.lengths[i][j]] = z
				self.lengths[i][j] += 1

	def plotAll(self, ax):
		x = []
		y = []
		z = []
		for i in range(0, self.s):
			for j in range(0, self.s):
				for k in range(0, self.lengths[i][j]):
					x.append(i)
					y.append(j)
					z.append(self.img[i][j][k])
		ax.scatter(x, y, z)

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

from numpy import *
from matplotlib import pyplot as p
import matplotlib
import mpl_toolkits.mplot3d.axes3d as p3
from fractions import *
from matplotlib import *
from scipy.interpolate import *

import math
import random
def f(x, y):
	d = math.sqrt(x**2 + y**2)
	return math.cos(d)*math.exp(-d)

def dist2(t1, t2):
	return (t1[0]-t2[0])**2 + (t1[1]-t2[1])**2

def dist(t1, t2):
	return math.sqrt(dist2(t1, t2))

# Setup the points
n = 100
m = 10
# n random x values between -m and m
x = [random.random()*2*m-m for _ in range(0, n)]
# n random y values between -m and m
y = [random.random()*2*m-m for _ in range(0, n)]
# z = f(x, y)
z = []
for i in range(0, n):
	z.append(f(x[i], y[i]))



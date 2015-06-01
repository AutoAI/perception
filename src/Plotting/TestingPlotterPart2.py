#!/usr/bin/env python

from TestingPlotter import *
from numpy import *
import pylab as p
import mpl_toolkits.mplot3d.axes3d as p3
from fractions import *


from scipy.interpolate import *

x = raw_input("Enter Points: ")

x = x.split("::")
i = 0
while i < len(x):
    x[i] = x[i].split(",")
  
    x[i][0] = int(x[i][0])
    x[i][1] = int(x[i][1])
    i+=1

w = tuplesort(x)

for l in w:
    #Needs to be repaired
    q1 = interpoints(l[0],l[1])
    q2 = interpoints(l[0],l[2])
    q3 = interpoints(l[1],l[2])
    x1 = linspace(l[0][0], l[1][0],20)
    x2= linspace(l[0][0], l[2][0],20)
    x3 = linspace(l[1][0], l[2][0],20)

    
    f1 = interp1d([m[0] for m in q1],[m[1] for m in q1])
    f2 = interp1d([m[0] for m in q2], [m[1] for m in q2])
    f3 = interp1d([m[0] for m in q3], [m[1] for m in q3])

    p.plot(co(x1),[f1(i) for i in co(x1)])
    p.plot(co(x2),[f2(i) for i in co(x2)])
    p.plot(co(x3),[f3(i) for i in co(x3)])

p.show()




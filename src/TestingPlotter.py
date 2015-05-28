from numpy import *
import pylab as p
#import matplotlib.axes3d as p3
import mpl_toolkits.mplot3d.axes3d as p3
from fractions import *


from scipy.interpolate import *

#plotting
plotting = '''
#Set up x, y, z
x = [2,1,3]
y = [2,1,3]
z = [2,1,3]

fig=p.figure()
ax = p3.Axes3D(fig)
ax.plot_wireframe(x,y,z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
p.show()'''

#Image normailztion

#EAch item comes a list 3-tuple (x,y,z)
#[[1,1,2], [2,2,3] ... ] along with predefined xsize, ysize
#Distributed System for 
#Consider generalizing to all possible x,y y,z x,z distances and then xyz
def dist(t1,t2): #(near by points based on x,y should be nearby
    
    return (t1[0]-t2[0])**2 + (t1[1]-t2[1])**2

def co(li):
    i = 0
    m = []
    while i < len(li):
        m.append(li[i])
        i+=1
    return m

#Integer points on a line specified by 2 points: p1 p2.
# m = (p2y - p1y)/(p2x - p1x) #define the case of m = infinity. Find the smallest n \in N s.t. nm \in N. 
#p2y p2x --> (p2y + nm), p2x+n is the next integer point. Repeat to detect all integer points until upper bound is discovered




def interpoints(p1,p2): #To be used once we have a discretized model in C++
    iarray = []
    xd = p2[0]-p1[0]
    yd = p2[1]-p1[1]

    if xd == 0: #infinity case:
        if p1[1] < p2[1]:
            g=p1
            while g[1] <= p2[1]:
                iarray.append(co(g))
                g[1]+=1
        
        else:
            g = p2
            while p1[1] >= g[1]:
                iarray.append(co(g))
                g[1]+=1
       

    else:
        m = Fraction(yd,xd)
        st = m.denominator #the step amount is that much

        if p1[0] <p2[0]: 
            g = p1
            while g[0] <= p2[0]: #bang out all the points
                iarray.append(co(g))
                g[0]+=st
                g[1]+=int(m*st)
                
           

        else:
            g = p2
            while p1[0] >= g[0]:
                iarray.append(co(g))
                g[0]+=st
                g[1]+=int(m*st)

    return iarray
            
        



def tuplesort(tlist): #Sids ghetto approximation of Delaunay triangulation
    dlist = [] #dlist is a list of tuples
    d1 = 0
    d2 = 0
    temtuple = [tlist[0], tlist[1], tlist[1]]
    temdist = [dist(tlist[0],tlist[1]),dist(tlist[0],tlist[1])]
    i = 0
    while i < len(tlist):
        j = 0
        while j < len(tlist):
            if j == i:
                j+=1
                if j >= len(tlist):
                    break
           
            cd = dist(tlist[i],tlist[j]) #distance from current point to nodej
            if cd < temdist[0]:
                temdist[1] = temdist[0]
                temdist[0] = cd
                temtuple[2] = temtuple[1]
                temtuple[1] = tlist[j]
            
                
                
            
            
            
            j+=1
        dlist.append(co(temtuple))
        i+=1
        if i < len(tlist):
            temtuple = [tlist[i],tlist[(i+1)%len(tlist)],tlist[(i+2)%len(tlist)]]
        
            temdist = [dist(tlist[i],tlist[(i+1)%len(tlist)]),dist(tlist[i],tlist[(i+2)%len(tlist)])] #distances

            if temdist[1] < temdist[0]:
                s = temtuple[1]
                temtuple[1] = temtuple[2]
                temtuple[2] = s
                
                s = temdist[0]
                temdist[0] = temdist[1]
                temdist[1] = s
            

    return dlist


w = '''
t =  interpoints([-1,0,6],[11,12,7])
f = interp1d([m[0] for m in t],[m[1] for m in t])
xt = linspace(0, 10, 40) #a to b, smapled with frequency c (a,b,c)
print xt
p.plot(xt,[f(i) for i in xt],'-')
p.show()'''


        
        





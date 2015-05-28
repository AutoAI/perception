from TestingPlotter import *

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

print w



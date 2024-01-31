# Wally Draw Axiom

import matplotlib.pyplot as plt
import numpy as np
import math

def straight(PresPos,NewPos, step):
    
# Calculating the x and y changes in position
    deltax = NewPos[0] - PresPos[0]
    deltay = NewPos[1] - PresPos[1]
    L = math.sqrt(deltax**2 + deltay**2)
    Div = int(L/math.ceil(step))
    stepx = deltax/Div
    stepy = deltay/Div
    
    X = [PresPos[0]]
    Y = [PresPos[1]]

    for i in range(1,Div):
        x = PresPos[0] + i*stepx
        y = PresPos[1] + i*stepy
        X.append(x)
        Y.append(y)
        
        
    return X,Y

X = []
Y = []

# A

XA = [-100,-60,-60]
YA = [220,280,220]

X.append(XA)
Y.append(YA)

# X

XX = [-50,-25,-5,-25,-50,-5]
YX = [220,250,280,250,280,220]

X.append(XX)
Y.append(YX)

# I

XI = [5,5,5]
YI = [220,280,220]

X.append(XI)
Y.append(YI)

# O

XO = [15,15,45,45,15,45]
YO = [220,280,280,220,220,220]

X.append(XO)
Y.append(YO)

# M

XM = [55,55,75,80,100,100]
YM = [220,280,260,260,280,220]

X.append(XM)
Y.append(YM)

fig,ax = plt.subplots()
ax.plot(X,Y)
plt.show()
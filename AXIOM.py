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

def StraightLines(X,Y):
    Xp = []
    Yp = []
    for i in range(1,len(X)):
        Xs,Ys = straight([X[i-1],Y[i-1]],[X[i],Y[i]],2)
        for j in range(len(Xs)):
            Xp.append(Xs[j])
            Yp.append(Ys[j])
    return Xp,Yp

def AXIOM():

    X = []
    Y = []

    # A

    XA = [-100,-60,-60]
    YA = [220,280,220]

    for i in range(len(XA)):
        X.append(XA[i])
        Y.append(YA[i])

    # X

    XX = [-50,-27.5,-5,-27.5,-50,-5]
    YX = [220,250,280,250,280,220]

    for i in range(len(XX)):
      X.append(XX[i])
      Y.append(YX[i])

# I

    XI = [5,5,5]
    YI = [220,280,220]

    for i in range(len(XI)):
       X.append(XI[i])
       Y.append(YI[i])

    # O

    XO = [15,15,45,45,15,45]
    YO = [220,280,280,220,220,220]

    for i in range(len(XO)):
        X.append(XO[i])
        Y.append(YO[i])

# M

    XM = [55,55,75,80,100,100]
    YM = [220,280,260,260,280,220]

    for i in range(len(XM)):
        X.append(XM[i])
        Y.append(YM[i])

    X,Y = StraightLines(X,Y)
    return X,Y

X,Y = AXIOM()

print(X)
print(Y)

fig,ax = plt.subplots()
ax.plot(X,Y)
plt.show()
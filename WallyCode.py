# Wally first code

from dynamixel_sdk import*
import math
import numpy as np
import matplotlib as mp
import matplotlib.pyplot as plt
import time
import os
    

class Motor:
    def __init__(self,DXL_ID,OFFSET,ORIENTATION, Kp, Ki, Kd, Kff1):
        self.ID = DXL_ID # DXL Motor ID
        self.OFFSET = OFFSET*11.375 # Offset (what angle should be considered zero)
        if ORIENTATION == -1: # Oreintation of the motor (is it upside down)
            self.ORIENTATION = ORIENTATION
        elif ORIENTATION == 1:
            self.ORIENTATION = ORIENTATION
        else:
            print('Error: Orientation must be 1 or -1')
            exit()
        self.Kp = Kp # P gain
        self.Ki = Ki # I gain
        self.Kd = Kd # D gain
        self.Kff = Kff1 # Feedforward gain
        # Update PID gains
        packetHandler.write2ByteTxRx(portHandler, self.ID, ADDR_P_GAIN, Kp)
        packetHandler.write2ByteTxRx(portHandler, self.ID, ADDR_I_GAIN, Ki)
        packetHandler.write2ByteTxRx(portHandler, self.ID, ADDR_D_GAIN, Kd)
        packetHandler.write2ByteTxRx(portHandler, self.ID, ADDR_FF1_GAIN, Kff1)
        
    
    def move(self,Pos):
        packetHandler.write4ByteTxRx(portHandler, self.ID, ADDR_GOAL_POSITION, round(self.ORIENTATION*Pos*11.375 + self.OFFSET))
        
    def read(self):
        Pos, Result, Error = packetHandler.read4ByteTxRx(portHandler, self.ID, ADDR_PRESENT_POSITION)
        if Error != 0:
            print(packetHandler.getRxPacketError(Error))
        return self.ORIENTATION*round(Pos/11.375 - self.OFFSET/11.375)

    def enable(self,switch):
        if switch == 1:
            packetHandler.write1ByteTxRx(portHandler, self.ID, ADDR_TORQUE_ENABLE, 1)
        else:
            packetHandler.write1ByteTxRx(portHandler, self.ID, ADDR_TORQUE_ENABLE, 0)
            
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
        
def ZigZag(x,y,n,H):
    K = np.empty([2*n,1])
    for i in range(0,2*n):
        if i == 0:
            K[i] = 1
        elif ((i % 2) != 0) & (K[i-1] == 1):
            K[i] = -1
        elif ((i % 2) == 0) & (K[i-1] == -1):
            K[i] = -1
        elif ((i % 2) != 0) & (K[i-1] == -1):
            K[i] = 1
        else:
            K[i] = 1
            
    Pos = np.empty([2*n,2])
    for i in range(0,2*n):
        Pos[i,0] = x*K[i]
        
        if ((i % 2) == 0) & (i > 0):
            Pos[i,1] = y - H*(i/2)
        elif i > 0:
            Pos[i,1] = Pos[i-1,1]
        else:
            Pos[i,1] = y;
    
    # print(Pos)
    X = [x]
    Y = [y]
    
    for i in range(1,2*n):
        Xs,Ys = straight(Pos[i-1,:],Pos[i,:],5)
        for j in range(len(Xs)):
            X.append(Xs[j])
            Y.append(Ys[j])
        
    return X,Y

def FwdKin(motor1,motor2,l1,l2):
    motor1 = motor1*math.pi/180
    motor2 = motor2*math.pi/180
    y1 = l1*math.sin(motor1)
    x1 = l1*math.cos(motor1)
    y2 = l2*math.cos(motor2)
    x2 = l2*math.sin(motor2)
    Y = y1 + l2*math.sin(motor1 + motor2)
    X = x1 + l2*math.cos(motor1 + motor2)
    return X,Y

def InvKin(x,y,l1,l2):
    if x**2 + y**2 <= (l1+l2)**2:
        ct2 = (1/(2*l1*l2))*(x**2+y**2-l1**2-l2**2)
        st2 = math.sqrt(1-ct2**2)
        theta2 = math.atan2(st2,ct2);
        st1 = (l1+l2*math.cos(theta2))*y-l2*math.sin(theta2)*x
        ct1 = (l1+l2*math.cos(theta2))*x+l2*math.sin(theta2)*y
        theta1 = math.atan2(st1,ct1)
        theta1 = theta1/math.pi*180
        theta2 = theta2/math.pi*180
    else:
        print('Error: Coordinates outside of range')
        exit()
    theta1 = theta1
    theta2 = theta2
    return theta1,theta2
            
def FollowPath(X,Y):
    L = len(X)
    Motor1a = []
    Motor2a = []
    for i in range(L):
        M1,M2 = InvKin(X[i],Y[i],150,150)
        Motor1a.append(M1)
        Motor2a.append(M2)
    Pos1 = np.empty([L,1])
    Pos2 = np.empty([L,1])
    for i in range(L):
        Motor1.move(Motor1a[i])
        Motor2.move(Motor2a[i])
        Pos1[i] = Motor1.read()
        Pos2[i] = Motor2.read()
        
    return Pos1,Pos2

def ShowPath(M1,M2):
    L = len(M1)
    X = []
    Y = []
    for i in range(L):
        x,y = FwdKin(M1[i],M2[i],150,150)
        X.append(x)
        Y.append(y)
    return X,Y

def WallyRead():
    M1 = Motor1.read()
    M2 = Motor2.read()
    X,Y = FwdKin(M1,M2,150,150)
    return [X,Y]

def WallyMove(X,Y):
    M1,M2 = InvKin(X,Y,150,150)
    Motor1.move(M1)
    Motor2.move(M2)
    
    
    

# Control Table Adresses
global ADDR_TORQUE_ENABLE
global ADDR_GOAL_POSITION
global ADDR_PRESENT_POSITION 
global ADDR_P_GAIN
global ADDR_I_GAIN
global ADDR_D_GAIN
global portHandler
global packetHandler
global CONVERSION
global ADDR_FF1_GAIN
ADDR_TORQUE_ENABLE = 64 # 1 byte
ADDR_GOAL_POSITION = 116 # 4 byte
ADDR_PRESENT_POSITION = 132 # 4 byte
ADDR_P_GAIN = 84 # 2 byte
ADDR_I_GAIN = 82 # 2 byte
ADDR_D_GAIN = 80 # 2 byte
ADDR_FF1_GAIN = 90 # 2 byte
MINPOS  = 0         # Refer to the Minimum Position Limit of product eManual
MAXPOS  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600
CONVERSION = 11.375 # 1 Degree Conversion

# Control gains

# Motor 1
Kp1 = 850
Ki1 = 60
Kd1 = 150
Kff1 = 20

# Motor 2
Kp2 = 700
Ki2 = 75
Kd2 = 200
Kff2 = 15

# Establishing serial communication

portHandler = PortHandler('COM3')
packetHandler = PacketHandler(2.0)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# Creating the motor objects
Motor1 = Motor(2,90,1,Kp1,Ki1,Kd1,Kff1) # Base motor
Motor2 = Motor(3,180,-1,Kp2,Ki2,Kd2,Kff2) # Forearm motor

# Zeroing the robot
Motor1.enable(1)
Motor2.enable(1)

print("Move Wally into 0 position (positive x direction). Press enter to continue")
input()

Motor1.move(0)
Motor2.move(0)

print(Motor1.read())
print(Motor2.read())

# Moving robot

print("Wally will move to (0,212). Press enter to continue")
input()

WallyMove(0,212)

time.sleep(3)

print(WallyRead())

# Testing Straight and Zig Zag Function

X,Y = ZigZag(-100,200,3,20)
fig,ax = plt.subplots()
ax.plot(X,Y)
plt.show()

input('Make sure path is right and area is clear. Press enter to proceed')

Pos1,Pos2 = FollowPath(X,Y)
Xp,Yp = ShowPath(Pos1,Pos2)

fig,ax = plt.subplots()
ax.plot(X,Y)
ax.plot(Xp,Yp)
plt.show()

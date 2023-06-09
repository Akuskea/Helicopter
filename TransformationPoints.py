#!/usr/bin/env python
# coding: utf-8

#Anais Ponsonnet
"""
Click on and adjust viewing angle to activate. 

Press Enter or Space to step through the animation. 

ESC: terminates the program.

"""
import numpy as np
import math as mth
import matplotlib.pyplot as plt
import os
import time 
from vedo import *
from os import path


def RotationZ(theta):

    # Matrix of 3-D rotation by angle=theta about the z-axis
    #Rotation Z
    Rz = np.array(
    [[mth.cos(theta), -mth.sin(theta),0.0, 0.0], 
     [mth.sin(theta),mth.cos(theta), 0.0, 0.0],
     [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
    
    return Rz

def RotationX(theta):
    #Rotation X
    Rx = np.array(
    [[1.0, 0.0, 0.0, 0.0],
    [0.0,mth.cos(theta),-mth.sin(theta), 0.0],
     [0.0, mth.sin(theta), mth.cos(theta), 0.0],
     [0.0, 0.0, 0.0, 1.0]])
    return Rx


def getNewPosition_topPoint(p: np.ndarray, angle: float, T_01: np.ndarray, a:int, b:int) -> np.ndarray:
    # Convert point to homogeneous coords
    n,m = p.shape
    p_tilde = np.vstack((p, (np.ones((1,m)))))

    # Motion of the top point z Rotation 
    Rz = RotationZ(angle)
    
    # This matrix transforms global frame F{0} to the local frame F{1}
    T_10 = np.linalg.inv(T_01)

    #motion of top points
    topRotor_new = translation(a,b) @ T_01 @Rz @ T_10 @p_tilde
    
    # Convert from homogeneous to Cartesian before returning     
    return topRotor_new[:3]

def getNewPosition_tailPoint(p: np.ndarray, angle: float, T_02: np.ndarray, a:int, b:int) -> np.ndarray:
    # Convert point to homogeneous coords
    n,m = p.shape
    p_tilde = np.vstack((p, (np.ones((1,m)))))

    # Motion of the top point z Rotation 
    Rx = RotationX(angle)

    # This matrix transforms global frame F{0} to the local frame F{2}
    T_20 = np.linalg.inv(T_02)
    
    #motion of tail points
    tailRotor_new = translation(a,b)@ T_02 @ Rx @ T_20 @ p_tilde
    
    # Convert from homogeneous to Cartesian before returning     
    return tailRotor_new[:3]


def getNewPosition_bodyPoint(p: np.ndarray,length, T_03: np.ndarray, a:int, b:int) -> np.ndarray:
    # Convert point to homogeneous coords
    n,m = p.shape
    p_tilde = np.vstack((p, (np.ones((1,m)))))
    
    # This matrix transforms global frame F{0} to the local frame F{3}
    T_30 = np.linalg.inv(T_03)

    #motion of body points
    body_new= translation(a,b) @ T_03 @ T_30 @  p_tilde
    
    # Convert from homogeneous to Cartesian before returning     
    return body_new[:3]
    
def translation(Y, Z):
    # Define the translation matrix
    translation_matrix = np.array([[1.0, 0.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0, Y],
                                   [0.0, 0.0, 1.0, Z],
                                   [0.0, 0.0, 0.0, 1.0]])
    return translation_matrix

# function to make the helicopter move up
def moveUp(event):
    global pts_topR,pts_tailR,pts_bodyR, T_02, T_01

    # Get updated position of point/object 
    pts_topR =getNewPosition_topPoint(pts_topR, theta, T_01, 0,1)
    T_01 =translation(0,1) @ T_01

    pts_tailR = getNewPosition_tailPoint(pts_tailR, theta, T_02, 0,1)
    T_02 =translation(0,1) @ T_02
    pts_bodyR = getNewPosition_bodyPoint(pts_bodyR, theta, T_03, 0,1)
        
    # Update object's position 
    newTop.points(pts_topR)
    newTail.points(pts_tailR)
    newBody.points(pts_bodyR)
    
    # Update the scene 
    plt.render()
    time.sleep(0.1)
    video.add_frame()

# function to make the helicopter move forward
def moveForward(event):
    global pts_topR,pts_tailR,pts_bodyR, T_02, T_01

    # Get updated position of point/object 
    pts_topR =getNewPosition_topPoint(pts_topR, theta, T_01, -1,0)
    T_01 =translation(-1,0) @ T_01

    pts_tailR = getNewPosition_tailPoint(pts_tailR, theta, T_02, -1,0)
    T_02 =translation(-1,0) @ T_02

    pts_bodyR = getNewPosition_bodyPoint(pts_bodyR, theta, T_03, -1,0)
        
    # Update object's position 
    newTop.points(pts_topR)
    newTail.points(pts_tailR)
    newBody.points(pts_bodyR)
    
    # Update the scene 
    plt.render()
    time.sleep(0.1)
    video.add_frame()

# function to make the helicopter move down
def moveDown(event):
    global pts_topR,pts_tailR,pts_bodyR, T_02, T_01
    
    # Get updated position of point/object 
    pts_topR =getNewPosition_topPoint(pts_topR, theta, T_01, 0,-1)
    T_01 =translation(0,-1) @ T_01

    pts_tailR = getNewPosition_tailPoint(pts_tailR, theta, T_02, 0,-1)
    T_02 =translation(0,-1) @ T_02

    pts_bodyR = getNewPosition_bodyPoint(pts_bodyR, theta, T_03, 0,-1)
        
    # Update object's position 
    newTop.points(pts_topR)
    newTail.points(pts_tailR)
    newBody.points(pts_bodyR)
    
    # Update the scene 
    plt.render()
    time.sleep(0.1)
    video.add_frame()

# Read mesh files of each part, and color-label them
mainBodyMesh = Mesh("./main_body.vtk").c("white")
topRotorMesh = Mesh("./top_rotor.vtk").c("red")
tailRotorMesh = Mesh("./tail_rotor.vtk").c("blue")

#New object
newTop= topRotorMesh
newTail= tailRotorMesh
newBody= mainBodyMesh

#Transpose
pts_topR = topRotorMesh.points().T
pts_tailR = tailRotorMesh.points().T
pts_bodyR = mainBodyMesh.points().T

# Transformation matrix from local frame to global F0
T_01 = np.array(
        [[1.0, 0.0, 0.0, -39.3254],
            [0.0, 1.0, 0.0,  58.8874],
            [0.0, 0.0, 1.0, 20.7563],
            [0.0, 0.0, 0.0, 1.0],])
print("\nTransformatiin from local frame F{1} to global frame F{0}:\nT_01 = ")
print(T_01)
    
# Transformation matrix from local 2 frame to global 0
T_02 = np.array(
        [[1.0, 0.0, 0.0, -37.715],
            [0.0, 1.0, 0.0, 155.514],
            [0.0, 0.0, 1.0, 4.44849],
            [0.0, 0.0, 0.0, 1.0],])
print("\nTransformatiin from local frame F{2} to global frame F{0}:\nT_02 = ")
print(T_02)
    
# Transformation matrix from local F3 frame to global F0
T_03 = np.array(
        [[1.0, 0.0, 0.0, -17.6389],
            [0.0, 1.0, 0.0,  41.6236],
            [0.0, 0.0, 1.0, -3.69994],
            [0.0, 0.0, 0.0, 1.0],])
print("\nTransformatiin from local frame F{3} to global frame F{0}:\nT_03 = ")
print(T_03)


# Rotation step
theta = np.pi/20

#Video file
video = Video("tmp.mp4", 
              backend='ffmpeg', 
              fps = 24
             ) 

# Build the graphical scene with all objects and axes
plt = Plotter(size=(1050, 600))
plt += [newBody, newTop, newTail,  __doc__]
plt.background("black", "w").add_global_axes(axtype=1).look_at(plane='yz')


# Up
plt.add_callback("timer", moveUp)
plt.timer_callback("create", dt=50)
plt.show()
plt.remove_callback("timer")

#Forward
plt.add_callback("timer", moveForward)
plt.timer_callback("create", dt=50)
plt.show()
plt.remove_callback("timer")

#Down
plt.add_callback("timer", moveDown)
plt.timer_callback("create", dt=50)
plt.show().close()

# merge all the recorded frames
video.close()                        

# Convert the video file spider.mp4 to play on a wider range of video players 
if path.exists("./animation.mp4"):
    os.system("rm animation.mp4")
    
os.system("ffmpeg -i tmp.mp4 -pix_fmt yuv420p animation.mp4")
os.system("rm tmp.mp4")




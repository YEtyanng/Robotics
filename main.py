# -*- coding: utf-8 -*-
# @created on : 2020/3/22 22:03
# @name       : mainBill.py
# @IDE        : PyCharm

# Import Libraries:
from zmqRemoteApi import RemoteAPIClient
import sys
import time  # used to keep track of time
import numpy as np  # array library
import math
import matplotlib.pyplot as plt  # used for image plotting
import socket
import cv2


#begin the main1 code next:

# following is initializing:
# initialize communition with VREP
PI = math.pi  # pi=3.14..., constant
global persAngle
persAngle = 70
def myFunc(input1, input2):
    print('Hello', input1, input2)
    return 21

print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')
            # When simulation is not running, ZMQ message handling could be a bit
            # slow, since the idle loop runs at 8 Hz by default. So let's make
            # sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)
            # Run a simulation in stepping mode:
client.setStepping(False)
sim.startSimulation()


#get handles of four wheels and vision sensor
wheelJoints = [1,1,1,1] #wheel joints handle
wheelJoints[0] = sim.getObject('./rollingJoint_fl')
wheelJoints[1] = sim.getObject('./rollingJoint_rl')
wheelJoints[2] = sim.getObject('./rollingJoint_rr')
wheelJoints[3] = sim.getObject('./rollingJoint_fr')
#youBot = sim.getObject('./youBot#0')
#youBot = youBot[1]
visionSensorHandle = sim.getObject('./Vision_sensor')

#errprCode,resolution,rawimage = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_streaming)
time.sleep(0.5)    #initialize the visionsensor


def actuate_car(forwBackVel,leftRightVel,rotVel):    #set four wheel speed according to vx,vy,w
    v0 = (-forwBackVel + leftRightVel + 0.38655 * rotVel) / r
    v1 = (-forwBackVel - leftRightVel + 0.38655 * rotVel) / r
    v2 = (-forwBackVel + leftRightVel - 0.38655 * rotVel) / r
    v3 = (-forwBackVel - leftRightVel - 0.38655 * rotVel) / r
    sim.setJointTargetVelocity(wheelJoints[0], v0)
    sim.setJointTargetVelocity(wheelJoints[1], v1)
    sim.setJointTargetVelocity(wheelJoints[2], v2 )
    sim.setJointTargetVelocity(wheelJoints[3], v3)
    time.sleep(0.2)  # loop executes once every 0.2 seconds (= 5 Hz)


def readVisionSensor():
    global resolution
    img, [resX, resY] = sim.getVisionSensorImg(visionSensorHandle)
    resolution, [resX, resY] = sim.getVisionSensorImg(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
    img = cv2.flip(img, 0)
    image = img
    return image



image = readVisionSensor()
#opencv ROI and establish KCFtracker
tracker = cv2.TrackerKCF_create()
bbox1 = cv2.selectROI(image, False)
tracker.init(image, bbox1)



# Initialize the tracker with the bounding box and the frame

#bbox2 = cv2.selectROI('tracking', image)
#ok = tracker.add(cv2.TrackerKCF_create(), image, bbox2)
#bbox3 = cv2.selectROI('tracking', image)
#ok = tracker.add(cv2.TrackerKCF_create(), image, bbox3)

# P controller for rotVel:
outlast = 0
errorlast = 0
N = 1.2
def guidance(error):
    global outlast,errorlast
    out = outlast + N * (error - errorlast)
    outlast = out
    errorlast = error
    return out * PI/180

forwBackVel = 0 # m/s
leftRightVel = 0  # m/s
rotVel =  0   #rotVel = 10 * PI/ 180  #rad/s
r = 0.05      #wheel radium (meters)


t= time.time()

while (time.time() - t) < 180:  #loop for 180 seconds
    # capture image and Process the image to the format (64,64,3)
    image = readVisionSensor()
    success, bbox = tracker.update(image)
    target_center_x = []
    target_center_y = []
    sight_angle = 0
    print(resolution[0])
    if success:
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(image, p1, p2, (200, 0, 0))
        centerx = (int(bbox[0]) + int(bbox[2]) // 2)
        centery = (int(bbox[1]) + int(bbox[3]) // 2)
        target_center_x.append(centerx)
        target_center_y.append(centery)
        theta1 = (math.atan(2.0 * (centerx - resolution[0] / 2) * math.tan(persAngle * PI / 360.0) / 128)) * 180 / PI  # sight angle(degree)
        sight_angle=theta1   #sight angle -- left minus;right plus
    print('error angle is: ',sight_angle)
    print('error x distance is: ',target_center_x)

    cv2.imshow('tracking', image)

    forwBackVel = 0.06  # m/s
    leftRightVel = 0  # m/s
    rotVel = guidance(-sight_angle) # rotVel = 10 * PI/ 180  #rad/s
    print('rotVel is: ',rotVel*180/PI)
    actuate_car(forwBackVel, leftRightVel, 0)

    k = cv2.waitKey(1)
    if k == 27: break  # esc pressed


# finish and shut down the car
actuate_car(0, 0, 0)

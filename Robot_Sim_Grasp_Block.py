# -*- coding: utf-8 -*-
"""
Created on Thu Dec  1 14:04:10 2022

@author: jakek
"""

import pybullet as pb
import pybullet_data
import time
import numpy as np
# from detect_distance import *

physicsClient = pb.connect(pb.GUI)

pb.setAdditionalSearchPath(pybullet_data.getDataPath())

planeID = pb.loadURDF("plane.URDF")

cubeStartPos = []
endEffectorHeight = []
endEffectorHeightGrab = []
boxes = []
colorSort = []
sortPos = []
sortBoxes = []
y = 0
gfinger = .6
pb.setGravity(0, 0, -9.81)
numBoxes = 1
sortingPosition = []
boxSortedPosition = []
# jointPositions=[0.45, 0.458, 0.5, -2.24, -0.30, 0.66, 3]
jointPositions=[0.5, 0.458, 0.32, -1.7, -0.30, 0.66, 2.32]
k = 0
movetime = 20
griptime = 20

sortPos = [0.6, y, 0.05]

endEffectorHeight.append([sortPos[0], sortPos[1], sortPos[2]+.3])
endEffectorHeightGrab.append([sortPos[0], sortPos[1], sortPos[2]+0.15])

cubeStartOrientation = pb.getQuaternionFromEuler([0,0,0])
orn=[0.0, 0.0, 1, 0.0]
flags = pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

boxes.append(pb.loadURDF("cube.urdf", sortPos, cubeStartOrientation, globalScaling = .1, flags = flags))
pb.changeVisualShape(boxes[k], -1, rgbaColor=[0, 0, 0, 1])
    
robot = pb.loadSDF('kuka_iiwa/kuka_with_gripper.sdf')
robot = robot[0]
numJoints = pb.getNumJoints(robot)

for i in range(pb.getNumJoints(robot)): #prints joint index, name, and type
    jointInfo = pb.getJointInfo(robot, i)
    # print('Index', jointInfo[0])
    # print('Name', jointInfo[1])
    # print('Type', jointInfo[2])
    # print()

planeID = pb.loadURDF("plane.URDF")
pb.setRealTimeSimulation(0)
Orientation = pb.getQuaternionFromEuler([np.pi, 0., 0.])
                                           
time.sleep(2)
pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
targetPositionsJoints = pb.calculateInverseKinematics(robot, numJoints - 7, [0, 0, 3], targetOrientation = Orientation)
pb.setJointMotorControlArray(robot, range(numJoints-7), pb.POSITION_CONTROL, targetPositions = jointPositions)
for _ in range(movetime): #move to "rest" position
    pb.stepSimulation()
    time.sleep(1./10.)
for i in range (0, numBoxes):
    targetPositionsJoints = pb.calculateInverseKinematics(robot, numJoints - 3, endEffectorHeight[i], targetOrientation = Orientation)
    pb.setJointMotorControlArray(robot, range(numJoints-3), pb.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    # print(targetPositionsJoints)
    for _ in range(movetime): #move to positions hovering over the blocks
        pb.stepSimulation()
        pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -.01 ,force= 10)
        pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, .01 ,force= 10)
        pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, .5 ,force= 10)
        pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, -.5 ,force= 10)
        pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
        time.sleep(1./10.)
  #  # print(targetPositionsJoints)
    pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -gfinger ,force= 100)
    pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, gfinger ,force= 100)
    pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, -gfinger ,force= 100)
    pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, gfinger ,force= 100)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    for _ in range(griptime): #open the gripper
        pb.stepSimulation()
        time.sleep(1./10.)
    targetPositionsJoints = pb.calculateInverseKinematics(robot, numJoints - 3, endEffectorHeightGrab[i], targetOrientation = Orientation)
    pb.setJointMotorControlArray(robot, range(numJoints-3), pb.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    for _ in range(movetime): #move down around the block
        pb.stepSimulation()
        time.sleep(1./10.)
    pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -.01 ,force= 10)
    pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, .01 ,force= 10)
    pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, .5 ,force= 10)
    pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, -.5 ,force= 10)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    for _ in range(griptime): #close the gripper
        pb.stepSimulation()
        time.sleep(1./10.)
    targetPositionsJoints = pb.calculateInverseKinematics(robot, numJoints - 3, endEffectorHeight[i], targetOrientation = Orientation)
    pb.setJointMotorControlArray(robot, range(numJoints-3), pb.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    # print(targetPositionsJoints)
    for _ in range(movetime): #move to positions hovering over the blocks
            pb.stepSimulation()
            pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -.01 ,force= 10)
            pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, .01 ,force= 10)
            pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, .5 ,force= 10)
            pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, -.5 ,force= 10)
            pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
            time.sleep(1./10.)
    targetPositionsJoints = pb.calculateInverseKinematics(robot, numJoints - 3, endEffectorHeightGrab[i], targetOrientation = Orientation)
    pb.setJointMotorControlArray(robot, range(numJoints-3), pb.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    for _ in range(movetime): #move down around the block
        pb.stepSimulation()
        time.sleep(1./10.)
    pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -gfinger ,force= 100)
    pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, gfinger ,force= 100)
    pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, -gfinger ,force= 100)
    pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, gfinger ,force= 100)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    for _ in range(griptime): #open the gripper
        pb.stepSimulation()
        time.sleep(1./10.)
pb.disconnect()



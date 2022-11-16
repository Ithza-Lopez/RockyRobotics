import pybullet as pb
import pybullet_data
import time
from random import randint
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
y = .5
gfinger = .15
color_dict = {'1 RED': [[[0.45, y, 0.05], [0.455, 0.169, 0.18, 1]]], '2 ORANGE': [[[0.55, y, 0.05], [0.588, 0.318, 0.2, 1]]], '3 YELLOW': [[[0.15, y, 0.05], [0.541, 0.463, 0.2, 1]]], '4 GREEN': [[[.25, y, 0.05], [0.125, 0.325, 0.216, 1]]], '5 BLUE': [[[0.05, y, 0.05], [0.094, 0.204, 0.325, 1]]]}
pb.setGravity(0, 0, -9.81)
numBoxes = 5
sortingPosition = []
boxSortedPosition = []
# jointPositions=[0.45, 0.458, 0.5, -2.24, -0.30, 0.66, 3]
jointPositions=[0.5, 0.458, 0.32, -1.7, -0.30, 0.66, 2.32]
k = 0
movetime = 20
griptime = 10

for i in range(numBoxes):
    colorSort.append([])
    sortPos.append([])
for i in range(numBoxes):
    if '1 RED' in color_dict:
        colorSort[0] = color_dict['1 RED'][0][1]
        sortPos[0] = color_dict['1 RED'][0][0]
    if '2 ORANGE' in color_dict:
        colorSort[1] = color_dict['2 ORANGE'][0][1]
        sortPos[1] = color_dict['2 ORANGE'][0][0]
    if '3 YELLOW' in color_dict:
        colorSort[2] = color_dict['3 YELLOW'][0][1]
        sortPos[2] = color_dict['3 YELLOW'][0][0]
    if '4 GREEN' in color_dict:
        colorSort[3] = color_dict['4 GREEN'][0][1]
        sortPos[3] = color_dict['4 GREEN'][0][0]
    if '5 BLUE' in color_dict:
        colorSort[4] = color_dict['5 BLUE'][0][1]
        sortPos[4] = color_dict['5 BLUE'][0][0]

for i in range(numBoxes):
    endEffectorHeight.append([sortPos[i][0], sortPos[i][1], sortPos[i][2]+.27])
    endEffectorHeightGrab.append([sortPos[i][0], sortPos[i][1], sortPos[i][2]+.23])
    sortingPosition.append([.4+.1*i, 0, 0.3])
    boxSortedPosition.append([.4+.1*i, 0, 0.05])

cubeStartOrientation = pb.getQuaternionFromEuler([0,0,0])
orn=[0.0, 0.0, 1, 0.0]
flags = pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
for n in colorSort:
    boxes.append(pb.loadURDF("cube.urdf", sortPos[k], cubeStartOrientation, globalScaling = .1, flags = flags))
    pb.changeVisualShape(boxes[k], -1, rgbaColor=n)
    k += 1
    
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
                                           
time.sleep(5)
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
    print(targetPositionsJoints)
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
    print(targetPositionsJoints)
    for _ in range(movetime): #move to positions hovering over the blocks
            pb.stepSimulation()
            pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -.01 ,force= 10)
            pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, .01 ,force= 10)
            pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, .5 ,force= 10)
            pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, -.5 ,force= 10)
            pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
            time.sleep(1./10.)
    targetPositionsJoints = pb.calculateInverseKinematics(robot, numJoints - 3, sortingPosition[i], targetOrientation = Orientation)
    pb.setJointMotorControlArray(robot, range(numJoints-3), pb.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    for _ in range(movetime): #move to "sorted" position
        pb.stepSimulation()
        pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -.01 ,force= 100)
        pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, .01 ,force= 100)
        pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, .5 ,force= 100)
        pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, -.5 ,force= 100)
        pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
        time.sleep(1./10.)
    pb.removeBody(boxes[i])
    sortBoxes.append(pb.loadURDF("cube.urdf", boxSortedPosition[i], cubeStartOrientation, globalScaling = .1, flags = flags))
    pb.changeVisualShape(boxes[i], -1, rgbaColor=colorSort[i])
    #make new blocks in the "sorted" positions
    targetPositionsJoints = pb.calculateInverseKinematics(robot, numJoints - 3, endEffectorHeight[i], targetOrientation = Orientation)
    pb.setJointMotorControlArray(robot, range(numJoints-3), pb.POSITION_CONTROL, targetPositions = targetPositionsJoints)
    pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
    for _ in range(movetime): #move to positions hovering over the blocks
            pb.stepSimulation()
            pb.setJointMotorControl2(robot, 8, pb.POSITION_CONTROL, -.01 ,force= 10)
            pb.setJointMotorControl2(robot, 11, pb.POSITION_CONTROL, .01 ,force= 10)
            pb.setJointMotorControl2(robot, 10, pb.POSITION_CONTROL, .5 ,force= 10)
            pb.setJointMotorControl2(robot, 13, pb.POSITION_CONTROL, -.5 ,force= 10)
            pb.setJointMotorControlMultiDof(robot, 7, pb.POSITION_CONTROL)
            time.sleep(1./10.)
pb.disconnect()



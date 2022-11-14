# -*- coding: utf-8 -*-
"""
Created on Sat Nov 12 17:24:56 2022

@author: Aditya
"""

import cv2
import numpy as np
import time
from realsense_camera import *

class Detector:
    def __init__(self,videoPath,configPath,modelPath, classesPath):
      self.videoPath = videoPath
      self.configPath = configPath
      self.modelPath = modelPath
      self.classesPath = classesPath
      
      # self.readClasses()
      
      
     ################################
     
      self.net = cv2.dnn_DetectionModel(self.modelPath, self.configPath)
      self.net.setInputSize(320,320)
      self.net.setInputScale(1.0/127.5)
      self.net.setInputMean((127.5,127.5,127.5))
      self.net.setInputSwapRB(True)
      
      self.readClasses()
      
      
    def readClasses(self):
         with open(self.classesPath, 'r') as f:
             self.classesList = f.read().splitlines()
             
         print(self.classesList)
     
     
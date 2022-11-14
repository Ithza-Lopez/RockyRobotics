# -*- coding: utf-8 -*-
"""
Created on Sat Nov 12 17:40:04 2022

@author: Aditya
"""

from Detector_test import *
import os
from realsense_camera import *

rs = RealsenseCamera()

def main():
    print("here")
    videoPath = rs.get_frame_stream()
    
    configPath = os.path.join("model_data", "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt")
    modelPath = os.path.join("model_data", "frozen_inference_graph.pb")
    classesPath = os.path.join("model_data", "coco.names")
    
    Detector(videoPath, configPath, modelPath, classesPath)
    # detector.onVideo()
    
if __name__ == '__main__':
    main()    
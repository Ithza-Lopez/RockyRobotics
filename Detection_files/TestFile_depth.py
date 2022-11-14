# -*- coding: utf-8 -*-

import cv2
from realsense_camera import *
from mask_rcnn import *

rs = RealsenseCamera()
mrcnn = MaskRCNN()

while True:
    ret,bgr_frame, depth_frame = rs.get_frame_stream()
    
    boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)
    
    bgr_frame = mrcnn.draw_object_mask(bgr_frame)
    
    mrcnn.draw_object_info(bgr_frame, depth_frame)
    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Bgr frame", bgr_frame)
    
    key = cv2.waitKey(1)
    if key == 27:
        break
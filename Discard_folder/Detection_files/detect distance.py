import cv2
import pyrealsense2
from realsense_depth import *
import sys
from pynput.mouse import Listener
import logging
import win32api 
import time 

point = (400, 300)

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()

# Create mouse event
cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)



 
state_left = win32api.GetKeyState(0x01)  # Left button down = 0 or 1. Button up = -127 or -128 
state_right = win32api.GetKeyState(0x02)  # Right button down = 0 or 1. Button up = -127 or -128 
distance_vals = []
boolean_val = False
# while True:  
#     b = win32api.GetKeyState(0x02) 
    
#     if b != state_right:  # Button state changed 
#         state_right = b 
#         print(b) 
#         if b < 0: 
#             print('Right Button Pressed') 
#         else: 
#             print('Right Button Released') 
#     time.sleep(0.001)

while True:
    ret, depth_frame, color_frame = dc.get_frame()

    # Show distance for a specific point
    cv2.circle(color_frame, point, 4, (0, 0, 255))
    distance = depth_frame[point[1], point[0]]

    cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Color frame", color_frame)
    
  
    b = win32api.GetKeyState(0x02) 
    
    if b != state_right:  # Button state changed 
        state_right = b 
        if b < 0: 
            print("this is the registered distance: ", distance)
            distance_vals.append(distance)
            print("this is the distance array length: ",len(distance_vals))
        else: 
            continue
    time.sleep(0.001)
    
    key = cv2.waitKey(1)
    if key == 27:
        break
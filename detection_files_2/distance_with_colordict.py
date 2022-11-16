import cv2
import pyrealsense2
from realsense_depth import *
import sys
# from pynput.mouse import Listener
# import logging
import win32api 
import time 
from detect_distance import *

point = (400, 300)

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)
    
def ColorDetection(frame, point):
    #we want to convert frame to hsv for color
    
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #changing to 
    center_x, center_y = point
    pixel_center = hsv_frame[center_y,center_x] 
    
    hue_value = pixel_center[0]#hue value is color value
    saturation_value = pixel_center[1]
    value_value = pixel_center [2]
    
    pixel_center_bgr = frame[center_y, center_x]
    b,g,r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
    color_rgb = [round(r/255,3), round(g/255,3), round(b/255,3), 1]
    #selecting color ranges
    
    if saturation_value < 30 or saturation_value <100 or value_value <100:
        color_name = 'uncategorized'
        
        
    elif hue_value < 7:
        color_name = '1 RED'
        # color_index = '1'
    
    elif hue_value < 22:
        color_name = '2 ORANGE'
    
    elif hue_value < 32:
        color_name = '3 YELLOW'
        
    elif hue_value < 90:
        color_name = '4 GREEN' 
        
    elif hue_value < 131:
        color_name = '5 BLUE' 
    
    # elif hue_value < 146:
    #     color_name = '6PURPLE' 
    
    else:
        color_name = '1 RED'
        
    
        
    return color_name, color_rgb 

# Initialize Camera Intel Realsense
dc = DepthCamera()

# Create mouse event
cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

state_left = win32api.GetKeyState(0x01)  # Left button down = 0 or 1. Button up = -127 or -128 
state_right = win32api.GetKeyState(0x02)  # Right button down = 0 or 1. Button up = -127 or -128 
distance_vals = []
color_vals = []
boolean_val = False

color_dict = {'1 RED':[], '2 ORANGE':[],'3 YELLOW':[], '4 GREEN':[], '5 BLUE':[], 'uncategorized': []}

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
            # print("this is the registered distance: ", distance)
            distance_vals.append(distance)
            color_name, color_rgb = ColorDetection(color_frame,point)
            color_dict[color_name].append([xyz_array, color_rgb])
            # print("this is the registered color: ", color_name)
            # print("this is the distance array length: ",len(distance_vals))
            print('\n this is the color dict: ', color_dict)
        else: 
            continue
    time.sleep(0.001)
    
    key = cv2.waitKey(1)
    if key == 27:
        break

dc.release() 
cv2.destroyAllWindows() #closes all windows


   
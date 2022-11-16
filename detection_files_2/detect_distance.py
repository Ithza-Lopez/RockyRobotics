import cv2
import pyrealsense2
from realsense_depth import *
import sys
# from pynput.mouse import Listener
# import logging
import win32api 
import time 
from math import sqrt
from distance_with_colordict import *
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
    #selecting color ranges
    
    if saturation_value < 30:
        color = 'WHITE'
        
    elif saturation_value <100:
        color = 'too pale'
        
    elif value_value <100:
        color = 'too dark'
        
    elif hue_value < 7:
        color = 'RED'
        
    elif hue_value < 22:
        color = 'ORANGE'
    
    elif hue_value < 32:
        color = 'YELLOW'
        
    elif hue_value < 90:
        color = 'GREEN'   
        
    elif hue_value < 131:
        color = 'BLUE' 
    
    elif hue_value < 146:
        color = 'PURPLE' 
        
    elif hue_value < 170:
        color = 'PINK'
    
    else:
        color = 'RED'
        
    return color 

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
global counter_click 
counter_click = 0

def xyz_values(z_val,L_val):
    y_val = sqrt((L_val**2)-(z_val**2))
    temp_xyz = [0,y_val,z_val]
    xyz_array = []
    xyz_array.append(temp_xyz)
    
    if counter_click>0:
        l_val = sqrt((L_val**2)-(z_val**2))
        x_val = sqrt((l_val**2)-(y_val**2))
        temp_xyz = [x_val,y_val,z_val]
        xyz_array.append(temp_xyz)
        
    print(xyz_array)
    return xyz_array
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
            color = ColorDetection(color_frame,point)
            color_vals.append(color)
            print("this is the registered color: ", color)
            print("this is the distance array length: ",len(distance_vals))
            counter_click+=1
            
            
        else: 
            continue
    time.sleep(0.001)
    
    key = cv2.waitKey(1)
    if key == 27:
        break

dc.release() 
cv2.destroyAllWindows() #closes all windows
print(distance_vals)
print(color_vals)
# def get_xyz(z_val,L_list):
#     y_val = sqrt((L_list[0]**2)-(z_val**2))
#     temp_xyz = [0,y_val,z_val,color_vals[0]]
#     xyz_array = []
#     xyz_array.append(temp_xyz)
    
#     for i in range (len(L_list)-1):
#         l_val = sqrt((L_list[i+1]**2)-(z_val**2))
#         x_val = sqrt((l_val**2)-(y_val**2))
#         temp_xyz = [x_val,y_val,z_val,color_vals[i+1]]
#         xyz_array.append(temp_xyz)
        
#     print(xyz_array)
#     return xyz_array

        
print(color_dict)
#get_xyz(165,distance_vals)
#xyz_values(165,)

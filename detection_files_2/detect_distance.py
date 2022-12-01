import cv2
import pyrealsense2
from realsense_depth import * # import file functions from realsense_depth
import sys
# from pynput.mouse import Listener
# import logging
import win32api 
import time 
from math import sqrt
from distance_with_colordict import *
point = (400, 300)
z_val = 160
y_val =[]
global y_valtemp

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)
    
def ColorDetection(frame, point):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #changing to 
    center_x, center_y = point
    pixel_center = hsv_frame[center_y,center_x] 
    
    hue_value = pixel_center[0]#hue value is color value
    saturation_value = pixel_center[1]
    value_value = pixel_center [2]
    
    pixel_center_bgr = frame[center_y, center_x]
    b,g,r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
    color_rgb = [round(r/255,3), round(g/255,3), round(b/255,3), 1]
    
    #if conditions below use the hue_value to determine color
    
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

    else:
        color_name = '1 RED'

    return color_name, color_rgb 


def xyz_values(z_val,L_val,counter_click):
    if counter_click == 0:
        y_val = sqrt((L_val**2)-(z_val**2))
        y_valtemp = y_val
        x_val = 0
        xyz = [x_val, y_val, z_val]
        
    # print('counter click under 1')
    y_val = y_valtemp
    if counter_click > 0:
        # print('counter click over 1')
        # y_val = y_valtemp
        l_val = sqrt((L_val**2)-(z_val**2))
        print('this is l', l_val)
        x_val = sqrt((l_val**2)-(y_val**2))
        print('this is x', x_val)
        xyz = [x_val, y_val, z_val]
        # xyz_array.append(temp_xyz)
    # counter_click += 1    
    # print(xyz_array)
    # counter_click =+1
    return xyz
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
            # distance_vals.append(distance)
            xyz = xyz_values(z_val, distance, counter_click)
            color_name, color_rgb = ColorDetection(color_frame,point)
            color_dict[color_name].append([xyz, color_rgb])
            # print(xyz, '\n')
            counter_click += 1
            # print('this is counter click', counter_click)
            # print("this is the registered color: ", color)
            print('\n this is the color dict: ', color_dict)
            # print("this is the distance array length: ",len(distance_vals))
            
            
            
        else: 
            continue
    time.sleep(0.001)
    
    key = cv2.waitKey(1)
    if key == 27:
        break

dc.release() 
cv2.destroyAllWindows() #closes all windows

# -*- coding: utf-8 -*-
"""
Created on Sun Nov  6 12:38:26 2022

@author: ithza

Tutorial from https://www.youtube.com/watch?v=t71sQ6WY7L4&ab_channel=Pysource
for simple color recognition
"""

import cv2

cap = cv2.VideoCapture(1) #webcam is 0, 1 for camera(if connected)

#increase resolution of frame
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True: 
    _, frame = cap.read() #takes frame
    
    #we want to convert frame to hsv for color
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #changing to 
    height,width,_ = frame.shape#gets the center point
   
    
    
    
    center_x = int(width/2) #getting the center point, cv only takes integers
    center_y = int(height/2)
    
    #pick pixel value
    pixel_center = hsv_frame[center_y,center_x] 
    
    hue_value = pixel_center[0]#hue value is color value
    saturation_value = pixel_center[1]
    #selecting color ranges
    if saturation_value < 30:
        color = 'WHITE'
        
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
    
    
    
    print(pixel_center) 
    pixel_center_bgr = frame[center_y, center_x]
    b,g,r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
    cv2.putText(frame, color, (10,70), 0, 1.5, (b,g,r), 2)#where, what, location, font, size of text, 
    cv2.circle(frame, (center_x,center_y), 5, (25,25,25), 3) #(255,0,0) is blue
    
    
    cv2.imshow("Frame", frame) #shows frame
    key = cv2.waitKey(1)#if zero it freezes the key
    if key == 27: #the esc key
        break
    
cap.release() 
cv2.destroyAllWindows() #closes all windows


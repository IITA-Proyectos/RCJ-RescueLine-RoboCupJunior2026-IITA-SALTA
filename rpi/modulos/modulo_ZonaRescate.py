"___________________________________________________________________________________________________________________________________________________________________________"
### 1.IMPORT ###
"___________________________________________________________________________________________________________________________________________________________________________"

import cv2
from camthreader import *
import numpy as np
import math
import serial
import time

"___________________________________________________________________________________________________________________________________________________________________________"
### 2.DEBUG FLAGS ###
"___________________________________________________________________________________________________________________________________________________________________________"
debugOriginal = False       # original frame
debugBlack = False          # black mask
debugGreen = False          # green mask
debugBlue = False           # blue mask
debugSliced = False         # sliced black mask to re-acquire line after obstacle
debugLinegapMask = False    # linegap mask (triangle)
debugSilver = False         # sliced frame for evac entrance (should be a silver strip)
debugSlicedGreen = False
"___________________________________________________________________________________________________________________________________________________________________________"
### 3.THRESHOLDS ###
"___________________________________________________________________________________________________________________________________________________________________________"
lower_black = np.array([0, 0, 0])     # BGR
upper_black = np.array([100,105,105])
 
lower_black_evac = np.array([0, 0, 0])     # BGR
upper_black_evac = np.array([60, 60, 60])
 
lower_black_silver = np.array([0, 0, 0])     # BGR
upper_black_silver = np.array([80, 80, 80])
 
lower_green = np.array([70, 80, 80])  # HSV
upper_green = np.array([80, 255, 255])
 
lower_green_evac = np.array([70, 125, 80])  # HSV
upper_green_evac = np.array([80, 255, 255])
 
lower_blue = np.array([110, 80, 50]) # HSV
upper_blue = np.array([125, 255, 255])
 
lower_red = np.array([0, 100, 100]) # HSV
upper_red = np.array([20, 255, 255])
 
lower_purple = np.array([115, 40, 140])
upper_purple = np.array([137, 255, 255])
 
lower_orange = np.array([5, 60, 70])
upper_orange = np.array([25, 255, 255])
 
lower_marker = np.array([115, 40, 140]) # PURPLE THIS TIME
upper_marker = np.array([137, 255, 255])
 
min_square_size = 20    # for filtering green squares # old: 361
min_cube_size = 100     # for filtering blue cube
"___________________________________________________________________________________________________________________________________________________________________________"
### 4.GET FRAME DIMENSIONS ###
"___________________________________________________________________________________________________________________________________________________________________________"
vs = WebcamVideoStream(src = 0).start()
ser = serial.Serial('/dev/serial0', 115200, timeout = 0)
 
test_frame = vs.read()
width, height = test_frame.shape[1], test_frame.shape[0]
print(width, height)
"___________________________________________________________________________________________________________________________________________________________________________"
### 5.CONSTANTS ###
"___________________________________________________________________________________________________________________________________________________________________________"
x_com = np.zeros(shape = (height, width))
y_com = np.zeros(shape = (height, width))
 
cam_x = width / 2 - 1   # 79 ~ middle coloumn
cam_y = height - 1      # 119 ~ bottom row
 
blue_state = False
 
for i in range(height):
    for j in range(width):
        x_com[i][j] = (j - cam_x) / (width / 2)   # [-1, 1]
        y_com[i][j] = (cam_y - i) / height        # [0, 1]
 
kernel = np.ones((5, 5), np.uint8)  # for denoising line mask
ball_kernel = np.ones((3, 3), np.uint8) # for denoising ball mask
 
x_scaling_factor = ((1 - y_com) ** 0.1) # for scaling x-values so lower pixels have higher weightage
 
linegap_mask = np.zeros(shape = (height, width), dtype = np.uint8)
pts = np.array([[-80, 0], [239, 0], [cam_x, cam_y]], np.int32)  # TUNE NEXT TIME
pts = pts.reshape((-1, 1, 2))
cv2.fillPoly(linegap_mask, [pts], 255)  # triangle where stuff at the side are masked out
"___________________________________________________________________________________________________________________________________________________________________________"
### 6.VARIABLES ###
"___________________________________________________________________________________________________________________________________________________________________________"
counter = 0             # for ignoring double green squares
prev_angle = 0          # for storing forced turn angle
greenSquare = False     # True if green square is detected
forcedTurnCounter = 0   # not used: for 135-turn and pacman forced turns
deposited = False       # if true, exit evac zone
lastreset = 0
blueCube = False
cx_black = width / 2    # for x-value of black centroid (green squares)
cccounter = 0

while True:
    while True:
        frame = vs.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        speed = 50  # default 30 for linetrack, varied for cube centering
        angle = 90   # fixed during green square and blue cube centering, varied for linetrack
        task = 5
        data = ser.read()
        if data == b'\xff': # switch is off
            "___________________________________________________________________________________________________________________________________________________________________________"
            ### 12.RESET ALL FLAGS ###
            "___________________________________________________________________________________________________________________________________________________________________________"
            counter = 0
            prev_angle = 0
            greenSquare = False
            deposited = False
            blueCube = False
            break
 
        evac_mask = cv2.inRange(frame, lower_black_evac, upper_black_evac)
        evac_mask = cv2.erode(evac_mask, kernel)
        evac_mask = cv2.dilate(evac_mask, kernel)
 
        evac_contours, heirarchy = cv2.findContours(evac_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        max_ratio = 0
 
        if len(evac_contours):
            max_index = np.argmax([cv2.contourArea(cnt) for cnt in evac_contours])
            max_evac_contour = evac_contours[max_index]
            max_area = cv2.contourArea(max_evac_contour)
            print('Area:', max_area)
 
            x,y,w,h = cv2.boundingRect(max_evac_contour)
            max_ratio = float(w)/h
 
        x_evac = 0
       
        if np.sum(evac_mask):
            x_evac = np.average(x_com, weights = evac_mask)
 
        if max_area < 1200 or max_area > 4000 or max_ratio < 6:
            angle = 90
            speed = 50
 
        else:
            if abs(x_evac) < 0.015 and np.sum(evac_mask) / 255.0 > 50 and (not deposited):
                task = 10
                deposited = True
           
            elif x_evac < 0:
                angle = 90
                speed = abs(x_evac) * 50
 
            else:
                angle = -90
                speed = abs(x_evac) * 50
       
        print('Red Sum:', np.sum(cv2.inRange(hsv_frame, lower_red, upper_red)) / 255.0)
        if np.sum(cv2.inRange(hsv_frame, lower_red, upper_red)) / 255.0 > 50 and deposited:
             # print('Red')
             speed = 0
             angle = 0
             task = 11
        "___________________________________________________________________________________________________________________________________________________________________________"
         ### 13.BLACK BALL DETECTION ###
        "___________________________________________________________________________________________________________________________________________________________________________"
        if np.sum(evac_mask):
            black_contours, hierarchy = cv2.findContours(evac_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # extract outermost contours
            black_contours = [c for c in black_contours if cv2.contourArea(c) > 100] # remove noise
            black_contours = sorted(black_contours, key = lambda x: cv2.contourArea(x), reverse = True)
 
            cx_ball = None
            cy_ball = None
 
            if len(black_contours) > 0:
                for black_contour in black_contours:
                    black_circle_mask = np.zeros(shape = (height,width), dtype = np.uint8)
                    black_circled_mask = np.zeros(shape = (height,width), dtype = np.uint8)
 
                    black_square_mask = np.zeros(shape = (height,width), dtype = np.uint8)
                    black_squared_mask = np.zeros(shape = (height,width), dtype = np.uint8)
 
                    M = cv2.moments(black_contour)
                    cx_contour = int(M['m10'] / M['m00'])
                    cy_contour = int(M["m01"] / M["m00"])
 
                    x,y,w,h = cv2.boundingRect(black_contour)
                    ratio = float(w)/h

                    radius = min(w,h)
                    black_circle_mask = cv2.circle(black_circle_mask, (cx_contour, cy_contour), radius, 255, -1)
                    black_square_mask = cv2.rectangle(black_square_mask, ((cx_contour - radius), (cy_contour - radius)), ((cx_contour + radius), (cy_contour + radius)), 255, -1)
                   
                    black_circled_mask = cv2.bitwise_and(evac_mask, evac_mask, mask = black_circle_mask)
                    black_square_mask = black_square_mask - black_circle_mask
                    black_squared_mask = cv2.bitwise_and(evac_mask, evac_mask, mask = black_square_mask)
 
                    percentage_background_black = np.sum(black_squared_mask) / np.sum(black_square_mask) if np.sum(black_square_mask) else 0
                    percentage_ball = np.sum(black_circled_mask) / np.sum(black_circle_mask) if np.sum(black_circle_mask) else 0
                   
                    print('Percentage Background Black:', percentage_background_black,'Percentage Ball:', percentage_ball)
                    # percentage ball threshold: 10%
                    print('ratio', ratio)
                    if percentage_background_black < 0.03 and percentage_ball > 0.3 and ratio > 0.95 and ratio < 1.2:
                        print('Black Ball Detected, centering')
                        cx_ball = cx_contour
                        cy_ball = cy_contour
                        break
 
            if cx_ball is not None:  
                task = 40      
                if cx_ball < 80:  # turn left
                    angle = 90  # max steer rate, turn on the spot
                else: # turn right
                    angle = -90
               
                speed = min(abs((cx_ball - 80) / 80) * 20, 20) # speed based on error (x-vectors)
                # print('Speed:', speed)
                print('cy_ba;;', cy_ball, cx_ball)
                if cx_ball > 70 and cx_ball < 90:  # cube is roughly centered
                    if cy_ball > 21:   # if ball is nearby, turn to center on it
                        print('Picking up Black Ball')
                        task = 50
 
                    else:
                        speed = 30
                        angle = 0
       
        print("Speed:", speed, "Angle:", angle, "Task:", task)
 
        black_mask = cv2.inRange(frame, lower_black_silver, upper_black_silver)
        "___________________________________________________________________________________________________________________________________________________________________________"
        ## 14.SILVER BALL DETECTION ###
        "___________________________________________________________________________________________________________________________________________________________________________"
        eroded_black_mask = cv2.erode(black_mask, ball_kernel)
        dilated_black_mask = cv2.dilate(eroded_black_mask, ball_kernel)
        subtracted_mask = black_mask - dilated_black_mask
        subtracted_mask[100:, :] = 0
       
        circle_mask = np.zeros(shape = (height,width), dtype = np.uint8)
        circled_mask = np.zeros(shape = (height,width), dtype = np.uint8)

        if (np.sum(subtracted_mask)) and (np.sum(black_mask)/255) > 40:
            M = cv2.moments(subtracted_mask)
 
            cx_ball = int(M['m10'] / M['m00'])
            cy_ball = int(M["m01"] / M["m00"])
           
            circle_mask = cv2.circle(circle_mask, (cx_ball, cy_ball), min(cy_ball + 5, 60), 255, -1)
            circled_mask = cv2.bitwise_and(subtracted_mask, subtracted_mask, mask = circle_mask)
       
            percentage_white = np.sum(circled_mask) / np.sum(circle_mask) if np.sum(circle_mask) else 0
            magicno = (percentage_white * (cy_ball) * 100)
            print(magicno)
            if magicno > 80:  # original: 0.01
                print('Silver Ball detected, centering')
               
                if cx_ball < 80:  # turn left
                    angle = 90  # max steer rate, turn on the spot
                else: # turn right
                    angle = -90
               
                task = 40
                speed = min(abs((cx_ball - 80) / 80) * 20, 20) # speed based on error (x-vectors)
                # print('Speed:', speed)
                if cx_ball > 70 and cx_ball < 90:  # cube is roughly centered
                    print('cy_ball:                                                            ', cy_ball)
                    if cy_ball > 25:   # if ball is nearby, turn to center on it
                        print('Picking up Silver Ball')
                        task = 60
                    else:
                        speed = 30
                        angle = 0
 
        output = [255, round(speed),
                254, round(angle) + 90,
                253, task,
                252, 0]
        ser.write(output)
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if debugOriginal:
            cv2.imshow('Original', frame)
        if debugBlack:
            cv2.imshow('Black Mask', black_mask)
cv2.destroyAllWindows()
vs.stop()
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
debugOriginal = True        # original frame
debugBlack = True          # black mask
debugGreen = True          # green mask
debugBlue = False           # blue mask
debugSliced = False         # sliced black mask to re-acquire line after obstacle
debugLinegapMask = False    # linegap mask (triangle)
debugSilver = False         # sliced frame for evac entrance (should be a silver strip)
debugSlicedGreen = False
"___________________________________________________________________________________________________________________________________________________________________________"
### 3.THRESHOLDS ###
"___________________________________________________________________________________________________________________________________________________________________________"
lower_black = np.array([0, 0, 0])     # BGR
upper_black = np.array([80,80,80])
 
lower_black_silver = np.array([0, 0, 0])     # BGR
upper_black_silver = np.array([100, 100, 100])
 
lower_green = np.array([90, 90, 95])  # HSV
upper_green = np.array([110, 140, 170])
 
 
lower_red = np.array([0, 100, 100]) # HSV
upper_red = np.array([20, 255, 255])
 
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
cx_black = width / 2    # for x-value of black centroid (green squares)
cccounter = 0

while True:
    ### FIRST LINE TRACK ###
    while True:
        data = ser.read()
        if cccounter > 0:
            cccounter -= 1
            data = b'\xff'
        if data == b'\xff': # switch is off
            if ser.in_waiting > 500:
                ser.reset_input_buffer()
                cccounter = 50
            counter = 0
            prev_angle = 0
            greenSquare = False
            deposited = False
            blueCube = False
 
        frame = vs.read()
        frame[:25, :, :] = 255  # block out horizon
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        "___________________________________________________________________________________________________________________________________________________________________________"
        ### 7.DATA SENT TO TEENSY ###
        "___________________________________________________________________________________________________________________________________________________________________________"
        speed = 30  # default 30 for linetrack, varied for cube centering
        angle = 0   # fixed during green square and blue cube centering, varied for linetrack
        task = 0    # 0 = no green (default), 1 = left green, 2 = right green, 3 = double green (reverse), 4 = pick up cube, 5 = wall track, 6 = move straight to enter evac zone, 7 = exit evac zone  

        ### DETECT BLACK ###
        black_mask = cv2.inRange(frame, lower_black, upper_black)
        black_mask = cv2.erode(black_mask, kernel)  # remove noisy pixels
        black_mask = cv2.dilate(black_mask, kernel)

        ### DETECT RED LINE ###
        red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
        #if np.mean(red_mask) > 50:
            # print('Red')
        black_mask = cv2.bitwise_or(black_mask, red_mask)
 
        black_mask_linegap = cv2.bitwise_and(black_mask, black_mask, mask = linegap_mask)   # mask outside pixels
        y_black_uncropped = cv2.bitwise_and(y_com, y_com, mask = black_mask_linegap)
        x_black_uncropped = cv2.bitwise_and(x_com, x_com, mask = black_mask_linegap) * x_scaling_factor
 
        ### EVAC ENTRANCE DETECTION ###
        summed_pixels = list(np.amax(black_mask, axis = 1)) # if row has black line: 1, else 0
 
        ### IGNORE LINES ABOVE/ON OTHER TILES ###
        summed_pixels.reverse()
        lineIndex = summed_pixels.index(max(summed_pixels)) # get index of lowest line
        gapIndex = summed_pixels.index(0, lineIndex)        # get index of lowest white gap above line
       
        # print('Line Index:', height - lineIndex)
        # print('Gap Index:', height - gapIndex)
        black_mask[:height - gapIndex, :] = 0   # mask everything above lowest line starting from white gap
        "___________________________________________________________________________________________________________________________________________________________________________"
        ## 8.EVAC ENTRANCE ###
        "___________________________________________________________________________________________________________________________________________________________________________"
        masked_summed_pixels = list(np.amax(black_mask, axis = 1)) # if row has black line: 1, else 0
        furthestLineIndex = masked_summed_pixels.index(max(masked_summed_pixels))   # find index of furthest black line
        print('Furthest Line Index:', furthestLineIndex)
        if furthestLineIndex > 75:  # last 1/5 of frame, line is ending
            slicedFrame = frame[int(0.8 * furthestLineIndex):furthestLineIndex + 1, :]
            std = np.std(slicedFrame) / np.mean(slicedFrame)
            print('Normalized Standard Deviation:', std)
 
            # cv2.imshow('Silver Sliced Frame', slicedFrame)
 
            # if std > 0.28 and np.mean(cv2.inRange(slicedFrame, lower_red, upper_red)) < 10 and np.mean(cv2.inRange(slicedFrame, lower_green, upper_green)) < 10:   # do roomba
            if std > 0.3 and std < 0.9 and data != b'\xff' and (not blue_state) and (not greenSquare):   # can loosen, and check if angle is close to 0
            #if std > 0.4:
                print('linea plateada')
                output = [255, round(speed),
                254, round(angle) + 90,
                253, 6,
                252, 0]
                ser.write(output)


        ### 135-TURNS & PACMAN (DISABLED FOR NOW) ###
        y_black = cv2.bitwise_and(y_com, y_com, mask = black_mask)
        x_black = cv2.bitwise_and(x_com, x_com, mask = black_mask) # weigh bottom pixels more
        x_max = np.max(x_black) # for detecting line gap later
        x_min = np.min(x_black) # have to be gotten first before top black stuff are masked out
        x_black = x_black * x_scaling_factor
        ### NEW GREEN SQUARES ###
        green_mask = cv2.inRange(hsv_frame[90:, :, :], lower_green, upper_green)    # shape = (30, 160)

        if np.sum(green_mask) > min_square_size * 255:
            green_pixels = np.amax(green_mask, axis = 0)    # for every column, if green: 1 else 0, shape = (160,)
           
            greenIndices = np.where(green_pixels == np.max(green_pixels))   # get indices of columns that have green
            leftIndex = greenIndices[0][0]
            rightIndex = greenIndices[0][-1]
            slicedGreen = frame[60:90, leftIndex:rightIndex + 1, :]
 
            greenCentroidX = (rightIndex + leftIndex) / 2
            slicedBlackMaskAboveGreen = black_mask[60:90, leftIndex:rightIndex + 1]
            blackM = cv2.moments(black_mask[90:, :])
 
            if np.sum(black_mask[90:, :]):  # if there is black, prevents divide by 0 error
                cx_black = int(blackM["m10"] / blackM["m00"])   # get x-value of black centroid
 
            if (np.sum(slicedBlackMaskAboveGreen) / (255 * 30 * (rightIndex - leftIndex))) > 0.32:   # if mean is high -> square before line
                greenSquare = False
                filtered_green_mask = cv2.erode(green_mask, kernel)
                filtered_green_mask = cv2.dilate(green_mask, kernel)
                green_contours, hierarchy = cv2.findContours(filtered_green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(green_contours) > 1 and cx_black > leftIndex and cx_black < rightIndex and counter == 0 and np.sum(green_mask) > (2 * min_square_size * 255):
                    task = 3
                    counter = 0
 
                elif greenCentroidX < cx_black:

                    counter = 200

                    angle = 46
 
                else:

                    counter = 200

                    angle = -46
 
            else:
                greenSquare = False
 
        else:
            greenSquare = False
 

        if counter > 0:
            counter -= 1

        "_______________________________________________________________________________________________________________________________________________________________________"
        ### 10.DETECT LINE AFTER OBSTACLE ###
        "___________________________________________________________________________________________________________________________________________________________________________"
        sliced_black_mask = black_mask[75:95, :]
        line_middle = round(np.mean(sliced_black_mask))
 
        ### SCALE ANGLE ###
        if counter == 0:    # don't power for green squares
            if not ((np.max(y_black) < 0.5) and (x_max - x_min) < 1.2): # only power if there's no line gap
                # print('Poweringgg') # the shorter the line, the greater the power (fractional root of decimal gives larger decimal)
                angle = ((abs(angle) / 90.0) ** (np.max(y_black) * (height/(height - 25)))) * 90.0 * (-1 if angle < 0 else 1)
        "___________________________________________________________________________________________________________________________________________________________________________"
        ### 11.SEND DATA TO TEENSY ###
        "___________________________________________________________________________________________________________________________________________________________________________"
        output = [255, round(speed),
                254, round(angle) + 90,
                253, task,
                252, line_middle]
        ser.write(output)
 
        ### DEBUGS ###
        print("Speed:", speed, "Angle:", angle, "Task:", task, "Line Middle:", line_middle)
       
        if debugOriginal:
            cv2.imshow('Original', frame)
        if debugBlack:
            cv2.imshow('Black Mask', black_mask)
        if debugGreen:
            cv2.imshow('Green Mask', green_mask)
        if debugSliced:
            cv2.imshow('Sliced Black Mask', sliced_black_mask)
        if debugLinegapMask:
            cv2.imshow('Linegap Mask', linegap_mask)
        if debugSlicedGreen:
            cv2.imshow('Sliced Green', slicedGreen)
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    """while True:
        print("detecte la linea plateada")
        print("si o no")
        if input("terminar?:")=="si":
            print("listo")
            break
        else:
            print("que triste lo termine igual XD")
            break"""
cv2.destroyAllWindows()
vs.stop()
import cv2
from camthreader import *
import numpy as np
import math
import serial
import time

### DEBUG FLAGS ###
debugOriginal = False       # original frame
debugBlack = False          # black mask
debugGreen = False          # green mask
debugBlue = False           # blue mask
debugSliced = False         # sliced black mask to re-acquire line after obstacle
debugLinegapMask = False    # linegap mask (triangle)
debugSilver = True         # sliced frame for evac entrance (should be a silver strip)
debugSlicedGreen = False

### THRESHOLDS ###
lower_black = np.array([0, 0, 0])     # BGR
upper_black = np.array([95, 95, 95])

lower_black_evac = np.array([0, 0, 0])     # BGR
upper_black_evac = np.array([20, 20, 20])

lower_green = np.array([70, 150, 100])  # HSV
upper_green = np.array([80, 255, 255])

lower_green_evac = np.array([70, 150, 80])  # HSV
upper_green_evac = np.array([80, 255, 255])

lower_blue = np.array([110, 80, 80]) # HSV
upper_blue = np.array([130, 255, 255])

lower_red = np.array([0, 100, 100]) # HSV
upper_red = np.array([20, 255, 255])
#lower_red = np.array([170, 70, 100])
#upper_red = np.array([180, 255, 255])

lower_purple = np.array([115, 40, 140])
upper_purple = np.array([137, 255, 255])

lower_orange = np.array([5, 60, 70])
upper_orange = np.array([25, 255, 255])

lower_marker = np.array([115, 40, 140]) # PURPLE THIS TIME
upper_marker = np.array([137, 255, 255])

min_square_size = 40    # for filtering green squares # old: 361
min_cube_size = 100     # for filtering blue cube

### GET FRAME DIMENSIONS ###
vs = WebcamVideoStream(src = 0).start()
#ser = serial.Serial('/dev/serial0', 57600)

test_frame = vs.read()
width, height = test_frame.shape[1], test_frame.shape[0]
print(width, height)

### CONSTANTS ###
x_com = np.zeros(shape = (height, width))
y_com = np.zeros(shape = (height, width))

cam_x = width / 2 - 1   # 79 ~ middle coloumn
cam_y = height - 1      # 119 ~ bottom row

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

### VARIABLES ###
counter = 0             # for ignoring double green squares
prev_angle = 0          # for storing forced turn angle

greenSquare = False     # True if green square is detected
forcedTurnCounter = 0   # not used: for 135-turn and pacman forced turns
evacCounter = 0         # for ignoring silver strip detection after exitting evac zone
deposited = False       # if true, exit evac zone

cx_black = width / 2    # for x-value of black centroid (green squares)

silverstripdetected = False
threshold = 1

### FIRST LINE TRACK ###
while not silverstripdetected:
    frame = vs.read()
    frame[:25, :, :] = 255  # block out horizon
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ### DETECT BLACK ###
    black_mask = cv2.inRange(frame, lower_black, upper_black)
    black_mask = cv2.erode(black_mask, kernel)  # remove noisy pixels
    black_mask = cv2.dilate(black_mask, kernel)

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

    ### EVAC ENTRANCE ###
    masked_summed_pixels = list(np.amax(black_mask, axis = 1)) # if row has black line: 1, else 0
    furthestLineIndex = masked_summed_pixels.index(max(masked_summed_pixels))   # find index of furthest black line
    if furthestLineIndex > 75:  # last 1/5 of frame, line is ending
        slicedFrame = frame[int(0.8 * furthestLineIndex):furthestLineIndex + 1, :]
        std = np.std(slicedFrame) / np.mean(slicedFrame)
        cv2.imshow('Silver Sliced Frame', slicedFrame)
        if std < threshold:
          threshold -= 0.01
        else:
          silverstripdetected = True
        print(std)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break


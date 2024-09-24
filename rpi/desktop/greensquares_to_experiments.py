import cv2
from camthreader import *
import numpy as np
import math
import serial

debugOriginal = False
debugBlack = False
debugHori = False
debugGreen = False
debugBlue = False
record = False

vs = WebcamVideoStream(src=0).start()
ser = serial.Serial('/dev/serial0', 57600)

noise_blob_threshold = 16
blue_pickup_threshold = 50

lower_black = np.array([0, 0, 0])  # BGR
upper_black = np.array([95, 95, 95])
lower_green = np.array([45, 64, 64])  # HSV
upper_green = np.array([75, 255, 255])
lower_blue = np.array([0, 40, 40])  # HSV
upper_blue = np.array([10, 255, 255])

test_frame = vs.read()
width, height = test_frame.shape[1], test_frame.shape[0]
print(width, height)

cam_x = width / 2 - 1   # 79 ~ middle coloumn
cam_y = height - 1      # 119 ~ bottom row

# GET X AND Y ARRAYS
# scaled by frame dimensions, use this for filtering pixels
x_com = np.zeros(shape=(height, width))
y_com = np.zeros(shape=(height, width))
for i in range(height):
    for j in range(width):
        x_com[i][j] = (j - cam_x) / (width / 2)   # [-1, 1]
        y_com[i][j] = (cam_y - i) / height        # [0, 1]

while True:
    frame = vs.read()
    # frame = cv2.flip(frame, -1)     # comment out if testing on robot
    frame[:30, :, :] = 255          # ignore top 1/8 of frame
    kernel = np.ones((3, 3), np.uint8)

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # FILTER BLUE PIXELS
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    # blue_mask = cv2.dilate(blue_mask, kernel, iterations=1)
    # blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
    x_blue = cv2.bitwise_and(x_com, x_com, mask=blue_mask)
    y_blue = cv2.bitwise_and(y_com, y_com, mask=blue_mask)
    blue_contours, hierarchy = cv2.findContours(
        blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours_middle = [
        c for c in green_contours_middle if cv2.contourArea(c) > noise_blob_threshold]
    # sort by contour area, only use largest one
    if len(blue_contours) > 0:
        blue_contours = sorted(
            blue_contours, key=lambda x: cv2.contourArea(x), reverse=True)
        blue_mask = cv2.drawContours(
            blue_mask, [blue_contours[0]], -1, 255, -1)
        blue_blob_area = cv2.contourArea(blue_contours[0])
    else:
        blue_blob_area = 0

    # FILTER BLACK PIXELS
    black_mask = cv2.inRange(frame, lower_black, upper_black)
    x_black = cv2.bitwise_and(x_com, x_com, mask=black_mask)
    # scale such that bottom pixels have more weight
    x_black *= (1 - y_com)
    y_black = cv2.bitwise_and(y_com, y_com, mask=black_mask)

    # DETECT HORIZONTAL INTERSECTION LINES
    blurred = cv2.GaussianBlur(black_mask, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 200)
    lines = cv2.HoughLines(edges, rho=1, theta=np.pi / 180, threshold=30)

    minY = 1000  # arbitrarily high value

    bottom_mask = np.zeros(shape=(height, width), dtype=np.uint8)

    if lines is not None:
        lines = [[line[0][0], line[0][1]] for line in lines]
        # theta closer to 90deg will be placed first
        lines = sorted(lines, key=lambda x: abs((x[1]/np.pi*180) - 90))
        # print(lines)
        rho, theta = lines[0]
        degrees = theta / np.pi * 180
        if degrees > 45 and degrees < 135:   # detect horizontal line
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho

            # 200 = sqrt(height ** 2 + width ** 2)
            x1 = int(x0 + 200 * (-b))
            y1 = int(y0 + 200 * a)
            x2 = int(x0 - 200 * (-b))
            y2 = int(y0 - 200 * a)

            # cv2.line(frame, (x1, y1), (x2, y2),(0, 0, 255), 1)  # draw line
            # get lowest y-value of line in vector form

            if height - max(y1, y2) < minY:
                minY = min(minY, height - y1)
                minY = min(minY, height - y2)

            pts = np.array([[x1, y1], [x2, y2], [
                           160, 120], [0, 120]], np.int32)
            bottom_mask = cv2.fillPoly(bottom_mask, [pts], 255)  # draw polygon

    if minY == 1000:    # no line detected
        minY = 0

    # FILTER GREEN PIXELS
    hsv_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=bottom_mask)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    green_mask_middle = green_mask.copy()
    # version of green mask with only middle area shown, used for double green square
    green_mask_middle[80:, :] = 0
    green_mask_middle[:40, :] = 0
    green_mask_bottom = green_mask.copy()
    # version of green mask with only bottom area shown, used for single green square
    green_mask_bottom[:85, :] = 0

    # DETECT GREEN SQUARES
    # extract outermost contours (ignore inner holes)
    green_contours_middle, hierarchy = cv2.findContours(
        green_mask_middle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours_bottom, hierarchy = cv2.findContours(
        green_mask_bottom, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # remove noise by limiting blob size
    green_contours_middle = [
        c for c in green_contours_middle if cv2.contourArea(c) > noise_blob_threshold]
    green_contours_bottom = [
        c for c in green_contours_bottom if cv2.contourArea(c) > noise_blob_threshold]
    green_mask_bottom = cv2.drawContours(
        green_mask_bottom, green_contours_bottom, -1, 255, -1)
    x_green = cv2.bitwise_and(x_com, x_com, mask=green_mask_bottom)

    # CALCULATE RESULTANT
    speed = 20
    green_state = 0
    blue_pickup = (blue_blob_area >= blue_pickup_threshold)
    if len(blue_contours) > 0:
        x_resultant = np.mean(x_blue)
        y_resultant = np.mean(y_blue)
    else:
        x_resultant = np.mean(x_black)
        y_resultant = np.mean(y_black)

    angle = (math.atan2(y_resultant, x_resultant) / math.pi * 180) - 90

    if len(green_contours_middle) == 2:
        green_state = 3  # leave to Teensy to hardcode 180deg
    elif len(green_contours_bottom) == 1:
        if np.mean(x_green) < 0:  # turn left
            green_state = 1
            angle = 45
        else:
            green_state = 2  # turn right
            angle = -45

    # stop robot from turning if it sees nothing
    if y_resultant == 0:
        angle = 0

    output = [255, speed,
              254, round(angle) + 90,
              253, green_state]
    # 252, blue_pickup
    # 250, black_pickup
    # 249, silver_pickup
    # 248, evac_enter
    # 247, evac_steer
    # 246, evac_rotate
    # 245, evac_exit
    ser.write(output)

  # DEBUGS
    print("Angle: ", angle, "Green Squares: ", green_state)
    if debugOriginal:
        cv2.imshow('Original', frame)
    if debugBlack:
        cv2.imshow('Black Mask', black_mask)
    if debugHori:
        cv2.imshow('Hori Mask', bottom_mask)
    if debugGreen:
        cv2.imshow('Green Mask', green_mask)
    if debugBlue:
        cv2.imshow('Blue Mask', blue_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
vs.stop()

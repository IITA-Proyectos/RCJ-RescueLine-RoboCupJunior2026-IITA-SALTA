import cv2
from camthreader import *
import numpy as np
import math
import serial

debugOriginal = True
debugBlack = True
debugHori = False
debugGreen = True
debugBlue = True
record = False

noise_blob_threshold = 16

vs = WebcamVideoStream(src=0).start()
ser = serial.Serial('/dev/serial0', 115200)

lower_black = np.array([0, 0, 0])  # BGR
upper_black = np.array([70, 70, 70])
lower_green = np.array([0, 100, 80])  # HSV
upper_green = np.array([255, 120, 150])

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
cccounter=0
    
while True:
    """
    data = ser.read()
    if cccounter > 0:
        cccounter -= 1
        data = b'\xff'
    if data == b'\xff': # switch is off
        print("apagado")
        ser.reset_input_buffer()
        while True:
            data = ser.read()
            if data == b'\xff':
                ser.reset_input_buffer()
                break
    """
    frame = vs.read()
    # frame = cv2.flip(frame, -1)     # comment out if testing on robot
    frame[:50, :, :] = 255          # ignore top 1/8 of frame
    kernel = np.ones((3, 3), np.uint8)

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)


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
    # hsv_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=bottom_mask)
    green_mask = cv2.inRange(lab, lower_green, upper_green)
    cv2.imshow('Green Mask', green_mask)
    # green_mask_middle = green_mask.copy()
    # version of green mask with only middle area shown, used for double green square (maybe)
    #green_mask_bottom = green_mask.copy()
    # version of green mask with only bottom area shown, used for single green square
    green_mask[:85, :] = 0

    # DETECT GREEN SQUARES
    # extract outermost contours (ignore inner holes)
    green_contours, hierarchy = cv2.findContours(
        green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #green_contours_bottom, hierarchy = cv2.findContours(
    #    green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # remove noise by limiting blob size
    green_contours = [c for c in green_contours if cv2.contourArea(
        c) > noise_blob_threshold]
    #green_contours_bottom = [
    #    c for c in green_contours if cv2.contourArea(c) > noise_blob_threshold]
    green_mask = cv2.drawContours(green_mask, green_contours, -1, 255, -1)
    # green_mask_bottom = cv2.drawContours(
    #    green_mask_bottom, green_contours_bottom, -1, 255, -1)
    x_green = cv2.bitwise_and(x_com, x_com, mask=green_mask)

    # CALCULATE RESULTANT
    green_state = 0

    
    x_resultant = np.mean(x_black)
    y_resultant = np.mean(y_black)
    angle = (math.atan2(y_resultant, x_resultant) / math.pi * 180) - 90
    speed = 20
    if len(green_contours) == 1:
        if np.mean(x_green) < 0:  # turn left
            green_state = 1
            angle = 45
            speed=40
        else:
            green_state = 2  # turn right
            angle = -45
            speed=40
    if len(green_contours) == 2:
        green_state = 3

    # stop robot from turning if it sees nothing
    if y_resultant == 0:
        angle = 0
    contornos, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contornos, -1, (255, 0, 0), 2) 
    contornos_izquierda = 0
    contornos_derecha = 0
    for contorno in contornos:
        x, y, w, h = cv2.boundingRect(contorno)  
        if x + w/2 < cam_x:  
            contornos_izquierda += 1
        else: 
            contornos_derecha += 1
    if contornos_izquierda > 0 and contornos_derecha > 0:
        green_state_contorno = 3  
    elif contornos_izquierda > 0:
        green_state_contorno = 1  
    elif contornos_derecha > 0:
        green_state_contorno = 2  
    else:
        green_state_contorno = 0 
    green_state_final = 0
    if green_state_contorno==green_state:
        green_state_final=green_state_contorno
    """if round(angle) + 90 <20 or round(angle) + 90>160:
        speed=40"""

    output = [255, speed,
            254, round(angle) + 90,
            253, green_state_final]
    ser.write(output)
    print(output)
    # DEBUGS
    #print("Angle: ", angle, "Green Squares: ", green_state)
    if debugOriginal:
        cv2.imshow('Original', frame)
    if debugBlack:
        cv2.imshow('Black Mask', black_mask)
    #if debugHori:
    #    cv2.imshow('Hori Mask', bottom_mask)
    if debugGreen:
        cv2.imshow('Green Mask', green_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.imshow('Detección de Contornos Verdes', frame)
cv2.destroyAllWindows()
vs.stop()
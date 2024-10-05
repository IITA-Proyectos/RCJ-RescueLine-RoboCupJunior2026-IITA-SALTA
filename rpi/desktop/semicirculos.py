from threading import Thread
import cv2
import numpy as np
from camthreader import *
import serial


min_area = 150 
max_area = 3700


video_stream = WebcamVideoStream().start()
ser = serial.Serial('/dev/serial0', 115200)

while True:
    green_state = 0
    frame = video_stream.read()
    if frame is None:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])

    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    output = np.zeros_like(frame)
    output[mask_black==0] = [0, 0, 0]
    output[mask_black==255] = [255, 255, 255]
    detected = False

    contours, _ = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    detected = False
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * (area / (perimeter ** 2))
            if circularity > 0.6:
                detected = True
                green_state = 15 #means "non green, just a black ball"
                cv2.circle(output, center, radius, (0, 0, 255), 2)

    speed = 20 
    angle = 0

    outpuT = [255, speed,
            254, round(angle) + 90,
            253, green_state]
    ser.write(outpuT)
    cv2.imshow('Detected Ball', output)
    cv2.imshow('original', frame)

    if detected == True:
        print('Pelota detectada')

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_stream.stop()
cv2.destroyAllWindows()
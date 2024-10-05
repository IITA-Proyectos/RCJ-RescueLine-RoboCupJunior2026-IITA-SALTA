import cv2
from threading import Thread
import os
from camthreader import *
import numpy as np
import serial
### CAMTHEREADER ###

vs = WebcamVideoStream(src=0).start()
# ser = serial.Serial('/dev/serial0', 115200)
rgb_frame = vs.read()
hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)


def rgbclick(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(x,y,rgb_frame[y][x])


def hsvclick(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(x,y,hsv_frame[y][x])


def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        frame = vs.read()
        img_name = f"capture_{x}_{y}.png"
        img_path = os.path.join(img_name)
        cv2.imwrite(img_path, frame)

cv2.namedWindow('RGB')
cv2.setMouseCallback('RGB', rgbclick)
cv2.namedWindow('HSV')
cv2.setMouseCallback('HSV', hsvclick)
cv2.namedWindow('Para-hacer-click')
cv2.setMouseCallback('Para-hacer-click', click_event)
green_state = 0
angle = 0
speed = 20

while True:
    rgb_frame = vs.read()
    hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)
    rgb_frame = cv2.line(rgb_frame, (80, 0), (80, 120), (255, 0, 0), 1)
    hsv_frame = cv2.line(hsv_frame, (80, 0), (80, 120), (255, 0, 0), 1)
    cv2.imshow("RGB", rgb_frame)
    gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)


    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
        param1=140, param2=40, minRadius=12, maxRadius=40)

    if circles is not None:
        green_state = 5
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            # Dibujar un cuadrado alrededor de la pelota
            cv2.rectangle(rgb_frame, (x - r, y - r), (x + r, y + r), (0, 255, 0), 2)
            # Mostrar el mensaje "ball detected"
            cv2.putText(rgb_frame, "ball detected", (x - r, y - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            


    output = [255, speed,
        254, round(angle) + 90,
        253, green_state,
              5] 
    #ser.write(output)
    
            
    if cv2.waitKey(1) == 27:
        break  # esc to quit

vs.stop()
cv2.destroyAllWindows()


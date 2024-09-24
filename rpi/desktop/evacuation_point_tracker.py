import cv2
import numpy as np
from camthreader import *
# import serial


video_stream = WebcamVideoStream().start()
# ser = serial.Serial('/dev/serial0', 115200)


min_area = 150
max_area = 3700



frame_width = 640  
frame_center_x = frame_width // 2

while True:
    frame = video_stream.read()
    if frame is None:
        break

   
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])

    
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

   
    output = np.zeros_like(frame)
    output[mask_black == 0] = [0, 0, 0]
    output[mask_black == 255] = [255, 255, 255]

    
    contours, _ = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    alignment = 0  #
    for contour in contours:
        area = cv2.contourArea(contour)
        
        if min_area < area < max_area:
           
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

           
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])  

                # Ver si es un triángulo (tres vértices)
                if len(approx) == 3:
                    cv2.drawContours(output, [approx], 0, (0, 255, 0), 2)  
                    detected = True
                    shape = 'Triángulo'
                    green_state = 20
                
                
                elif len(approx) >= 4:
                    _, _, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if aspect_ratio > 4.0:  
                        cv2.drawContours(output, [approx], 0, (255, 0, 0), 2) 
                        detected = True
                        shape = 'Línea'
                        green_state = 20
                alignment = cX - frame_center_x

    speed = 20
    angle = alignment / frame_width * 100  


    """outpuT = [255, speed,
              254, round(angle) + 90,
              253, green_state]
    ser.write(outpuT)"""


    cv2.imshow('Detected Shape', output)
    cv2.imshow('Original', frame)

    if detected:
        print(f'{shape} detectado. Desviación del centro: {alignment} píxeles')


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


video_stream.stop()
cv2.destroyAllWindows()

import cv2
from camthreader import *
import numpy as np
import math
import time
import serial

debugOriginal = False
debugBlack = False
debugHori = False
debugGreen = False
debugBlue = False
record = False
noise_blob_threshold = 16

vs = WebcamVideoStream(src=0).start()
ser = serial.Serial('/dev/serial0', 115200)

lower_black = np.array([0, 0, 0])  # BGR
upper_black = np.array([70, 70, 70])
lower_green = np.array([0, 100, 80])  # lab
upper_green = np.array([255, 120, 150])
lower_silver_hsv = np.array([20,2,90])  # Valores en espacio HSV para detectar el plateado
upper_silver_hsv = np.array([35, 20, 175])  # Ajusta estos valores si es necesario

test_frame = vs.read()
width, height = test_frame.shape[1], test_frame.shape[0]
print(width, height)

cam_x = width / 2 - 1   # 79 ~ middle column
cam_y = height - 1      # 119 ~ bottom row

timer_active = False
green_output_duration = 1  # Duración en segundos para mostrar el estado
green_output_cooldown_duration = 2.5  # Duración en segundos para estar en 0
green_state_final = 0
timer_active = False
timer_start_time = 0

# GET X AND Y ARRAYS
# scaled by frame dimensions, use this for filtering pixels
x_com = np.zeros(shape=(height, width))
y_com = np.zeros(shape=(height, width))

for i in range(height):
    for j in range(width):
        x_com[i][j] = (j - cam_x) / (width / 2)   # [-1, 1]
        y_com[i][j] = (cam_y - i) / height        # [0, 1]

cccounter = 0

while True:
    frame = vs.read()
    frame[:50, :, :] = 255          # ignore top 1/8 of frame
    kernel = np.ones((3, 3), np.uint8)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    # FILTER BLACK PIXELS
    black_mask = cv2.inRange(frame, lower_black, upper_black)
    x_black = cv2.bitwise_and(x_com, x_com, mask=black_mask)
    x_black *= (1 - y_com)
    y_black = cv2.bitwise_and(y_com, y_com, mask=black_mask)

    # FILTER GREEN PIXELS
    green_mask = cv2.inRange(lab, lower_green, upper_green)
    green_mask[:90, :] = 0

    # DETECT GREEN SQUARES
    green_contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours = [c for c in green_contours if cv2.contourArea(c) > noise_blob_threshold]
    green_mask = cv2.drawContours(green_mask, green_contours, -1, 255, -1)

    x_green = cv2.bitwise_and(x_com, x_com, mask=green_mask)

    # CALCULATE RESULTANT
    green_state = 0
    x_resultant = np.mean(x_black)
    y_resultant = np.mean(y_black)
    angle = (math.atan2(y_resultant, x_resultant) / math.pi * 180) - 90
    speed = 25

    if len(green_contours) == 1:
        if np.mean(x_green) < 0:  # turn left
            green_state = 1
            speed = 30
            angle = 45  
        else:
            green_state = 2  # turn right
            speed = 30
            angle = -45

    if len(green_contours) == 2:
        green_state = 3

    if y_resultant == 0:
        angle = 0

    # DETECTION OF SILVER LINE (IN HSV)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convertir la imagen de BGR a HSV
    silver_mask = cv2.inRange(hsv_frame, lower_silver_hsv, upper_silver_hsv)

    silver_contours, _ = cv2.findContours(silver_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    silver_line = False
    for contour in silver_contours:
        area = cv2.contourArea(contour)
        if area > 2:  # Define un umbral para el área para evitar ruido
            silver_line = True
            break

    # CONTINUE WITH GREEN STATE CALCULATION
    contornos_izquierda = 0
    contornos_derecha = 0
    for contorno in green_contours:
        x, y, w, h = cv2.boundingRect(contorno)
        if x + w / 2 < cam_x:
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

    green_state_final = green_state

    if green_state_final != 0:
        if not timer_active:
            green_output = green_state_final
            timer_active = True
            timer_start_time = time.time()
        else:
            elapsed_time = time.time() - timer_start_time
            if elapsed_time < green_output_duration:
                green_output = green_state_final
            elif elapsed_time < (green_output_duration + green_output_cooldown_duration):
                green_output = 0
            else:
                timer_active = False
                green_output = green_state_final
    else:
        green_output = green_state_final
    
    # SEND OUTPUT WITH SILVER LINE DETECTION
    output = [255, speed,
              254, round(angle) + 90,
              253, green_output,
              252, int(silver_line)]  # 1 si se detecta la línea plateada, 0 en caso contrario

    print(output)
    ser.write(output)

    # DEBUGS
    if debugOriginal:
        cv2.imshow('Original', frame)
    if debugBlack:
        cv2.imshow('Black Mask', black_mask)
    if debugGreen:
        cv2.imshow('Green Mask', green_mask)
    if debugHori:
        cv2.imshow('Silver Mask', silver_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
vs.stop()

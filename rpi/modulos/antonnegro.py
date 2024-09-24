import cv2
import numpy as np
import serial

# Tracks a black line. Use (128, 255) for tracking a white line.
GRAYSCALE_THRESHOLD = (0, 64)

# Actualizamos las ROIs para la resolución 160x120.
WROIS = [
    (0, 70, 160, 20, 15),   # Área inferior
    (20, 50, 120, 20, 10),  # Área media
    (0, 25, 35, 25, 7),     # Izquierda del medio
    (125, 25, 35, 25, 7),   # Derecha del medio
    (30, 15, 100, 20, 1),   # Área superior
]
N = len(WROIS)

ser = serial.Serial('/dev/serial0', 115200, timeout=0)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

def preprocess(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, GRAYSCALE_THRESHOLD[1], 255, cv2.THRESH_BINARY_INV)
    return thresh

def find_largest_contour(contours):
    max_contour = max(contours, key=cv2.contourArea) if contours else None
    return max_contour

while True:
    ret, img = cap.read()
    if not ret:
        break

    thresh = preprocess(img)
    
    centroids = [0] * N
    pixels = [0] * N
    weights = [0] * N
    angles = [0] * N

    for i in range(N):
        x, y, w, h, weight = WROIS[i]
        roi = thresh[y:y+h, x:x+w]

        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = find_largest_contour(contours)

        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = 0, 0
            
            centroids[i] = (cx + x) * weight
            pixels[i] = cv2.contourArea(largest_contour)
            weights[i] = weight

            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img, [box], 0, (0, 255, 0), 2)

            angle = rect[2]
            if angle < -45:
                angle = 90 + angle
            angles[i] = angle

    if all(pixels[2:4]):
        if pixels[3] > pixels[2]:
            weights[2] = 0
            centroids[2] = 0
        else:
            weights[3] = 0
            centroids[3] = 0

    speed = int(250 - abs(angles[1]) * 1.7) if any(angles) else 80

    if any(pixels[2:4]):
        speed = 80

    if sum(weights) > 0:
        center_pos = sum(centroids) / (sum(weights) + 0.001)
        turn_rate = round(center_pos - 80)
    else:
        turn_rate = 0
        speed = 80
    if turn_rate >= 0:
        line_midle = 0
    else:
        line_midle = 1
        
    # Adaptamos las coordenadas para dibujar las flechas y texto en la imagen de 160x120.
    cv2.arrowedLine(img, (80, 5), (80 + turn_rate, 5), (255, 255, 255), 3)
    cv2.putText(img, f"spd:{speed} trn:{turn_rate} lm{line_midle}", (0, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
        
    # Send the data to the Teensy
    ser.write([255, 20])
    ser.write([254, round(abs(turn_rate))])
    ser.write([253, line_midle])
    ser.write([252, 0])

    for r in WROIS:
        x, y, w, h = r[:4]
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)

    cv2.imshow('Line Follower', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

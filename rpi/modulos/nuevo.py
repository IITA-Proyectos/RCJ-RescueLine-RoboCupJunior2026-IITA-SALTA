import cv2
import numpy as np

def preprocess_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_color = np.array([0, 0, 200])
    upper_color = np.array([180, 50, 255])
    mask = cv2.inRange(hsv, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

def detect_line_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None, None
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    if len(contours) < 2:
        return contours[0], None
    return contours[0], contours[1]

def calculate_center_line(contour1, contour2):
    if contour2 is None:
        return contour1
    points1 = contour1[:, 0, :]
    points2 = contour2[:, 0, :]
    if len(points1) > len(points2):
        points1 = points1[:len(points2)]
    else:
        points2 = points2[:len(points1)]
    
    center_points = (points1 + points2) // 2
    
    return center_points

def draw_trajectory(frame, points):
    if points is not None:
        cv2.polylines(frame, [points], isClosed=False, color=(0, 255, 0), thickness=5)
        for point in points[::50]:  
            cv2.circle(frame, tuple(point), 5, (255, 0, 0), -1)

def main():
    cap = cv2.VideoCapture(0)
    
    # Configurar la resolución de la cámara
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        mask = preprocess_frame(frame)
        contour1, contour2 = detect_line_contours(mask)
        if contour1 is not None and contour2 is not None:
            center_line = calculate_center_line(contour1, contour2)
            draw_trajectory(frame, center_line)
        
        cv2.imshow('Mascara', mask)
        cv2.imshow('Seguidor de Linea', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import numpy as np
import cv2

# Capturar video desde la cámara
webcam = cv2.VideoCapture(0)

# Iniciar el bucle para capturar frames
while True:
    # Leer un frame de la cámara
    _, imageFrame = webcam.read()

    # Definir el rango para el color verde en el espacio BGR
    # El formato BGR significa (Azul, Verde, Rojo)
    green_lower = np.array([0, 100, 0], np.uint8)  # Umbral mínimo para verde
    green_upper = np.array([100, 255, 100], np.uint8)  # Umbral máximo para verde

    # Crear una máscara que solo deje pasar los píxeles que estén en el rango de verde
    green_mask = cv2.inRange(imageFrame, green_lower, green_upper)

    # Encontrar contornos para los objetos verdes detectados
    contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Dibujar rectángulos alrededor de los objetos verdes detectados
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 300:  # Filtrar objetos pequeños
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Dibujar el rectángulo verde
            cv2.putText(imageFrame, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))

    # Mostrar el frame con las detecciones
    cv2.imshow("Detección de color verde en BGR", imageFrame)

    # Salir del bucle si se presiona 'q'
    if cv2.waitKey(10) & 0xFF == ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break

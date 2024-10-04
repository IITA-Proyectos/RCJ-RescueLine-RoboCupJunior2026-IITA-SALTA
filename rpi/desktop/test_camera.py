import cv2
import numpy as np
# Cargar la imagen desde la ruta especificada
ruta_imagen = 'rpi//desktop//roboliga//IMG-20240913-WA0082.jpg'

imagen = cv2.imread(ruta_imagen)
imagen[:50, :, :] = 255 
# Convertir la imagen a espacio de color LAB para detectar el color verde
lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
# Definir el rango de color verde en LAB (ajustado con base en la imagen)
bajo_verde = np.array([0, 100, 80])   # Ajusta estos valores si es necesario
alto_verde = np.array([255, 120, 150])  # Ajusta estos valores si es necesario
# Crear una máscara que detecte el color verde
mascara_verde = cv2.inRange(lab, bajo_verde, alto_verde)
# Encontrar los contornos en la imagen basada en la máscara
contornos, _ = cv2.findContours(mascara_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Dibujar todos los contornos detectados en una copia de la imagen
imagen_contornos = imagen.copy()
cv2.drawContours(imagen_contornos, contornos, -1, (255, 0, 0), 2)  # Dibuja todos los contornos (en azul)
# Dibujar un recuadro cuadrado perfecto alrededor de **todos** los contornos detectados
print(len(contornos))
print(np.mean(contornos[0]))
if len(contornos) == 1:
    if np.mean(contornos[0]) < 70:  # turn left
        green_state = 1
        angle = 45
        speed=40
    else:
        green_state = 2  # turn right
        angle = -45
        speed=40
    if len(contornos) == 2:
        green_state = 3
# Mostrar la imagen con los contornos en azul y la máscara
print(green_state)
cv2.imshow("Contornos detectados", imagen_contornos)
cv2.imshow("Máscara Verde LAB", mascara_verde)
cv2.waitKey(0)
cv2.destroyAllWindows()
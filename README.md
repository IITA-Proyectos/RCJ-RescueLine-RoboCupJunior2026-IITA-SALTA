# RescueBot

El equipo está preparándose para la competencia "Roboliga" en la categoría **Rescue Line**. Esta competencia implica diseñar un robot autónomo capaz de seguir una línea mientras evita obstáculos, y luego desempeñarse correctamente en una etapa de rescate de víctimas vivas o muertas.


## Integrantes:
- Benjamin Villagrán (Electronica)
- Valentina Guaymás (Diseño 3D)
- Laureano Monteros (Programacion en C++)
- Agustín Figueroa (Programacion en C++)
- Lucio Saucedo (Programacion en Python)

## Lista de Materiales

A continuación, se detallan los componentes que estamos utilizando para el proyecto:

### Cámara: 0.3 MegaPixels USB Camera for Raspberry Pi / NVIDIA Jetson Nano / UNIHIKER
- **Uso:** Reconocimiento de imagen
- **Cantidad:** 1

### MPU6050
- **Uso:** Giroscopio y acelerómetro
- **Cantidad:** 1*

### Sensor ToF VL53L0X
- **Uso:** Detectar distancias
- **Cantidad:** 2*

### Sensor de Ultrasonido
- **Uso:** Detección de obstáculos
- **Cantidad:** 3*

### Placa de Procesamiento: Raspberry Pi 4 (8 GB de RAM)
- **Uso:** Procesamiento de imágenes con Python
- **Cantidad:** 1*

### Teensy 4.1
- **Uso:** Microcontrolador para sensores, motores y servomotores
- **Cantidad:** 1*

### Brushless DC Motor with Encoder 12V 159RPM
- **Uso:** Motor
- **Cantidad:** 4*

### Servo 2kg 300º Clutch
- **Uso:** Activación de mecanismos
- **Cantidad:** 5*

### Rueda Omniwheel 58mm y 12kg
- **Uso:** Desplazamientos laterales y mayor tracción
- **Cantidad:** 2*

### Ruedas Fijas
- **Uso:** Desplazamiento frontal
- **Cantidad:** 2*

### Batería 11.1V
- **Uso:** Alimentación
- **Cantidad:** 1*

### Reguladores 6V y 5V
- **Uso:** Reducción de tensión para sensores y la Teensy
- **Cantidad:** 2*

### Ficha XT 60 PLUG
- **Uso:** Conector para la batería
- **Cantidad:** 1*

## Hardware del robot
![alt text](imagenes\image.png)
- Diseño 3D del Robot hecho en Fusion360

![alt text](imagenes\image-1.png)
- Diseño PCB del Robot hecho en EasyEda

## Software del Robot
### Raspberry pi (Python)
Es una computadora de placa unica con su potente CPU, RAM nos facilita el funcionamiento de programa 
Usamos la raspberry pi para procesamiento de imagen con una camara usando la libreria Open CV envia datos a la Teensy por medio de un puerto serial (UART)
![alt text](imagenes\image-2.png)
### Teensy (C++)
Es una placa de microprocesamiento mas potente que las arduino comunes consta de gran variedad de pines lo que facilita la conexion de los mismos.
Recibe datos de la raspberry pi para realizar diversas acciones (detecciones de obstaculo, movimiento) a partir de los componentes
![alt text](imagenes\image-3.png)

## Para más:
- https://github.com/IITA-Proyectos/Roboliga-2024-Rescue-Line-Team-RescueBot/tree/beb59b8b4d3ad57cc9bd6ae8f23ab345ed6bcc90/Dise%C3%B1o-Cad(Diseño cad)
- https://github.com/IITA-Proyectos/Roboliga-2024-Rescue-Line-Team-RescueBot/tree/beb59b8b4d3ad57cc9bd6ae8f23ab345ed6bcc90/Creacion_PCB (Diseño PCB)
- https://github.com/IITA-Proyectos/Roboliga-2024-Rescue-Line-Team-RescueBot/tree/beb59b8b4d3ad57cc9bd6ae8f23ab345ed6bcc90/rpi (Programacion Python)
- https://github.com/IITA-Proyectos/Roboliga-2024-Rescue-Line-Team-RescueBot/tree/beb59b8b4d3ad57cc9bd6ae8f23ab345ed6bcc90/test (Programacion C++)
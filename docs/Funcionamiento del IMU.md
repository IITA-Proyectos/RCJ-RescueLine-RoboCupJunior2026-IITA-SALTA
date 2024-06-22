## Introducción
Este README explica la importancia del IMU (Unidad de Medición Inercial) en el funcionamiento de un rescue bot. Este robot está diseñado para seguir una línea negra, esquivar obstáculos y llegar a una zona de rescate donde detecta y deposita pelotas en un lugar específico.

Elegimos el MPU6050 como giroscopio y acelerómetro para el IMU del proyecto por su precisión, facilidad de integración y costo accesible. Este sensor nos permite implementar funcionalidades avanzadas de medición inercial en el robot de manera efectiva.



#### Código y Funcionamiento

El código proporcionado consta de dos módulos principales: `IMU.cpp` y `runtimempu.cpp`, cada uno desplegando funciones clave para el control y la navegación del robot.

#### IMU (IMU.cpp)

El módulo `IMU.cpp` se encarga de integrar el IMU MPU6050 al sistema del robot. Aquí se inicializa y calibra el MPU6050 para obtener datos precisos de los giros y movimientos del robot. Algunas funciones importantes son:

- **Inicialización y Configuración del MPU6050:** Se configura el MPU6050 para medir a 2000 DPS (grados por segundo) y un rango de ±2G. Se realiza una calibración inicial del giroscopio para asegurar mediciones precisas.

- **Función `leermpu()`:** Esta función se ejecuta a intervalos regulares para leer y normalizar los valores del giroscopio. Calcula los ángulos de Pitch, Roll y Yaw, que son cruciales para determinar la orientación del robot en tiempo real.

- **Uso del IMU en `runTime()`:** En la función `runTime()`, se utiliza el IMU para controlar el movimiento del robot durante un tiempo determinado. Esto permite ajustes precisos de dirección y evasión de obstáculos basados en los ángulos calculados por el IMU.

#### runTime MPU (runtimempu.cpp)

En el módulo `runtimempu.cpp`, también se integra el MPU6050 pero con algunas diferencias clave en la configuración y manejo de interrupciones de hardware. Las funciones principales incluyen:

- **Configuración de Interrupciones:** Se utiliza `attachInterrupt()` para manejar las interrupciones de los encoders de los motores, lo cual es fundamental para el control preciso de la velocidad y posición del robot.

- **Función `runTime3()`:** Introduce una nueva función `runTime3()` que utiliza el IMU para controlar el ángulo de giro del robot de manera precisa. Esto es esencial para movimientos direccionales específicos como girar 90 grados en sentido horario o antihorario.

#### Funcionamiento del runTime

En ambos módulos (`IMU.cpp` y `runtimempu.cpp`), las funciones `runTime()` y `runTime3()` son fundamentales para el control temporal del movimiento del robot. Estas funciones utilizan el IMU para ajustar la dirección y velocidad del robot durante períodos definidos (`time`), asegurando un desplazamiento preciso y controlado.

#### Conclusión

El IMU juega un papel crucial en el control y la navegación del rescue bot. Proporciona datos precisos sobre la orientación y movimiento del robot, permitiendo así que pueda esquivar obstáculos y realizar maniobras específicas como girar con precisión. La integración del IMU con las funciones de tiempo de ejecución (`runTime`) asegura que el robot pueda cumplir con sus tareas de manera efectiva y segura en entornos dinámicos y cambiantes.


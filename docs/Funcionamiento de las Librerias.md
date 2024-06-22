# Que Librerias Utilizamos?

## OpenCV

![imagen](/docs/imagenes/opencv.png)

**OpenCV** (cv2) es una herramienta sumamente esencial en el ámbito del procesamiento de imágenes y visión por computadora. Al ser combinada con la librería camthreader (*la cual aunque no se encuentra disponible en Pylance*) cumple con la importante función de gestionar y capturar los fotogramas provenientes de la cámara. OpenCV, por su parte, ofrece una vasta variedad de funciones para el procesamiento de imágenes, la detección de objetos y el análisis de vídeo, aspectos fundamentales para las distintas tareas relacionadas con la cámara dentro del programa.



Dentro del programa, OpenCV desempeña un papel crucial al permitirnos operar la cámara y realizar el procesamiento de imágenes. Además de facilitar la captura de imágenes de la cámara, esta potente librería nos brinda las herramientas necesarias para realizar operaciones de procesamiento de imágenes de manera eficiente y efectiva. Utilizamos las funciones de OpenCV para llevar a cabo tareas como el filtrado de imágenes, la detección de bordes, la segmentación y la extracción de características, entre otras.
Una de las funcionalidades más importantes que implementamos con OpenCV es la generación de máscaras. Estas máscaras son esenciales para nuestro programa, ya que nos permiten aislar y resaltar los objetos de interés en las imágenes capturadas por la cámara. Utilizamos diferentes técnicas de procesamiento de imágenes, como la umbralización y la segmentación, para crear máscaras que nos ayudan a detectar los objetivos necesarios para el funcionamiento del programa.

### CamThreader (Liberia desarrollada por el equipo)

Por otro lado, camthreader (una librería desarrollada internamente por el equipo) se encarga de controlar el encendido de la cámara y la captura de los fotogramas.

 Aunque no está disponible en Pylance, esta librería es 
esencial para asegurar una interacción eficiente con la cámara y proporcionar una interfaz adecuada para la 
adquisición de imágenes que posteriormente serán procesadas por OpenCV. Su papel radica en garantizar un flujo de datos coherente y eficaz desde la cámara hacia el programa principal.

## NumPy

![NumPy](/docs/imagenes/numpy.png)

**NumPy**, una de las librerías más populares en el entorno de Python, desempeña un papel crucial en este programa al permitir el manejo de arreglos numéricos y la realización de operaciones matemáticas complejas. En este contexto, NumPy se utiliza principalmente para la creación de matrices, una estructura de datos esencial para almacenar elementos de manera homogénea y multidimensional. Su eficiencia en la manipulación de datos numéricos resulta invaluable en diversas partes del programa, facilitando la implementación de algoritmos y cálculos complejos.


En el programa, se emplea NumPy para una variedad de tareas fundamentales en el procesamiento de imágenes y el análisis de datos. Esta biblioteca ofrece estructuras de datos eficientes para representar y manipular matrices multidimensionales, lo que resulta crucial para el manejo de imágenes en el programa. NumPy se utiliza extensivamente para calcular momentos de imágenes, determinar centroides y realizar operaciones matriciales necesarias para el procesamiento de imágenes, como el filtrado y la segmentación. Su eficiencia y rendimiento optimizados son especialmente valiosos en aplicaciones de procesamiento de imágenes en tiempo real, donde cada milisegundo cuenta.

Además, NumPy se integra perfectamente con OpenCV, otra herramienta esencial en el programa, lo que facilita la interoperabilidad entre estas dos bibliotecas. La combinación de NumPy y OpenCV permite un flujo de trabajo fluido al combinar el procesamiento de imágenes de OpenCV con las operaciones numéricas de NumPy. En conjunto, NumPy desempeña un papel central en la optimización del rendimiento y la velocidad del procesamiento de imágenes, brindando las herramientas 

necesarias para manipular eficientemente datos numéricos en el contexto del análisis de imágenes y visión por computadora.


### Math

La librería Math, por su parte, se convierte en una herramienta indispensable para llevar a cabo cálculos numéricos dentro del programa. Específicamente, se utiliza para funciones como el cálculo del ángulo de giro del robot basado en la información capturada por la cámara, así como para definir constantes numéricas esenciales en el programa. Con una amplia gama de funciones matemáticas estándar, Math simplifica la implementación de operaciones numéricas complejas, contribuyendo así a la eficacia y precisión del código.

La biblioteca Math es una herramienta esencial en el programa, ya que se utiliza para realizar diversos cálculos numéricos necesarios en el procesamiento de imágenes y la visión por computadora. Esta biblioteca se emplea principalmente para calcular ángulos con precisión, lo que resulta fundamental para que el robot realice movimientos precisos y siga rutas específicas basadas en la información capturada por la cámara. Además, Math se utiliza para definir constantes numéricas utilizadas en el programa, lo que permite ajustar y adaptar los algoritmos de manera eficiente a diferentes escenarios y condiciones.


Además, la biblioteca Math proporciona una amplia gama de funciones matemáticas estándar que facilitan la implementación de operaciones numéricas complejas en el código. Estas funciones son útiles para realizar cálculos avanzados y manipular datos numéricos de manera efectiva durante el procesamiento de imágenes y la detección de objetos. En resumen, Math desempeña un papel crucial junto con otras bibliotecas como NumPy y OpenCV al proporcionar las herramientas necesarias para realizar cálculos precisos y eficientes en el contexto de la visión artificial y el control de robots.

## PySerial

![pyserial](/docs/imagenes/pyserial.png)



La biblioteca **PySerial** desempeña un papel fundamental en el programa al permitir el establecimiento de una comunicación serial entre la Raspberry Pi y la Teensy. Esta comunicación serial es esencial para enviar y recibir datos entre los dispositivos, lo que facilita una interacción efectiva entre ellos en el programa. PySerial proporciona una interfaz simple para trabajar con puertos serie, lo que permite la transmisión confiable de datos entre la Raspberry Pi y la Teensy a través de un canal serial.

Al utilizar PySerial, el programa puede enviar comandos y recibir información de la Teensy, lo que permite controlar el comportamiento del robot en función de los datos capturados por la cámara y los algoritmos de procesamiento de imágenes. Además, PySerial facilita la configuración de la comunicación serial (incluida la velocidad de transmisión y otros parámetros importantes) lo que garantiza una comunicación estable y eficiente entre los dispositivos.

 En resumen, PySerial es crucial para establecer una conexión serial confiable entre la Raspberry Pi y la Teensy, lo que permite una interacción fluida y coordinada entre la visión por computadora y el control del robot.

## NewPing

![NewPing](/docs/imagenes/NewPing.png)

La librería NewPing es una herramienta fundamental para proyectos de robots autónomos que dependen de sensores ultrasónicos para detectar y evitar obstáculos. Esta librería optimiza el manejo de los sensores en plataformas Arduino al utilizar una técnica de temporización sin bloqueo, lo cual permite al microcontrolador realizar otras tareas mientras espera la respuesta del sensor. Esto es crucial para mantener la capacidad de respuesta del robot en tiempo real y gestionar eficientemente múltiples sensores ultrasónicos.

Al integrar NewPing en un robot autómata, los desarrolladores pueden acceder fácilmente a funciones que facilitan la medición precisa de la distancia hasta los objetos detectados. La librería ofrece métodos como `ping()` que devuelven la distancia en centímetros, permitiendo al robot tomar decisiones rápidas y precisas basadas en la proximidad de los obstáculos. Además, NewPing incluye características avanzadas como la detección de eventos, que ayuda a manejar situaciones de fallo o errores en la comunicación con los sensores, mejorando la fiabilidad operativa del sistema.

Otra ventaja significativa de NewPing es su compatibilidad con una amplia gama de sensores ultrasónicos disponibles en el mercado, lo cual facilita la selección y configuración del hardware según las necesidades específicas del robot y del entorno. Esto proporciona flexibilidad en el diseño y la implementación de robots autónomos, garantizando un rendimiento óptimo en diversas condiciones ambientales y operativas. En resumen, NewPing no solo simplifica la integración y gestión de sensores ultrasónicos en robots autónomos, sino que también mejora su eficiencia, fiabilidad y capacidad de adaptación en aplicaciones prácticas.

## MPU6050

![MPU6050](/docs/imagenes/mpu6050.png)

La librería MPU6050 es fundamental para robots autónomos que requieren un control preciso de la orientación y movimiento. Este sensor de movimiento y giroscopio de seis ejes integra acelerómetro y giroscopio en un solo chip, proporcionando mediciones precisas de la aceleración lineal y la velocidad angular en tres dimensiones. Esto permite al robot determinar su posición, orientación y movimiento con gran exactitud, esencial para la navegación autónoma y la estabilización dinámica.

Al utilizar la librería MPU6050 con plataformas como Arduino, los desarrolladores pueden acceder fácilmente a funciones que gestionan la comunicación y calibración del sensor. La librería facilita la lectura de datos del acelerómetro y giroscopio, realizando automáticamente la compensación de errores y la fusión de datos necesarias para obtener mediciones precisas y estables. Esto es crucial para mantener la estabilidad del robot durante el movimiento y para ejecutar algoritmos de control que respondan de manera eficiente a cambios en el entorno o en las condiciones de operación.

Además de su funcionalidad básica de medición de movimiento, la librería MPU6050 también soporta características avanzadas como el DMP (Digital Motion Processor), que permite realizar cálculos complejos internamente en el sensor, reduciendo la carga computacional en el microcontrolador principal. Esto optimiza el rendimiento del sistema global del robot al liberar recursos para otras tareas críticas, como el procesamiento de datos de otros sensores o la toma de decisiones basadas en la información ambiental.

## PID

![PID](/docs/imagenes/PID.png)

La librería PID (Proportional-Integral-Derivative) es fundamental en el control de sistemas automatizados como robots usando Arduino. Este tipo de controlador ajusta la salida del sistema en función de la diferencia entre el valor deseado (setpoint) y el valor medido actualmente (feedback). La acción proporcional (P) corrige proporcionalmente el error actual, la acción integral (I) considera la acumulación de errores pasados y la acción derivativa (D) predice la tendencia del error en el futuro. Juntos, estos componentes permiten al controlador PID ajustar la salida de manera precisa y rápida para minimizar el error y estabilizar el sistema.

En el contexto de un robot controlado por Arduino, el PID se emplea para mejorar la precisión del movimiento y la respuesta ante perturbaciones externas. Por ejemplo, al mover un robot hacia un punto específico, el control PID ajusta continuamente los valores de velocidad y dirección de los motores en función de cómo se desvía la posición real del punto deseado. Esto asegura que el robot pueda llegar al destino de manera eficiente y con mínima oscilación.

Implementar la librería PID en Arduino requiere ajustar adecuadamente los parámetros P, I y D mediante pruebas y ajustes iterativos para adaptarse a las características específicas del robot y las condiciones del entorno. Este proceso de sintonización es crucial para lograr un comportamiento estable y preciso del robot en diferentes situaciones operativas. Además, la librería PID en Arduino es flexible y permite adaptarse a diversas aplicaciones, desde el control de trayectorias de robots móviles hasta sistemas de control de temperatura o posicionamiento de actuadores.

## LiDAR

![lidar](/docs/imagenes/lidar.png)

La librería LiDAR para robots automatizados en Arduino facilita la integración de sensores LiDAR (Light Detection and Ranging) para la percepción del entorno. Estos sensores emiten pulsos láser y miden el tiempo que tarda en regresar la luz reflejada, permitiendo así calcular distancias precisas a objetos en el entorno cercano del robot. La librería simplifica la comunicación entre el sensor LiDAR y la placa Arduino, proporcionando funciones para inicializar el sensor, adquirir datos de distancia y manejar interrupciones.

Al utilizar un LiDAR con Arduino, el robot puede realizar tareas como mapeo de entornos, evitación de obstáculos y navegación autónoma de manera más eficiente y precisa. La librería facilita la implementación de algoritmos de control que utilizan datos del LiDAR para tomar decisiones en tiempo real, como ajustar la trayectoria para evitar colisiones o calcular la posición relativa de objetos para fines de mapeo. Esto es especialmente útil en aplicaciones de robótica móvil donde la percepción del entorno es crucial para el rendimiento seguro y eficaz del robot.

Configurar la librería LiDAR en Arduino implica inicializar el sensor correctamente, establecer los parámetros de comunicación serial y realizar pruebas de funcionamiento para garantizar la precisión de las mediciones. Además, la librería puede incluir métodos para filtrar datos de distancia y manejar condiciones de iluminación adversas, mejorando así la robustez y confiabilidad de los sistemas de percepción del robot. En resumen, la librería LiDAR para Arduino amplía las capacidades de percepción sensorial de los robots automatizados, permitiéndoles interactuar de manera más inteligente y segura con su entorno.

## VL53L1X

![tof](/docs/imagenes/vl53l1x.png)

La librería VL53L1X para Arduino facilita la integración del sensor de distancia de tiempo de vuelo (ToF) VL53L1X en robots automatizados. Este sensor utiliza tecnología láser para medir distancias con alta precisión y rapidez, operando en rangos de hasta varios metros. La librería simplifica la comunicación entre el sensor VL53L1X y la placa Arduino, proporcionando funciones para inicializar el sensor, configurar parámetros como la distancia máxima de medición y el modo de medición, así como para adquirir datos de distancia en tiempo real.

Al implementar el VL53L1X con Arduino, los robots pueden realizar diversas tareas que requieren percepción de distancia, como evitar obstáculos, realizar mapeo de entornos o detectar la posición de objetos móviles. La alta precisión del sensor ToF permite una navegación más segura y eficiente, adaptándose dinámicamente a cambios en el entorno. Además, la librería VL53L1X puede incluir métodos avanzados para gestionar múltiples sensores o para filtrar datos de distancia, mejorando la robustez del sistema en condiciones variables de iluminación o reflexión.

Configurar la librería VL53L1X en Arduino implica conectar físicamente el sensor al microcontrolador, cargar la librería en el entorno de desarrollo Arduino IDE y utilizar las funciones proporcionadas para inicializar y leer los datos del sensor. A través de esta integración, los robots automatizados pueden mejorar significativamente sus capacidades de percepción espacial y responder de manera más inteligente a su entorno, lo que es crucial para aplicaciones de robótica móvil y sistemas de automatización industrial.


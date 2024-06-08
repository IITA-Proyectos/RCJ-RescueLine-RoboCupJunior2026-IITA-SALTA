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



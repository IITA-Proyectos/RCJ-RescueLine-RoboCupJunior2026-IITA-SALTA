# Documentación del Código "runtime"

## Descripción General del Código

El código proporcionado controla el movimiento de un robot utilizando una placa Teensy. Aquí hay una descripción general de su funcionamiento:

- **Inicialización:** El código comienza con la inicialización de funciones y pines necesarios para el funcionamiento del robot, como la configuración de pines de entrada/salida, la inicialización de motores, la configuración de interrupciones y la inicialización de la comunicación serial.

- **Bucle Principal (`loop()`):** El bucle principal del código (`loop()`) gestiona el funcionamiento continuo del robot mientras esté encendido. Este bucle verifica si el robot está encendido o apagado utilizando un interruptor, y ejecuta acciones específicas en función de su estado.

- **Manejo del Estado del Robot:** El código maneja tres estados principales del robot: encendido, apagado y reinicio. Cada estado activa diferentes comportamientos en el robot. Por ejemplo, cuando el robot está encendido, ejecuta un movimiento específico en bucle para realizar un cuadrado.

- **Funciones de Control de Movimiento (`runTime()`):** El código utiliza las funciones `runTime()` y `runTime2()` para controlar el movimiento del robot durante un período de tiempo específico. Estas funciones permiten al robot moverse a una velocidad, dirección y ángulo de dirección definidos durante un tiempo determinado.

## Función `runTime() y runtime2()`

La función `runTime()` se utiliza para controlar el movimiento del robot durante un período de tiempo específico. Aquí está la descripción de sus parámetros y su funcionamiento:

- **Parámetros:**
  - `speed`: La velocidad a la que el robot debe moverse, expresada en unidades específicas del sistema de movimiento.
  - `dir`: La dirección del movimiento del robot, que puede ser hacia adelante o hacia atrás.
  - `steer`: El ángulo de dirección del robot, que determina la curvatura de su movimiento (puede ser 0 para movimiento recto).
  - `time`: El tiempo durante el cual el robot debe realizar el movimiento, expresado en milisegundos.

- **Funcionamiento:**
  - La función `runTime()` activa el movimiento del robot con la velocidad, dirección y ángulo de dirección especificados.
  - El robot se mueve de acuerdo con estos parámetros durante el tiempo especificado.
  - Una vez transcurrido el tiempo especificado, la función detiene el movimiento del robot.
  -Determina el estado del switch 
  - Runtime utiliza una condición adicional dentro del bucle while para verificar el estado del interruptor (digitalRead(32) == 1). Si el interruptor está en la posición "off", envía un byte con valor 255 a través de la comunicación serial (Serial5.write(255)) para informar que el robot está apagado. Esto significa que la función detiene el movimiento del robot si el interruptor se apaga mientras está en ejecución.
  -Runtime2 no incluye la verificación del estado del interruptor dentro del bucle while. En cambio, envía el byte 255 constantemente a través de la comunicación serial (Serial5.write(255)) independientemente del estado del interruptor. Esto significa que la función no detiene el movimiento del robot incluso si el interruptor se apaga mientras está en ejecución.

La función `runTime()` es fundamental para el control preciso del movimiento del robot durante diversas operaciones, como desplazamientos, giros y otras maniobras específicas. Su flexibilidad y facilidad de uso la hacen esencial para el funcionamiento eficiente del robot en diferentes situaciones. Debido a que se le pueden asignar tiempos determinados

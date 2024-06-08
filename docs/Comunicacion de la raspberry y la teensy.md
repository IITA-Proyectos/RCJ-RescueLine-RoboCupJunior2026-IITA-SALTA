# Como es la comunicacion entre la Raspberry y la Teensy?

La Raspberry envía tareas (Tasks), la teensy las entiende y las convierte en acciones (Actions) y estas representan un cambio en el comportamiento del robot (Case).

### **Por ejemplo:**

### *El seguidor de linea detecta un cuadrado verde a la izquierda.*

 **Task:** La Raspberry le envia una tarea a la Tennsy.
 
 **Action**: Esta la interpreta y la convierte en una accion. Esta accion le dirá al robot que al encontrarse con un cuadrado verde al lado izquierdo de la linea, deberá tomar la ruta del lado izquierdo siguiendo la linea negra.
  
 **Case**: Ya al saber que accion debe tomar, la Tennsy generará un cambio en el movimiento del robot, para que pueda resolver la tarea, es decir, que cuando encuentre un cuadrado verde de la parte izquierda de la linea negra, este cambie de ruta y se dirija para la linea negra de su lado izquierdo.


## Tareas y Acciones


|   Tasks   | Actions |        Lo que detecta         |   Lo que hace   |
|-----------|---------|-------------------------------|-----------------|
|  Task 0   |    --    | No detecta verde            | Sigue su recorrido normal
|  Task 1   |   --     | Verde a la izquierda          | Gira el robot para que siga la linea negra de su izquierda
|  Task 2   |   --     | Verde a la derecha            | Gira el robot para que siga la linea negra de su derecha
|  Task 3   |    14   | Doble verde                   | Hace que el robot vuelva por su recorrido anterior
|  Task 4   |    3    | Detecta el cubo azul          | Debe recojer el cubo azul
|  Task 5   |    10   | Busca paredes                 | Tiene que buscar las paredes
|  Task 6   |    8    | Mueve a la zona de evacuacion | Hay que ir derecho para entrar a la zona de evacuación
|  Task 7   |    11   | Detecta rojo                  | Debe salir de la zona de evacuación
|  Task 8   |    7    | No hay cubo azul              | Si no encuentra el cubo azul, sigue con el seguidor de linea
|  Task 10  | 6 // 10 | Deposita pelota               | Si no depositó el cubo azul o las pelotitas, hace que las deposite
|  Task 11  |    9    | Linea roja                    | Debe salir de la zona de evacuación
|  Task 40  |    7    | Busca las pelotas negra y plateada| Se acomoda para agarrar las pelotas negras o plateadas
|  Task 50  |    30   | Detecta pelota negra          | Si esta cerca de una pelota negra, girara para centrar y cuando esté centrada este buscará recogerla
|  Task 60  |    40   | Detecta pelota plateada       | Si esta cerca de una pelota plateada, girara para centrar y cuando esté centrada este buscará recogerla

Hay algo en principal que está muy relacionado con la comunicación que hay entre la Teensy 4.1, qué es el **Void Runtime** que sirve para declarar a una velocidad, dirección y un ángulo. Además cuenta la cantidad de tiempo en lo que se demora en hacer una acción según el tiempo que tu le diste.

## Resumen basico del encendido del robot


En el **Void Loop** (*El void loop es una parte del codigo que se repite en un bucle constante*) hay un **“if”** (*El if sirve para que determinada parte del codigo solo funcione si pasa un suceso especifico*) consiste que si el switch está apagado nada se mueve.

Luego su **“else if”** (*El else if significa 
que si no se cumple el "if" inicial, pasa a esa parte del codigo*) que indica cuando comienza a partir de una variable, **“startUp”** (*Que es el primer inicio del robot*) acá los motores giran para adelante y para atrás durante un tiempo muy corto (esto sirve para calibrar). 

Por ultimo está el **“else”** (*El else sirve por si no se cumple ninguna condicion de if o else if*) que es la parte más elaborada donde ya se encuentra todo el resto del codigo.

![alt text](/docs/imagenes/encendido.png)

# Sistema de Encendido

## Inicialización del Sistema de Encendido

El sistema de encendido se activa a través de un interruptor conectado a un pin específico en la teensy, cuando el interruptor cambia de estado, se inician distintas secuencias de encendido o apagado.

### Configuración del Interruptor

El interruptor está conectado al **pin digital 32** y se configura como un pin de entrada con pull-up interno. Cuando el interruptor está en la posición de encendido, el pin lee un valor lógico alto (1), y cuando está en la posición de apagado, el pin lee un valor lógico bajo (0).

## Bucle Principal

En el bucle principal, se manejan tres estados distintos relacionados con el sistema de encendido:

- **Apagado**: Cuando el interruptor está en la posición de apagado, todos los motores se detienen y se envía un mensaje de estado a través de la comunicación serie.
- **Reinicio**: Cuando el interruptor cambia de la posición de apagado a la de reinicio, se activa una secuencia de reinicio, indicada por el parpadeo de un LED incorporado.
- **Encendido**: Cuando el interruptor está en la posición de encendido, el LED incorporado permanece encendido, indicando que el sistema está activo y listo para funcionar.

### Manejo de Estados

- **Apagado**: Si el interruptor está en la posición de apagado, se detienen todas las funciones y se espera hasta que el interruptor cambie de estado.
- **Reinicio**: Durante el reinicio, el LED incorporado parpadea brevemente para indicar que el sistema se está reiniciando. Esto puede ser útil para diagnosticar problemas con el cambio de estado del interruptor.
- **Encendido**: Cuando el interruptor está en la posición de encendido, el sistema está activo y listo para funcionar. Esto se indica mediante el LED incorporado, que permanece encendido. En el else del codigo se puede empezar a agregar codigo.


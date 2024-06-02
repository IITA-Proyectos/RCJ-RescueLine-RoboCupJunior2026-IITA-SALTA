# Como es la comunicacion entre la Raspberry y la Teensy?

La Raspberry envía tareas (Tasks), la teensy las entiende y las convierte en acciones (Actions) y estas representan un cambio en el comportamiento del robot (Case).

## Por ejemplo:

### La el seguidor de linea detecta un cuadrado verde a la izquierda.

 **Task:** La Raspberry le envia una tarea a la Tennsy.
 
 **Action**: Esta la interpreta y la convierte en una accion. Esta accion le dirá al robot que al encontrarse con un cuadrado verde al lado izquierdo de la linea, deberá tomar la ruta del lado izquierdo siguiendo la linea negra.
  
 **Case**: Ya al saber que accion debe tomar, la Tennsy generará un cambio en el movimiento del robot, para que pueda resolver la tarea, es decir, que cuando encuentre un cuadrado verde de la parte izquierda de la linea negra, este cambie de ruta y se dirija para la linea negra de su lado izquierdo.


## Tareas y Acciones


|   Tasks   | Actions |        Lo que detecta         |   Lo que hace   |
|-----------|---------|-------------------------------|-----------------|
|  Task 0   |        | No detecta verde            | Sigue su recorrido normal
|  Task 1   |        | Verde a la izquierda          | Gira el robot para que siga la linea negra de su izquierda
|  Task 2   |        | Verde a la derecha            | Gira el robot para que siga la linea negra de su derecha
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

## Resumen basico del encendido del robot

Hay algo en principal que está muy relacionado con la comunicación que hay entre la Teensy 4.1, qué es el **Void Runtime** que sirve para declarar a una velocidad, dirección y un ángulo. Además cuenta la cantidad de tiempo en lo que se demora en hacer una acción según el tiempo que tu le diste. En el **Void Loop** (*El void loop es una parte del codigo que se repite en un bucle constante*) hay un **“if”** (*El if sirve para que determinada parte del codigo solo funcione si pasa un suceso especifico*) consiste que si el switch está apagado nada se mueve. Luego su **“else if”** (*El else if significa que si no se cumple el "if" inicial, pasa a esa parte del codigo*) que indica cuando comienza a partir de una variable, **“startUp”** (*Que es el primer inicio del robot*) acá los motores giran para adelante y para atrás durante un tiempo muy corto (esto sirve para calibrar). Luego está el **“else”** (*El else sirve por si no se cumple ninguna condicion de if o else if*) que es la parte más elaborada donde ya se encuentra todo el resto del codigo.

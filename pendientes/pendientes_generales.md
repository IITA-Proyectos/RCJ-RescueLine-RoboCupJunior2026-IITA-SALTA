# Pendientes 

## Equipo:
- Laureano
- Lucio
- Benjamin

Los tres trabajamos en el mismo objetivo. La unica diferencia es que Benjamin hace pruebas de IA por separado.

Objetivo principal: optimizar el rescate con IA para aumentar FPS en Raspberry, manteniendo precision y estabilidad.

## Resumen reglamento 2026 (zona de rescate)

Resumen del reglamento 2026:
- Zona de evacuacion de 120 cm x 90 cm con paredes de al menos 10 cm.
- Cinta plateada reflectiva en la entrada (25 mm x 250 mm).
- Cinta negra en la salida (25 mm x 250 mm).
- La linea negra termina en la entrada y vuelve a empezar en la salida.
- Dos zonas altas (triangulos): rojo (victima muerta) y verde (victimas vivas).
- Triangulos de 30 cm x 30 cm, paredes de 6 cm, centro hueco.
- Paredes de cualquier color excepto rojo, verde y negro.
- Zonas en cualquier esquina que no sea entrada/salida.
- Puede haber obstaculos o speed bumps dentro de la zona (no suman puntos).
- Puede haber luces LED blancas en la parte alta de las paredes.
- Puede haber victimas falsas que deben ignorarse.
- Victimas reales: esferas de 4-5 cm, masa descentrada, max 80 g.
  - Vivas: plateadas, reflectivas y conductoras.
  - Muerta: negra, no conductora.


## Trabajo en equipo (tareas comunes)

- Definir decision sobre segunda camara y documentarla.
- Evaluar si la camara wide fija cubre rescate y si la segunda camara mejora la linea.
- Prototipo rapido de segunda camara con soporte temporal (si se aprueba).
- Probar linea en curvas 135 con la camara baja.
- Evaluar el impacto mecanico de mover el servo lift.
- Implementar y testear la estrategia de re-enganche con ROIs laterales.
- Verificar estabilidad del tracking (MOSSE o fallback) en secuencias largas.
- Confirmar que el pipeline multihilo no tiene colas saturadas ni latencias acumuladas.
- Registrar consumo de CPU y memoria para identificar cuellos de botella.

## Sistema de re-enganche en curvas cerradas (una sola camara)

1. La Raspberry calcula el porcentaje de linea negra visible.
2. Si baja de un umbral minimo, la Raspberry envia un green_state especial a la Teensy.
3. La Teensy retrocede una distancia corta para recuperar contexto visual.
4. La Raspberry analiza ROIs laterales (izquierda y derecha) buscando linea negra.
5. Si hay linea en un ROI, se gira hacia ese lado.
6. Si no hay linea, la Teensy avanza recto hasta recuperar un porcentaje confiable.
7. Al recuperar el porcentaje durante varios frames seguidos, se vuelve al modo normal.

Parametros por definir:
- Umbral de poca linea negra.
- Umbral de linea confiable.
- Tiempo o distancia de retroceso.
- Tamano y ubicacion exacta de ROIs laterales.

## Tareas especificas de Benjamin (IA)

- Probar TFLite con NMS en Raspberry Pi 5.
- Resultado actual: 16 FPS en pruebas internas.
- Comparar FPS vs precision con el modelo ONNX FP32.
- Ajustar augmentations con linternas intermitentes.
- Medir impacto de los augmentations en detecciones reales.

## Pendientes tecnicos prioritarios

- [ ] Corregir la salida de la zona de evacuacion.
- [ ] Medir FPS reales en rescate con ONNX FP32 y pipeline completo.
- [ ] Probar augmentation de iluminacion simulando linternas intermitentes.
- [ ] Grabar videos del robot en zona de rescate con diferentes paredes para entrenarlo con diferentes fondos.
- [ ] Validar comportamiento de detecciones en pista con cambios de luz.
- [ ] Resolver pendientes laterales inclinadas (falta grip).

## Bloqueos actuales

- La segunda camara requiere definicion mecanica antes de programar.

## Criterios de exito

- FPS superiores a 15 con rescate estable.
- Detecciones consistentes en diferente iluminacion.
- Mejora clara en curvas 135 sin salidas de linea.
- Salida de zona de evacuacion funcionando de forma confiable.
- No perder mucho tiempo probando las cosas

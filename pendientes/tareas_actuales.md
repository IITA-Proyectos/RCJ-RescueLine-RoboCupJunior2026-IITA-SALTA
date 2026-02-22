# Tareas actuales (2026-02-22)

## Objetivo inmediato

Optimizar el rescate con IA para aumentar FPS sin perder precision y estabilidad.

## Trabajo en equipo

Los tres (Laureano, Lucio y Benjamin) estamos trabajando **juntos** en el mismo objetivo general. Benjamin trabajando en adaptar nuestro modelo de IA al nuevo reglamento
## Tareas en curso 

- Definir decision sobre segunda camara y documentarla.
- Evaluar si el rendimiento de la camara wide no empeora el rendimiento por incluir otra camara y si la segunda camara mejora la linea.
- Prototipo rapido de segunda camara con soporte temporal (si se aprueba).
- Probar linea en curvas 135 con la camara baja.
- Evaluar el impacto mecanico de mover el servo `lift`.
- Implementar y testear la **estrategia de re-enganche** con ROIs laterales sin la segunda camara
## Tareas especificas de IA

- Probar un nuevo formato **TFLite con NMS** en Raspberry Pi 5.
- Resultado actual: **16 FPS** en pruebas internas.
- Continuar comparando FPS vs precision con el modelo ONNX FP32.
- Ajustar augmentations con **linternas intermitentes**.
- Medir impacto de los augmentations en detecciones reales.

## Tareas tecnicas detalladas

- Medir FPS en Raspberry con el modelo actual ONNX FP32 y pipeline completo.
-  Medir FPS con TFLite + NMS en Raspberry Pi 5 y comparar precision.
-  Revisar si el modelo pierde deteccion con menos imagenes en el dataset.
-  Probar augmentation de iluminacion simulando linternas intermitentes.
-  Validar el comportamiento de detecciones en pista con cambios de luz.
-  Definir decision sobre segunda camara y documentarla.
-  Prototipo rapido de segunda camara con soporte temporal (si se aprueba).
-  Probar linea en curvas 135 con la camara baja.
-  Evaluar el impacto mecanico de mover `lift`.
-  Implementar y testear la **estrategia de re-enganche** con ROIs laterales.


## Bloqueos actuales

- La segunda camara requiere definicion mecanica antes de programar cambios en serio.
- Las pruebas de dataset necesitan nuevos videos con linterna fondo diferente.
- Ajustes por **nuevo reglamento de rescate** aun en revision (prioridad alta).
- Pendientes laterales inclinadas: falta grip, se necesita solucion mecanica.

## Criterios de exito

- FPS sostenidos superiores a 15 con rescate estable.
- Detecciones consistentes en condiciones reales de luz.
- Mejora clara en curvas 135 sin salidas de linea.
- Cumplimiento del **nuevo reglamento de rescate**.

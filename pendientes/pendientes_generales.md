# Pendientes generales (2026-02-22)

## Contexto del equipo

Actualmente el equipo activo esta formado por 3 integrantes:
- Laureano
- Lucio
- Benjamin

Los tres trabajamos en el mismo objetivo general. La unica diferencia es que Benjamin tambien hace pruebas de IA por separado.

El foco principal es **optimizar el rescate con IA** para aumentar FPS en Raspberry, manteniendo precision y estabilidad en pista.

## Estado general del problema

- El modelo actual funciona pero el FPS es limitado porque corre en CPU y con un pipeline pesado.
- Se intento Edge Impulse y no es viable por el tamano del dataset.
- Reducir la cantidad de imagenes degrada la deteccion.
- Se evalua cambiar el setup de camaras para mejorar linea y rescate en simultaneo.
- La camara wide actual **queda fija** en rescate. La duda actual es si agregar una segunda camara.
- El entorno de pista variara mucho este ano, por eso se prioriza **robustez** antes que una metodologia unica de medicion.

## Pendientes de alto nivel

- [ ] Definir el objetivo de FPS minimo aceptable para rescate (ej: 15+ FPS sostenidos en CPU).
- [ ] Revisar si se mantiene el pipeline actual con `ultralytics` + `onnxruntime` o si se migra a otro runtime.
- [ ] Definir si se agrega una **segunda camara** (la wide actual queda fija).
- [ ] Validar que el cambio de camara soluciona perdidas en curvas 135 grados sin generar nuevos problemas.
- [ ] Prepararse con el **nuevo reglamento de la zona de rescate** (esto es prioridad maxima).

## Pendientes de software

- [ ] Registrar FPS reales en rescate con el modelo ONNX FP32 y el pipeline completo.
- [ ] Medir FPS con variaciones de `IMGSZ` y `DETECT_EVERY` para encontrar el mejor balance.
- [ ] Revisar thresholds por clase para evitar falsos positivos en rescate.
- [ ] Verificar estabilidad del tracking (MOSSE o fallback) en secuencias largas.
- [ ] Confirmar que el pipeline multihilo no tiene colas saturadas ni latencias acumuladas.
- [ ] Registrar consumo de CPU y memoria para identificar cuellos de botella.
- [ ] Disenar y validar logica alternativa para **una sola camara** cuando se pierde linea (ver detalle en tareas actuales).

## Pendientes de dataset

- [ ] Revisar distribucion de clases (balance entre pelotas y zonas).
- [ ] Agregar augmentation con **linternas intermitentes** para robustez de iluminacion.
- [ ] Validar si el augmentation produce mejoras reales en pista.
- [ ] Definir criterio de stop para no degradar el dataset con augmentation excesivo.

## Pendientes de hardware

- [ ] Disenar y validar el montaje de la segunda camara si se aprueba.
- [ ] Evaluar el impacto de mover el servo `lift` en recoleccion y deposito.
- [ ] Verificar cableado y consumo de energia con dos camaras.
- [ ] Comprobar que el nuevo angulo de camara inferior mejora curvas 135.
- [ ] Resolver **pendientes laterales inclinadas**: falta grip en pendientes; buscar una solucion mecanica/ruedas que asegure agarre.

## Riesgos actuales

- Riesgo de perder precision al intentar optimizar solo FPS.
- Riesgo de introducir vibracion o movimiento adicional con la nueva camara.
- Riesgo de empeorar la mecanica del `lift` si se cambia su posicion.
- Riesgo de no cumplir el **nuevo reglamento de rescate** si no se prioriza a tiempo.

## Entregables minimos esperados

- Documentacion final de pipeline con FPS medidos.
- Decision formal sobre segunda camara (si/no, y razon).
- Dataset actualizado con augmentation validado.
- Codigo actualizado y sincronizado en repo.

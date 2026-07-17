# AGENTS.md — Proyecto de graduación: Brazo cartesiano XYZ

## Rol de Codex

Este repositorio contiene copias de trabajo del código más reciente de la Portenta H7 y la ESP32 para el proyecto de graduación de un brazo cartesiano XYZ.

Codex debe actuar como asistente de desarrollo embebido. Su trabajo es analizar, proponer, modificar y documentar cambios en el código, pero siempre de forma controlada, segura y revisable.

Antes de modificar archivos, Codex debe:
1. Leer este archivo completo.
2. Revisar la estructura del proyecto.
3. Identificar qué archivos pertenecen a la Portenta H7 y cuáles pertenecen a la ESP32.
4. Explicar el plan de cambios.
5. Esperar confirmación si el cambio puede afectar movimiento físico, calibración, pines, comunicación o seguridad.

---

## Contexto general del proyecto

El proyecto es un brazo cartesiano XYZ para automatización de una línea de empaquetado o clasificación en laboratorio.

El sistema busca mover una herramienta o efector final hacia posiciones específicas para agarrar o manipular piezas. En el futuro se integrará visión artificial con cámara para detectar objetos, colores o figuras, y luego mover el brazo al punto correspondiente.

Por ahora el desarrollo se enfoca en:
- Control de movimiento manual.
- Calibración automática.
- Movimiento en coordenadas XYZ.
- Implementación progresiva de cinemática inversa.
- Comunicación entre Portenta H7 y ESP32.
- Seguridad con finales de carrera.
- Preparación para futura integración con cámara/HuskyLens.

---

## Hardware principal

### Control principal

- Placa principal: Arduino Portenta H7.
- La Portenta controla los movimientos del brazo cartesiano.
- La Portenta maneja motores, finales de carrera, modos de operación y calibración.

### Interfaz/control auxiliar

- Placa auxiliar: ESP32.
- La ESP32 se usa como interfaz/control remoto.
- Ha trabajado con pantalla y control remoto.
- La ESP32 se comunica con la Portenta para enviar comandos o datos de control.

### Visión artificial futura

- Se planea usar cámara o HuskyLens 2 para detección de objetos.
- La cámara detectará piezas, figuras o colores.
- El sistema deberá convertir la detección visual en una posición objetivo para que el brazo se mueva.

No implementar cambios de visión artificial todavía si el usuario no lo pide explícitamente.

---

## Estado actual del brazo cartesiano

El sistema es un brazo cartesiano con ejes X, Y y Z.

### Eje X

- Ya existe implementación de movimiento.
- Ya existe calibración previa.
- Rango físico de referencia: aproximadamente 496 mm.
- El sistema debe trabajar preferiblemente en milímetros para posiciones.

### Eje Y

- Ya existe implementación de movimiento.
- Ya existe calibración previa.
- Rango físico de referencia: aproximadamente 337 mm.
- El sistema debe trabajar preferiblemente en milímetros para posiciones.

### Eje Z

El eje Z está en proceso de implementación.

Ya se agregaron dos finales de carrera para el eje Z:

- Entrada digital 4: final de carrera superior del eje Z.
- Entrada digital 5: final de carrera inferior del eje Z.

Requisitos para eje Z:

- En modo manual, el eje Z debe detenerse si se activa un final de carrera en la dirección correspondiente.
- En modo calibración, debe implementarse la lógica de calibración del eje Z usando sus dos finales de carrera.
- No modificar otros mecanismos pendientes si el usuario indica que todavía no se deben tocar.
- No asumir que el eje Z ya está completamente calibrado.
- Si se trabaja en posiciones XYZ, puede aceptarse una posición Z como entrada, pero si Z aún no está listo, debe evitarse moverlo físicamente hasta que el usuario confirme.

---

## Modos de operación esperados

### Modo manual

El usuario puede mover los ejes manualmente.

Reglas importantes:
- Si un final de carrera se activa, el movimiento debe detenerse para evitar daño mecánico.
- La detención debe aplicarse solo al movimiento que empuja contra el límite.
- Si el eje está en un límite, debe permitirse moverse en la dirección contraria para salir del límite.
- No bloquear todo el sistema innecesariamente si solo un eje llegó a un límite.
- Evitar movimientos simultáneos inseguros.

### Modo calibración

El sistema debe buscar los finales de carrera para establecer referencias.

Reglas importantes:
- Calibrar de forma controlada y a baja velocidad.
- Detectar finales de carrera.
- Detener el motor al tocar el límite.
- Establecer posición de referencia.
- Si se calcula rango físico, mantener X = 496 mm e Y = 337 mm como referencias conocidas.
- Para Z, usar los finales de carrera en D4 y D5.
- No asumir dimensiones físicas de Z si no están definidas. Preguntar o dejar constante editable.

### Modo posición XYZ

Existe o se está implementando un menú donde el usuario puede enviar una posición XYZ por terminal o interfaz.

Requisitos:
- El usuario puede ingresar posiciones en milímetros.
- El sistema debe convertir milímetros a pasos, pulsos o unidades internas.
- X debe escalarse con referencia a 496 mm.
- Y debe escalarse con referencia a 337 mm.
- Z debe manejarse con cuidado si aún no está calibrado.
- Si Z todavía no está implementado completamente, mover solo XY y reportar que Z fue recibido pero no ejecutado.

### Cinemática inversa

Para un brazo cartesiano, la cinemática inversa es directa en concepto:
- La posición deseada del efector final en X, Y, Z corresponde directamente a la posición objetivo de cada eje.
- Lo importante es convertir coordenadas físicas en milímetros a las unidades internas de movimiento.

No complicar la cinemática con modelos de brazo articulado. Este proyecto es cartesiano.

---

## Comunicación Portenta - ESP32

El sistema puede tener dos códigos separados:

1. Código de la Portenta H7:
   - Control de movimiento.
   - Motores.
   - Finales de carrera.
   - Calibración.
   - Recepción de comandos.
   - Ejecución de posiciones.

2. Código de la ESP32:
   - Interfaz de usuario.
   - Pantalla.
   - Control remoto.
   - Envío de comandos hacia la Portenta.
   - Posible comunicación por UART/I2C u otro método existente en el código.

Codex debe identificar automáticamente, por los archivos del repositorio, qué código corresponde a cada placa.

No cambiar protocolos de comunicación sin justificarlo y sin confirmación.

---

## HuskyLens 2 / cámara

El proyecto ha trabajado con HuskyLens 2 para reconocimiento de objetos.

Contexto:
- Se han usado modelos entrenados.
- Se ha trabajado con datasets YOLO.
- Se han usado clases como pieza6 y pieza7.
- Se ha trabajado con Mind+ y exportación/deploy de modelos.
- Se ha probado comunicación I2C con ESP32.
- En ESP32 se han usado pines I2C personalizados como D27 y D26 en algunos ensayos.

Importante:
- No implementar integración de HuskyLens en el código principal del brazo salvo que el usuario lo pida.
- Si se analiza el proyecto, solo mencionar dónde podría integrarse en el futuro.
- Mantener separación entre control de movimiento y visión artificial.

---

## Seguridad del sistema

Este es un sistema físico con motores. Codex debe priorizar seguridad.

Reglas de seguridad:
- Nunca eliminar validaciones de finales de carrera.
- Nunca aumentar velocidades sin confirmación.
- Nunca desactivar límites de software o hardware sin una razón explícita.
- Toda función de movimiento debe tener alguna condición de parada.
- En calibración, usar velocidades bajas.
- En modo manual, impedir movimiento hacia un límite activo.
- Si hay duda sobre el estado de un final de carrera, asumir condición segura.
- No modificar pines sin confirmación.
- No invertir lógica de finales de carrera sin verificar cómo están cableados.

---

## Pines importantes conocidos

### Eje Z

- D4: final de carrera superior.
- D5: final de carrera inferior.

Codex debe buscar en el código cómo se nombran los pines y adaptar la implementación al estilo existente.

Si el código usa constantes, definir constantes para estos pines.
Si el código usa `#define`, seguir ese estilo.
Si usa `const int`, seguir ese estilo.
Si usa estructuras o clases, mantener el patrón actual.

---

## Preferencias de programación

El usuario prefiere cambios claros, directos y explicados.

Reglas:
- No borrar comentarios existentes.
- No reescribir todo el código si solo se necesita una modificación puntual.
- Mantener la estructura actual del proyecto.
- Mantener nombres de variables y estilo tanto como sea posible.
- Comentar cambios nuevos de forma breve y útil.
- Evitar cambios innecesarios de formato.
- Antes de cambiar código grande, explicar qué se va a tocar.
- Después de cambiar código, explicar:
  - Qué archivos se modificaron.
  - Qué funciones se modificaron.
  - Qué comportamiento cambió.
  - Cómo probarlo.
- Si hay ambigüedad, preguntar antes de modificar.

---

## Forma esperada de trabajo

Cuando el usuario pida una modificación:

1. Analizar el código relevante.
2. Identificar archivos y funciones afectadas.
3. Explicar el plan.
4. Hacer cambios mínimos.
5. Mostrar resumen de cambios.
6. Dar pasos de prueba.
7. Advertir si hay algo que debe verificarse físicamente.

---

## Pruebas esperadas

Cuando sea posible, Codex debe proponer pruebas de terminal o pruebas manuales.

Ejemplo para finales de carrera:
1. Encender sistema con motores deshabilitados si es posible.
2. Verificar lectura de D4 y D5 por Serial.
3. Presionar final superior y confirmar cambio de estado.
4. Presionar final inferior y confirmar cambio de estado.
5. Probar modo manual a baja velocidad.
6. Confirmar que el eje Z se detiene al tocar límite.
7. Confirmar que puede moverse en dirección contraria para salir del límite.
8. Probar calibración Z lentamente.

---

## Cosas que Codex NO debe hacer sin permiso

- No cambiar pines.
- No cambiar velocidades máximas.
- No cambiar protocolo entre Portenta y ESP32.
- No eliminar funciones existentes.
- No mezclar código de ESP32 dentro del código de Portenta.
- No activar movimiento Z si el usuario indicó que aún no se debe mover.
- No modificar integración de cámara/HuskyLens sin solicitud explícita.
- No tocar archivos fuera de la carpeta del proyecto.
- No asumir que todos los archivos están actualizados; debe revisar los archivos presentes.

---

## Objetivos próximos más probables

Los próximos trabajos sobre este proyecto probablemente serán:

1. Analizar estructura actual del código de Portenta y ESP32.
2. Documentar qué hace cada archivo.
3. Identificar dónde está el modo manual.
4. Identificar dónde está el modo calibración.
5. Identificar dónde se reciben comandos de posición XYZ.
6. Implementar finales de carrera del eje Z.
7. Implementar calibración Z.
8. Mejorar movimiento por posición en mm.
9. Preparar integración futura con visión artificial.
10. Mantener seguridad en movimientos físicos.

---

## Respuesta esperada de Codex

Codex debe responder en español técnico claro.

Debe evitar respuestas demasiado generales. Debe aterrizar sus explicaciones al código real.

Cuando analice, debe entregar algo como:

- Archivos encontrados.
- Función de cada archivo.
- Flujo general del programa.
- Dónde se controla cada modo.
- Dónde se controlan motores.
- Dónde se leen finales de carrera.
- Riesgos detectados.
- Próximos cambios recomendados.

Cuando modifique, debe entregar algo como:

- Archivos modificados.
- Resumen de cambios.
- Cómo probar.
- Advertencias de hardware.
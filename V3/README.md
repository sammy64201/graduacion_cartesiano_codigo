# Sistema de brazo cartesiano con visión artificial

## 1. Descripción general

Este repositorio contiene el firmware de un sistema mecatrónico compuesto por:

- Una **Arduino Portenta H7 con Machine Control**, encargada del movimiento del brazo cartesiano, lectura de finales de carrera, calibración de los ejes y ejecución de movimientos por coordenadas.
- Una **ESP32**, encargada del control Bluetooth, pantalla OLED, servos, comunicación con la Portenta y, después de esta integración, la cámara HUSKYLENS 2.
- Una **HUSKYLENS 2**, conectada a la ESP32 por UART, utilizada para calibrar el plano de trabajo mediante cuatro AprilTags y detectar piezas con un modelo personalizado.

El objetivo final es que el sistema pueda arrancar, comprobar sus componentes, calibrar la cámara y el brazo, esperar la conexión del control y permitir operación manual o automática.

Este proyecto forma parte de un trabajo de graduación de Ingeniería Mecatrónica.

---

## 2. Archivos principales

### `ESP_SLAVE.ino`

Firmware actual de la ESP32.

Funciones que ya trabajan y deben conservarse:

- ESP32 como esclavo I²C de la Portenta.
- Dirección I²C: `0x40`.
- Bus I²C hacia la Portenta:
  - SDA: GPIO27.
  - SCL: GPIO14.
  - Frecuencia: 100 kHz.
- Segundo bus I²C para la OLED SH1106:
  - SDA: GPIO21.
  - SCL: GPIO22.
  - Dirección: `0x3C`.
- Control Bluetooth mediante Bluepad32.
- Servo de rotación en GPIO25.
- Servo de pinza en GPIO26.
- Inicialización no bloqueante de la OLED.
- Envío de joystick, botones, estado Bluetooth y posiciones de servos a la Portenta.
- Recepción del estado de la Portenta para mostrarlo en la OLED.

La ESP32 debe seguir funcionando como **esclavo I²C**. No debe intentar iniciar transmisiones I²C espontáneas hacia la Portenta. La Portenta continuará solicitando los datos mediante `Wire.requestFrom()`.

### `MAster.ino`

Firmware actual de la Portenta H7 con Machine Control.

Funciones que ya trabajan y deben conservarse:

- Portenta como único maestro del bus I²C hacia la ESP32.
- Lectura de finales de carrera de X, Y y Z.
- Generación de pulsos para motores paso a paso.
- Movimiento manual con joystick.
- Calibración completa de X, Y y Z.
- Definición de HOME en el centro físico del recorrido.
- Movimiento cartesiano por coordenadas en milímetros.
- Escala automática de los ejes:
  - Recorrido X de referencia: 496 mm.
  - Recorrido Y de referencia: 337 mm.
- Comunicación de estado hacia la OLED por medio de la ESP32.
- Comandos de diagnóstico por terminal, incluyendo `GOTO`, `HOME`, `POS`, `RANGO`, `STOP` y `AYUDA`.

La Portenta debe continuar siendo el **coordinador general del sistema**, porque controla los motores y conoce el estado real del brazo.

### `Pasted code.cpp`

Algoritmo funcional e independiente de la HUSKYLENS 2.

Funciones actuales:

- Comunicación con HUSKYLENS 2 mediante UART2 de la ESP32.
- RX de la ESP32: GPIO32, conectado al TX de la HUSKYLENS.
- TX de la ESP32: GPIO33, conectado al RX de la HUSKYLENS.
- Velocidad UART: 115200 baudios.
- Calibración con AprilTags 0, 1, 2 y 3.
- Promedio de 25 muestras por tag.
- Cálculo de homografía de píxeles a milímetros.
- Apertura de un modelo personalizado de reconocimiento de piezas.
- Conversión de la detección a coordenadas físicas X,Y.
- Validación de que la pieza se encuentre dentro del área calibrada y sobre la banda blanca.

Datos físicos actuales:

- Ancho de la banda blanca: 292 mm.
- Ancho total incluyendo aluminio: 412 mm.
- Distancia entre centros de las filas de tags: 382 mm.
- Posición lateral estimada de los centros de tags: ±176 mm respecto al centro.
- Modelo personalizado seleccionado: índice 1.

Este algoritmo funciona actualmente. Su matemática de calibración, cálculo de homografía y conversión píxel-milímetro no debe modificarse sin una razón técnica documentada.

---

## 3. Restricción principal de esta integración

La cámara debe mantenerse conectada a la ESP32 mediante **UART**.

No cambiar la cámara a I²C.

La integración debe incorporar el algoritmo de `Pasted code.cpp` dentro del firmware de la ESP32 sin romper:

- Bluepad32.
- La comunicación I²C con la Portenta.
- La pantalla OLED.
- Los servos.
- La calibración y movimiento existentes de la Portenta.

La cámara no debe bloquear el programa si está desconectada, tarda en responder o está cargando un algoritmo.

No deben existir ciclos como:

```cpp
while (!huskylens.begin(...)) {
    delay(...);
}
```

ni esperas largas con `delay()` dentro del flujo principal. La conexión, carga del algoritmo, calibración y detección deben manejarse con una máquina de estados y temporizadores basados en `millis()`.

Los callbacks I²C de la ESP32 deben ser cortos. Dentro de `requestEvent()` y `receiveEvent()` no se debe:

- Consultar la cámara.
- Imprimir por Serial.
- Usar `delay()`.
- Ejecutar cálculos pesados.
- Cambiar de algoritmo de la HUSKYLENS.

---

## 4. Arquitectura deseada

### Responsabilidad de la Portenta

- Coordinar la secuencia de arranque.
- Verificar la comunicación I²C.
- Ordenar la calibración de cámara.
- Ejecutar la calibración del brazo.
- Esperar la conexión del control.
- Ejecutar el checklist final.
- Mostrar y administrar el menú principal.
- Validar coordenadas y límites.
- Ejecutar los movimientos X,Y.
- Detener los motores ante timeout, error o pérdida de comunicación.

### Responsabilidad de la ESP32

- Mantener Bluetooth activo.
- Mantener la OLED activa.
- Controlar los servos.
- Mantenerse como esclavo I²C.
- Comunicarse con HUSKYLENS 2 por UART2.
- Calibrar la cámara cuando la Portenta lo ordene.
- Abrir el modelo personalizado después de calibrar.
- Detectar piezas y calcular X,Y en milímetros.
- Publicar estado, errores y nuevas coordenadas en el paquete I²C solicitado por la Portenta.

---

## 5. Secuencia automática de arranque requerida

Al encender el sistema:

1. Inicializar todos los subsistemas en estado seguro y mantener los motores detenidos.
2. La ESP32 debe iniciar Bluetooth, servos, ambos buses I²C y UART de la cámara sin bloquear el arranque.
3. La Portenta debe verificar que la ESP32 responda por I²C.
4. Cuando exista comunicación I²C válida, iniciar una espera de 5 segundos.
5. Después de esos 5 segundos, la Portenta debe ordenar a la ESP32 iniciar la calibración de la cámara.
6. La ESP32 debe:
   - Conectarse o reconectarse a HUSKYLENS.
   - Abrir Tag Recognition.
   - Tomar las muestras de los cuatro tags.
   - Calcular la homografía.
   - Abrir el modelo personalizado.
   - Informar que la cámara quedó lista para detectar piezas.
7. Cuando la Portenta reciba confirmación de cámara lista, debe iniciar automáticamente la calibración del brazo X/Y/Z.
8. La calibración del brazo debe funcionar aunque el control Bluetooth todavía no esté conectado.
9. Al finalizar la calibración, el brazo debe llegar a HOME.
10. El sistema debe esperar la conexión del control Bluetooth.
11. Cuando el control se conecte, ejecutar un checklist final.
12. Si todo está correcto, mostrar el menú principal.
13. Si ocurre un error crítico, mantener los motores detenidos, mostrar el error y permitir reintento seguro.

La espera de 5 segundos debe comenzar después de confirmar comunicación I²C válida, no simplemente desde el encendido si todavía no existe comunicación.

---

## 6. Checklist final requerido

Antes de habilitar el menú, verificar como mínimo:

- Comunicación I²C Portenta–ESP32 activa.
- Paquetes I²C con versión, longitud y checksum válidos.
- HUSKYLENS conectada por UART.
- Homografía válida.
- Modelo personalizado abierto.
- Calibración X/Y válida.
- Calibración Z válida.
- Brazo detenido y en HOME.
- Ausencia de error de calibración.
- Control Bluetooth conectado.
- Finales de carrera disponibles y sin condición incoherente.

El checklist debe reportar claramente qué elemento falla.

---

## 7. Menú principal requerido

El menú final debe tener cuatro opciones:

1. **Modo manual**
2. **Modo automático**
3. **Calibración de brazo**
4. **Calibración de cámara**

### Opción 1: modo manual

Debe conservar el comportamiento actual de joystick, finales de carrera, servos y retorno al menú.

### Opción 2: modo automático

Debe:

- Requerir cámara y brazo calibrados.
- Mantener activo el modelo de piezas.
- Esperar una detección estable.
- Recibir de la ESP32 una coordenada X,Y en milímetros.
- Validar que la posición sea alcanzable.
- Transformar la coordenada de cámara al sistema del brazo mediante parámetros configurables.
- Mover solamente X,Y.
- No ordenar movimiento automático de Z.
- Detenerse al alcanzar la posición.
- No ejecutar agarre, descenso, retorno automático ni secuencia de pinza por ahora.
- Evitar enviar repetidamente la misma pieza como un objetivo nuevo.
- Permitir cancelar y volver al menú de forma segura.

### Opción 3: calibración de brazo

Debe iniciar nuevamente la calibración X/Y/Z y regresar al menú cuando termine o mostrar un error si falla.

### Opción 4: calibración de cámara

Debe ordenar a la ESP32 borrar la calibración anterior, repetir la captura de tags, recalcular la homografía, abrir de nuevo el modelo y regresar al menú cuando quede lista.

---

## 8. Coordenadas y transformación cámara–brazo

La cámara utiliza actualmente:

- Origen en el centro geométrico de los cuatro tags.
- X positivo hacia la derecha de la imagen.
- Y positivo hacia abajo de la imagen.

El brazo utiliza:

- HOME como X=0, Y=0, Z=0.
- HOME de X/Y en el centro físico del recorrido calibrado.

Aunque ambos orígenes deberían aproximarse, se deben agregar parámetros configurables, por ejemplo:

```cpp
constexpr bool CAMERA_SWAP_XY = false;
constexpr int8_t CAMERA_SIGN_X = 1;
constexpr int8_t CAMERA_SIGN_Y = 1;
constexpr float CAMERA_OFFSET_X_MM = 0.0f;
constexpr float CAMERA_OFFSET_Y_MM = 0.0f;
```

La transformación debe estar aislada en una función fácil de ajustar. Toda coordenada debe validarse contra el rango real y el margen de seguridad antes de mover motores.

---

## 9. Comunicación I²C requerida

La Portenta sigue siendo maestra y la ESP32 sigue siendo esclava.

Se debe ampliar el protocolo actual para incluir, como mínimo:

### ESP32 → Portenta

- Joystick X/Y/Z.
- Estado Bluetooth.
- Botones.
- Ángulos de servos.
- Estado de cámara.
- Código de error de cámara.
- Indicador de homografía válida.
- Indicador de modelo listo.
- Indicador de nueva pieza válida.
- ID o clase de la pieza.
- Coordenada X en milímetros escalada como entero.
- Coordenada Y en milímetros escalada como entero.
- Número de secuencia del objetivo.
- `magic`, versión y checksum.

Las coordenadas pueden enviarse como `int16_t` multiplicadas por 10:

- 23.4 mm → 234.
- -15.8 mm → -158.

No enviar `float` directamente por I²C.

### Portenta → ESP32

- Estado general del sistema.
- Opción de menú.
- Estado de calibración del brazo.
- Comando para la cámara.
- Indicador de modo automático activo.
- Indicador de brazo ocupado.
- Confirmación o número de secuencia del objetivo recibido.
- Estado necesario para la OLED.
- `magic`, versión y checksum.

Mantener cada paquete dentro del límite real del buffer de `Wire`. Usar `static_assert` para verificar el tamaño. Si el paquete Portenta → ESP32 llega a 32 bytes, no agregar más campos sin rediseñarlo.

Las estructuras deben ser idénticas en ambos firmwares. Es preferible definirlas en un archivo común `ProtocoloI2C.h` si la estructura del proyecto permite compilarlo correctamente. Si no es posible compartir físicamente el archivo entre los dos proyectos de Arduino, mantener copias claramente marcadas y comprobar con `static_assert` que sus tamaños coincidan.

---

## 10. Estados sugeridos para la cámara

La implementación puede usar nombres equivalentes, pero debe distinguir estados como:

```text
CAMERA_OFFLINE
CAMERA_CONNECTING
CAMERA_STANDBY
CAMERA_OPENING_TAGS
CAMERA_CALIBRATING
CAMERA_CALCULATING
CAMERA_OPENING_MODEL
CAMERA_READY
CAMERA_ERROR
```

Comandos sugeridos:

```text
CAM_CMD_NONE
CAM_CMD_STANDBY
CAM_CMD_CALIBRATE
CAM_CMD_OPEN_MODEL
CAM_CMD_RESET_ERROR
```

La ESP32 debe intentar reconectar la cámara periódicamente sin bloquear sus demás tareas.

---

## 11. Filtrado de detecciones para modo automático

Una sola lectura no debe iniciar un movimiento.

Implementar un filtro simple y configurable, por ejemplo:

- Exigir entre 3 y 5 detecciones consecutivas de la misma pieza.
- Exigir que la variación X,Y permanezca dentro de una tolerancia aproximada de 3 a 5 mm.
- Aceptar solamente piezas dentro del área calibrada y sobre la banda blanca.
- Crear un nuevo número de secuencia cuando la detección quede estable.
- Mantener el objetivo latched hasta que la Portenta lo confirme.
- No publicar otro objetivo mientras el brazo esté ocupado.
- Después de llegar al objetivo, exigir que la pieza desaparezca durante un tiempo configurable antes de rearmar otra detección.

Todos estos parámetros deben quedar concentrados en constantes fáciles de modificar.

---

## 12. Seguridad y manejo de errores

- Todos los motores deben detenerse ante pérdida de comunicación I²C.
- No mover el brazo si la calibración correspondiente no es válida.
- No usar un objetivo viejo después de reconectar la ESP32.
- No aceptar coordenadas con checksum incorrecto.
- No aceptar coordenadas fuera del espacio de trabajo.
- No mover Z en modo automático.
- No ejecutar movimientos mientras la cámara se está calibrando.
- Un error de cámara no debe congelar Bluetooth, OLED o I²C.
- Un error de OLED no debe detener cámara, Bluetooth o comunicación con Portenta.
- Evitar `delay()` largos; se permite únicamente una cesión breve como `delay(1)` si es necesaria para Bluepad32.
- Los mensajes de diagnóstico deben incluir prefijos claros, por ejemplo `[BOOT]`, `[I2C]`, `[CAM]`, `[CAL]`, `[AUTO]` y `[ERROR]`.

---

## 13. Restricciones actuales del prototipo

- El eje Z existe y se calibra, pero **no debe moverse automáticamente** hacia la pieza.
- Todavía no está instalada la herramienta o garra final.
- No implementar secuencia de agarre.
- No implementar retorno automático a HOME después de detectar una pieza, salvo que se solicite posteriormente.
- No cambiar la cámara de UART a I²C.
- No reemplazar la arquitectura Portenta maestra / ESP32 esclava.
- No eliminar los comandos de terminal existentes.
- No cambiar pines, escalas, sentidos de motores o lógica de finales de carrera sin justificarlo y documentarlo.

---

## 14. Criterios de aceptación

La integración se considera correcta cuando:

1. La ESP32 arranca aunque la HUSKYLENS esté desconectada.
2. Bluetooth, OLED, servos e I²C continúan funcionando durante intentos de conexión a la cámara.
3. La Portenta detecta a la ESP32 por I²C.
4. Después de 5 segundos desde la conexión I²C válida, comienza la calibración de cámara.
5. La cámara termina la homografía y abre el modelo.
6. La Portenta inicia automáticamente la calibración X/Y/Z sin requerir control conectado.
7. El brazo llega a HOME.
8. El sistema espera el control Bluetooth.
9. El checklist final permite entrar al menú de cuatro opciones.
10. El modo manual conserva su funcionamiento actual.
11. La calibración de brazo puede ejecutarse desde la opción 3.
12. La calibración de cámara puede ejecutarse desde la opción 4.
13. El modo automático acepta una detección estable y mueve únicamente X,Y.
14. La misma pieza no genera órdenes repetitivas continuamente.
15. Cualquier timeout o error crítico detiene los motores y muestra un diagnóstico.
16. Ambos firmwares usan exactamente la misma versión del protocolo I²C.
17. No existen esperas bloqueantes prolongadas en el flujo principal de la ESP32.

---

## 15. Forma recomendada de trabajar

Antes de modificar código:

1. Leer completos los tres archivos.
2. Identificar todas las funciones, estados, paquetes y dependencias.
3. Crear una copia de respaldo si no existe control de versiones.
4. Documentar un plan de integración.
5. Hacer cambios por etapas pequeñas.
6. Mantener los códigos compilables después de cada etapa.
7. Entregar un resumen de cambios, supuestos, pruebas y parámetros pendientes de ajuste físico.

No entregar pseudocódigo como resultado final. Los archivos finales deben contener implementaciones completas y coherentes.

# Prueba aislada RS485: Portenta Machine Control y ESP32

**Diagnostico anterior.** Estos sketches usan RX27/TX14 y protocolo de
prueba 2. Desde el 2026-10-08, el banco final y la variante completa usan
RX14/TX27, conforme a `../nuevo test funcional/`. Para verificar la migracion
vigente usar `../RS485_FINAL_MIGRACION/README.md` (aplicacion v17 real).

Solo comprueba comunicacion bidireccional. Cargar estos sketches sustituye
temporalmente el firmware de cada placa. No inicializan servos, camara, OLED,
Bluetooth ni el ciclo del brazo. No ordenan movimiento ni leen el encoder.

- `ESP/ESP.ino`: recibe los datos de Portenta, los confirma y, al recibir su
  turno, envia un mensaje propio con secuencia independiente y su tiempo local.
- `PORTENTA/PORTENTA.ino`: envia sus propios datos, exige confirmacion y cede
  el turno a la ESP; recibe los datos de la ESP y los confirma.
- Bus RS485 y ambos monitores seriales: **115200 baudios, 8N1**.
- Paquetes de **32 bytes**, patron de prueba y CRC16-CCITT-FALSE.
- Protocolo exclusivo de diagnostico, **version 2**, firma `R485`. No es el
  protocolo 16 de la maqueta ni contiene sus ordenes. Ambos `PruebaRS485.h`
  deben permanecer identicos.

Actualizacion: la version 1 hacia PING/PONG. La version 2 comprueba mensajes
propios de ambas placas, con ACK separado para cada origen. Cargar **ambos**
sketches actualizados: mezclar versiones se rechaza. RS485 de dos hilos es
bidireccional por turnos, no transmite simultaneamente en ambos sentidos.
Portenta coordina el bus, como coordinaba el enlace I2C.

Cada ciclo, iniciado una vez por segundo, intercambia cinco paquetes:

```text
PORTENTA -- DATOS_PORTENTA: secuencia P, reloj P --> ESP
PORTENTA <-- ACK_ESP: confirma secuencia y valor -- ESP
PORTENTA -- TURNO_ESP: permiso para transmitir ---> ESP
PORTENTA <-- DATOS_ESP: secuencia E, reloj E ------ ESP
PORTENTA -- ACK_PORTENTA: confirma datos ESP ----> ESP
```

Los mensajes propios llevan `millis()` de la placa emisora, no datos de motores
o sensores. Los relojes de las placas son independientes. Cada ACK debe
coincidir en secuencia, valor y turno y llegar dentro de **500 ms**.
Un mensaje ESP recibido fuera de plazo se descarta en Portenta. Tras un timeout, Portenta
inicia otro ciclo; la ESP deja de esperar el ACK al vencer su plazo o al
recibir un nuevo ciclo. Cada cambio de emisor tiene un margen de 2 ms.

## Conexion ESP32

Se requiere un transceptor UART-RS485 compatible con logica de **3.3 V**.
GPIO27/14 son UART hacia el modulo; nunca se conectan directamente a A/B.

| ESP32 | Modulo RS485 |
|---|---|
| GPIO27 (antiguo SDA) | RO, salida del receptor |
| GPIO14 (antiguo SCL) | DI, entrada del transmisor |
| GPIO18 | DE y /RE unidos, si la direccion es manual |
| GND | GND del lado logico |

La prueba usa UART2. No inicia el I2C antiguo. Retirar sus conexiones a la
Portenta y sus pull-ups externas antes de reutilizar GPIO27/14.

Por defecto, `DIRECCION_MANUAL = true` en `ESP.ino`: unir DE y /RE a GPIO18.
LOW recibe; HIGH transmite. Para un modulo con direccion automatica, cambiar
la constante a `false` y dejar GPIO18 sin conectar. Confirmar el modelo y el
pinout del modulo antes de alimentarlo. Alimentar segun su especificacion;
no asumir que cualquier modulo MAX485 de 5 V tiene salida RO de 3.3 V.

### Modulo mostrado por el usuario: MAX485 con RO/RE/DE/DI

Cableado vigente con el convertidor de nivel A/B utilizado en la prueba que
funciono. A es el lado de 3.3 V y B el lado de 5 V del convertidor de nivel;
estos canales no son los bornes A/B del MAX485:

| ESP / convertidor de nivel | MAX485 / destino |
|---|---|
| GPIO14 -> A4; B4 -> | DI (TX de la ESP) |
| GPIO27 <- A5; B5 <- | RO (RX de la ESP) |
| GPIO18 -> A2; B2 -> | DE y /RE puenteados |
| ESP 3.3 V | VA/VCCA del convertidor de nivel |
| Fuente regulada de 5 V | VB/VCCB del convertidor y VCC del MAX485 |
| ESP 3.3 V, si el convertidor es TXS0108E | OE del convertidor |
| GND ESP y negativo de la fuente de 5 V | GND convertidor, GND MAX485 y GND de comunicaciones de Machine Control |
| MAX485 A | Machine Control RS485 TX P |
| MAX485 B | Machine Control RS485 TX N |

Machine Control RX P y RX N quedan libres. GPIO18 es una senal de direccion;
no conectarlo a la salida de un regulador de alimentacion. Retirar las
conexiones y pull-ups del antiguo I2C externo en GPIO14/27. Cablear sin
alimentacion. El OLED conserva su cableado I2C GPIO21/22, pero esta prueba
aislada no lo inicializa.

Revision del 2026-10-08: se encontraron RX/TX invertidos en ESP.ino y se
restauro RX=27 / TX=14. Los dos sketches y cabeceras se compararon con los
fuentes guardados de la compilacion bidireccional v2 del 2026-10-07; no se
encontraron otros cambios de contenido. Se conserva la prueba bidireccional
que el usuario verifico, con direccion manual GPIO18 y 115200 baudios en
ambos monitores y en el bus. El estado anterior a restaurar se respaldo en
`../../tmp/rs485-prueba/restauracion-20261008/antes/`.

Verificacion de la restauracion: ambos sketches compilaron el 2026-10-08
con `esp32-bluepad32:esp32:esp32` y `arduino:mbed_portenta:envie_m7`.
Portenta conserva advertencias de las bibliotecas Arduino_MachineControl.
Los logs estan en `../../tmp/rs485-prueba/restauracion-20261008/`.
Se verifico igualdad byte a byte de las dos cabeceras y coincidencia del
contenido de los sketches con la compilacion anterior, excluyendo solamente
las directivas/prototipos generados por Arduino. No se cargaron placas ni se
repitio fisicamente la prueba sobre la placa perforada.

Las fotos aportadas el 2026-10-07 corresponden a un modulo anunciado como
MAX485, con direccion manual. Mantener `DIRECCION_MANUAL = true`.
Para un MAX485 genuino, VCC es **5 V regulados**, no 3.3 V ni 24 V.
Su salida RO puede alcanzar el nivel de 5 V: la conexion directa de la tabla
anterior solo aplica a transceptores con RO de 3.3 V, no a este MAX485.

| Conexion | Cableado para este modulo |
|---|---|
| GPIO14 | DI |
| GPIO18 | DE y RE unidos (RE es /RE, activo en LOW) |
| GPIO27 | RO mediante adaptador logico de 5 V a 3.3 V |
| VCC del modulo | Fuente regulada de 5 V |
| GND del modulo | GND ESP32 y GND de comunicaciones de Machine Control |
| A del modulo | RS485 TX P de Machine Control |
| B del modulo | RS485 TX N de Machine Control |

Un ejemplo de adaptacion de RO es un buffer **SN74LVC1G17**: VCC a 3.3 V,
GND a masa comun, entrada A a RO, salida Y a GPIO27 y condensador de 100 nF
entre VCC y GND cerca del buffer. Verificar los pines de su encapsulado.
DI, DE y /RE del MAX485 aceptan niveles TTL de 3.3 V; no unirlos a VCC de 5 V
porque el sketch debe controlar la direccion desde GPIO18.

Referencias:
[MAX485 y niveles TTL](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf),
[limites de entrada ESP32](https://documentation.espressif.com/esp32_datasheet_en.html),
[buffer 5 V a 3.3 V](https://www.ti.com/lit/ds/symlink/sn74lvc1g17.pdf).

La foto muestra R7 con marca `121`, indicio de terminacion de 120 ohm.
Comprobar que la unidad fisica la incorpora antes de agregar otra resistencia
entre A/B. En Machine Control la terminacion ya esta activada en el sketch.
Realizar el cableado sin alimentacion y retirar la antigua conexion I2C.

## Conexion Portenta Machine Control

Usar la interfaz integrada en **RS485 half duplex**, de dos hilos:

| Machine Control | Transceptor ESP32 |
|---|---|
| RS485 TX P (Data+) | Linea positiva del par RS485 |
| RS485 TX N (Data-) | Linea negativa del par RS485 |
| GND de comunicaciones | Referencia del lado bus, segun el modulo |

Los bornes **RX P y RX N quedan sin conectar** en half duplex: no se requieren
puentes a TX. Identificar la polaridad del modulo por su documentacion; las
etiquetas A/B no son uniformes entre fabricantes. En modulos no aislados,
compartir GND de ESP32, transceptor y comunicaciones de Machine Control.
En modulos aislados, respetar la separacion entre masa logica y masa del bus.

La terminacion interna de Machine Control esta activada mediante
`TERMINACION_120_OHM = true`. Usar una terminacion de 120 ohm en el extremo
del modulo ESP32; si ya la incorpora, no agregar otra en paralelo. Conectar
el par con cable trenzado y alimentar Machine Control segun su instalacion
normal. La alimentacion de potencia de los actuadores puede permanecer
desconectada durante esta prueba.

Referencia de bornes y terminaciones:
[pinout oficial, pagina 2](https://docs.arduino.cc/resources/pinouts/AKX00032-full-pinout.pdf).
La biblioteca utilizada es `Arduino_MachineControl` 1.1.2, la misma instalada
para el proyecto, con `ArduinoRS485` 1.1.1.

## Ejecutar e interpretar

Si el montaje sobre placa perforada falla, usar primero el diagnostico
separado de [canales GPIO/convertidor](../RS485_CANALES/README.md).
Ese sketch se carga solo en la ESP y exige desconectar las senales del
MAX485. No comunica con Portenta; al terminar, quitar jumpers y cargar
nuevamente ambos sketches de esta prueba.

1. Abrir `ESP/ESP.ino` y seleccionar la misma placa ESP32 del proyecto.
2. Abrir `PORTENTA/PORTENTA.ino` y seleccionar **Portenta H7 (M7)**.
3. Revisar el modo del transceptor y realizar el cableado con alimentacion apagada.
4. Cargar los dos sketches de prueba y abrir ambos monitores a 115200 baudios.
5. Observar al menos 100 ciclos confirmados en ambas vias despues de arrancar
   ambas placas. No enviar texto manual desde los monitores al bus RS485.

Ejemplo ilustrativo (el tiempo real puede variar):

```text
[RS485 PORTENTA] TX PORTENTA->ESP seq=1 portenta_ms=3500
[RS485 ESP] RX PORTENTA->ESP OK seq=1 portenta_ms=3500 ACK enviado
[RS485 PORTENTA] TX PORTENTA->ESP CONFIRMADO seq=1 ida_vuelta_ms=6
[RS485 ESP] TX ESP->PORTENTA seq=1 esp_ms=1620
[RS485 PORTENTA] RX ESP->PORTENTA OK seq=1 esp_ms=1620 ACK enviado
[RS485 ESP] TX ESP->PORTENTA CONFIRMADO seq=1 ida_vuelta_ms=6
```

Cada 10 segundos aparecen contadores:

| Monitor | Contador | Significado |
|---|---|---|
| Portenta | `enviados` | Mensajes propios enviados a la ESP |
| Portenta | `confirmado_portenta` | La ESP confirmo exactamente esos datos |
| Portenta | `rx_esp` | Mensajes propios de la ESP recibidos y validados |
| Portenta | `timeout_portenta` | Falto el ACK de la ESP |
| Portenta | `timeout_turno_esp` | Falto el mensaje propio de la ESP |
| ESP | `rx_portenta` | Mensajes de Portenta recibidos y validados |
| ESP | `tx_esp` | Mensajes propios enviados a Portenta |
| ESP | `confirmado_esp` | Portenta confirmo exactamente esos datos |
| ESP | `timeout_esp` | Falto el ACK de Portenta |
| Ambas | `rx_bytes` | Bytes recibidos, incluso si no forman paquetes validos |

`CONFIRMADO` exige CRC, patron y coincidencia exacta de secuencia, valor y turno.
`TIMEOUT` indica que no llego el paquete esperado dentro de 500 ms.
`invalidos` cuenta paquetes con firma reconocible pero integridad/version/patron
incorrectos; ruido sin firma puede aparecer solamente como timeout.
`inesperados` cuenta tipos/secuencias/valores/turnos incorrectos o respuestas
fuera de plazo.
Los contadores comienzan de cero al reiniciar cada placa.

Para la comprobacion inicial, buscar **100 confirmaciones consecutivas de cada
origen**, sin incrementos de timeouts, invalidos ni inesperados despues de
conectar ambas placas. Deben crecer `confirmado_portenta` en Portenta y
`confirmado_esp` en la ESP. Los contadores pueden diferir brevemente mientras
un ciclo esta en curso o porque los resumenes se imprimen en instantes distintos.
Los timeouts previos al arranque de la ESP no invalidan ese tramo posterior.
Si `rx_bytes=0`, revisar alimentacion, habilitacion del convertidor de nivel,
polaridad A->TX P / B->TX N, referencia de masa y GPIO RX/TX.
Si llegan bytes sin mensajes validos, revisar voltajes, cableado y que ambas
placas ejecuten version 2 a 115200. Si solo falla una confirmacion o el turno
ESP, revisar direccion DE y /RE y el correspondiente camino de retorno.

Opcionalmente, desconectar el par RS485 durante unos segundos y reconectarlo:
deben aparecer timeouts y despues volver a crecer ambas confirmaciones sin
reiniciar las placas. Esta comprobacion es manual; no fue ejecutada localmente.

Esta prueba comprueba el enlace con trafico ligero; no demuestra inmunidad al
ruido de motores/variador ni valida la futura migracion del ciclo automatico.
Para regresar a Automatico V2, restaurar el cableado I2C y cargar sus dos
sketches vigentes en `pruebas de automatico v2/`.

## Verificacion local

Verificacion del 2026-10-07:

- La version 1 compilo con `esp32-bluepad32:esp32:esp32`, core 4.1.0, y
  `arduino:mbed_portenta:envie_m7`, core 4.6.0. Es un resultado historico.
- **Version 2:** ambos sketches compilaron correctamente con esas mismas
  placas/cores y `--warnings more`. ESP: 712397 bytes de programa y 86820
  bytes de memoria global. Portenta: binario de aplicacion de 187520 bytes.
  No aparecieron advertencias ni errores en los logs de esta recompilacion.
- Las dos copias de `PruebaRS485.h` son identicas (SHA256 verificado).
- Las dos copias vigentes de `ProtocoloI2C.h` siguen identicas y con el mismo
  SHA256 previo a esta tarea; Automatico V2 sigue usando I2C/version 16.

Los logs vigentes estan en `../../tmp/rs485-prueba/compilacion-esp-v2.log`
y `../../tmp/rs485-prueba/compilacion-portenta-v2.log`.
La prueba fisica queda pendiente. No se cargaron placas ni se acciono hardware.

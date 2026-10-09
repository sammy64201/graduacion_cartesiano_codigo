# Diagnostico por canales: ESP, convertidor de nivel y MAX485

**Montaje anterior: TX14/RX27/DE18.** Las etiquetas A4/B4, A5/B5 y A2/B2
de este diagnostico corresponden a ese montaje. La referencia que funciono
posteriormente en `../nuevo test funcional/` usa **RX14/TX27/DE18**, con
RO/DI/direccion por canales 1/2/3. El banco `../RS485_FINAL_MIGRACION/` y
el firmware completo conservan esa asignacion posterior. No usar las
instrucciones de este diagnostico para definir el cableado vigente.

Esta prueba separada no reemplaza ni modifica la prueba bidireccional de
`../RS485_COMUNICACION/`. Se carga solamente `ESP/ESP.ino` en la ESP32.
Portenta queda desconectada del bus durante las primeras etapas. No inicia
UART RS485, motores, servos, camara, pantalla ni Bluetooth. No modifica los
protocolos de la maqueta.

El objetivo inicial es encontrar donde deja de propagarse un nivel estatico.
Un canal que pasa esta prueba todavia puede fallar con pulsos UART, ruido o
capacitancia. El monitor de la ESP debe estar a **115200 baudios**.

## 0. Preparar sin alimentacion

Desconectar USB y fuentes antes de modificar conexiones. Separar fisicamente
los tres cables entre convertidor y MAX485: B4-DI, B5-RO y B2-DE/RE. El puente
DE/RE puede quedar en el MAX485, que para las pruebas 1-3 permanece desconectado.
Desconectar A/B de Portenta. Las etiquetas A4/B4 y A5/B5 pertenecen al
convertidor de nivel, no a los bornes diferenciales A/B del MAX485.

Mantener en el convertidor:

| Conexion | Destino |
|---|---|
| ESP GPIO14 | A4 |
| ESP GPIO27 | A5 |
| ESP GPIO18 | A2 |
| ESP 3.3 V | VA/VCCA |
| Fuente regulada 5 V | VB/VCCB |
| ESP GND y negativo fuente 5 V | GND convertidor |
| ESP 3.3 V, si es TXS0108E | OE |

Comprobar continuidad GPIO14-A4, GPIO27-A5 y GPIO18-A2. Comprobar ausencia
de puentes de soldadura no previstos. No medir continuidad A4-B4 como si
fuera un cable: hay un traductor electronico entre los lados.

Cargar el sketch, encender y medir VA~3.3 V, VB~5 V y OE~3.3 V si es TXS0108E.
Si no coinciden, corregir alimentacion/habilitacion antes de seguir.

## 1. Canal de salida TX: GPIO14 -> A4 -> B4

Enviar `1` en el monitor. GPIO14 alterna LOW/HIGH cada cinco segundos;
GPIO18 permanece LOW y GPIO27 es entrada. Con punta negra en GND ESP:

| Estado impreso | GPIO14 | A4 | B4 |
|---|---:|---:|---:|
| LOW | ~0 V | ~0 V | ~0 V |
| HIGH | ~3.3 V | ~3.3 V | ~5 V |

`ORDENADO` es el nivel pedido por el programa, no una medicion de voltaje.
Si GPIO14 cambia y A4 no, revisar cable/soldadura. Si A4 cambia y B4 no,
revisar VA, VB, OE y el canal del convertidor. Si GPIO14 no cambia,
desconectar su cable de A4 sin alimentacion y repetir la medicion sobre la
ESP: eso distingue carga/corto externo de un problema del pin, carga del
sketch o identificacion del GPIO.

Enviar `0` al terminar.

## 2. Canal de direccion: GPIO18 -> A2 -> B2

Enviar `2`. Repetir la tabla anterior para GPIO18/A2/B2, con el mismo
criterio. GPIO14 permanece LOW. No reconectar B2 al MAX485 todavia.
Enviar `0` al terminar.

## 3. Canal de entrada RX: B5 -> A5 -> GPIO27

Verificar que **RO esta fisicamente desconectado de B5**. Enviar `3`.
GPIO27 es entrada sin pull-up. Usar un jumper para fijar B5 a GND o a los
5 V de VB; cambiar el jumper con alimentacion apagada y no puentear ambas
alimentaciones. No conectar 5 V a A5 ni a GPIO27.
Tras cada encendido, volver a enviar `3`: el programa arranca detenido.

| B5 fijado a | Voltaje A5 | Voltaje GPIO27 | Monitor |
|---|---:|---:|---|
| GND | ~0 V | ~0 V | D27 LEIDO=LOW |
| 5 V | ~3.3 V | ~3.3 V | D27 LEIDO=HIGH |

Si B5 cambia y A5 no, el fallo apunta a convertidor/alimentacion. Si A5
cambia y GPIO27 no, revisar continuidad. Si el voltaje en GPIO27 cambia
correctamente pero la lectura no, revisar pin/placa/sketch y posible dano
del GPIO. Una entrada flotante no da un resultado valido.

Enviar `0` y quitar el jumper de B5 antes de restaurar RO.

## 4. MAX485 separado: primero niveles estaticos

Solo despues de aprobar los canales anteriores. Para comprobar el modulo
sin UART ni Portenta, desconectarlo completamente de la ESP y convertidor.
Quitar el puente DE/RE, alimentar VCC con 5 V regulados y GND con el negativo
de la fuente. Conectar DE a 5 V y /RE a GND. DI se conecta mediante 1 kohm a
GND o a 5 V. A/B quedan desconectados de Portenta; no unirlos entre si.

| DI | A-B, punta roja A y negra B | RO respecto GND |
|---|---|---|
| GND | Negativo | LOW, cercano a 0 V |
| 5 V | Positivo | HIGH, normalmente cercano a 5 V |

No conectar RO de 5 V a la ESP. Un cambio correcto de A/B y RO comprueba
funcionamiento estatico basico, no inmunidad al ruido ni funcionamiento
UART. Si falla, confirmar los niveles en los propios pines DI, DE, /RE y
VCC y ausencia de cortos antes de concluir que el chip esta danado.
Apagar antes de quitar estas conexiones de prueba y restaurar DE/RE.

## 5. Volver al enlace bidireccional

Quitar TODOS los jumpers de prueba, sobre todo el de B5 a GND/5 V y los de
DE a 5 V y /RE a GND. Con todo apagado, restaurar:

| Trayecto | Conexion |
|---|---|
| TX | GPIO14 -> A4; B4 -> DI |
| RX | RO -> B5; A5 -> GPIO27 |
| Direccion | GPIO18 -> A2; B2 -> DE y /RE puenteados |
| Bus | MAX485 A -> Machine Control TX P; B -> TX N |
| Tierra | ESP, convertidor, MAX485, negativo fuente 5 V y GND comunicaciones de Machine Control |

VCC MAX485=5 V, VA=3.3 V, VB=5 V y OE=3.3 V si TXS0108E. RX P/N de
Machine Control quedan libres. Mantener desconectados los antiguos cables
y pull-ups I2C externos en GPIO14/27.

Cargar los DOS sketches de `../RS485_COMUNICACION/`, monitores 115200,
antes de probar UART. El sketch de canales no entiende paquetes RS485.
Si los niveles estaticos pasan pero la prueba bidireccional falla, revisar
soldaduras bajo carga, cables cortos, polaridad A/B, referencia comun y
terminaciones. Con osciloscopio se puede buscar donde se pierde la senal
entre GPIO14/A4/B4/DI o RO/B5/A5/GPIO27. Un multimetro no valida los pulsos
a 115200 baudios. Ver el README de la prueba bidireccional para interpretar
rx_bytes, invalidos y timeouts.

## Referencias y verificacion

- [TXS0108E: niveles y OE](https://www.ti.com/lit/ds/symlink/txs0108e.pdf).
- [MAX485: alimentacion, DI, DE, /RE y RO](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf).
- [Machine Control: bornes RS485](https://docs.arduino.cc/resources/pinouts/AKX00032-full-pinout.pdf).

Las instrucciones del convertidor con OE aplican a TXS0108E; confirmar el
modelo real. Las mediciones las realiza el usuario: el programa no mide
voltajes ni declara aprobado un canal por el nivel ordenado.
No se cargaron placas ni se ejecutaron pruebas fisicas desde Codex.

Verificacion local del 2026-10-08: el sketch ESP compilo correctamente con
`esp32-bluepad32:esp32:esp32`, core 4.1.0: 711681 bytes de programa y
86668 bytes de memoria global. Log: `../../tmp/rs485-canales/compilacion-esp.log`.
La nueva prueba solo tiene un sketch ESP y no requiere firmware Portenta.
La pareja de la prueba bidireccional fue compilada el mismo dia durante su
restauracion y no se modifico al agregar este diagnostico. La validacion
fisica de cada canal queda pendiente de las mediciones del usuario.

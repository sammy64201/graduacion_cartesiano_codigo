# Comunicacion bidireccional Nano + MAX485 y Portenta Machine Control

Esta es la prueba de **comunicacion real entre las dos placas**. La prueba
anterior `../RS485_NANO/` es solamente de niveles estaticos del modulo.
Se usan DOS sketches de esta carpeta: `NANO/NANO.ino` y
`PORTENTA/PORTENTA.ino`. No mezclarlos con los sketches ESP/Portenta de
115200 ni con el firmware completo de la maqueta.

Para **Nano clasico ATmega328P, logica de 5 V**. No aplicar este cableado
directo a Nano ESP32, Nano 33 ni otras placas de 3.3 V. El sketch exige
ATmega328P al compilar. Nano usa SoftwareSerial RX D2 / TX D3; USB usa
Serial y quedan libres D0/D1.

## Conexiones, con alimentacion apagada

Separar MAX485 de la ESP y del convertidor de nivel. Para esta prueba el
Nano se conecta directamente al MAX485. Quitar el cable D5-/RE usado en
la prueba estatica anterior. **Restaurar el puente DE-/RE**; ambos van a D4.
Quitar tambien cualquier jumper de DI/DE/RE/B5 a las alimentaciones.

| Nano | MAX485 |
|---|---|
| D2 (RX) | RO |
| D3 (TX) | DI |
| D4 (direccion) | DE y /RE unidos |
| 5 V | VCC |
| GND | GND |

Alimentar Nano por USB y usar su pin 5 V para el MAX485. No unir otra
salida de fuente de 5 V al pin 5 V del Nano al mismo tiempo.
La Portenta conserva su alimentacion habitual.

| MAX485 | Portenta Machine Control |
|---|---|
| A | RS485 TX P (Data+) |
| B | RS485 TX N (Data-) |
| GND | GND del bloque de comunicaciones |

Tierra comun: Nano, MAX485 y comunicaciones de Portenta. RX P y RX N de
Machine Control quedan sin conexion. El modulo incorpora su transceptor;
Machine Control usa el integrado de la placa. Un MAX485 externo es suficiente.
Usar par corto trenzado A/B. La terminacion de 120 ohm de Machine Control
esta habilitada; si el MAX485 ya incorpora 120 ohm (por ejemplo R7=121),
no agregar otra resistencia en paralelo en ese extremo.

## Velocidades y carga

| Interfaz | Velocidad |
|---|---|
| Bus RS485 de AMBAS placas | **9600 baudios, 8N1** |
| Monitor USB del Nano | **115200 baudios** |
| Monitor USB de Portenta | **115200 baudios** |

9600 es la velocidad de diagnostico elegida para SoftwareSerial en el
Nano clasico. No mide el funcionamiento del montaje ESP a 115200.
La escritura SoftwareSerial es sin buffer y retorna tras el bit de parada;
se mantiene DE hasta terminar la escritura y se deja margen antes de recibir.

1. Abrir NANO/NANO.ino; placa Arduino Nano, procesador ATmega328P.
   Algunos clones requieren ATmega328P (Old Bootloader) para cargar.
2. Abrir PORTENTA/PORTENTA.ino; placa Portenta H7 M7.
3. Cargar ambos sketches y abrir ambos monitores a 115200.
4. No enviar comandos desde los monitores: la prueba empieza automaticamente.

No inicializa motores, servos, OLED, camara, Bluetooth ni encoder.
Cargar estos sketches sustituye temporalmente los firmwares de las placas.

## Que se prueba

Cada segundo Portenta coordina cinco tramas de 32 bytes, con CRC16:

```text
PORTENTA -- datos propios --> NANO
PORTENTA <-- ACK ---------- NANO
PORTENTA -- turno ---------> NANO
PORTENTA <-- datos propios - NANO
PORTENTA -- ACK -----------> NANO
```

Cada placa envia su contador y reloj local; cada ACK debe coincidir en
secuencia, valor y turno y llegar dentro de 500 ms. El par es half duplex,
por turnos. Firma R485, protocolo de diagnostico **version 3**; las dos
copias de PruebaRS485.h deben ser identicas. No cambia el protocolo de
aplicacion de Automatico V2. Es incompatible con la prueba ESP version 2.

Ejemplo ilustrativo, no resultado de una prueba fisica:

```text
[RS485 PORTENTA] TX PORTENTA->NANO CONFIRMADO seq=1 ida_vuelta_ms=...
[RS485 PORTENTA] RX NANO->PORTENTA OK seq=1 nano_ms=... ACK enviado
[RS485 NANO] RX PORTENTA->NANO OK seq=1 portenta_ms=... ACK enviado
[RS485 NANO] TX NANO->PORTENTA CONFIRMADO seq=1 ida_vuelta_ms=...
```

Los resumenes cada 10 s muestran bytes recibidos, invalidos e inesperados;
Portenta imprime enviados, confirmado_portenta, rx_nano, timeout_portenta
y timeout_turno_nano. Nano imprime rx_portenta, tx_nano, confirmado_nano
y timeout_nano. Buscar al menos 100 ciclos consecutivos confirmados en
ambas vias, sin nuevos errores/timeouts despues de arrancar ambas placas.

Si Nano rx_bytes=0, revisar RO-D2, DE/RE-D4 en LOW cuando recibe, alimentacion,
tierra y A/B. Si Nano recibe pero Portenta no obtiene ACK, revisar D3-DI
y habilitacion DE al transmitir. Si llegan bytes sin tramas validas,
confirmar ambos sketches nuevos y bus 9600, y revisar ruido/continuidad.
La falta de comunicacion por si sola no demuestra que el MAX485 este quemado.

## Referencias

- [Nano clasico, logica 5 V](https://docs.arduino.cc/resources/datasheets/A000005-datasheet.pdf).
- [MAX485, DE-/RE y niveles](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf).
- [Bornes de Machine Control, pagina 2](https://docs.arduino.cc/resources/pinouts/AKX00032-full-pinout.pdf).

No se cargaron placas ni se acciono hardware desde Codex. La prueba fisica
de esta pareja queda pendiente del usuario.

## Verificacion local del 2026-10-08

Ambos sketches compilaron correctamente: Nano con Arduino AVR 1.8.8,
`arduino:avr:nano:cpu=atmega328`, y Portenta con mbed_portenta 4.6.0,
`arduino:mbed_portenta:envie_m7`. Nano: 5764 bytes de programa, 420 bytes
globales. Portenta conserva advertencias de Arduino_MachineControl.
Las dos cabeceras son identicas byte a byte, version 3 y paquetes de 32 bytes.
Logs en `../../tmp/rs485-nano-portenta/compilacion-nano.log` y
`../../tmp/rs485-nano-portenta/compilacion-portenta.log`.

La regresion `../rs485_nano_portenta_test.cpp` ejecuta ambos sketches reales
con UART/reloj simulados. Paso 100 ciclos confirmados en ambas vias, ACK
corrupto, ACK con secuencia ajena, perdida de ACK del Nano, perdida de datos
del Nano, perdida de ACK de Portenta, desconexion/reconexion y rechazo de
version 2 con CRC correcto. Tambien verifica el control de direccion y
configuracion half duplex. Log: `../../tmp/rs485-nano-portenta/regresion.log`.
Esta simulacion no reproduce interrupciones de SoftwareSerial, niveles,
ruido ni el montaje fisico. No confirma que el modulo real funcione.

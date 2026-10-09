# UART TTL Nano clasico y Portenta Machine Control

UART directa **sin MAX485 externo**. Se conserva la prueba RS485 separada
en `../RS485_NANO_PORTENTA/`; no mezclar sus conexiones ni sketches.

Requiere acceso fisico identificado a **TP78 y TP83** de Machine Control,
segun esquema oficial revision V1.1, hoja 7. Son puntos de prueba de la
placa, no bornes HMI. Confirmar revision/identificacion antes de cablear.
Si no tienes acceso a esos puntos, no puedes conectar esta prueba usando
solamente TX P/TX N: para esos bornes usar RS485 con transceptor.

## Conexion, con alimentacion apagada

Nano clasico ATmega328P de 5 V. Portenta usa TTL 3.3 V. Usar el convertidor
de nivel ya disponible, si es TXS0108E, con A a 3.3 V y B a 5 V:

| Origen | Convertidor | Destino |
|---|---|---|
| Nano D3 TX | B4 -> A4 | Machine Control TP83 RX / PI9 |
| Machine Control TP78 TX / PA0 | A5 -> B5 | Nano D2 RX |
| Machine Control 3V3 del Grove | VA/VCCA | Alimentacion lado A |
| Nano 5 V | VB/VCCB | Alimentacion lado B |
| Machine Control 3V3 | OE | Habilitacion, si TXS0108E |
| GND Nano y GND Machine Control | GND | Referencia comun |

No aplicar 5 V directamente a TP83. **No conectar a TX P, TX N, RX P ni
RX N de Machine Control.** Tampoco a GPIO industriales de 24 V. TP78/TP83
no son D14/D13 de la cabecera H7: este sketch usa UART4, no Serial1.

Desconectar MAX485 externo y sus cables del convertidor. Nano D4/D5 quedan
libres. Nano por USB y Portenta con su alimentacion habitual; separar sus
positivos. Confirmar VA~3.3 V, VB~5 V. Los puntos de prueba son pequenos;
no cablear hasta identificarlos ni permitir contacto con puntos vecinos.

El sketch Portenta configura PG9=LOW (SHDN), PA10=HIGH (modo RS485),
PI10=HIGH (/RE) y PI13=LOW (DE), deshabilitando el SP335 integrado. Usa
directamente `comm_protocols._UART4_` (PA0/PI9); su salida RO no debe
conducir el punto RX TTL. No inicializa servos, motores, camara ni OLED.

## Carga y prueba

1. Cargar `NANO/NANO.ino`: Nano, ATmega328P. Algunos clones requieren
   ATmega328P (Old Bootloader) para cargar.
2. Cargar `PORTENTA/PORTENTA.ino`: Portenta H7 M7.
3. Ambos monitores **115200 baudios**. UART entre placas **9600, 8N1**.
4. La prueba comienza automaticamente; no enviar comandos del monitor.

Cada segundo Portenta envia datos y exige ACK, cede turno al Nano, recibe
sus datos propios y los confirma. SoftwareSerial no recibe mientras
transmite, por eso se coordina por turnos aunque TX/RX son lineas separadas.
Secuencias independientes, reloj local, patron y CRC16; paquetes de 32 bytes.

Firma **UTTL**, protocolo de diagnostico **1**; las dos PruebaUART.h deben
ser identicas. Firma distinta de R485; no cambia el protocolo de la maqueta.

Ejemplo ilustrativo, no registro fisico:

```text
[UART PORTENTA] TX PORTENTA->NANO CONFIRMADO seq=1 ...
[UART PORTENTA] RX NANO->PORTENTA OK seq=1 ... ACK enviado
[UART NANO] RX PORTENTA->NANO OK seq=1 ... ACK enviado
[UART NANO] TX NANO->PORTENTA CONFIRMADO seq=1 ...
```

Buscar 100 ciclos consecutivos confirmados en ambas vias sin nuevos
timeouts/invalidos/inesperados despues de arrancar ambas placas. Resumenes
cada 10 s incluyen rx_bytes. ACK enviado por si solo no prueba recepcion
en la otra placa. Para regresar a RS485, retirar cables TTL, cargar los dos
sketches de `../RS485_NANO_PORTENTA/` y restaurar su MAX485 y conexiones.

## Referencias

- [Machine Control V1.1, hoja 7 TP78/TP83](https://content.arduino.cc/assets/AKX00032-schematics.pdf).
- [SP335: modo RS485, /RE y shutdown](https://www.maxlinear.com/ds/sp335e.pdf).
- [TXS0108E: VA/VB y OE](https://www.ti.com/lit/ds/symlink/txs0108e.pdf).

No se cargaron placas ni se hicieron pruebas fisicas desde Codex.

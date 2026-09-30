# Diagnostico HUSKYLENS 2

Este sketch aisla la HUSKYLENS 2 del resto del sistema. No usa Bluepad32,
OLED, servos ni el enlace I2C con la Portenta, por lo que permite saber si el
problema esta en la camara/UART o en la integracion completa.

## Conexion

Con el sistema sin alimentacion:

- TX de HUSKYLENS -> GPIO32 (RX) del ESP32.
- RX de HUSKYLENS <- GPIO33 (TX) del ESP32.
- GND de HUSKYLENS y GND del ESP32 en comun.
- Alimente la HUSKYLENS segun la especificacion de su modulo/fuente.
- En la HUSKYLENS seleccione protocolo `UART` y velocidad `115200`.

No conecte TX con TX ni RX con RX.

## Uso

1. Compile y cargue `HUSKYLENS2_DIAGNOSTICO.ino` en la ESP32.
2. Abra el monitor serie a 115200 baudios y final de linea opcional.
3. El arranque realiza un handshake una sola vez.
4. Envie `c` para reintentar la conexion.
5. Envie `t` para abrir Tag Recognition.
6. Envie `0` para abrir el modelo personalizado usado por el proyecto
   (ID 128), o `1`/`2` para los IDs 129/130.
7. Envie `p` para solicitar una lectura. Cero detecciones es una respuesta
   valida; no significa perdida de comunicacion.
8. Envie `a` para probar consecutivamente los tres modelos personalizados.

## Separar una falla de placa de una falla de camara

La orden `l` prueba fisicamente UART1 del ESP32:

1. Desconecte por completo la HUSKYLENS.
2. Envie `l` en el monitor serie.
3. Durante la espera de cinco segundos, una GPIO33 con GPIO32.
4. Un resultado `Loopback correcto` demuestra que UART2 y ambos pines pueden
   transmitir y recibir. Quite el puente antes de volver a conectar la camara.

Si el loopback pasa pero el handshake falla, concentre la revision en la
configuracion UART de la HUSKYLENS, los cables cruzados, GND y alimentacion. Si
el loopback falla aun con el puente bien hecho, pruebe otra pareja de GPIO o
otra ESP32 antes de concluir que la placa esta danada.

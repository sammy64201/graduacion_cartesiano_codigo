# Prueba MAX485 con Arduino Nano clasico

Para comunicar **Nano con Portenta**, usar la prueba separada
`../RS485_NANO_PORTENTA/README.md`. El sketch de esta carpeta es estatico.

Para Nano clasico **ATmega328P de 5 V**. No aplicar este cableado a Nano
ESP32, Nano 33 u otras placas de 3.3 V. El sketch rechaza una MCU distinta
de ATmega328P al compilar. No necesita librerias adicionales.

Es un diagnostico de niveles estaticos, no una prueba UART ni una pareja
compatible con el firmware Portenta. No inicia motores ni funciones del
brazo. Las pruebas anteriores ESP/Portenta quedan sin modificaciones.

## Conexion directa del MAX485

Con todo apagado, separar MAX485 de ESP, convertidor de nivel y Portenta.
Quitar el puente DE-/RE: aqui se controlan por separado. No conectar A/B
a Portenta ni unir A con B. Alimentar Nano por USB y usar su pin 5 V para
el modulo; no conectar a la vez otra salida de fuente al pin 5 V.

| Nano | MAX485 |
|---|---|
| 5 V | VCC |
| GND | GND |
| D2, entrada | RO |
| D3, salida | DI |
| D4, salida | DE |
| D5, salida | /RE (RE en la serigrafia) |

No se necesita convertidor para esta conexion con Nano clasico de 5 V.
D0/D1 quedan reservados para USB/monitor. No se usa SoftwareSerial.

## Cargar y probar

Abrir `NANO/NANO.ino`. En Arduino IDE seleccionar **Arduino Nano**,
procesador **ATmega328P**, puerto del Nano y cargar. Para algunos clones,
si falla la carga, seleccionar **ATmega328P (Old Bootloader)**; el codigo y
las conexiones son iguales. Monitor serial **115200 baudios**.
Arranca detenido; enviar `?` para ver el menu si no se vio al abrir el monitor.

1. Enviar `1`: activa transmisor y desactiva receptor. DI alterna LOW/HIGH
   cada cinco segundos. Medir DI respecto GND y A-B con roja A, negra B:

   | DI ordenado | DI medido | A-B esperado |
   |---|---:|---|
   | LOW | ~0 V | Negativo |
   | HIGH | ~5 V | Positivo |

   En este modo RO esta en alta impedancia; su lectura no seria valida.
   Si DI cambia pero A-B no, revisar DE~5 V, VCC y soldaduras/cortos antes
   de sospechar del driver. ORDENADO no es un voltaje medido por el Nano.

2. Enviar `2`: mantiene transmisor activo y habilita el receptor propio
   (DE=HIGH, /RE=LOW). El mismo cambio lento de DI debe leerse en RO:

   ```text
   [NANO] DI ORDENADO=LOW (~0 V); A-B esperado NEGATIVO; RO LEIDO=LOW; OK_ESTATICO
   [NANO] DI ORDENADO=HIGH (~5 V); A-B esperado POSITIVO; RO LEIDO=HIGH; OK_ESTATICO
   ```

   Se imprimen contadores de lecturas y fallos. Deben funcionar ambos niveles
   repetidamente. Si A-B invierte signo pero RO no cambia, revisar /RE~0 V,
   continuidad RO-D2 y voltaje real RO: el problema puede ser el receptor
   o la conexion al Nano. Una coincidencia aislada no aprueba el modulo.

3. Enviar `0` para detener: DE=LOW, /RE=HIGH y DI=LOW. Desconectar USB antes
   de cambiar cables. Al volver a la maqueta, quitar estos cuatro cables
   Nano, restaurar el puente DE-/RE y el convertidor de nivel de la ESP.

La prueba usa los propios A/B del transceptor como retorno, sin puentes
externos. Pasar la prueba confirma funcionamiento estatico basico; no
demuestra funcionamiento UART a 115200 ni inmunidad al ruido.
Un fallo tampoco prueba por si solo que el chip este quemado.

## Usar el Nano como generador para el convertidor de nivel

Es opcional y se realiza con ESP y MAX485 totalmente separados de las
senales del convertidor. Desconectar tambien los cables D2/D4/D5 del Nano.
Alimentar VA con 3.3 V verificados, VB con 5 V y compartir GND; OE a 3.3 V
si el convertidor es TXS0108E. Enviar `1` y usar solo D3 como generador 0/5 V:

| Nano D3 conectado a, uno por vez | Medir con multimetro |
|---|---|
| B4 | B4 alterna ~0/5 V; A4 debe alternar ~0/3.3 V |
| B2 | B2 alterna ~0/5 V; A2 debe alternar ~0/3.3 V |
| B5 | B5 alterna ~0/5 V; A5 debe alternar ~0/3.3 V |

Cambiar el cable con alimentacion apagada y volver a enviar `1` al encender.
No conectar una salida de 5 V del Nano a A2/A4/A5 ni a ningun GPIO de ESP.
Esto comprueba B->A con niveles lentos, no A->B a la velocidad de UART.
Para comprobar A->B con 3.3 V, usar la prueba ESP de `../RS485_CANALES/`.

## Referencias

- [Nano clasico: logica 5 V y pines](https://docs.arduino.cc/resources/datasheets/A000005-datasheet.pdf).
- [MAX485: DE, /RE, DI, RO y A/B](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf).
- [TXS0108E: VA, VB y OE](https://www.ti.com/lit/ds/symlink/txs0108e.pdf).

No se cargaron placas ni se hicieron mediciones fisicas desde Codex.

Verificacion local del 2026-10-08: compilo correctamente con
`arduino:avr:nano:cpu=atmega328`, core Arduino AVR 1.8.8 y warnings more.
Programa: 3684 bytes; variables globales: 202 bytes. Log:
`../../tmp/rs485-nano/compilacion-nano.log`.
El firmware de la maqueta y las pruebas ESP/Portenta no fueron modificados.
La prueba fisica del MAX485 sigue pendiente.

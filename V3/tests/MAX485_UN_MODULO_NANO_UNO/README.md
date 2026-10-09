# Prueba Nano + Uno con UN modulo MAX485

Prueba exclusiva de diagnostico, para Nano clasico y Uno R3 con ATmega328P de
5 V. No usa ESP, Portenta, pantalla, servos ni motores. No es compatible con
Nano 33, Nano ESP32 ni Uno R4 sin adaptar y verificar la prueba.

Con un solo MAX485 se prueba su transmisor y su receptor locales, activos
simultaneamente: `Nano D3 -> DI -> A/B internos -> RO -> Uno D2`.
El retorno `Uno D3 -> Nano D2` es UART TTL directo. Por eso una prueba correcta
NO valida un enlace RS485 entre dos extremos, el control de direccion DE/RE
ni la instalacion de la Machine Control. Para eso hacen falta dos transceptores
(la Machine Control ya integra uno; Nano y Uno R3 no).

## Conexiones para esta prueba

Cambiar cables con ambas placas desconectadas del USB. Desconectar fuente
externa, ESP, Portenta y convertidor de niveles de este montaje.

| Origen | Destino |
| --- | --- |
| Nano D3 (TX) | MAX485 DI |
| MAX485 RO | Uno D2 (RX) |
| Uno D3 (TX) | Nano D2 (RX), directo |
| Nano 5 V | MAX485 VCC y DE |
| Nano GND | MAX485 GND y /RE (puede aparecer como RE) |
| Uno GND | Nano GND |
| MAX485 A y B | Sin cables externos; no unirlos entre si |
| Nano y Uno D4 | Sin conexion; no se usan |

**Quitar el puente entre DE y RE.** DE a 5 V habilita el transmisor; RE a GND
habilita el receptor. Ambos comparten A/B dentro del integrado, sin un puente
externo. No poner A/B en pines del Arduino. La resistencia de terminacion que
traiga el modulo puede quedarse; esta prueba no requiere agregar otra.

Alimentar Nano y Uno por sus propios USB. El modulo toma 5 V solamente del
Nano. No unir los positivos 5 V de ambos Arduino ni usar el regulador externo
durante este diagnostico. No hace falta convertidor de nivel con estas placas
clasicas de 5 V. Los pines D0/D1 se reservan para los monitores USB.

## Cargar y observar

1. Abrir `NANO/NANO.ino` y seleccionar Arduino Nano, ATmega328P y su puerto.
   Si el Nano requiere Old Bootloader, elegirlo para la carga; el codigo es el
   mismo. Abrir `UNO/UNO.ino`, seleccionar Arduino Uno y el puerto del Uno.
2. Mantener `PruebaMAX485.h` en la carpeta de cada sketch. No mezclarlo con el
   header de pruebas anteriores. Ambas copias son identicas: firma M485,
   version diagnostica 4, CRC16 y paquetes de 32 bytes.
3. Cargar cada sketch en su placa. Abrir ambos monitores a **115200 baud**.
   Los pines D2/D3 se comunican a **9600 baud, 8N1**; el monitor USB usa otra
   velocidad. Despues de cualquier reinicio, esperar algunos ciclos.
4. El Uno inicia un ciclo cada segundo, el Nano confirma, el Uno concede el
   turno, el Nano envia su propio contador/reloj y el Uno confirma. Los datos
   y confirmaciones enviados por el Nano pasan por el MAX485.

Mensajes esperados (secuencias y tiempos varian):

```text
[MAX485 UNO] TX UNO->NANO CONFIRMADO seq=1 ida_vuelta_ms=...
[MAX485 UNO] RX NANO->UNO OK seq=1 nano_ms=... ACK enviado
[MAX485 NANO] RX UNO->NANO OK seq=1 uno_ms=... ACK enviado
[MAX485 NANO] TX NANO->UNO CONFIRMADO seq=1 ida_vuelta_ms=...
```

Cada 10 segundos aparecen los contadores. Tras arrancar juntos, deben crecer
`confirmado_uno`, `rx_nano`, `rx_uno` y `confirmado_nano`, con `invalidos`,
`inesperados` y los `timeout` en cero durante funcionamiento estable.
Un reinicio/desconexion puede producir timeouts; observar si dejan de crecer
despues de recuperar el enlace. Hay recuperacion automatica sin reiniciar.

## Si falla: comparar con UART directa

Con todo apagado, quitar **RO del Uno D2**, y conectar **Nano D3 a Uno D2**.
Conservar Uno D3 a Nano D2 y GND comun. Los mismos sketches funcionan asi.
No conectar RO y Nano D3 simultaneamente al Uno D2: son dos salidas.

- Si falla tambien en directo, revisar carga de sketches, pines, GND y cables.
- Si funciona en directo y falla por el modulo, revisar VCC-GND (~5 V),
  DE-GND (~5 V), RE-GND (~0 V), continuidad de DI/RO y ausencia de cortos A/B.
  Eso identifica el modulo o sus conexiones como causa probable; no prueba
  por si solo que el chip se haya quemado.
- Puede usarse ademas `../RS485_NANO/NANO/NANO.ino` para la prueba estatica
  canal por canal, siguiendo su README: sus conexiones de DE/RE son distintas.

El resultado correcto aqui demuestra paso local de datos por DI/transmisor/
receptor/RO a 9600 baud. No demuestra margen electrico con cable largo,
recepcion desde otro transceptor ni cambio de direccion. El siguiente paso
para simular ESP y Portenta por RS485 es usar dos MAX485.

## Verificacion del codigo

Compilar ambos sketches con Arduino AVR. La regresion
`tests/max485_un_modulo_nano_uno_test.cpp` ejecuta los dos sketches reales con
UART simulada: 100 ciclos, CRC corrupto, ACK de otra secuencia, perdida de ACK
en ambos sentidos, perdida de datos, desconexion/reconexion y rechazo de la
version anterior aunque tenga CRC valido. No simula niveles electricos ni
interrupciones de SoftwareSerial y no sustituye la prueba fisica del modulo.

El 2026-10-08 ambos sketches compilaron con Arduino AVR 1.8.8: Nano
5730 bytes de programa / 396 bytes globales; Uno 5990 / 400. La regresion
anterior paso y ambas cabeceras se verificaron identicas. No se cargaron
placas ni se verifico fisicamente el MAX485.

Referencia electrica: [hoja de datos MAX485, controles DE y /RE y circuito
interno](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf).

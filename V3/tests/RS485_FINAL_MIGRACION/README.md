# Test final del enlace para migrar Automatico V2

Preparado el 2026-10-08 a partir de `../nuevo test funcional/` y corregido
desde los sketches `ESP32_RS485_115200.ino` y `PORTENTA_RS485_115200.ino`
entregados por el usuario en Downloads. El usuario confirmo el funcionamiento
de esos dos tests a **115200**; intercambian mensajes ASCII de 8 bytes con
CRC8. La prueba anterior a 9600 se conserva como antecedente. Este banco usa
**el transporte del firmware completo**, sin motores, servos, camara, OLED,
control Bluepad32, encoder ni calibraciones de arranque.

- Aplicacion: `ProtocoloRS485.h`, **version 17**, paquetes de **32 bytes**.
- Transporte: `EnlaceRS485.h`, COBS + sesion + solicitud + CRC16,
  **44 bytes en cable**, **115200 baudios, 8N1, half duplex**.
- Portenta coordina y mantiene una solicitud pendiente; ESP responde.
- Portenta exige **3 ms sin bytes RX** antes de iniciar otra solicitud,
  incluyendo las fases de carga y los casos de fallo/recuperacion.
- Timeout de respuesta **100 ms**, fragmentos **80 ms**, igual que produccion.
- Los paquetes contienen valores sinteticos de encoder, coordenadas, tags,
  joystick y servos; se comparan los **32 bytes completos** en la respuesta.
  Nadie aplica esos valores a hardware en estos sketches.

Las cabeceras de protocolo/transporte son copias identicas de
`../../automatico v2 rs485/`. `ConfiguracionRS485.h` tambien se comparte con
su ESP; la regresion verifica las copias. `FinalRS485.h` es exclusivo del banco:
sus fallos artificiales por numero de solicitud nunca se incorporan al ciclo
autonomo. No hay una nueva orden de aplicacion ni cambio de version.

## Cableado de la prueba del usuario

Se conservan **RX14, TX27 y DE18**, como en `ESP32_RS485_115200.ino`.
La asignacion anterior RX27/TX14 de la variante completa fue corregida antes
de esta revision; no cambiar los cables para usar este banco o la variante completa.

| ESP32 | Convertidor 3.3 V / 5 V | MAX485 |
|---|---|---|
| GPIO14 RX | Canal de RO utilizado en tu prueba | RO |
| GPIO27 TX | Canal de DI utilizado en tu prueba | DI |
| GPIO18 | Canal de direccion utilizado en tu prueba | DE y /RE unidos |
| GND | GND | GND |

MAX485 A/B van a TX P/TX N de Machine Control, respectivamente, y GND comun.
Conservar alimentacion/conversion de nivel del montaje probado. La Portenta
configura RS485 half duplex y deja desactivada la terminacion interna,
como el test confirmado tras `comm_protocols.init()`. No agregar terminaciones
sin revisar las existentes. GPIO14/27 deben estar libres del antiguo I2C de
control hacia Portenta. El detalle de alimentacion esta en
`../../automatico v2 rs485/README_RS485.md`.

Se comparten los tiempos del firmware completo: ESP mantiene DE activo
200 us antes de TX y 200 us despues de `flush()`; Portenta inicia el bus
con `begin(115200, 0, 2000)` mediante las constantes de `EnlaceRS485.h`.
El segundo argumento de esa sobrecarga es el retardo previo; 8N1 es implicito.
La respuesta espera **15 ms** para ceder el bus. En el test confirmado
responde Portenta; aqui responde ESP y Portenta mantiene la coordinacion.
No se trasladan el formato ASCII ni el timeout de 400 ms del test al banco.

La correccion posterior del 2026-10-08 agrega el retorno de turno: tras cada
byte recibido, Portenta aplaza otra solicitud hasta cumplir 3 ms de silencio.
Esto permite que ESP libere DE despues de sus 200 us de post-TX. La condicion
vive en `Cliente::registrarRecepcion()`, `puedeIniciar()` e `iniciar()` del
transporte compartido y no bloquea el loop. Se mantienen 15 ms antes de
responder, timeout de 100 ms y formato v17/32 bytes/COBS 44 bytes.
El test ASCII confirmado separaba solicitudes con 100 ms y no comprobaba
este retorno inmediato bajo carga. Ver la evidencia y sus limites en
`../../automatico v2 rs485/DIAGNOSTICO_RS485_2026-10-08.md`.

## Ejecucion en placas

1. Cargar **los dos sketches de esta carpeta**, con todas sus cabeceras:
   `ESP32/ESP32.ino` (`esp32-bluepad32:esp32:esp32`) y
   `PORTENTA/PORTENTA.ino` (`arduino:mbed_portenta:envie_m7`). No mezclar este
   banco con firmware completo o con los tests anteriores.
2. Abrir ambos monitores a **115200**. Conservar los logs de las dos placas.
3. Enviar **T mayuscula a Portenta**. Solo entonces comienza el intercambio.
   El banco ejecuta una sola ronda; para repetirla reiniciar ambas placas.
4. Esperar aproximadamente **7 minutos**, segun los tiempos reales de loop.
   No hay botones que accionen el brazo.
   No cortar el bus ni reiniciar placas durante la ronda automatica.

Para registrar ambos USB y enviar T automaticamente, cerrar ambos monitores
y ejecutar el registrador del banco. Reiniciar ambas placas antes de cada ronda:

```powershell
powershell -ExecutionPolicy Bypass -File ".\tests\RS485_FINAL_MIGRACION\registrar_test.ps1" -PuertoESP COM14 -PuertoPortenta COM8
```

Sustituir COM14/COM8 por los puertos reales. El script abre ambos a 115200,
envia T despues de 3 segundos, conserva cada linea con origen/hora UTC en
`registros/final_rs485_*.log` y espera el resultado y el ultimo resumen ESP.
El limite total es 900 s; si no llega resultado, la ronda no esta aprobada.
Este script es exclusivo del banco y envia T: no usarlo con firmware completo.

## Criterios de aprobacion

La fase 0 envia **1000** solicitudes, periodo 150 ms, limite total 200 s.
La fase 1 envia **10000**, periodo nominal **10 ms**, limite total 1500 s.
La separacion real depende del loop y de una solicitud pendiente. El banco
reporta duracion, RTT medio/maximo y separacion maxima; los 15 ms de giro,
las dos tramas y los retardos de TX impiden alcanzar 100 Hz. El registrador
tiene un limite global de 900 s, mas estricto que el limite interno de fase 1.
En **cada fase de carga** se exige:

- Todos los intercambios correctos, cero timeout, respuesta ajena o dato distinto.
- Cero TX incompleto y cero nuevos errores CRC, longitud o fragmento en Portenta.
- Terminar dentro del limite de duracion. No se reintenta para esconder perdidas.

La fase 2 comprueba los siguientes casos, con 300 ms de observacion por caso:

| Paso | Caso | Resultado esperado |
|---|---|---|
| 0 | Paquete valido | Respuesta valida |
| 1 | CRC16 alterado | Silencio y timeout |
| 2 | Recuperacion | Respuesta valida |
| 3 | Version 16 con CRC8/CRC16 correctos | Silencio y timeout |
| 4 | Recuperacion | Respuesta valida |
| 5 | CRC8 malo dentro de CRC16 correcto | Silencio y timeout |
| 6 | Recuperacion | Respuesta valida |
| 7 | Fragmento sin delimitador | Silencio; vencer fragmento; cero de resincronizacion |
| 8 | Recuperacion | Respuesta valida |
| 9 | Ruido mayor que la trama + delimitador + paquete | Respuesta valida |
| 10 | Duplicado exacto de paso 9 | Respuesta, sin volver a aplicar |
| 11 | Misma solicitud con carga distinta y CRC correctos | Silencio y timeout |
| 12 | Solicitud antigua | Silencio y timeout |
| 13 | Respuesta de sesion ajena | Descartada, un timeout y una ajena |
| 14 | Recuperacion | Respuesta valida |
| 15 | Respuesta retrasada 115 ms adicionales antes del giro | Descartada, un timeout y una ajena |
| 16 | Recuperacion | Respuesta valida |
| 17 | Respuesta perdida deliberadamente | Un timeout |
| 18 | Recuperacion | Respuesta valida |
| 19 | Nueva sesion de Portenta | Respuesta valida |

Portenta debe imprimir **dos `CARGA PASS`**, veinte casos `PASS` y
**`RESULTADO=PASS fallos=0`**. En fase 2 los 9 timeouts y 2 ajenas son
**deliberados**. En carga se exige cero. Revisar tambien el ultimo resumen ESP:
`nuevas=11013 duplicadas=1 descartadas=2 sem=2 crc=1 len=1 fragmentos=1
txOK=11013 txError=0`. Esos errores ESP son las inyecciones previstas;
incrementos adicionales o valores diferentes impiden aprobar la ronda.
La sesion de arranque ESP se mantiene estable; un reinicio durante la ronda
produce fallo por datos distintos. Desconexion/ausencia de ESP produce FAIL.

## Ultima comprobacion con el firmware completo

Un PASS aqui aprueba el enlace del banco. Para cerrar la migracion, cargar
ambos sketches de `../../automatico v2 rs485/` y guardar otra captura completa
con su registrador (ESP **460800**, Portenta **115200**; bus **115200**).
El firmware completo inicia calibraciones y puede mover el brazo: ejecutarlo
en el montaje preparado siguiendo su README.

Comprobar arranque, OLED, control, camara, calibracion/telemetria de encoder y
al menos 10 minutos de enlace bajo esa carga, con cero incrementos de CRC,
longitud, semantica, TX fallido, timeout o respuesta ajena tras estabilizarse.
Registrar el ciclo Automatico V2 a velocidad baja: DIN04, alineacion,
seguimiento, cierre, entrega, apertura y retirada. Verificar por observacion
el agarre; `CICLO_ENTREGADO_NO_VERIFICADO` solo confirma secuencia de software.
Comprobar perdida/reinicio de cada extremo en condiciones controladas y
recalibracion obligatoria tras movimiento interrumpido. Estas guardas ya
tienen regresiones locales; el banco no acciona hardware para ensayarlas.

Si falla un criterio, conservar la variante I2C/protocolo 16 como retorno y
resolver el fallo antes de dar la migracion por terminada. No se sobrescriben
`pruebas de automatico v2/ESP`, `pruebas de automatico v2/PORTENTA` ni raiz.

## Verificacion local reproducible

```powershell
python tests/rs485_final_migracion_test.py --compiler "C:\Program Files\Webots\msys64\mingw64\bin\g++.exe"
python tests/rs485_migracion_test.py --compiler "C:\Program Files\Webots\msys64\mingw64\bin\g++.exe"
```

El primer runner ejecuta **los dos sketches completos** con UART/reloj
simulados: 11000 intercambios, 20 casos, contadores de rechazo exactos,
duplicado sin reaplicar, liberacion de DE y una ronda con bus desconectado
que debe dar FAIL. Verifica identidad de cabeceras con produccion. El segundo
runner verifica las guardas y los modos de la variante completa RS485.
Esto no comprueba niveles electricos, ruido real ni agarre fisico.
No se cargaron placas ni se acciono hardware durante la preparacion.

### Revision vigente: silencio antes de otra solicitud (2026-10-08)

Pasaron las regresiones de esta revision. El banco completo simulado cumple
11000 intercambios y 20 casos de fallo/recuperacion; la ronda desconectada
da FAIL como corresponde. En cada respuesta se procesa el loop Portenta
desde `flush()` de ESP, antes de liberar DE despues de los 200 us de post-TX.
No se solapan transmisores. La variante completa tambien pasa transporte,
guardas, envios reales y cinco suites heredadas; comprueba urgente diferida
3 ms, espera renovada por bytes invalidos/ajenos, wrap del reloj y bytes
parciales que no renuevan el plazo de seguridad. Logs en
`../../tmp/rs485-giro-inverso/regresion-banco.log` y `regresiones-v2.log`.

Compilaron los cuatro sketches con codigo de salida 0:

| Sketch | Core | Resultado |
|---|---|---|
| Banco ESP32 | Bluepad32 4.1.0 | 712561 bytes programa, 86932 globales |
| Banco Portenta M7 | mbed_portenta 4.6.0 | Binario de aplicacion 190368 bytes |
| Automatico V2 RS485 ESP | Bluepad32 4.1.0 | 805617 bytes programa, 104356 globales |
| Automatico V2 RS485 Portenta M7 | mbed_portenta 4.6.0 | Binario de aplicacion 258008 bytes |

Logs en `../../tmp/rs485-giro-inverso/`: `compilacion-banco-esp.log`,
`compilacion-banco-portenta.log`, `compilacion-esp.log` y
`compilacion-portenta.log`. Portenta conserva las advertencias heredadas
de Ticker/Arduino_MachineControl; no se modificaron esas bibliotecas.

Los resultados que siguen corresponden a revisiones anteriores.
La nueva confirmacion en placas sigue pendiente. No se cargaron placas
ni se acciono hardware durante esta correccion.

### Verificacion previa: perfil del test a 115200 (2026-10-08)

Pasaron las regresiones sobre los fuentes corregidos: ambos sketches del
banco, 11000 intercambios, 20 casos y FAIL por desconexion; tambien pasaron
las regresiones del perfil/transporte/guardas y las cinco suites heredadas
de la variante completa. La base I2C original paso sus regresiones.
Logs en `../../tmp/rs485-115200-correccion/regresion-banco.log`,
`regresiones-v2.log` y `regresion-v2-i2c.log`. Se verificaron las cuatro
copias identicas de cada cabecera `ProtocoloRS485.h` y `EnlaceRS485.h`, y
hashes sin cambios de los 12 fuentes originales I2C/raiz.

Compilaciones de los fuentes corregidos, con Arduino CLI:

| Sketch | Core | Resultado |
|---|---|---|
| Banco ESP32 | Bluepad32 4.1.0 | Compilo: 712561 bytes programa, 86932 globales |
| Banco Portenta M7 | mbed_portenta 4.6.0 | Compilo: binario de aplicacion 190304 bytes |
| Automatico V2 RS485 ESP | Bluepad32 4.1.0 | Compilo: 805617 bytes programa, 104356 globales |
| Automatico V2 RS485 Portenta M7 | mbed_portenta 4.6.0 | Compilo: binario de aplicacion 258008 bytes |

Logs en `../../tmp/rs485-115200-correccion/`: `compilacion-banco-esp.log`,
`compilacion-banco-portenta.log`, `compilacion-esp.log` y
`compilacion-portenta.log`. Las cuatro compilaciones terminaron con codigo
de salida 0; Portenta conserva advertencias heredadas de Arduino_MachineControl.
La confirmacion fisica del usuario
corresponde al test ASCII a 115200; el banco final y el firmware completo
corregido siguen pendientes de comprobar en placas. No se cargaron placas
ni se acciono hardware durante esta correccion.

### Resultados historicos anteriores a esta correccion

Resultados del **2026-10-08**, sobre los fuentes de la revision anterior:

| Sketch | Core | Resultado |
|---|---|---|
| Banco ESP32 | Bluepad32 4.1.0 | Compilo: 712545 bytes programa, 86932 globales |
| Banco Portenta M7 | mbed_portenta 4.6.0 | Compilo: binario de aplicacion 190312 bytes |
| Automatico V2 RS485 ESP | Bluepad32 4.1.0 | Compilo: 805461 bytes programa, 104356 globales |
| Automatico V2 RS485 Portenta M7 | mbed_portenta 4.6.0 | Compilo: binario de aplicacion 257880 bytes |

Pasaron la simulacion de ambos sketches del banco, la ronda desconectada y
los 8 grupos de regresion de la variante completa. El registrador paso el
parser PowerShell sin abrir puertos. Las advertencias de compilacion provienen
de las bibliotecas instaladas Arduino_MachineControl/DFRobot_HuskylensV2 y
del Ticker existente del firmware completo; no se modificaron bibliotecas.
Logs en `../../tmp/rs485-final-migracion/`: `compilacion-esp.log`,
`compilacion-portenta.log`, `compilacion-v2-esp.log`,
`compilacion-v2-portenta.log`, `regresion-banco.log` y `regresiones-v2.log`.

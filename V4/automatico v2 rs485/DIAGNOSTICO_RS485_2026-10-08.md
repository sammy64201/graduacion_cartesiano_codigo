# Diagnostico de estabilizacion y perdida de datos RS485

Fecha: 2026-10-08. Firmware completo `ESP/ESP.ino` y
`PORTENTA/PORTENTA.ino`, bus 115200 8N1, protocolo 17, aplicacion de 32 bytes
y transporte COBS de 44 bytes. Analisis de registros entregados por el usuario;
no se cargaron placas ni se acciono hardware durante esta correccion.

## Politica vigente posterior a las capturas

El usuario aclaro despues que otro episodio de estabilizacion ocurrio con el
brazo en movimiento y no fue su reinicio manual. Esa aclaracion no viene
acompanada de una nueva captura sincronizada; el analisis de lineas siguiente
describe solamente los archivos enumerados y no explica todos los episodios.

Por su nueva instruccion del 2026-10-08, la recuperacion por ausencia de enlace
solo comienza tras **mas de 1000 ms desde la ultima respuesta valida**.
Hasta 1000 ms inclusive, el enlace permanece vigente. Se exige CRC, semantica
y correlacion de sesion/solicitud RS485; bytes sueltos, errores o respuestas
ajenas no renuevan el reloj. El timeout de solicitud permanece en **100 ms**
para seguir sondeando. Los errores consecutivos y la secuencia congelada no
adelantan recuperacion; con respuestas validas continuas, la secuencia
congelada no provoca recuperacion.

Al superar ese umbral se detienen motores y se cancela el objetivo, sin
reanudarlo automaticamente. La recuperacion conserva calibracion XY/Z,
pasos por mm, escalas y rangos aunque se interrumpa movimiento, por peticion
expresa del usuario aun si la referencia fisica deja de ser fiable. El cambio
de sesion de arranque ESP sigue provocando recuperacion inmediata, separada
de una ausencia transitoria, y tambien conserva esas referencias en Portenta.
Finales, limites, DIN04, STOP/cancelacion, Bluetooth, camara y encoder
mantienen sus comprobaciones; el reintento completo desde error conserva su
ruta propia. Se integra la politica en Automatico V2 I2C y RS485 con dos
copias identicas de `RecuperacionEnlace.h`. No cambia protocolo 16/17 ni
paquetes de 32 bytes.

La guarda de **3 ms sin bytes RX** para liberar DE se conserva. Los 150 ms y
la invalidacion por movimiento interrumpido descritos en las capturas son
**politica historica anterior a esta instruccion**, no instrucciones de uso
actuales. Los resultados previos de la correccion de turno tampoco acreditan
la nueva politica. Pasaron las regresiones de recuperacion sobre RX real
I2C/RS485, incluidas fronteras 999/1000/1001 ms, respuestas invalidas/ajenas,
snapshot congelado con respuestas validas, errores consecutivos, wrap,
referencias conservadas, cancelacion sin reanudacion y cambio de sesion ESP.
Tambien pasaron setup, transporte, perfil, envio real sin solapamiento de DE
y cinco suites RS485 heredadas, y ciclo/handshake I2C. Logs de esta revision:
`../tmp/recuperacion-enlace-1s/regresiones-v2-rs485.log` y
`regresiones-v2-i2c.log`. Compilaron los cuatro sketches completos I2C/RS485,
ESP/Portenta, con codigo de salida 0 y cores Bluepad32 4.1.0 /
mbed_portenta 4.6.0. Logs en la misma carpeta: `compilacion-rs485-esp.log`,
`compilacion-rs485-portenta.log`, `compilacion-i2c-esp.log` y
`compilacion-i2c-portenta.log`. Portenta conserva advertencias heredadas
de bibliotecas. La nueva captura y comprobacion en placas siguen pendientes;
no se cargaron placas ni se acciono hardware.

## Capturas examinadas

Los dos ultimos archivos amplian exactamente los prefijos de los primeros;
no son un nuevo ensayo independiente. Las lineas se cuentan por saltos LF,
desde 1, incluidos los caracteres ilegibles iniciales de ESP.

| Captura | Adjunto en `C:/Users/samue/.codex/attachments/` | Muestras RS485 |
|---|---|---:|
| ESP inicial | `f903dec3-7738-4d21-a576-3e77a78e0e7e/Pasted text.txt` | 88 |
| Portenta inicial | `db20e360-1d50-4f80-a46b-5321701b80bf/Pasted text.txt` | 116 |
| ESP ampliada | `dff61eed-8116-44b8-aa8e-410ef7b33581/Pasted text.txt` | 328 |
| Portenta ampliada | `21e136ab-25d9-4a22-827a-53124ca7f055/Pasted text.txt` | 358 |

No hay fechas/horas en estas capturas. La sesion ESP 18235 relaciona ambos
extremos, pero no permite sincronizar cada muestra o medir la duracion
exacta de cada incidencia. Las muestras de cada segundo son contadores
acumulados; sus maximos tampoco indican un fallo en cada segundo.

## Estabilizacion, arranque y recalibracion

La pantalla de estabilizacion corresponde al estado 2: el programa espera
cinco segundos despues de recuperar el enlace. Los mensajes
`[BOOT] Estado general -> ...` registran transiciones de estado y no son
por si solos evidencia de reset de CPU. `reinicios` mide reinicializaciones
del bus UART; permanece en 1 durante toda la captura Portenta y en 0 en ESP.

Hay dos cambios de sesion ESP en Portenta: 51030 -> 23895 (linea 15) y
23895 -> 18235 (37), ambos antes de calibrar el brazo. ESP muestra una
cabecera de arranque con sesion 18235 (29) y contadores que vuelven de
rx=360 (20) a rx=8 (32). La cabecera de reset legible dice
`rst:0xc (SW_CPU_RESET)` (3); no permite atribuirlo a alimentacion,
watchdog, monitor USB o una causa concreta. No aparece otro arranque ESP
en la ampliacion ni reinicio de los contadores Portenta.

Las seis lineas Portenta con `Enlace perdido; reintentando sin reinicio fisico`
son 12, 20, 42, 173, 417 y 476. No aparece
`Movimiento interrumpido; recalibrar brazo antes de mover`.
El corte de la linea 173 llega desde menu, despues de HOME completo (160);
el de 417 ocurre despues de completar los pulsos del movimiento (414).
El firmware de aquellas capturas conservaba la calibracion al recuperar el
enlace con motores detenidos. Su politica previa invalidaba la calibracion
si interrumpia movimiento; esa politica fue reemplazada como se indica arriba.

El episodio final tiene una causa de seguridad distinta:

| Linea Portenta ampliada | Evidencia |
|---:|---|
| 64 | Inicio de la primera calibracion X/Y/Z |
| 130-131 | CALIBRACION OK; HOME X=0 Y=0 Z=0 |
| 285-307 | Calibracion de camara; homografia valida y modelo listo |
| 452 | Movimiento detenido: final de carrera inesperado |
| 453-454 | Estado 10; ERROR codigo=8, final de carrera durante movimiento XY |
| 464 | Reintento seguro solicitado |
| 466 | Estado 0, reinicio de la secuencia segura |
| 490-493 | Falta calibracion; inicia X/Y/Z y busca X- |
| 495 | Vuelve al menu, estado 7 |

Los contadores continúan de rxOK=9190 (463) a 9219 (467), con reinicios=1;
este reintento no reinicia la CPU Portenta. Reinicia la secuencia desde error
e invalida calibraciones. **La segunda calibracion solo empieza**: no hay
otra `CALIBRACION OK` y la captura no explica el retorno al menu en 495.
El final inesperado requiere comprobar por separado el movimiento y los
sensores; el registro no demuestra que RS485 lo haya causado.

## Evidencia de solicitudes recortadas

En el tramo ESP posterior al ultimo arranque se cumple
`tramas validas + lenTrama = ceros`, con CRC de transporte, fragmentos y
solicitudes antiguas en cero. Cada trama del formato vigente ocupa 44 bytes.

| Captura ESP | Validas | Malformadas | Delimitadores | Bytes recibidos | Bytes esperados | Deficit |
|---|---:|---:|---:|---:|---:|---:|
| Inicial, ultima muestra | 2371 | 31 | 2402 | 105649 | 105688 | 39 |
| Ampliada, linea 447 | 9196 | 155 | 9351 | 411236 | 411444 | 208 |

En la ampliacion, 90 intervalos con exactamente una trama malformada permiten
restar los bytes de las tramas validas: **59 tramas de 43 bytes y 31 de
42 bytes**, frente a 44 esperados. Otros 29 intervalos contienen 65 tramas
malformadas con un deficit conjunto de 87 bytes; sus longitudes individuales
no se pueden separar. La tasa acumulada de tramas malformadas es 1.66 %.
Es un patron compatible con perder 1-2 bytes al cambiar el turno; el log no
localiza esos bytes dentro de cada trama ni mide las senales electricas.

`errLen=0` y `errCRC=0` de ESP corresponden a la aplicacion de 32 bytes.
No contradicen `lenTrama=155`: la aplicacion solo inspecciona mensajes que
el receptor de transporte ya decodifico correctamente.

En Portenta ampliada, `rxOK` crece de 371 a 10314, `txOK` de 444 a 10573
y timeout de 72 a 258: **186 nuevos timeouts entre 10129 solicitudes**,
aproximadamente 1.84 %. En todas las muestras `rxError=timeout`, con longitud,
CRC y semantica en cero. Esto encaja con solicitudes rechazadas en ESP que
no producen respuesta, aunque no demuestra que todas tengan la misma causa.

Los RTT aceptados tienen maximo 73 ms, inferior al plazo de respuesta de
100 ms. `pausaLoopMax=1963ms` aparece alrededor del ultimo cambio de sesion
y permanece como maximo historico; no significa una pausa repetida de dos
segundos. Los caracteres ilegibles del monitor ESP no prueban corrupcion
RS485: la consola USB usa 460800 y el bus usa 115200.

## Defecto de turno encontrado y correccion

La version anterior podia iniciar la siguiente solicitud al reconocer el
ultimo byte de respuesta, porque ya no habia una solicitud pendiente.
ESP conserva DE activo durante 200 us despues de `flush()`; Portenta usa
retardo previo de TX de 0 us. Faltaba una condicion de retorno del turno que
impidiera comenzar a transmitir antes de que ESP liberara el bus.
La prueba ASCII confirmada por el usuario incluye 100 ms entre solicitudes
y no reproduce este envio inmediato de la migracion.

En half duplex debe haber un solo transmisor activo cada vez; el software
controla los turnos mediante DE y /RE. Esta restriccion se describe en
[The RS-485 Design Guide, TI, pagina 2](https://www.ti.com/lit/an/slla272d/slla272d.pdf).

La correccion exige **3 ms de silencio RX en Portenta** antes de otra
solicitud. Cada byte recibido renueva ese intervalo, aunque el mensaje sea
invalido o ajeno. `Cliente::registrarRecepcion()`, `puedeIniciar()` e
`iniciar()` comparten la condicion entre el banco final y todos los modos
del firmware completo, incluidas las ordenes urgentes de cierre/apertura.
Se comprueba por reloj sin bloquear el loop; ESP conserva giro de 15 ms
y post-TX de 200 us. Aquella revision conservaba timeout de respuesta de
100 ms, guardas de seguridad de 150 ms y protocolo 17/32 bytes/COBS 44 bytes.
La revision vigente conserva el timeout de solicitud y el transporte,
pero usa mas de 1000 ms sin respuesta valida para la recuperacion general.

El defecto de programacion esta identificado y es compatible con el patron
del registro. **No se ha medido una colision electrica ni demostrado una
unica causa fisica.** Se necesita una nueva captura con esta revision para
comprobar si desaparecen los recortes. Si persisten, las senales DE/RO/DI
y A/B alrededor del cambio de turno permitirian distinguir manejo UART,
transceptores, conversion de nivel y cableado.

La correccion de DE es exclusiva de RS485 y se incorpora al Automatico V2
completo de esta carpeta. En aquella revision, la base I2C/protocolo 16 y los
sketches de raiz conservaron sus fuentes. La nueva politica de recuperacion
si se integra tambien en Portenta de la base I2C, conservando protocolo 16;
los sketches de raiz siguen sin sobrescribirse. El paro/cancelacion al declarar
perdida y el criterio de ciclo sin agarre fisico verificado se mantienen.

## Verificacion historica de la correccion del retorno de turno

Pasaron las regresiones del firmware completo: setup, transporte, perfil,
guardas, envio y cinco suites heredadas. El banco final paso 11000
intercambios y 20 casos; la ronda desconectada produce FAIL como corresponde.
En la simulacion, `flush()` de ESP ejecuta el loop Portenta antes de
liberar DE y de completar los 200 us posteriores a TX: no se permite
solapamiento de transmisores en ninguna de las 11000 respuestas.
Las pruebas tambien comprueban la urgente diferida 3 ms, la espera renovada
por cualquier byte RX, wrap del reloj y que los bytes parciales no renuevan
las guardas de seguridad. Logs en `../tmp/rs485-giro-inverso/regresiones-v2.log`
y `regresion-banco.log`. Los resultados anteriores a 115200 son historicos.

Compilaron los cuatro sketches con codigo 0: banco y firmware completo ESP
con Bluepad32 4.1.0 (712561/86932 y 805617/104356 bytes de programa/globales),
y ambos Portenta con mbed_portenta 4.6.0 (binarios 190368 y 258008 bytes).
Logs en la misma carpeta: `compilacion-banco-esp.log`,
`compilacion-banco-portenta.log`, `compilacion-esp.log` y
`compilacion-portenta.log`. Las advertencias Portenta proceden del Ticker
y Arduino_MachineControl existentes; sus bibliotecas no se modificaron.
La simulacion confirma el control de turnos del codigo; la confirmacion
en placas y la nueva captura bajo carga quedan pendientes.

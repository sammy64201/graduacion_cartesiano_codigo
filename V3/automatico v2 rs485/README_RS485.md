# Automatico V2 con enlace ESP32-Portenta por RS485

Version completa creada el **2026-10-07** a partir de los sketches vigentes de
`../pruebas de automatico v2/`. Los sketches de esta carpeta migran el enlace
de control entre ESP32 y Portenta Machine Control a **RS485, 115200 baudios,
8N1, half duplex**. La pantalla SH1106 sigue usando I2C.

## Archivos que se cargan

- **ESP32:** `ESP/ESP.ino`, placa `esp32-bluepad32:esp32:esp32`, core 4.1.0.
- **Portenta H7 M7:** `PORTENTA/PORTENTA.ino`, placa
  `arduino:mbed_portenta:envie_m7`, core 4.6.0.
- Abrir cada `.ino` desde su carpeta completa: las cabeceras forman parte del
  sketch. Cargar **los dos sketches juntos**, protocolo de aplicacion **17**.
- Monitor serial de ESP: **460800**. Monitor serial de Portenta: **115200**.
  El bus RS485 permanece en **115200** aunque el monitor ESP use 460800.

Esta es la version completa del brazo, con sus modos y calibraciones de
arranque; la prueba anterior en `../tests/RS485_COMUNICACION/` solo verifica
el enlace. No mezclar firmware I2C/version 16 ni prueba RS485/version 2 con
estos sketches. La version anterior se conserva en su carpeta original.

## Recuperacion vigente: mas de un segundo sin respuesta (2026-10-08)

Por la nueva instruccion del usuario, Portenta conserva vigente el enlace
hasta **1000 ms inclusive desde la ultima respuesta valida**. Solo declara
perdida y vuelve a la estabilizacion cuando transcurren **mas de 1000 ms**
sin respuesta con CRC y semantica validos y con la sesion/solicitud RS485
pendiente correctas. Bytes sueltos, tramas corruptas o respuestas ajenas no
renuevan ese reloj. La politica comun esta en las dos copias identicas de
`RecuperacionEnlace.h`, tambien integrada en Automatico V2 I2C.

El timeout de **100 ms de cada solicitud** sigue liberando el turno para
sondear de nuevo. Ese timeout, los errores consecutivos y una secuencia de
snapshot congelada no fuerzan recuperacion anticipada; mientras lleguen
respuestas validas, una secuencia congelada tampoco declara perdida.
La guarda de **3 ms de silencio RX** para liberar DE se conserva.

Cuando finalmente se declara perdida, se detienen motores, se cancela el
objetivo pendiente y se vuelve a la espera, estabilizacion y checklist/menu.
Se conservan la calibracion XY/Z, los pasos por mm, escalas y rangos de la
sesion, incluso si se interrumpio movimiento. El movimiento cancelado no se
reanuda automaticamente. Conservar esas referencias fue solicitado por el
usuario aun si dejan de coincidir con la posicion fisica; la ultima orden
puede mantenerse mas tiempo durante una ausencia transitoria de respuestas.

Un **cambio de sesion de arranque ESP** sigue provocando cancelacion y
recuperacion inmediata, como evento separado de un corte transitorio;
tampoco invalida las referencias guardadas por Portenta. Un reinicio real de
Portenta o un reintento completo desde error conserva su comportamiento
propio y puede exigir calibrar de nuevo. Siguen vigentes finales de carrera,
limites, DIN04, STOP/cancelacion, Bluetooth, camara y encoder.

No se agrega una orden ni se cambia la semantica de los paquetes:
protocolo **17**, aplicacion de **32 bytes**, COBS de **44 bytes**.
Esta politica reemplaza la anterior de 150 ms e invalidacion de calibracion
por movimiento interrumpido descrita en el historial y en las capturas.
Pasaron las regresiones de recuperacion con RX real en I2C y RS485:
vigencia a 999/1000 ms y perdida a 1001 ms, respuesta que renueva el plazo,
snapshot congelado con respuestas validas, errores consecutivos, rechazo
de respuestas invalidas/ajenas, wrap de reloj, cancelacion sin reanudacion,
conservacion de referencias y cambio de sesion ESP. Tambien pasaron setup,
transporte, perfil, envio real sin solapamiento de DE y cinco suites heredadas
RS485, y el ciclo/handshake de Automatico V2 I2C. Logs en
`../tmp/recuperacion-enlace-1s/regresiones-v2-rs485.log` y
`regresiones-v2-i2c.log`; los resultados anteriores no acreditan esta politica.

Compilaron los cuatro sketches completos, codigo de salida 0: ESP y Portenta
de RS485 y de la base I2C, con Bluepad32 4.1.0 y mbed_portenta 4.6.0.
ESP RS485 usa 805617 bytes de programa y 104356 globales; ESP I2C usa
807781 y 104124. Ambos Portenta conservan advertencias heredadas de sus
bibliotecas. Logs en la misma carpeta: `compilacion-rs485-esp.log`,
`compilacion-rs485-portenta.log`, `compilacion-i2c-esp.log` y
`compilacion-i2c-portenta.log`. Falta comprobar esta revision en placas.
No se cargaron placas ni se acciono hardware.

## Correccion conservada del retorno de turno (2026-10-08)

Los registros del firmware completo mostraron solicitudes recortadas en ESP
y timeouts en Portenta. Se encontro una omision en el coordinador: al recibir
el ultimo byte de una respuesta podia iniciar otra solicitud mientras ESP
aun conservaba DE activo durante los 200 us posteriores a `flush()`.
La prueba ASCII del usuario separaba sus solicitudes con una pausa de 100 ms
y no ejercitaba este retorno inmediato del turno.

Portenta ahora exige **3 ms sin recibir bytes** antes de iniciar otra
solicitud. `Cliente::registrarRecepcion()` registra cada byte, incluso si la
trama resulta invalida o ajena; `puedeIniciar()` e `iniciar()` aplican la
misma condicion a solicitudes normales y ordenes urgentes de pinza. La espera
se comprueba sin bloquear el loop y se comparte con el banco final mediante
las cuatro copias de `EnlaceRS485.h`.

Se conserva el giro ESP de 15 ms, su DE posterior de 200 us y el plazo de
respuesta de 100 ms. La recuperacion general usa el umbral vigente de mas
de 1000 ms indicado arriba. La correccion de turno no
cambia protocolo 17, paquetes de 32 bytes ni tramas COBS de 44 bytes.
El defecto de programacion es compatible con los recortes observados; las
capturas no demuestran que sea la unica causa del montaje. El analisis de
reinicios, reconexiones y recalibracion esta en
[DIAGNOSTICO_RS485_2026-10-08.md](DIAGNOSTICO_RS485_2026-10-08.md).

Las regresiones de la revision del retorno de turno pasaron: transporte,
guardas, envios reales,
cinco suites heredadas y banco final con 11000 intercambios y 20 casos.
La simulacion procesa la respuesta en Portenta antes de que ESP libere DE,
sin solapamiento de transmisores, e incluye solicitudes urgentes diferidas.
Logs en `../tmp/rs485-giro-inverso/regresiones-v2.log` y `regresion-banco.log`.
Compilaron los cuatro sketches (banco y firmware completo), codigo 0,
Bluepad32 4.1.0 / mbed_portenta 4.6.0. Logs en la misma carpeta:
`compilacion-esp.log`, `compilacion-portenta.log`,
`compilacion-banco-esp.log` y `compilacion-banco-portenta.log`.
Portenta conserva advertencias heredadas de Ticker/Arduino_MachineControl.
Estos resultados acreditan la correccion de turno anterior a la politica
de recuperacion de un segundo; no acreditan esta ultima.
Falta repetir la captura en placas. No se cargaron placas ni se acciono hardware.

## Configuracion del montaje confirmado a 115200 (2026-10-08)

El usuario confirmo que funcionaron los sketches
`C:/Users/samue/Downloads/ESP32_RS485_115200/ESP32_RS485_115200.ino` y
`C:/Users/samue/Downloads/PORTENTA_RS485_115200/PORTENTA_RS485_115200.ino`.
Se toma su configuracion fisica como referencia para corregir la migracion:

| Parametro | Firmware completo y banco final |
|---|---|
| Bus y pines ESP | 115200, 8N1; RX14, TX27, DE18 |
| DE de ESP antes/despues de TX | 200 us / 200 us; `flush()` antes de liberar DE |
| RS485 Portenta | `begin(BAUD, PORTENTA_PRE_TX_US, PORTENTA_POST_TX_US)`, valores 115200, 0, 2000 |
| Terminacion interna Portenta | Desactivada, como tras `comm_protocols.init()` en el test funcional |
| Espera del extremo que responde | 15 ms antes de transmitir |
| Retorno de turno a Portenta | 3 ms de silencio desde el ultimo byte RX antes de otra solicitud |
| Timeout / fragmento | 100 ms desde fin de TX / 80 ms |

El test tiene ESP como solicitante y Portenta como respondedor; la variante
completa mantiene Portenta como coordinador y ESP como respondedor. Por eso
la espera de 15 ms del test se aplica a la respuesta ESP. Los tiempos de TX
son propios de cada transceptor y se conservan por placa. Portenta ahora usa
la misma sobrecarga de tres argumentos del test: el segundo es el retardo
previo y 8N1 es implicito. La llamada anterior de cuatro argumentos era
valida, pero usaba retardos 200/200 us en lugar de 0/2000 us.

Los parametros comunes estan en las copias identicas de `EnlaceRS485.h` y
los pines en `ConfiguracionRS485.h`, compartidos con el banco final. Se
conservan protocolo 17, paquetes de aplicacion de 32 bytes y tramas COBS de
44 bytes. El test funcional ASCII/CRC8 y su timeout de 400 ms son exclusivos
del diagnostico; no reemplazan el protocolo ni las guardas del brazo.

La confirmacion fisica del usuario corresponde a esos dos tests. La
comprobacion en placas del banco final y del firmware completo corregido
queda pendiente. Las verificaciones locales de esta revision se registran
al final; los resultados anteriores estan identificados como historicos.

## Cableado para soldar

Conservar el cableado que funciono en la prueba, usando el convertidor de
nivel entre ESP32 (lado A, 3.3 V) y MAX485 (lado B, 5 V):

| ESP32 | Convertidor de nivel | MAX485 |
|---|---|---|
| **GPIO27: TX** | Canal de DI usado en la prueba funcional | DI |
| **GPIO14: RX** | Canal de RO usado en la prueba funcional | RO |
| **GPIO18: direccion** | Canal de direccion usado en la prueba funcional | DE y RE unidos |
| GND | GND | GND |

`RS485_DIRECCION_MANUAL = true` es la configuracion entregada. GPIO18 bajo
habilita recepcion; alto habilita transmision. RE en el modulo corresponde a
**/RE**, activo en LOW. RO del MAX485 pasa por el convertidor, nunca directo
a GPIO14. Para un transceptor con direccion automatica, ajustar la constante
a `false` y omitir GPIO18; no es el montaje MAX485 mostrado por el usuario.

| Alimentacion | Conexion |
|---|---|
| VA/VCCA del convertidor | **3.3 V de ESP32** |
| VB/VCCB del convertidor | **5 V regulados** |
| OE, si el convertidor es TXS0108E | **3.3 V** |
| VCC del MAX485 | **5 V regulados** |
| GND | Comun entre ESP32, convertidor, MAX485 y Machine Control |

| MAX485 | Borne de Machine Control |
|---|---|
| **A** | **RS485 TX P** |
| **B** | **RS485 TX N** |
| GND | GND de comunicaciones |

RX P y RX N de Machine Control quedan libres: no se necesitan puentes en
half duplex. El sketch deja desactivada la terminacion interna de Machine
Control para conservar el montaje del test confirmado. El modulo mostrado
parece incluir R7 `121` (120 ohm); verificar la
unidad real antes de agregar otra terminacion en paralelo. Usar par trenzado
para A/B. La alimentacion VCC del MAX485 es 5 V, nunca 24 V.

Retirar el antiguo cableado SDA/SCL hacia Portenta y sus pull-ups externas
antes de reutilizar GPIO27/14. Realizar las soldaduras con alimentacion apagada.

Referencia vigente: `ESP32_RS485_115200.ino` entregado por el usuario declara
RO por canal 1, DI por canal 2 y DE-/RE por canal 3. Esa numeracion describe
los canales usados en la prueba; no permite deducir etiquetas fisicas A2/B2
del convertidor. Conservar los canales que funcionaron en el montaje real.
Trenzar juntos los dos conductores diferenciales A/B entre MAX485 y Portenta;
llevar GND aparte y separar el par del cableado de motores/variador. Los
cables locales GPIO14/27/18 no constituyen el par diferencial RS485.

RX14/TX27 reemplazan la asignacion RX27/TX14 documentada el 2026-10-07.
La prueba anterior `../tests/nuevo test funcional/` usaba esos mismos pines
a 9600; queda como antecedente. Se usa `ESP/ConfiguracionRS485.h`, tambien
verificada en el banco final. El ultimo test antes de validar el firmware completo esta
en `../tests/RS485_FINAL_MIGRACION/README.md`: 11000 intercambios con protocolo
17 real y 20 casos de fallo/recuperacion, sin actuadores. Sus monitores usan
115200 en ambas placas; el monitor ESP del firmware completo sigue en 460800.
Los resultados locales de la revision previa se conservan como historicos
en el README del banco; no acreditan los fuentes corregidos en esta revision.

**Pantalla sin cambios:** SDA=**GPIO21**, SCL=**GPIO22**, I2C a 100 kHz,
direccion `0x3C` en el bus independiente `I2C_Pantalla(1)`.
**HUSKYLENS sin cambios:** UART1 RX=GPIO32, TX=GPIO33, 115200.
**Servos sin cambios:** giro=GPIO25, pinza=GPIO26.
El encoder sigue conectado al canal 0 de Machine Control, no a la ESP.

### Correccion de arranque y terminal USB (2026-10-08)

El firmware completo Portenta conserva el **I2C interno de Machine Control**:
se ejecutan `Wire.begin()` y `Wire.setClock(100000)` antes de inicializar/leer
`digital_inputs`. El enlace con la ESP permanece exclusivamente RS485.
La primera migracion omitio esa inicializacion: el expansor DIN usaba Wire
sin un objeto I2C maestro, causando un acceso nulo en el core mbed que puede
detener el firmware y hacer inaccesible el USB serial. Los bancos de enlace
no leen ese expansor y por eso no reproducian este fallo de arranque.

Se agrega a `tests/rs485_migracion_test.py` la ejecucion de `setup()` real
con el bus y las entradas simulados, exigiendo inicializacion previa del I2C
interno. Esta comprobacion y los ocho grupos anteriores pasaron. La base
original en `../pruebas de automatico v2/` ya inicializa Wire correctamente.
El sketch Portenta corregido compilo con mbed_portenta 4.6.0; log en
`../tmp/rs485-final-migracion/correccion-usb-compilacion-portenta.log`.
La reapertura del COM se debe confirmar en la placa.

Para comprobar la terminal del firmware completo, abrir el puerto de
**Portenta a 115200** y el de **ESP a 460800**. Portenta debe emitir `[RS485]`
cada segundo y acepta `AYUDA` con fin de linea. Si una aplicacion de registro
tiene abierto el mismo COM, cerrarla antes de abrir el monitor del IDE.
La correccion no espera indefinidamente a que se abra el monitor USB.

Referencias de hardware:
[Machine Control, pagina 2](https://docs.arduino.cc/resources/pinouts/AKX00032-full-pinout.pdf),
[MAX485](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1487-MAX491.pdf),
[TXS0108E](https://www.ti.com/lit/ds/symlink/txs0108e.pdf).

## Funciones migradas

Portenta envia estado, menu, calibraciones, limites, comandos de camara,
ACK de objetivo/pinza y la telemetria de encoder/Z. ESP responde con control
Bluepad32, servos, estado y ACK de camara, muestras de tags, objetivo estable,
coordenadas, clase, secuencia y referencia de encoder. Todos los modos de
Automatico V2 usan este mismo transporte.

Se conservan Automatico V2, Automatico, Manual, calibraciones, pruebas de
encoder/seguimiento, registro de angulo, ensenanza ML/ML V2, ajuste catch y
cambios catch. Se conservan velocidades, geometria, escalas, homografia,
calibracion de giro, pulsos de pinza, entrega y guardas DIN04/finales/control.
Las etiquetas humanas y catch por X siguen exclusivos de sus modos de
ensenanza; no pasan a ser requisitos ni excepciones del ciclo autonomo.

## Transporte y seguridad

- `ProtocoloRS485.h` conserva exactamente **32 bytes por paquete**, mismas
  posiciones de campos y CRC-8/ATM de la aplicacion. Version nueva: **17**.
  Los nombres de algunos enums conservan el sufijo historico `Wire`; sus
  numeros se mantienen para preservar los estados de la maqueta.
- `EnlaceRS485.h` encapsula cada paquete con sesion aleatoria de Portenta,
  solicitud de 32 bits, CRC16-CCITT-FALSE y delimitacion COBS. Cada trama
  en el cable ocupa **44 bytes**. Las dos copias de cada cabecera son identicas.
- Portenta coordina los turnos y mantiene **una solicitud pendiente**. ESP
  responde con su snapshot actualizado y la misma sesion/solicitud.
- Periodo nominal **10 ms**, sujeto al bus libre y al tiempo real de loop.
  Portenta procesa la respuesta sin una espera bloqueante; ESP espera 15 ms
  para ceder el bus antes de responder. Portenta espera 3 ms de silencio RX
  antes de solicitar otra vez. Ambos esperan el TX local antes de
  liberar DE. A 115200, cada trama tarda aproximadamente 3.82 ms en cable;
  dos tramas, el giro y los retardos de TX superan el periodo nominal de 10 ms.
- Respuesta dentro de **100 ms**, sin aceptar paquetes de otra solicitud ni
  respuestas tardias. Los fragmentos incompletos vencen a los **80 ms**;
  el receptor vacia los bytes disponibles antes de aplicar ese vencimiento.
- ESP rechaza solicitudes antiguas/duplicadas con carga diferente; una
  repeticion exacta puede responderse sin volver a aplicar control ni renovar
  la vigencia del encoder. Si se acumulan solicitudes, se aplica la ultima.
- Las ordenes inmediatas de cierre/apertura se envian al quedar libre el bus
  y cumplirse los 3 ms de silencio desde el ultimo byte recibido,
  y se construyen desde el estado actual, conservando los ACK idempotentes.
- Portenta declara perdida solo tras **mas de 1000 ms sin respuesta valida**.
  El snapshot congelado y los errores consecutivos son diagnosticos y no
  adelantan esa recuperacion. El cambio de sesion ESP sigue causando
  cancelacion inmediata. La recuperacion conserva calibracion XY/Z, pasos por
  mm, escalas y rangos aunque se interrumpa movimiento; detiene y cancela,
  vuelve al checklist/menu y no reanuda el movimiento cancelado.
- La recuperacion de UART Portenta solo se hace con ejes detenidos. ESP
  invalida el control remoto al perder Portenta. Un fallo de sesion aleatoria
  en Portenta deja el enlace detenido y se reintenta desde el estado seguro.
- CRC o recibir bytes no demuestra aplicacion de camara ni agarre fisico:
  siguen vigentes los ACK de la aplicacion y las guardas del ciclo.

## Registros y comprobacion en placas

`registrar_v2.ps1` y `analizar_catch_v2.py` estan copiados en esta carpeta.
El registrador conserva los CSV de ensayos y guarda el log continuo como
`registros_v2/rs485_AAAA-MM-DD_HH-mm-ss.log`; incluye las lineas `[RS485]` de
ambas placas. Ejemplo con los puertos observados por el usuario:

```powershell
powershell -ExecutionPolicy Bypass -File ".\automatico v2 rs485\registrar_v2.ps1" -PuertoESP COM14 -PuertoPortenta COM8
```

Los resumenes `[RS485]` aparecen cada segundo. En Portenta, `rxOK` confirma
respuestas correlacionadas; `timeout` cuenta solicitudes sin respuesta,
`ajena` respuestas atrasadas/de otra solicitud, `rtt`/`rttMax` los tiempos
desde el fin de TX. `len`/`crc` corresponden a errores de trama y `sem` al
protocolo/valores de aplicacion. En ESP, `ok` cuenta solicitudes nuevas validas,
`txOK` respuestas enviadas, `antigua` solicitudes descartadas y `crcTrama`,
`lenTrama`, `fragmentos` los errores de transporte. `rxBytes` ayuda a distinguir
ausencia de datos de datos rechazados. TX local completo no confirma recepcion.

`reinicios` cuenta reinicializaciones del bus UART, no reinicios de CPU.
`[BOOT] Estado general -> ...` informa un cambio de la maquina de estados.
La pantalla de estabilizacion aparece al recuperar el enlace durante cinco
segundos. Para distinguir un reinicio ESP, revisar cambios de `sesionArranque`,
la cabecera de arranque y los contadores. La perdida del enlace se declara
solo a mas de 1000 ms sin respuesta valida y conserva la calibracion y escalas
tanto con motores detenidos como con movimiento interrumpido. Un cambio de
sesion ESP se recupera inmediatamente conservando esas referencias. Un
reintento completo desde error reinicia la secuencia y exige calibrar de nuevo.

Tras cargar ambos sketches completos, registrar el arranque, verificar OLED,
conexion de control, camara y encoder, y observar que `rxOK`/`ok` avanzan sin
incrementos de errores. Registrar los tiempos reales bajo esa carga; la prueba
basica de un intercambio por segundo no mide este firmware a periodo nominal
10 ms ni el ruido de variador/motores. El firmware completo conserva sus
calibraciones y secuencias del brazo, por lo que el montaje y finales deben
estar listos antes de ejecutarlo. Esta migracion no fue cargada en placas.

## Verificacion local previa: correccion inicial a 115200

Pasaron las regresiones del perfil y el firmware completo (setup, transporte,
guardas, envio y cinco suites heredadas), el banco final (11000 intercambios,
20 casos y FAIL por desconexion) y las regresiones de Automatico V2 I2C
original. Se comprobo que los 12 fuentes originales de la base I2C y raiz
conservan sus hashes y que las cuatro copias de cada cabecera
`ProtocoloRS485.h` y `EnlaceRS485.h` son identicas. Logs en
`../tmp/rs485-115200-correccion/regresiones-v2.log`,
`regresion-banco.log` y `regresion-v2-i2c.log`.

Compilaron los cuatro sketches (banco y firmware completo) con Arduino CLI,
codigo de salida 0, Bluepad32 4.1.0 / mbed_portenta 4.6.0. Portenta conserva
advertencias heredadas de Arduino_MachineControl. Los resultados y logs de
esta revision se detallan en el README del banco final.
No se cargaron placas ni se acciono hardware. La prueba funcional confirmada
por el usuario verifica el montaje a 115200; sigue pendiente ejecutar el
banco final y el firmware completo con camara, encoder y control.

## Verificacion local historica, anterior a esta correccion

Resultados del **2026-10-07** sobre los fuentes de aquella revision:

- **ESP compilo:** core Bluepad32 4.1.0; 805461 bytes de programa (61 %) y
  104356 bytes globales (31 %). Compilacion con advertencias `default`, como
  configuracion de trabajo; `more` activa errores de variables sin uso en
  la biblioteca ESP32Servo 3.2.1 instalada. Esa biblioteca no fue modificada.
- **Portenta compilo:** core mbed_portenta 4.6.0; binario de aplicacion de
  257880 bytes. Advertencias heredadas por `Ticker::attach(float)` y por
  retorno/constructores en Arduino_MachineControl; no son errores de RS485.
- **8 grupos de regresion pasaron:** transporte, guardas, envio prioritario y
  cinco suites heredadas (ciclo/handshake/protocolo, giro, catch ML V2,
  calibraciones ML y vision modelo 129) ejecutadas sobre los fuentes nuevos.
- Transporte: **1000 viajes de ida/vuelta**, **3440 alteraciones de un bit**,
  truncado, ruido, fragmentos, CRC conocido, correlacion, timeout, duplicados,
  wrap de secuencia/millis y rechazo del protocolo 16 aun con CRC correcto.
- Las guardas reales detienen/cancelan e invalidan calibraciones ante perdida,
  secuencia congelada, reinicio ESP y paquetes invalidos. El envio real con
  UART simulada conserva cierre/apertura durante una solicitud pendiente y
  recupera TX incompleto sin dejar DE activo.
- Cabeceras nuevas identicas; **7 archivos originales** de los sketches V2
  conservan su SHA256 anterior. Registrador PowerShell verificado por parser.

Logs: `../tmp/rs485-migracion/compilacion-esp.log`,
`../tmp/rs485-migracion/compilacion-portenta.log` y
`../tmp/rs485-migracion/regresiones.log`.
La migracion completa no se cargo en placas ni se acciono hardware; su
verificacion fisica bajo carga de camara/encoder queda pendiente.

Comando reproducible para las regresiones (sin hardware):

```powershell
python tests/rs485_migracion_test.py --compiler "C:\Program Files\Webots\msys64\mingw64\bin\g++.exe"
```

El runner configura las DLL del compilador en su proceso, verifica OLED/pines,
ejecuta el arranque, el transporte compartido, las guardas y el envio reales de Portenta, y vuelve a
ejecutar las cinco suites existentes sobre los **fuentes nuevos**. Las capturas
historicas de vision/ML se leen desde su ubicacion original, sin copiarlas ni
alterarlas. Las compilaciones usan Arduino CLI y las bibliotecas instaladas
del proyecto; no requieren instalar otras bibliotecas.

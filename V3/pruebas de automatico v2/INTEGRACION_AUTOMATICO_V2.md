# Automatico V2 como modo integrador

Desde 2026-10-06, Automatico V2 recibe las mejoras aplicables de los otros
modos en la misma modificacion. La politica permanente esta en `../AGENTS.md`.
Esta integracion corresponde a los sketches ESP y PORTENTA de esta carpeta.

Actualizacion posterior del 2026-10-06: **AJUSTE CATCH V2** es una opcion
independiente que dispara automaticamente, entrega y espera LA AGARRO/ANTES/DESPUES.
Cada etiqueta adapta la espera del siguiente ensayo. **CAMBIOS CATCH** muestra
el valor probado, el proximo y la linea para llevarlo al codigo.
La mejora aplicable a Automatico V2 es el parametro compartido
`V2_AJUSTE_DISPARO_CATCH_MS`, inicial 0; conserva el disparo normal a 300 ms.
Los ensayos usan una copia temporal del parametro, sin cambiar seguimiento,
encoder, distancia ni giro. La confirmacion humana sigue siendo exclusiva de
la prueba. Ver `AJUSTE_CATCH_V2.md`. Protocolo **16**, paquetes de 32 bytes;
cargar ambos sketches juntos. El resumen tiene semantica propia y no se
publica en ESP como datos de encoder.

## Estado de integracion

### Politica vigente de recuperacion del enlace (2026-10-08)

La nueva instruccion expresa del usuario reemplaza el umbral previo de
150 ms y la invalidacion de calibraciones por movimiento interrumpido.
Se integra en los dos firmwares completos Portenta: esta base Automatico V2
I2C y `../automatico v2 rs485/`, para que todos sus modos compartan el
criterio. La politica comun se define en dos copias identicas de
`PORTENTA/RecuperacionEnlace.h`: `SIN_RESPUESTA_MS=1000` y `vigente()`.
Los sketches `ESP/` y `PORTENTA/` de la raiz no se sobrescriben.

Portenta mantiene el enlace vigente hasta **1000 ms inclusive desde la
ultima respuesta valida** y declara perdida/recuperacion/estabilizacion
solo cuando pasan **mas de 1000 ms** sin respuesta valida. En I2C exige CRC
y semantica de aplicacion; en RS485 tambien exige CRC de transporte y
correlacion con la sesion/solicitud pendiente. Ningun byte suelto, paquete
corrupto, respuesta ajena o tardia renueva ese reloj. Los errores consecutivos
y el snapshot congelado siguen disponibles como diagnostico y no fuerzan
recuperacion anticipada. Si llegan respuestas validas con snapshot congelado,
no se declara perdida por esa secuencia.

El timeout **100 ms por solicitud RS485** se conserva para seguir sondeando;
es distinto del umbral de recuperacion general. Las cuatro copias de
`EnlaceRS485.h` mantienen la correccion de **3 ms de silencio RX** antes de
otra solicitud, incluidas las urgentes; ampliar el umbral no evita la
perdida de bytes por solapamiento de DE.

Al declarar perdida, ambos Portenta detienen motores, cancelan el objetivo
pendiente y vuelven a espera/estabilizacion/checklist/menu; no reanudan el
movimiento cancelado. Conservan calibracion XY/Z, pasos por mm, escalas y
rangos aun si habia movimiento. Se conserva tambien esa referencia ante
un cambio de sesion ESP, cuya cancelacion/recuperacion sigue siendo inmediata
y separada de la caida transitoria. El usuario acepta que la posicion fisica
pueda no coincidir con el conteo conservado y que la ultima orden pueda
mantenerse mas tiempo durante una ausencia transitoria de respuestas.
Un reinicio real de Portenta o un reintento completo desde error mantienen
su ruta propia; esta politica no convierte esos eventos en recuperaciones
de enlace con datos persistidos.

Siguen vigentes finales, limites, DIN04, STOP/cancelacion, Bluetooth,
CRC/correlacion y las comprobaciones propias de camara y encoder.
Se conservan las etiquetas humanas de ensenanza, el giro ambiguo y el
criterio de ciclo sin agarre fisico verificado. No hay nueva orden ni
semantica incompatible: base **protocolo 16**, RS485 **17**, ambos paquetes
de **32 bytes** y sus copias de protocolo identicas. Exclusivos de RS485:
COBS de 44 bytes, turnos DE, correlacion y timeout de solicitud. Exclusivos
del banco: inyecciones y ensayos sinteticos, que no pasan al ciclo autonomo.

Verificacion de esta revision: paso `tests/recuperacion_enlace_test.py`
sobre RX/recuperacion reales I2C y RS485. Comprueba 999/1000 ms vigentes,
1001 ms perdido, respuesta que renueva el plazo, snapshot congelado con
respuestas validas, errores consecutivos, rechazo CRC/semantica/correlacion,
wrap del reloj, referencias conservadas, cancelacion sin reanudacion y
cambio de sesion ESP. Tambien pasaron setup, transporte, perfil, envio real
sin solapamiento de DE y cinco suites heredadas RS485, y ciclo/handshake I2C.
Las dos copias de `RecuperacionEnlace.h` son identicas. Logs en
`../tmp/recuperacion-enlace-1s/regresiones-v2-rs485.log` y
`regresiones-v2-i2c.log`. Los resultados anteriores no acreditan esta politica.

Compilaron los cuatro sketches completos ESP/Portenta I2C y RS485, codigo
de salida 0, con Bluepad32 4.1.0 y mbed_portenta 4.6.0. ESP RS485 usa
805617 bytes de programa y 104356 globales; ESP I2C usa 807781 y 104124.
Portenta conserva advertencias heredadas de sus bibliotecas. Logs en la
misma carpeta: `compilacion-rs485-esp.log`, `compilacion-rs485-portenta.log`,
`compilacion-i2c-esp.log` y `compilacion-i2c-portenta.log`.
Falta comprobar esta revision en placas. No se cargaron placas ni se
acciono hardware.

### Revision previa: correccion del retorno de turno RS485 (2026-10-08)

Se incorpora a todos los modos de Automatico V2 de
`../automatico v2 rs485/` y a su banco `../tests/RS485_FINAL_MIGRACION/`
una espera de **3 ms sin bytes RX en Portenta** antes de otra solicitud.
El coordinador podia volver a transmitir tras el ultimo byte de respuesta
mientras ESP todavia conservaba DE activo durante 200 us. El test ASCII
confirmado por el usuario esperaba 100 ms entre solicitudes y no exponia
este retorno inmediato. Los recortes de 1-2 bytes observados son compatibles
con esa omision, sin demostrar una unica causa electrica.

La condicion comun se implementa en `Cliente::registrarRecepcion()`,
`puedeIniciar()` e `iniciar()` de las cuatro copias de `EnlaceRS485.h`.
Cada byte recibido renueva la espera, tambien si pertenece a una trama
invalida o ajena. Se aplica a envios periodicos y urgentes de cierre/apertura
sin bloquear el loop. Aquella revision mantenia respuesta ESP con giro de
15 ms, post-TX de 200 us, timeout de 100 ms y guardas de seguridad de 150 ms.
No cambia protocolo 17, los 32 bytes de aplicacion ni los 44 bytes COBS.

Esta mejora de conmutacion half duplex es **exclusiva del transporte RS485**:
la base I2C de esta carpeta conserva protocolo 16, sus dos cabeceras
identicas y sus fuentes; no necesita una guarda de DE. Los sketches de raiz
tampoco se sobrescriben. Las inyecciones y paquetes sinteticos del banco
siguen exclusivos de diagnostico.

Los registros analizados ampliaban las capturas anteriores. En aquel firmware
la estabilizacion del enlace conservaba la calibracion si no interrumpia
movimiento. El episodio observado que vuelve al arranque y solicita calibracion se debe a un final
de carrera inesperado y un reintento seguro; los contadores no se reinician.
Solo se registra el inicio de esa segunda calibracion, no su finalizacion.
Esa cadena no explica todos los episodios: el usuario aclaro posteriormente
que otro corte ocurrio con el brazo en movimiento y no fue su reinicio manual.
El analisis con lineas y contadores esta en
`../automatico v2 rs485/DIAGNOSTICO_RS485_2026-10-08.md`.
En aquella revision se conservaban las guardas que invalidaban calibraciones
al interrumpir movimiento; la nueva politica vigente descrita arriba las
reemplaza para las recuperaciones del enlace.
Tambien se conservan la ensenanza, el giro ambiguo y la entrega sin agarre
fisico verificado.

Verificacion de esta revision: pasaron transporte, guardas, envios reales,
cinco suites heredadas y el banco completo con 11000 intercambios y 20 casos.
La simulacion ejecuta el loop Portenta dentro de `flush()` de ESP, antes
de liberar DE, y exige cero solapamientos. Verifica tambien urgente diferida
3 ms, bytes parciales sin renovar guardas y wrap del reloj. Logs en
`../tmp/rs485-giro-inverso/regresiones-v2.log` y `regresion-banco.log`.
Compilaron los cuatro sketches, codigo 0, Bluepad32 4.1.0 y
mbed_portenta 4.6.0. Logs en la misma carpeta: `compilacion-esp.log`,
`compilacion-portenta.log`, `compilacion-banco-esp.log` y
`compilacion-banco-portenta.log`. Se conservan advertencias heredadas
de Ticker/Arduino_MachineControl.
No se atribuyen a este cambio los resultados de la revision anterior.
La comprobacion del firmware
corregido en placas sigue pendiente. No se cargaron placas ni se acciono hardware.

### Revision previa: migracion desde el test confirmado a 115200 (2026-10-08)

El usuario confirmo que funcionaron
`C:/Users/samue/Downloads/ESP32_RS485_115200/ESP32_RS485_115200.ino` y
`C:/Users/samue/Downloads/PORTENTA_RS485_115200/PORTENTA_RS485_115200.ino`.
Esta es la referencia vigente del montaje: bus **115200, 8N1**, ESP
**RX14/TX27/DE18**, DE activo 200 us antes de TX y 200 us despues de flush;
Portenta `begin(115200, 0, 2000)` y terminacion interna desactivada, igual
que tras `comm_protocols.init()` en la prueba confirmada. La migracion tenia
BAUD=9600 y retardos Portenta de 200/200 us. Se corrigen velocidad, tiempos
y terminacion; Portenta usa la misma sobrecarga de tres argumentos del test
con 8N1 implicito. La llamada anterior de cuatro argumentos era valida.

La mejora aplicable se integra en todos los modos de la variante completa
`../automatico v2 rs485/` y en su banco `../tests/RS485_FINAL_MIGRACION/`.
Los parametros compartidos se mantienen en copias identicas de
`EnlaceRS485.h`: espera de giro **15 ms**, retardo previo/posterior ESP
**200/200 us**, retardo previo/posterior Portenta **0/2000 us** y timeout
de respuesta/fragmento **100/80 ms**. Los pines compartidos permanecen en
`ConfiguracionRS485.h`. En el test responde Portenta; en la migracion
responde ESP, que adopta la espera de 15 ms antes de transmitir. Portenta
conserva su papel de coordinador y una sola solicitud pendiente.

Se conservan protocolo **17**, paquetes de aplicacion de **32 bytes** y
tramas COBS de **44 bytes**. No hay nueva orden ni semantica incompatible.
Las dos copias del protocolo RS485 siguen identicas. La base I2C de esta
carpeta conserva protocolo **16** y sus dos copias identicas, como retorno;
los sketches ESP/PORTENTA de la raiz tampoco se sobrescriben. La
inicializacion Wire del expansor DIN interno sigue presente en Portenta.

Exclusivos del diagnostico: formato ASCII/CRC8, ESP como solicitante,
timeout de 400 ms, serie de 1000 mensajes y pausa de 100 ms. El banco
final conserva sus paquetes sinteticos e inyecciones PASS/FAIL, periodos
150/10 ms, limites internos 200/1500 s y ventana de fallo de 300 ms.
Esas inyecciones no pasan al firmware completo. Los 15 ms de giro y los
tiempos de TX impiden cumplir 100 Hz aunque el periodo nominal sea 10 ms.
Se conservan ciclos, guardas, calibraciones, etiquetas humanas exclusivas
de ensenanza y el criterio de entrega sin agarre fisico verificado.

Verificacion de esta correccion: pasaron setup, perfil, transporte, guardas,
envio y cinco suites heredadas del firmware completo RS485, el banco final
(11000 intercambios, 20 casos y FAIL por desconexion) y las regresiones de
la base Automatico V2 I2C. Las cuatro copias de cada cabecera
`ProtocoloRS485.h` y `EnlaceRS485.h` siguen identicas y los hashes de los
12 fuentes originales I2C/raiz no cambiaron. Logs en
`../tmp/rs485-115200-correccion/regresiones-v2.log`,
`regresion-banco.log` y `regresion-v2-i2c.log`.
Compilaron los cuatro sketches (banco y firmware completo), con codigo de
salida 0 y Bluepad32 4.1.0 / mbed_portenta 4.6.0. Portenta conserva
advertencias heredadas de Arduino_MachineControl. Los logs estan en
`../tmp/rs485-115200-correccion/compilacion-esp.log`,
`compilacion-portenta.log`, `compilacion-banco-esp.log` y
`compilacion-banco-portenta.log`.
La confirmacion fisica del usuario
valida el test ASCII a 115200; quedan pendientes el banco final y el
firmware completo corregido bajo carga. No se cargaron placas ni se
acciono hardware. Las instrucciones vigentes y el registro de verificacion
estan en `../automatico v2 rs485/README_RS485.md` y
`../tests/RS485_FINAL_MIGRACION/README.md`.

**Historial anterior a esta correccion:** las entradas siguientes describen
el estado y los resultados de cada revision previa. Sus asignaciones de
velocidad, tiempos, terminacion y referencias a pruebas pendientes no
sustituyen la configuracion vigente indicada arriba.

### Correccion de USB serial en el firmware completo RS485 (2026-10-08)

Se restauro `Wire.begin()` y `Wire.setClock(100000)` en el setup de
`../automatico v2 rs485/PORTENTA/PORTENTA.ino`, antes de `digital_inputs.init()`
y de leer finales. La migracion habia eliminado el enlace I2C externo y
tambien, por error, la inicializacion requerida por el expansor DIN interno.
La biblioteca Arduino_MachineControl lo accede mediante Wire; el core mbed
desreferencia su maestro I2C sin comprobar null. Esto explica un fallo de
arranque que puede dejar inaccesible el USB serial. No implica que UART4
de RS485 ocupe el puerto USB. Los bancos RS485 no acceden al expansor DIN.

La mejora aplicable a Automatico V2 queda en su variante completa RS485;
la base I2C de esta carpeta ya contiene ambas llamadas en el orden correcto.
Se mantienen protocolo 17/32 bytes de la variante, sus dos cabeceras
identicas, RX14/TX27/DE18 y todas las guardas/ciclos. No se modifica la ESP,
el firmware de raiz ni los tests funcionales entregados por el usuario.

Verificacion: `tests/rs485_migracion_test.py` ahora ejecuta el setup real
con simulacion de la dependencia Wire/DIN y comprueba inicializacion previa,
USB a 115200 y arranque seguro; paso junto a los ocho grupos anteriores.
El sketch Portenta corregido compilo con mbed_portenta 4.6.0, codigo de
salida 0; log en `../tmp/rs485-final-migracion/correccion-usb-compilacion-portenta.log`.
La validacion de reapertura del COM en la placa queda pendiente.
No se cargaron placas ni se acciono hardware.

### Revision de pines desde el test del usuario (2026-10-08)

La referencia es `../tests/nuevo test funcional/ESP32/ESP32.ino`: RX14
(RO, canal 1), TX27 (DI, canal 2), DE18 (DE y /RE, canal 3), UART2 a 9600.
El banco final y Automatico V2 de `../automatico v2 rs485/` coinciden en
RX14/TX27/DE18; ambos usan bus 115200. La UART recibe RX antes de TX en
begin(); la ESP inicializa DE en LOW y vuelve a LOW despues de flush(),
con DE alto solo durante TX. OLED21/22, camara32/33 y servos25/26 no colisionan.

Mejora integrada en la variante completa: el mensaje de arranque de la ESP
ahora muestra las constantes de pines reales; antes decia RX27/TX14 aunque
la configuracion ya era correcta. El README conserva el canal de direccion
probado sin imponer A2/B2 del montaje anterior y precisa el trenzado de A/B.
No cambia protocolo 17, paquetes de 32 bytes ni funciones/guardas autonomas.
La base I2C de esta carpeta conserva sus fuentes y protocolo 16.

Exclusivos/historicos: `RS485_COMUNICACION` y `RS485_CANALES` conservan
RX27/TX14 del montaje anterior; sus instrucciones no definen el montaje
vigente. Los tests Nano/Uno usan sus propios D2/D3/D4, no los GPIO de ESP.
Se identifica el diagnostico de canales como anterior sin alterar sus fuentes
ni el test funcional entregado por el usuario.

Verificacion: pasaron de nuevo `tests/rs485_final_migracion_test.py`
(11000 intercambios, 20 casos y FAIL por desconexion) y
`tests/rs485_migracion_test.py` (transporte, guardas, envio y cinco suites
heredadas). Los runners verifican pines y cabeceras identicas de protocolo
y transporte, incluyendo las dos copias del protocolo I2C original.
La confirmacion fisica a 115200 sigue pendiente; el test funcional del
usuario usa 9600. No se cargaron placas ni se acciono hardware.
Ambos sketches completos RS485 compilaron nuevamente con Arduino CLI:
ESP Bluepad32 4.1.0 (805545 bytes de programa, 104356 globales) y Portenta
mbed_portenta 4.6.0. Logs de esta revision en
`../tmp/rs485-final-migracion/revision-pines-compilacion-esp.log` y
`../tmp/rs485-final-migracion/revision-pines-compilacion-portenta.log`;
la compilacion Portenta termino con codigo 0 sin salida de diagnostico.

### Banco final de migracion RS485 (2026-10-08)

Se agrego `../tests/RS485_FINAL_MIGRACION/` partiendo del cableado de
`../tests/nuevo test funcional/`. Usa exactamente ProtocoloRS485 v17 y
EnlaceRS485 de la variante completa: 32 bytes de aplicacion y 44 en cable.
La documentacion inicial indicaba 115200 8N1, timeout 40 ms y fragmentos
20 ms; la correccion vigente arriba fija 115200 y 100/80 ms. Verifica 1000 intercambios
lentos, 10000 a periodo nominal 10 ms y 20 casos de corrupcion, version,
duplicados, respuestas ajenas/tardias/perdidas, recuperacion y nueva sesion.

Mejora aplicable integrada en Automatico V2 de `../automatico v2 rs485/`:
se corrigio **RX14/TX27/DE18** para coincidir con la prueba funcional del
usuario. `ConfiguracionRS485.h` se mantiene identica con el banco y su
regresion comprueba las copias. Se conserva protocolo 17, sin nueva orden
o semantica de aplicacion. Las dos copias de ProtocoloRS485/EnlaceRS485
y las dos del protocolo I2C original siguen identicas.

Exclusivo del banco: paquetes sinteticos, inicio con T, contadores/criterios
PASS/FAIL e inyecciones por numero de solicitud. No se trasladan las
inyecciones ni la ausencia de actuadores al firmware completo. Las guardas,
enseñanza, giro ambiguo y evento de entrega sin agarre verificado se conservan.
Los sketches I2C originales y los de raiz no se sobrescriben.

Verificacion local: `tests/rs485_final_migracion_test.py` ejecuta ambos
sketches completos con UART/reloj simulados; pasaron 11000 intercambios,
20 casos, contadores exactos, duplicado sin reaplicar y liberacion de DE.
La ronda con bus desconectado produce FAIL como corresponde. La aprobacion
fisica y el firmware completo bajo carga siguen pendientes; instrucciones
y criterios en `../tests/RS485_FINAL_MIGRACION/README.md`.
Compilaron los cuatro sketches finales (banco y variante completa; Bluepad32
4.1.0 / mbed_portenta 4.6.0). Pasaron tambien los 8 grupos de regresion de
la variante completa RS485 y el parser del registrador del banco, sin abrir
puertos. Logs en `../tmp/rs485-final-migracion/`; tamaños y advertencias
heredadas de bibliotecas en el README del banco.
No se cargaron placas ni se acciono hardware.

### Variante completa RS485 en carpeta nueva (2026-10-07)

Por solicitud expresa del usuario, la migracion completa del enlace
ESP32-Portenta se entrega en `../automatico v2 rs485/`, partiendo de los dos
sketches vigentes de Automatico V2. Conserva todos los modos, calibraciones,
coordenadas, motores, orientacion, catch/entrega, guardas y registros de esta
base. La pantalla conserva I2C GPIO21/22; HUSKYLENS conserva UART1 GPIO32/33.
La ESP usa UART2 RX14/TX27 (corregido el 2026-10-08) y direccion GPIO18 para el MAX485, con conversion
de nivel de 3.3 a 5 V. Machine Control usa TX P/TX N, half duplex.

La mejora integrada en **Automatico V2 de la nueva variante** es el transporte
RS485 comun de todos los modos: solicitudes/respuestas correlacionadas,
CRC16+COBS, rechazo de respuestas tardias y duplicados, prioridad para pinza,
recuperacion segura de UART y diagnostico de ambos extremos. Se mantienen
paquetes de aplicacion de 32 bytes; protocolo **17**, dos copias identicas de
ProtocoloRS485.h y EnlaceRS485.h. La trama fisica mide 44 bytes. La variante
original de esta carpeta conserva I2C/protocolo 16 como retorno; sus sketches
no se sobrescriben. Tampoco se modifican los sketches ESP/PORTENTA de la raiz.

La prueba anterior de comunicacion basica, exclusiva de diagnostico, queda en
`../tests/RS485_COMUNICACION/`. El registro del usuario mostro 162 ciclos
confirmados y sin nuevos invalidos despues de establecer el enlace, a 1 Hz.
Eso no valida el firmware completo con camara, encoder, control y actuadores.

Ver cableado, compilacion, regresiones y uso del registrador en
`../automatico v2 rs485/README_RS485.md`. La nueva version conserva las
restricciones de la ensenanza y la incertidumbre de agarre fisico de esta base.
No se cargaron placas ni se acciono hardware durante la migracion.

Verificacion de la variante completa: compilaron ESP (Bluepad32 4.1.0) y
Portenta (mbed_portenta 4.6.0). Pasaron 8 grupos de regresion sobre los fuentes
nuevos, incluyendo transporte corrupto/tardio, guardas reales, envio de pinza,
ciclo automatico/entrega, giro, catch ML V2, calibraciones y vision. Se
verifico igualdad de las cabeceras y SHA256 sin cambios de los 7 archivos
originales V2. Ver detalle y advertencias heredadas en README_RS485.

El 2026-10-07 se agrego una prueba **aislada de comunicacion RS485** en
`../tests/RS485_COMUNICACION/`. Por alcance expreso del usuario, solo intercambia
mensajes de 32 bytes, con secuencia, patron y CRC; no incorpora funciones del
brazo. Usa GPIO27/14 como UART2 de la ESP y RS485 integrado de Machine Control.
La direccion del transceptor ESP es configurable (manual en GPIO18 o automatica).
Esta prueba es exclusiva de diagnostico: no migra el transporte de Automatico
V2 ni cambia sus sketches, guardas o las dos copias identicas de ProtocoloI2C.h
(version 16). Su firma/version de diagnostico no es compatible con la maqueta.
Ver cableado, resultados de verificacion y criterio de prueba fisica en
`../tests/RS485_COMUNICACION/README.md`. No se cargaron placas ni acciono hardware.

La segunda revision del 2026-10-07 eleva el protocolo de **esta prueba** a
version 2 (misma firma R485 y 32 bytes). Cada placa envia datos propios
(contador independiente y reloj local) y exige un ACK exacto. Portenta cede
un turno a la ESP para evitar colisiones en el par half duplex. Los registros
distinguen confirmacion Portenta->ESP y ESP->Portenta, timeout por etapa y
bytes recibidos sin paquete valido. Esta revision sigue siendo exclusiva de
diagnostico previo a migrar el transporte: Automatico V2 conserva I2C y su
protocolo 16. Los resultados de compilacion y verificaciones locales estan
en su README; la validacion fisica del enlace bidireccional sigue pendiente.

Revision del 2026-10-08: se restauro exclusivamente la prueba aislada
`../tests/RS485_COMUNICACION/ESP/ESP.ino`: RX=27 y TX=14 estaban invertidos.
Se compararon ambos sketches y las cabeceras con los fuentes guardados de
la compilacion bidireccional v2 que funciono el 2026-10-07. Las cabeceras
siguen identicas, version 2, paquetes de 32 bytes. El cableado con convertidor
de nivel y la tierra comun se detalla en el README de la prueba. Esta
restauracion no modifica los firmwares completos: la variante Automatico V2
RS485 ya usa RX27/TX14/direccion18 y la base original conserva I2C. Es una
correccion exclusiva del diagnostico; no cambia funciones ni guardas del
brazo. No se cargaron placas ni se acciono hardware.

Verificacion de esta restauracion: ambos sketches aislados compilaron el
2026-10-08 con sus cores habituales. Las dos cabeceras se verificaron
identicas byte a byte y los fuentes coinciden con la compilacion anterior
tras excluir el preprocesado generado por Arduino. No se ejecuto una nueva
prueba fisica sobre la placa perforada.

Diagnostico del 2026-10-08: se agrega `../tests/RS485_CANALES/`, exclusivo
de comprobacion electrica. Solo usa la ESP: GPIO14 y GPIO18 alternan
niveles cada cinco segundos por separado y GPIO27 permanece como entrada
para comprobar B5->A5. Exige desconectar MAX485 y Portenta durante estas
etapas. No transmite ordenes del brazo ni modifica protocolos o funciones
de Automatico V2. La mejora aplicable ya esta presente en la variante
RS485: RX27/TX14/direccion18. Se documenta como aislar soldaduras,
convertidor y MAX485 antes de repetir la prueba bidireccional. No se
cargaron placas ni se acciono hardware.
El sketch ESP de canales compilo con Bluepad32 4.1.0 el 2026-10-08;
GPIO27 se verifico configurado solo como INPUT. Esta prueba no tiene
sketch Portenta. La pareja bidireccional ya verificada ese dia permanece
sin cambios; las mediciones fisicas de los canales estan pendientes.

Se agrega tambien `../tests/RS485_NANO/` el 2026-10-08 para Nano clasico
ATmega328P de 5 V. Es diagnostico exclusivo: permite comprobar salida A/B
y retorno estatico DI->A/B->RO del MAX485, con DE y /RE separados. El Nano
tambien puede generar niveles lentos de 0/5 V por el lado B del convertidor,
con ESP/MAX485 desconectados. No se integran estos gestos electricos al
firmware autonomo; no cambian protocolos ni funciones de Automatico V2.
No se cargaron placas ni se acciono hardware.
El sketch Nano compilo correctamente con Arduino AVR 1.8.8 para
`arduino:avr:nano:cpu=atmega328` el 2026-10-08. La pareja ESP/Portenta
ya verificada permanece sin modificaciones. Las mediciones fisicas del
modulo y convertidor estan pendientes.

Por aclaracion del usuario se agrega `../tests/RS485_NANO_PORTENTA/` el
2026-10-08, exclusiva de comunicacion entre Nano clasico y Machine Control.
Se adapta la prueba bidireccional a SoftwareSerial RX D2/TX D3, direccion
D4 y MAX485 directo de 5 V, DE-/RE unidos. Ambos extremos usan bus 9600
y monitores 115200. Cada placa envia datos propios y exige ACK por turnos;
paquetes de 32 bytes, protocolo de diagnostico 3 y dos cabeceras identicas.
No se cambia la velocidad o protocolo del firmware Automatico V2 RS485
ni sus funciones. Las pruebas estaticas anteriores se conservan identificadas
como diagnosticos electricos. No se cargaron placas ni acciono hardware.
Verificacion: ambos sketches nuevos compilaron el 2026-10-08 (Arduino AVR
1.8.8 y mbed_portenta 4.6.0), y se verificaron cabeceras identicas. La
regresion `../tests/rs485_nano_portenta_test.cpp` ejecuta los dos fuentes
reales con UART simulada: paso 100 ciclos bidireccionales, CRC corrupto,
ACK ajeno/perdido, datos perdidos, reconexion, version incompatible y
control half duplex. El montaje fisico sigue pendiente de comprobar.

Por la consulta de UART directa se agrega `../tests/UART_NANO_PORTENTA/`
el 2026-10-08. Es diagnostico exclusivo: TTL UART4 PA0/PI9 en TP78/TP83
de Machine Control segun esquema V1.1, adaptacion 5 V/3.3 V para Nano y
SP335 deshabilitado mediante SHDN, modo RS485, /RE y DE. No usa bornes
diferenciales TX P/TX N. Firma UTTL version 1, paquetes de 32 bytes y
cabeceras identicas; ACK en ambas vias a 9600. No cambia RS485 ni guardas
de Automatico V2. Requiere acceso identificado a los puntos TTL; no es
una sustitucion del modulo manteniendo sus bornes. No se cargaron placas
ni acciono hardware.

El 2026-10-08, al confirmar el usuario que dispone de un solo MAX485,
se agrega `../tests/MAX485_UN_MODULO_NANO_UNO/`. Es diagnostico local:
Nano TX D3 -> DI -> A/B internos -> RO -> Uno RX D2; el retorno Uno D3
-> Nano D2 es UART directo. DE a 5 V y /RE a GND, separados; A/B sin
cables externos. No simula un enlace RS485 entre dos extremos ni valida
conmutacion de direccion. Para esto se necesitan dos transceptores.
Mantiene 32 bytes, firma M485 y version diagnostica 4, cabeceras identicas,
bus 9600 y monitores 115200. No contiene mejoras funcionales aplicables
a Automatico V2; se conservan su firmware, protocolo y guardas.
Verificacion: ambos sketches compilaron con AVR 1.8.8 (Nano 5730 bytes
de programa/396 globales; Uno 5990/400). La regresion
`../tests/max485_un_modulo_nano_uno_test.cpp` paso 100 ciclos, CRC corrupto,
ACK ajeno, perdidas de ACK y datos, reconexion y version incompatible.
Es una simulacion logica: no verifica niveles electricos o interrupciones.
Se documento comparacion con UART directa para aislar el modulo.
No se cargaron placas ni se acciono hardware; prueba fisica pendiente.

| Mejora / origen | Comportamiento en Automatico V2 |
|---|---|
| Camara modelo 129 | Ya integrado: solo nombres exactos pieza6/pieza7, clases internas 6/7, homografia, cajas duplicadas y filtro compensado por encoder. |
| Coordenadas y encoder | Ya integrado: desfase X=-5 mm, Y=0, rueda de 49 mm, contador vivo, filtro de velocidad y referencia de imagen bloqueada durante el ciclo. |
| Calibracion de ejes de ML V2 | Integrado: misma funcion `CalibracionAnguloMLV2::sugerir`; X=59 para ambas clases, Y=155/166. Seguimiento Y tambien usa esta calibracion. |
| Giro ambiguo | Conserva el giro anterior y registra NA; no sustituye una caja ambigua por 90 grados. |
| Seguimiento Y | Integrado: comparte `seguirPiezaY` con el modo de prueba; Y sigue durante descenso final y cierre. |
| Disparo autonomo | Exige alineacion X/Y dentro de 5 mm durante 300 ms + `V2_AJUSTE_DISPARO_CATCH_MS` (actual 0), y recorrido restante para descenso, cierre y reserva de 200 ms. X no dispara. |
| Descenso y catch | Ya integrado: precaptura de 3000 pasos, cierre solo con DIN04, busqueda lenta limitada y cancelacion si falta el sensor. Ahora tambien valida alineacion Y al cerrar. |
| Entrega de ML | Integrado: comparte `iniciarTrasladoEntrega`; retirada Z y traslado X/Y en paralelo hacia X=max-10 mm, Y=0; baja a DIN04, abre, retira Z y rearma. |
| Seguridad / Manual | Ya integrado: calibraciones de sesion, limites, cancelacion con triangulo/STOP, perdida de control/I2C y retorno seguro de Z. El HOME por cuadrado sigue siendo una accion del modo Manual. |
| OLED y registrador | Integrado: fases 12..16, angulo aplicado, sugerencia y fuente, seguimiento, entrega y apertura. |

## Configuracion y compatibilidad

`AUTO_V2_APLICAR_GIRO_POR_CAJA=true` usa la calibracion de ejes. Con `false`
conserva el giro elegido en Manual. Las cajas de piezas diagonales pueden
aparentar un eje: la calibracion es inicial y no resuelve el giro continuo.

`AUTO_V2_SEGUIMIENTO_Y=true` activa el seguimiento autonomo; `false` conserva
el catch por umbral en la estacion fija Y=0 para comparar. Ambas rutas ejecutan
la entrega completa. El seguimiento usa pulsos ordenados del brazo, sin sensor
de posicion real de Y; el encoder mide la banda.

La integracion inicial uso **version 14**, conservando ambos paquetes
en 32 bytes. `ACK_OBJ_ABRIR_PINZA=6` abre en la entrega y mantiene la reserva;
`ACK_OBJ_COMPLETADO` se publica al terminar la retirada final. Cargar ambos
sketches juntos. ML tambien conserva su objetivo hasta la retirada final.

Las fases del ciclo con seguimiento son:
`0 -> 1 -> 4 -> 16 -> 11 -> 10 -> 12 -> 13 -> 14 -> 15 -> 7 -> 0`.
La fase 9 prepara Z al entrar y la fase 8 retira Z al cancelar. La ruta fija usa
6 en lugar de 16. Las fases anteriores 0..11 mantienen sus numeros.

Los controles manuales y la etiqueta humana de ML V2 siguen siendo propios
de ensenanza. Su excepcion de continuar tras rebase cuando X fue confirmado
no se aplica al catch automatico fijo. Las calibraciones sugeridas por nuevos
CSV requieren analizar los ensayos. El modo de ajuste aprende exclusivamente
una espera temporal dentro de limites; no infiere escala o distancia fisica.

## Verificacion y prueba fisica

`tests/auto_v2_integracion_test.py` ejecuta la maquina de estados real y el
handshake real con hardware simulado. Comprueba ciclo completo, seguimiento,
ruta fija, cierre unico, reserva durante apertura, fallos de sensor, encoder,
control, limites, timeout y destino de entrega.
`tests/auto_v2_giro_test.py` verifica calibracion, ambiguedad, clases y giro fijo.
Se conservan las pruebas de modelo 129, calibracion ML y catch manual ML V2.

Compilar ESP con `esp32-bluepad32:esp32:esp32` y Portenta con
`arduino:mbed_portenta:envie_m7`. Ambos sketches finales compilaron el
2026-10-06, con cores 4.1.0 y 4.6.0 respectivamente; pasaron las siete pruebas
de regresion indicadas en README_PRUEBAS, incluyendo CRC y rechazo de version
13 aun con checksum valido. Esos resultados corresponden a la integracion
inicial; el nuevo modo usa version 16 y tiene sus regresiones adicionales.
No se han cargado placas ni probado
fisicamente el seguimiento o la entrega de esta integracion.

El evento final `CICLO_ENTREGADO_NO_VERIFICADO` significa que termino la
secuencia de software; no etiqueta EXITO fisico. Probar primero a velocidad
baja, confirmar giro y recorrido Y, DIN04, destino derecho, apertura y retirada.

# Contexto vigente del proyecto: Automatico V2 por RS485

Fecha de corte: **2026-10-09**. Contexto preparado a partir de los fuentes y la
documentacion de esta carpeta. Describe la implementacion de software y el
montaje documentado; no constituye una inspeccion fisica del equipo.

## 1. Proyecto y alcance actual

Se desarrolla un brazo cartesiano de tres ejes **X, Y y Z**, con una garra
orientable, para detectar, capturar y entregar piezas de una banda
transportadora. La vision reconoce las clases `pieza6` y `pieza7`, convierte
sus coordenadas de imagen a milimetros y relaciona cada objetivo con las
cuentas del encoder de la banda.

**Automatico V2 es el modo integrador del proyecto.** El firmware vigente es:

- [ESP/ESP.ino](ESP/ESP.ino), junto con las cabeceras de su carpeta.
- [PORTENTA/PORTENTA.ino](PORTENTA/PORTENTA.ino), junto con sus cabeceras.

El enlace entre las dos placas es exclusivamente **RS485 a 115200 baudios,
8N1, half duplex**, con protocolo de aplicacion **19**. I2C se utiliza
localmente para la OLED de ESP y el expansor interno de Machine Control.
Todo el contexto de este documento se refiere a esa implementacion RS485.

El objetivo actual es ajustar y verificar la **captura autonoma en una
estacion Y fija**, manteniendo deteccion, calibraciones, coordenadas, giro,
encoder, motores, cierre, retirada, entrega, guardas, OLED y registros dentro
del mismo firmware.

## 2. Arquitectura y reparto de responsabilidades

```text
Mando Bluetooth ----------------------> ESP32
HUSKYLENS 2 <-- UART1 ----------------> ESP32 --> PWM --> servo de giro
OLED SH1106 <-- I2C local ------------> ESP32 --> PWM --> servo de pinza
                                           |
                                      UART2 + nivel logico
                                           |
                                         MAX485
                                           |
                                   RS485 A/B + GND
                                           |
                              Portenta H7 + Machine Control
                                  |          |          |
                              STEP/DIR    DIN00..05   encoder canal 0
                                  |          |          |
                           drivers X/Y/Z  seis finales  encoder de banda
                                  |
                           motores X/Y/Z
```

**Portenta coordina el sistema:** arranque, calibraciones mecanicas, lectura
de finales y encoder, generacion de pulsos, posicion cartesiana, checklist,
modos, reserva del objetivo, prediccion del catch, cancelacion y entrega.
Tambien inicia todos los intercambios RS485.

**ESP32 atiende los perifericos:** mando mediante Bluepad32, OLED, dos servos,
camara y homografia. Publica snapshots de control y vision y responde a
Portenta. La camara se atiende en una tarea FreeRTOS exclusiva; el loop
continua atendiendo Bluetooth, RS485, servos y pantalla durante esperas de
la biblioteca de vision. La telemetria compartida se publica con proteccion
de acceso concurrente.

La posicion del brazo se estima mediante pasos y calibracion de finales.
El encoder mide el movimiento de la banda; no mide la posicion de los tres
ejes ni detecta pasos perdidos. La confirmacion de servo transmitida es un
estado logico del comando aplicado.

## 3. Inventario de electronica

| Componente | Funcion y conexion vigente | Identificacion disponible |
|---|---|---|
| ESP32 | Bluetooth, vision, OLED, PWM de servos y UART del transceptor RS485 | Familia ESP32; la variante exacta del modulo/placa no se identifica en estos fuentes. |
| Arduino Portenta H7 | Coordinador; sketch compilado para el nucleo M7 | `arduino:mbed_portenta:envie_m7`. |
| Arduino Portenta Machine Control | Interfaz industrial: salidas de motores, entradas de finales, encoder y transceptor RS485 integrado | Se usa `Arduino_MachineControl`; las salidas son canales de la placa, no GPIO ordinarios. |
| HUSKYLENS 2 de DFRobot | Vision; UART1 de ESP32, 115200 | Biblioteca `DFRobot_HuskylensV2`; modelo personalizado 129. |
| Pantalla OLED SH1106 | Interfaz local de estado, menu, calibraciones y fases | `Adafruit_SH1106G`, 128x64, direccion `0x3C`. |
| Mando Bluetooth | Joysticks X/Y/Z y botones X, triangulo, circulo y cuadrado | Bluepad32; modelo comercial del mando no especificado. |
| Servo de rotacion | Orientacion de la garra; PWM GPIO25 | Modelo, torque, tension y fuente no especificados. |
| Servo de pinza | Apertura y cierre; PWM GPIO26 | Modelo, torque, tension y fuente no especificados. |
| Tres motores paso a paso X/Y/Z | Movimiento cartesiano | El firmware define STEP/DIR; no identifica fabricante, modelo, corriente ni paso mecanico de los motores. |
| Etapas/driver de X/Y/Z | Reciben STEP/PUL y DIR de Machine Control | Modelo, alimentacion, microstepping y cableado de potencia no identificados en la documentacion vigente. |
| Seis finales de carrera | Limites negativo/positivo de X/Y y abajo/arriba de Z | Logica NC conservada; software activa el limite con `!digital_inputs.read(...)`. Modelo fisico no especificado. |
| Encoder de banda | Conteo A/B e indice Z en canal encoder 0 de Machine Control | El sketch declara **E6B2-CWZ6C, 1024 P/R**, lectura nominal X2. |
| Rueda/acoplamiento del encoder | Convierte recorrido de banda a cuentas | Diametro nominal 49 mm, relacion encoder/rueda 1:1; requiere contraste fisico. |
| Modulo MAX485 del lado ESP | Conversion UART a par diferencial RS485 | Alimentacion documentada de 5 V; direccion manual con DE y /RE unidos. |
| Convertidor de nivel logico | Adapta TX, RX y direccion entre ESP32 3.3 V y MAX485 5 V | Modelo exacto no confirmado; la guia contempla OE si la unidad es TXS0108E. |
| Transceptor RS485 de Machine Control | Extremo Portenta del enlace half duplex | Integrado en Machine Control; se usa su API `comm_protocols.rs485`. |
| Alimentacion y referencia comun | 3.3 V para el lado ESP del convertidor, 5 V regulados para MAX485/lado B; GND comun del enlace | Fuentes, potencias y distribucion de alimentacion del conjunto no estan completamente inventariadas. |
| Accionamiento de la banda | Movimiento externo cuya velocidad se mide con encoder | La calibracion pide operar la banda al 50%; este firmware no define una salida de consigna al variador. Modelo y alimentacion pendientes de documentar. |
| PC y conexiones USB serie | Configuracion, diagnostico y registro de ambas placas | ESP: 460800; Portenta: 115200. Los numeros COM dependen del equipo. |

Los componentes sin identificacion completa se incluyen porque forman parte
de las interfaces del montaje documentado. No se les atribuyen modelos,
tensiones de potencia ni prestaciones ausentes de las fuentes vigentes.
En particular, no deducir la alimentacion de servos, camara, motores o
drivers a partir de las tensiones del convertidor RS485.

## 4. Conexiones y parametros electricos documentados

### ESP32 y perifericos

| Senal | Conexion |
|---|---|
| RS485 RX | GPIO14, UART2, desde RO de MAX485 por el convertidor de nivel. |
| RS485 TX | GPIO27, UART2, hacia DI de MAX485 por el convertidor. |
| RS485 direccion | GPIO18 hacia DE y /RE unidos, pasando por el convertidor. LOW recibe; HIGH transmite. |
| OLED SDA / SCL | GPIO21 / GPIO22; `I2C_Pantalla(1)`, 100 kHz, `0x3C`. |
| HUSKYLENS TX -> ESP RX | TX de camara -> GPIO32, UART1. |
| ESP TX -> HUSKYLENS RX | GPIO33 -> RX de camara, UART1, 115200. |
| Servo de giro | GPIO25. |
| Servo de pinza | GPIO26. |

`RS485_DIRECCION_MANUAL=true` corresponde al montaje actual. RO no va
directamente al GPIO de ESP; pasa por la adaptacion 5 V/3.3 V. El lado A del
convertidor usa 3.3 V y el lado B usa 5 V regulados. Si el convertidor real
es TXS0108E, la guia indica OE a 3.3 V. MAX485 VCC usa 5 V regulados.

### Bus diferencial

| MAX485 | Machine Control |
|---|---|
| A | RS485 TX P |
| B | RS485 TX N |
| GND | GND de comunicaciones |

RX P y RX N quedan libres en el montaje half duplex documentado. La
terminacion interna de Machine Control esta **desactivada** para conservar
el montaje de la prueba funcional confirmada. El README indica que el
modulo mostrado parece incluir una resistencia R7 marcada `121` (120 ohm);
eso debe comprobarse en la unidad real antes de afirmar la terminacion
instalada. El par trenzado es A/B; GND se lleva aparte y se comparte entre
ESP32, convertidor, MAX485 y Machine Control.

### Motores y finales de Machine Control

| Eje | Canal STEP/PUL | Canal DIR | Entrada limite negativo / abajo | Entrada limite positivo / arriba |
|---|---|---|---|---|
| X | DO4 | DO5 | DIN00 | DIN01 |
| Y | DO2 | DO3 | DIN02 | DIN03 |
| Z | DO0 | DO1 | DIN04, abajo | DIN05, arriba |

El ticker conmuta STEP cada **100 us**. Un paso completo requiere dos ticks;
con divisor 1 la frecuencia nominal es aproximadamente **5000 pasos/s**.
Los divisores son manual=1, posicion=1, calibracion rapida=2, HOME=2 y
calibracion lenta=8. El ultimo pulso HIGH se completa hasta el siguiente
tick, incluso ante una parada, sin agregar pasos.

La direccion positiva de X/Y usa DIR HIGH; la positiva de Z usa DIR LOW.
No hay una salida ENA separada definida entre estos seis canales. Los
limites fisicos de referencia configurados son **X=446 mm, Y=336 mm**, con
margen de seguridad de **2 mm**. Los pasos/mm se obtienen de la calibracion;
estos valores no verifican por si solos el recorrido ni la ausencia de
perdidas mecanicas.

DIN04 es el final de altura inferior de Z. Su activacion habilita la
confirmacion de altura para el cierre; no es un detector de pieza ni un
sensor de fuerza, contacto o retencion. No existe en esta revision un
sensor adicional que confirme agarre fisico.

## 5. Protocolo RS485 vigente

### Capa de aplicacion

Ambas direcciones usan **32 bytes**, version **19**, enteros multibyte
little-endian y **CRC-8/ATM** sobre los primeros 31 bytes. El checksum ocupa
el ultimo byte; no se transmiten floats, punteros ni padding. Los magic son
`0xE3` para ESP->Portenta y `0xA7` para Portenta->ESP.

| Direccion | Contenido principal |
|---|---|
| Portenta -> ESP | Estado general/menu/fase, flags de calibracion y finales, movimiento de ejes, error, comando y secuencia de camara, ACK/orden de objetivo, conteo/velocidad/escala/secuencia/estado/signo de encoder y distancia Z desde DIN04. |
| ESP -> Portenta | Secuencia y sesion de arranque ESP, estados de perifericos, joystick/botones, comandos aplicados de servos, estado/error/ACK de camara, muestras de cuatro tags, calidad del objetivo V2, clase, X/Y, secuencia de objetivo y cuenta de encoder de referencia. |

X/Y se codifican en decimas de mm, la velocidad en um/s y la escala en
nm/cuenta. `nmPorCuentaEncoder` comparte 18 bits de escala con 14 bits de
Z desde DIN04. En el estado CAMBIOS CATCH determinados campos se reutilizan
para mostrar el ajuste de ese modo; no interpretarlos como encoder real.

`reservadoV2` publica los flags `REFERENCIA_APROXIMADA=1`,
`ORIENTACION_AXIAL=2` y `GIRO_APLICADO=4`. La combinacion 7 identifica esos
tres atributos; no prueba asentamiento del servo, contacto ni agarre.

Los ACK de objetivo son: ninguno=0, aceptado=1, rechazado por rango=2,
cancelado=3, completado=4, cerrar pinza=5 y abrir pinza=6. Cierre y apertura
conservan la secuencia del objetivo reservado. El comando de camara tiene
su secuencia y confirmacion propias.

### Capa de transporte y turnos

```text
sesion Portenta uint32 + solicitud uint32 + aplicacion 32 bytes + CRC16
                          42 bytes crudos
                             |
                         COBS: 43 bytes
                             |
                     delimitador 0x00: 44 bytes en cable
```

El CRC exterior es **CRC16-CCITT-FALSE** (`poly=0x1021`, `init=0xFFFF`).
Portenta crea la sesion del transporte e inicia una solicitud de 32 bits;
ESP responde con esa misma sesion/solicitud. Solo hay una solicitud
pendiente. La sesion de arranque ESP es un campo de aplicacion distinto
de la sesion del transporte.

| Parametro | Valor vigente |
|---|---|
| Bus | 115200, 8N1, half duplex. |
| Periodo solicitado | 10 ms si el bus y el loop lo permiten; no es una tasa efectiva garantizada. |
| Espera ESP antes de responder | 15 ms. |
| DE ESP antes / despues de TX | 200 us / 200 us, con TX completado antes de liberar DE. |
| Portenta antes / despues de TX | 0 us / 2000 us. |
| Portenta antes de otra solicitud | 3 ms sin recibir bytes, tambien para ordenes urgentes de pinza. |
| Timeout de solicitud | 100 ms desde fin de TX. |
| Timeout de fragmento | 80 ms. |
| Perdida general de enlace | Mas de 1000 ms desde la ultima respuesta aceptada. |

A 115200, cada trama de 44 bytes requiere aproximadamente 3.82 ms en cable.
El intercambio completo, los 15 ms de giro y los retardos de TX superan
10 ms. La frecuencia real y las latencias deben observarse en los registros.

CRC correcto no basta: se verifican version, longitud, valores de aplicacion
y correlacion con la solicitud pendiente. Ruido, paquetes corruptos,
respuestas ajenas y tardias no renuevan la vigencia. Una solicitud repetida
exactamente puede responderse sin reaplicar control ni renovar una muestra
de encoder; una repeticion con otra carga se rechaza.

### Recuperacion

El timeout de una solicitud libera el turno para un nuevo sondeo. Portenta
conserva el enlace hasta 1000 ms inclusive; lo declara perdido al superar
ese plazo sin respuesta valida. Errores consecutivos o una secuencia de
snapshot congelada no adelantan por si solos esa recuperacion.

Al perder el enlace se detienen motores, se cancela el objetivo y se vuelve
a espera/estabilizacion/checklist/menu. Se conservan calibraciones XY/Z,
pasos/mm, escalas y rangos de la sesion, incluso si se interrumpio movimiento;
el movimiento cancelado no se reanuda automaticamente. Conservar referencias
no demuestra que sigan coincidiendo con la posicion fisica.

Un cambio de sesion de arranque ESP cancela y recupera inmediatamente,
conservando esas referencias. Un reinicio real de Portenta o un reintento
completo desde error tiene su propia secuencia y puede exigir recalibrar.
La recuperacion de UART Portenta se realiza con ejes detenidos. ESP invalida
el control remoto al perder Portenta.

## 6. Arranque, calibracion y vision

El arranque inicia salidas en estado seguro, espera la ESP y estabiliza el
enlace durante cinco segundos. Portenta inicializa `Wire` a 100 kHz antes
de utilizar las entradas del expansor interno. La secuencia integra
calibracion de camara, brazo y encoder, espera de mando y checklist antes
del menu.

La camara usa **cuatro tags**, **25 muestras por tag** y una homografia
pixel->mm sobre coordenadas de imagen 640x480. La geometria configurada es:

- Banda blanca de **292 mm**: X admisible del centro +/-146 mm.
- Ancho total de referencia de **412 mm**.
- Centros laterales de tags a **X=+/-176 mm**.
- Separacion de filas de tags de **382 mm**: Y calibrada +/-191 mm.

Se distingue `FUERA_BANDA` de `FUERA_CALIBRACION`. La ruta autonoma exige
centro y caja dentro del area calibrada y caja completa dentro de imagen.
Agrupa duplicados, pero rechaza varias piezas unicas, caja recortada,
orientacion ambigua y referencia no admisible.

El modelo vigente es **129** (`ALGORITHM_CUSTOM_BEGIN + 1`). La clase se
obtiene del nombre exacto `pieza6` o `pieza7`; un ID numerico aislado no
identifica la clase. V2 usa tres detecciones estables y comprueba trayectoria
frente al encoder.

La transformacion de montaje configurada no intercambia X/Y, aplica
signo X=-1 e Y=-1 y offset X=-5 mm, Y=0 mm. La distancia camara-HOME Y
compilada es **845 mm = 510 + 335**. Son parametros actuales del codigo,
cuya coherencia debe contrastarse con geometria fisica y registros.

El encoder nominal usa **2048 cuentas/vuelta**, rueda de 49 mm y relacion
1:1: aproximadamente **0.07517 mm/cuenta**. La calibracion de arranque pide
banda al 50%, estabiliza 2 s y mide 5 s para determinar sentido y velocidad;
no sustituye la comprobacion independiente de mm/cuenta contra recorrido.

El giro busca cerrar sobre el ancho menor. El consenso axial de la caja usa
relacion minima 1.35 y los angulos de montaje de `CalibracionAnguloMLV2`:
eje largo X -> 59 grados para ambas clases; eje largo Y -> 155 para clase 6
y 166 para clase 7. Estos angulos son calibracion preliminar del servo.
La caja del modelo 129 no mide giro diagonal continuo; sin consenso se
conserva el ajuste previo y se rechaza el objetivo autonomo.

## 7. Ciclo autonomo de captura fija

1. Validar y reservar una pieza con identidad, coordenadas y cuenta de
   encoder de referencia.
2. Preparar X de la pieza, **Y=0**, garra abierta, giro y Z en precaptura
   sobre DIN04. Esperar asentamiento de X y giro.
3. Mantener Y fija y actualizar la posicion de pieza por delta real de
   cuentas. Recalcular la prediccion mientras espera.
4. Disparar el descenso final cuando la prediccion de contacto es viable;
   seguir vigilando encoder, velocidad/aceleracion, finales y ventana.
5. Confirmar DIN04 y viabilidad antes de ordenar cierre. Si Z termina por
   conteo, se permite hasta 40 ms de confirmacion con Z quieto. No se busca
   por debajo del limite en esta ruta.
6. Confirmar respuesta ESP posterior a la orden, misma secuencia y
   `servoPinza=130`; completar el tiempo de cierre antes de retirar.
7. Subir Z, trasladar a entrega derecha, bajar, abrir y subir a posicion
   segura. Registrar ciclo y volver a espera.

La apertura/cierre configurados son 0/130 grados, con pulsos de pinza
**937/1816 us**. No se requiere catch manual con X ni etiqueta humana para
que el ciclo autonomo continúe.

La posicion ya recorrida se calcula con cuentas:

```text
y_actual = y_referencia + signo * mm_por_cuenta * delta_cuentas
y_contacto = y_actual + v * t_futuro + 0.5 * a * t_futuro^2
```

Velocidad y aceleracion predicen el horizonte futuro corto; no reconstruyen
el recorrido pasado. La referencia de encoder alrededor de una consulta de
camara es aproximada. No existe timestamp real de exposicion sincronizado:
retardo y jitter calibrados estan en -1, y los registros mantienen
`capture_timestamp=NA`, `image_age_known=0` y `camera_delay_compensated=0`.

## 8. Configuracion de ensayo y trabajo mas reciente

Actualmente estan compilados `V2_HABILITAR_PRUEBAS_CATCH=true` y
`V2_CAPTURA_FIJA_VALIDADA=false`. La OLED muestra **AUTOMATICO V2**, fase y
**VALORES NOMINALES**. El ensayo decide con prediccion nominal y conserva
la envolvente provisional en registros. El perfil fisico permanece pendiente.

| Parametro | Configuracion actual |
|---|---|
| Catch Y | 0 mm; seguimiento Y autonomo desactivado. |
| Precaptura Z | 3000 pasos por encima de DIN04; descenso nominal final de 0.600 s con divisor 1. |
| Cierre nominal | 450 ms. |
| Presupuesto de orden de pinza | 158 ms; no equivale a una latencia fisica medida. |
| Horizonte inicial sin ajuste | 1.208 s, recalculado con Z y cierre restantes. |
| Ventana nominal Y | +/-5 mm. |
| Velocidad nominal de ensayo | 1..150 mm/s; perfil estricto configurado con maximo 100 mm/s. |
| Limite de aceleracion | 200 mm/s2. |
| Ventana de estimacion / edad maxima | 100 ms / 150 ms. |
| Asentamiento X / giro | 80 ms / 450 ms. |
| Permanencia maxima abajo | 80 ms. |
| Confirmacion final DIN04 | Hasta 40 ms, Z quieto. |
| Ajuste de catch por defecto | 0 ms; rango por terminal -500..+500 ms. |

La revision mas reciente integra:

- **Ajuste temporal desde terminal Portenta:** `CATCH +100` adelanta,
  `CATCH -100` retrasa, `CATCH` consulta y `CATCH 0` elimina el ajuste.
  El valor es absoluto, queda en RAM y se reserva para la siguiente pieza;
  no cambia una reserva activa. `V2_DESFASE_CATCH_MS` permite fijar el
  predeterminado en codigo. El perfil validado no aplica este ajuste.
- **Pulsos STEP completos:** el ultimo HIGH de X/Y/Z dura hasta el siguiente
  tick; la mejora beneficia todos los modos de este firmware RS485.
- **Confirmacion DIN04 posterior al ultimo paso:** registros `DIN04_WAIT`
  y `DIN04_TIMEOUT`; cancela sin cerrar si no aparece o se pierde viabilidad.
- **Prediccion nominal ante cuantizacion:** suprime aceleracion que no supera
  la incertidumbre de la propia ventana; preserva datos originales y
  cancelaciones ante cambios distinguibles.
- **Diagnostico diferenciado:** ancho de banda, region calibrada, velocidad,
  ambiguedad, apertura, edad de muestra y causa final de cancelacion.
- **Registro del ajuste configurado y reservado**, prediccion nominal,
  envolvente provisional y `DESCEND_ABORT`.

Las guardas de DIN04, finales, limites, mando/enlace, encoder fresco,
pinza abierta, asentamiento, parada/reversa, aceleracion y confirmacion ESP
siguen vigentes. El ensayo no aplica como mediciones las cotas provisionales
de arrastre de 10 mm y retirada de 0.25 s; el perfil estricto si utiliza su
envolvente y esos limites cuando se configure como validado.

## 9. Modos y herramientas disponibles en este mismo firmware

El menu vigente incluye Manual, Automatico, calibracion de brazo, calibracion
de camara, Automatico V2, Calibraciones, Prueba servos, Diagnostico,
Ensenanza ML, Prueba encoder, Registro angulo, Ensenanza ML V2,
Prueba seguimiento, Ajuste catch y Cambios catch. Todos utilizan el enlace
RS485 descrito.

Las correcciones por joystick, cierre manual con X y etiquetas EXITO/FALLO
pertenecen a sus modos de ensenanza/ensayo. No se transfieren como requisitos
del ciclo autonomo ni excepciones a sus guardas.

La terminal Portenta a **115200** acepta `AYUDA`, `POS`, `RANGO`,
`ENC ESTADO`/`ENCODER`, `ENC VUELTA`, `ENC CERO`, `CATCH`, `STOP` y
`REINTENTAR`. `GOTO X Y Z` y `HOME` tienen restricciones de menu/enlace/
calibracion; el comando de coordenadas mueve X/Y y no acciona Z.

[registrar_v2.ps1](registrar_v2.ps1) lee los dos puertos USB, guarda logs
continuos con lineas `[RS485]` y CSV de ensayos en `registros_v2/`.
[analizar_catch_v2.py](analizar_catch_v2.py) analiza esos registros.
`rxOK`/`ok` cuentan intercambios aceptados; `timeout`, `ajena`, errores de
longitud/CRC/semantica y `rtt` ayudan a diagnosticar el enlace. `reinicios`
del resumen cuenta reinicializaciones de UART, no reinicios de CPU.

## 10. Verificacion disponible y pendientes

La documentacion vigente registra compilacion correcta de los dos sketches
con **Bluepad32 4.1.0** y **mbed_portenta 4.6.0**. Dependencias principales:
Arduino_MachineControl, mbed, Wire, Bluepad32, ESP32Servo, Adafruit_GFX,
Adafruit_SH110X y DFRobot_HuskylensV2.

Tambien registra regresiones offline de transporte/correlacion,
arranque/recuperacion, deteccion/OLED, ciclo/entrega, encoder/prediccion,
terminal y CSV. La revision de desfase documenta nueve ciclos a 20/40/128
mm/s con ajustes -100/0/+100 ms y 60 casos de pulsos/paradas X/Y/Z.
El banco final documenta 11000 intercambios y 20 casos, sin actuadores.
Estos son resultados documentados de revisiones de software; no se
repitieron esas suites al preparar este contexto.

La prueba funcional RS485 basica a 115200 fue confirmada por el usuario.
Eso no acredita el firmware completo bajo carga de camara, encoder,
Bluetooth, motores y banda. Las revisiones mas recientes documentan
verificacion offline y dejan la comprobacion fisica pendiente.

Falta medir y documentar:

- Inventario comercial y cableado de potencia: motores, drivers,
  microstepping, servos, fuentes, mando, finales y variador; modelo exacto
  del convertidor y terminaciones instaladas.
- Escala/signo del encoder contra recorrido real, acoplamiento y posible
  deslizamiento; homografia, centro de agarre y distancia hasta Y=0.
- Desfase/jitter de imagen, asentamiento de X/giro, descenso, orden hasta
  contacto, cierre y retirada con carga representativa.
- Ventana util de garra, deriva lateral, variacion admisible de v/a,
  altura libre y arrastre admisible.
- Comportamiento del enlace con el montaje completo y observacion de
  retencion y entrega fisicas.

La meta de aceptacion es **al menos 95 agarres fisicos de 100 intentos
ejecutados** dentro de un perfil validado. El disparo de descenso inicia
el intento; una cancelacion posterior cuenta como fallo. Rechazos previos
y cobertura se registran por separado. Un ciclo completado, DIN04, PWM,
ACK o prediccion centrada no demuestran agarre fisico. Esa meta no esta
acreditada por la revision vigente.

## 11. Fuentes y reglas para continuar

| Fuente | Contenido que debe consultarse |
|---|---|
| [AGENTS.md](../AGENTS.md) | Automatico V2 integrador, desarrollo RS485, compatibilidad y restricciones de hardware. |
| [README_RS485.md](README_RS485.md) | Montaje RS485, firmware, cableado, transporte, recuperacion y resultados registrados. |
| [CAPTURA_FIJA_V2.md](CAPTURA_FIJA_V2.md) | Ciclo actual, ensayo nominal, ajuste de catch, guardas y validacion pendiente. |
| [ESP/ESP.ino](ESP/ESP.ino) | Perifericos, vision, homografia, reserva, servos, OLED y respuesta RS485. |
| [PORTENTA/PORTENTA.ino](PORTENTA/PORTENTA.ino) | Canales industriales, calibraciones, motores, encoder, ciclo, entrega y terminal. |
| [ProtocoloRS485.h](ESP/ProtocoloRS485.h) | Version, paquetes de 32 bytes, campos, flags, ACK y CRC de aplicacion. |
| [EnlaceRS485.h](ESP/EnlaceRS485.h) | COBS, CRC16, sesiones, turnos, tiempos y correlacion. |
| [ConfiguracionRS485.h](ESP/ConfiguracionRS485.h) | RX14, TX27 y DE18. |
| [VisionModelo129.h](ESP/VisionModelo129.h) | Nombres de clases, modelo y limitacion del giro por caja. |
| [CalibracionAnguloMLV2.h](ESP/CalibracionAnguloMLV2.h) | Mapeo axial preliminar del servo. |
| [CapturaFijaV2.h](PORTENTA/CapturaFijaV2.h) | Modelo portable de estimacion y prediccion. |
| [AjusteTemporalCapturaV2.h](PORTENTA/AjusteTemporalCapturaV2.h) | Rango, parser y reserva del ajuste temporal. |
| [RecuperacionEnlace.h](PORTENTA/RecuperacionEnlace.h) | Politica de perdida general a mas de 1000 ms. |

Las cabeceras `ProtocoloRS485.h` y `EnlaceRS485.h` de ESP y Portenta se
comprobaron identicas por SHA256 al preparar este documento. La version
autoritativa es la constante del protocolo (**19**); algunos textos
literales de diagnostico no estan actualizados y no deben usarse para
deducir la version del paquete.

Al continuar, mantener iguales las copias del protocolo y transporte.
Una orden nueva o semantica incompatible requiere incrementar version
conservando 32 bytes, verificar ambos sketches y sus regresiones.
Integrar en Automatico V2 las mejoras aplicables de otros modos y
documentar sus exclusividades. Conservar el giro previo ante falta de
consenso. No cargar placas ni accionar hardware sin solicitud del usuario.

Este contexto se preparo mediante lectura y contraste de archivos; no
modifica firmware, parametros, cableado ni estado de las placas.

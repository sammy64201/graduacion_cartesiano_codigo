# Pruebas de Automático V2

Esta carpeta es una maqueta independiente. Los sketches finales de `ESP/` y
`PORTENTA/` no forman parte de estas pruebas ni deben sobrescribirse al cargar
las placas.

La escala cartesiana usada después de recorrer los finales es actualmente
X=446 mm y Y=336 mm.

## Orientacion aproximada y rueda de 49 mm

Automatico y Automatico V2 orientan el servo de rotacion cuando publican un
objetivo confirmado. El Automatico original mantiene su movimiento XY; V2
mantiene ademas la secuencia de catch con encoder y cierre de pinza.
Compara el ancho y alto de la caja detectada, transformados al plano calibrado,
en las detecciones consecutivas (cuatro en Automatico, tres en V2). Si al menos
dos cajas coinciden y el eje
mayor supera al otro por un factor de 1.35, ordena `ANGULO_GARRA_EJE_X` (90°)
o `ANGULO_GARRA_EJE_Y` (0°). Si la forma es casi cuadrada, diagonal o las
lecturas discrepan, usa el angulo inicial de 90°. Los dos angulos de servo son
parametros de montaje en `ESP/ESP.ino`; verificar su correspondencia fisica
con los ejes antes de un catch. La caja del modelo no contiene una rotacion
firmada: este metodo solo distingue aproximadamente los ejes X e Y. En
Ensenanza ML se usa la regla experimental de cajas anchas/altas descrita abajo
y se conserva el ajuste manual de la rotacion.

La rueda del encoder ahora mide 49 mm. Con 2048 cuentas X2 por vuelta y
acoplamiento 1:1, la escala geometrica es aproximadamente 0.075165 mm/cuenta.
El catch automatico existente sigue usando esa escala y la velocidad medida;
su resultado de software no confirma por si solo que la pieza haya quedado
sujeta. Tras cargar ambos sketches, comprobar una distancia conocida y luego
probar el catch a baja velocidad observando el log de orientacion, el evento
`GRIP_COMMAND` y la posicion real de la pieza. Revisar tambien
`V2_AJUSTE_DISTANCIA_CATCH_MM`: se ajusto con mediciones anteriores y puede
necesitar correccion despues del cambio de rueda.

## Contenido

- `ESP/ESP.ino`: HUSKYLENS, Bluepad32, OLED e I²C esclavo `0x40`.
- `PORTENTA/PORTENTA.ino`: calibración X/Y/Z, encoder ABZ, motores y máquina
  de estados de interceptación.
- Los dos archivos `ProtocoloI2C.h` son copias idénticas del protocolo de
  prueba versión 12. Ambos paquetes miden exactamente 32 bytes y terminan con
  CRC-8/ATM. Los 14 bits superiores de `nmPorCuentaEncoder` llevan los pasos
  de Z desde DIN04; los 18 inferiores mantienen la escala del encoder.
- `registrar_v2.ps1`: abre simultáneamente los USB de ambas placas y crea un
  registro combinado continuo desde el arranque, además de un CSV sincronizado
  por cada entrada a Automático V2 o Enseñanza ML/ML V2.

Después de este cambio se deben cargar **ambos** sketches: la versión 12 del
protocolo rechaza una placa que todavía ejecute una versión anterior.

## Arranque y menú

Al encender, los motores esperan a que ESP32 y Portenta completen tres
comprobaciones I²C de ida y vuelta con CRC. La ESP debe informar además que la
OLED está inicializada. Solo después comienza la estabilización de cinco
segundos. Al terminar, la Portenta envía el estado de confirmación y la OLED
permanece en una checklist que muestra únicamente `PORTENTA` y `CAMARA`. Con
ambos en `OK`, se conecta el control y se presiona `X` para entrar al menú.

Si el enlace se pierde después de haber quedado estable, el movimiento se
detiene y no se inicia otra estabilización automáticamente. La Portenta recupera
el bus, repite el triple check y espera que el control esté conectado y se pulse
`X`. Si había un motor en movimiento, se invalida la calibración del brazo para
volver a medir sus límites antes de moverlo.
Ninguna de las tres calibraciones se ejecuta por obligación al arrancar.

El menú contiene `MANUAL`, `AUTOMATICO`, `AUTOMATICO V2`,
`REGISTRAR ANGULO`, `CALIBRACIONES`, `PRUEBA SERVOS`, `ENSENANZA ML`,
`ENSENANZA ML V2`, `SEGUIMIENTO Y`,
`PRUEBA DE ENCODER` y `CHECKLIST`. Se navega con el joystick izquierdo vertical,
X entra y triángulo regresa. Al elegir un modo se ejecutan, una sola vez por
sesión, las calibraciones que todavía falten:

| Modo | Calibraciones necesarias |
|---|---|
| Manual | Brazo X/Y/Z y sus finales |
| Automático | Brazo y cámara |
| Automático V2 | Brazo, cámara y encoder |
| Registrar ángulo | Brazo y cámara |
| Enseñanza ML | Brazo, cámara y encoder |
| Enseñanza ML V2 | Brazo, cámara y encoder |
| Seguimiento Y | Brazo, cámara y encoder |

En **Manual**, el boton **cuadrado** devuelve el brazo al HOME que fijo la
calibracion (`X=0, Y=0, Z=0`). Detiene el movimiento manual actual, lleva Z
a 0 y despues X/Y a 0. Los joysticks quedan ignorados durante el retorno y
deben soltarse antes de volver al control continuo. Triangulo o la perdida del
control cancelan el retorno. Portenta informa inicio, etapas y final por la
terminal. Cargue las versiones ESP y Portenta juntas porque comparten la
version 13 del protocolo I2C.

Al volver al menú y reentrar, una calibración válida se conserva. `CALIBRACIONES`
permite repetir independientemente brazo, cámara o encoder. La calibración de
cámara conserva la verificación de las cuatro marcas, homografía y apertura y
confirmación del modelo personalizado 128. El `CHECKLIST` del menú muestra comunicación I²C con Portenta, conexión
de cámara, modelo, control, estado de calibraciones, pulsos de encoder y
coherencia de finales. X alterna sus dos páginas; no bloquea el acceso al menú.

Automático V2 permite reentrar siempre que las tres calibraciones de la sesión
sigan válidas. La entrada ya no depende de que la banda se esté moviendo, de
una bandera instantánea del encoder ni de que Z se encuentre exactamente en
HOME. Si Z quedó en otra altura válida, V2 la lleva automáticamente a la
posición segura antes de habilitar la detección.

## Registrar ángulo de catch

Este modo usa la deteccion y el posicionamiento XY del Automatico normal, con
la banda detenida. Detener la banda antes de entrar: este firmware no controla
el motor de la banda. No requiere movimiento ni calibracion del encoder.
Como prueba experimental, si la caja de una pieza de clase 0 es muy ancha
(`width/height >= 1,5`) sugiere 60 grados; si es muy alta
(`width/height <= 0,75`) sugiere 156 grados. En las demas cajas no sugiere
angulo y conserva el ajuste manual. Estas reglas provienen solo de cinco
etiquetas, sin evaluacion fisica del agarre. La OLED muestra `SUG:` cuando
hay sugerencia o `MANUAL` cuando no la hay, junto con `ROT:`. El joystick
derecho siempre permite corregir el giro.
Al llegar a XY, el brazo espera al menos 2,5 segundos y hasta que el servo
lleve 1 segundo sin cambiar. Luego baja Z hasta DIN04, cierra la pinza y
retira Z automaticamente. X no dispara el catch; triangulo cancela y vuelve
al menu. La sugerencia no reemplaza el angulo que el operador eligio.

Iniciar `registrar_v2.ps1` antes de entrar. Al seleccionar `REGISTRAR ANGULO`
se crea `registros_v2/angulo_AAAA-MM-DD_HH-mm-ss.csv`. La fila `ANGLE_SAMPLE`
guarda `objective_seq`, `class`, `camera_x_mm`, `camera_y_mm` y `label_rot_deg`
cuando termina el cierre ordenado. La fila `ANGLE_DETECTION` de la ESP32 con
el mismo `objective_seq` contiene `pixel_x`, `pixel_y`, `width_px`, `height_px`
y `confidence`, ademas de `approx_rot_deg`, `suggested_rot_deg` y los votos
del estimador actual. La fila `ANGLE_FEEDBACK` registra el angulo sugerido,
el aplicado al cerrar y la magnitud de la correccion manual.
El modelo personalizado de esta prueba devolvio `confidence=-128`, que no es
una probabilidad utilizable; las siguientes versiones registran `NA` en ese
caso. Las cajas son paralelas a la imagen y no muestran el sentido diagonal
de la pieza. Por eso dos cajas parecidas pueden requerir angulos distintos:
No se ha actualizado el giro de AUTOMATICO ni AUTOMATICO V2; esta sugerencia
solo se prueba en REGISTRAR ANGULO y admite correccion manual.
Retirar la pieza del campo de vision despues de cada intento
para que el detector pueda reconocer la siguiente. `physical_result=NO_VERIFICADO`
indica que no hay sensor que confirme el agarre.
Subir ese CSV al terminar las pruebas para ajustar los modos automaticos.

## Prueba de encoder (sin movimiento del brazo ni catch)

Cargar los sketches de esta carpeta en **ambas placas** y reiniciar el
registrador `registrar_v2.ps1`. Seleccionar **PRUEBA DE ENCODER** en el menu
principal y entrar con X. El brazo queda detenido y no se ordena cierre de
garra. La banda se controla externamente. Este modo cuenta
cuentas del hardware sin convertir a mm ni estimar velocidad. No requiere
calibracion de camara, brazo o escala del encoder.

- **X (primera pulsacion):** inicia el conteo relativo desde cero y el cronometro.
  La OLED muestra pulsos y segundos transcurridos en vivo.
- **X (segunda pulsacion):** detiene la medicion y deja pulsos y segundos fijos
  en la OLED. En ese instante se agrega una fila al CSV.
- **Circulo:** pone a cero la medicion y la deja lista para comenzar de nuevo
  con X. No borra el contador fisico del encoder ni el CSV ya guardado.
- **Triangulo:** sale y cierra el CSV. Desconectar el mando tambien termina
  la prueba. Solo los ensayos terminados quedan en el CSV.

Para la prueba al 50%: pulsar X al inicio del tramo y X otra vez al final.
La OLED permite comprobar el intervalo medido; no hay un temporizador
automatico de 2 segundos. La sincronizacion incluye el tiempo de reaccion
de ambas pulsaciones.

El archivo es `registros_v2/encoder_AAAA-MM-DD_HH-mm-ss.csv`. Se abre al
entrar al modo o recibir su primer evento si PowerShell se inicio tarde.
Tiene las columnas `ensayo,contador_pulsos,tiempo_s`, una fila por cada
segunda pulsacion de X. El tiempo usa segundos con tres decimales. No incluye
velocidad, distancias, camara ni filas de diagnostico. Circulo prepara otro
tramo en el mismo CSV. El `.log` tecnico separado conserva las muestras en
vivo y otros eventos.

El contador muestra la magnitud del desplazamiento neto desde el primer X,
para que la lectura no salga negativa en el sentido habitual de la banda.
Mantener un solo sentido durante cada medicion: no es un acumulador de ida
y vuelta. Son cuentas de lectura X2, no los 1024 pulsos/vuelta nominales del
encoder: con la configuracion actual hay 2048 cuentas por vuelta.

## Modo de enseñanza ML

### Ajustes de camara y error de catch segun velocidad

En `PORTENTA/PORTENTA.ino`, `DESFASE_CAMARA_X_MM` y
`DESFASE_CAMARA_Y_MM` corrigen la deteccion en milimetros del sistema del brazo,
despues de aplicar los signos y el intercambio de ejes. Un valor positivo suma
posicion hacia +X/+Y. Se aplican en los tres modos automaticos y no modifican
la homografia. Para Y movil, positivo estima la pieza mas adelantada y dispara
antes. Mantenga la distancia fisica camara-catch separada de estos ajustes.

`CATCH_ADELANTO_EXTRA_MS` suma tiempo al adelanto del cierre en V2 y ML.
Por ejemplo, 100 ms extra adelantan 10 mm a 100 mm/s y 20 mm a 200 mm/s.
El calculo del disparo ahora incluye el descenso final de Z:
`umbralY = catchY - velocidad * (tiempoZFinal + tiempoCierre)`.
Los valores iniciales siguen siendo 450 ms mecanicos + 50 ms de transporte,
con adelanto extra en cero hasta medirlo. El filtro de velocidad responde ahora
con alpha 0.5 y la posicion usa el contador vivo. La orden I2C se envia antes
de imprimir los registros para que Serial no retrase la garra. La retirada
espera tambien el margen de transporte, no solo el recorrido del servo.

Inicie `registrar_v2.ps1` antes de entrar en `ENSENANZA ML`: abre
`ml_*.csv` al entrar y lo cierra al salir. Cada captura guarda
`ML_ANGLE_SUGGESTION`, los tramos `ML_ANGLE_ADJUST` si se giro manualmente,
`ML_CATCH_TRIGGER`, `ML_SAMPLE`, `ML_GRIP_APPLIED` y `ML_CLOSE`.
La terminal describe la sugerencia, los cambios de angulo y si X o el
encoder ordeno el catch. Los tiempos `ms` de ESP y Portenta son relojes
distintos; compare los eventos entre placas mediante `pc_utc` del CSV.

- `servo_rot_deg`: angulo ordenado de orientacion; no es una lectura fisica.
- `trigger`: X manual o ENCODER automatico.
- `catch_type`: MANUAL o AUTOMATICO, segun quien disparo el cierre. No confirma
  que la pieza haya sido agarrada. El registrador PowerShell muestra cada
  `ML_SAMPLE` como `CATCH MANUAL (X)` o `CATCH AUTOMATICO (ENCODER)`, incluso
  en vivo. El CSV de ML incluye `catch_type`, `trigger`, el instante de la
  orden en Portenta y el instante en que la ESP aplico el pulso.
- `suggested_rot_deg`: sugerencia inicial de 60 o 156 grados para cajas
  claramente anchas o altas de clase 0. `NA` significa que la regla de cinco
  ejemplos no tiene una sugerencia para esa pieza; el angulo previo se conserva.
- `angle_before_deg`, `angle_after_deg`, `adjust_start_ms`, `adjust_end_ms`:
  cada ajuste continuo con el joystick derecho, agrupado al soltarlo 120 ms.
  `angle_origin=MANUAL` distingue estos cambios de la sugerencia inicial.
- `delta_x`, `delta_y`: correccion del joystick respecto a la posicion inicial.
- `error_disparo_mm`: pieza estimada menos umbral automatico en el disparo.
- `error_disparo_ms`: la misma diferencia dividida por la velocidad; negativo
  significa que X se pulso antes de lo que habria disparado el encoder.
- `adelanto_extra_sugerido_ms`: ajuste total sugerido por esa pulsacion manual.
  Usarlo solo cuando el catch manual fue bueno; ENCODER produce nan porque no
  es una etiqueta humana. Promediar varias capturas exitosas a distintas velocidades.
- `cierre_y_predicha`: posicion prevista al completar el tiempo mecanico y transporte.
- `ML_CLOSE.error_y_estimado_mm`: posicion por encoder menos catchY al retirar Z.

Estos errores comparan el gesto humano con la prediccion; el sistema no tiene
un sensor que mida si la pieza quedo fisicamente centrada o agarrada. Por eso
se informa `error_fisico_medido=NO`. Si el error en mm crece aproximadamente
en proporcion a la velocidad, ajuste el tiempo. Si permanece parecido en mm,
revise el desfase Y. Si crece con la distancia recorrida, revise diametro,
relacion de transmision y deslizamiento de la rueda (49 mm, 1:1, X2).

`ENSENANZA ML` es un modo supervisado separado de Automático V2. Requiere la
calibración del brazo, de la cámara y del encoder. Puede abrirse con la banda
detenida, pero solo acepta una detección cuando la banda avanza en sentido
cámara→brazo y el encoder entrega velocidad y pulsos válidos. Su ciclo es:

1. La cámara confirma una pieza en movimiento y conserva su clase, coordenadas
   X/Y y el conteo de encoder asociado a la imagen.
2. El brazo alinea X con la detección, lleva Y a `Y=0` y baja Z en paralelo
   hasta la precaptura, actualmente 3000 pasos por encima del final inferior
   DIN04. La OLED muestra la distancia real `Z-DIN04` en pasos mientras Z se
   mueve. El encoder sigue estimando la pieza.
3. La camara aplica una sugerencia inicial de giro si la caja es claramente
   ancha o alta (solo clase 0). El operador puede corregir el giro con el eje
   horizontal del joystick derecho desde que se acepta la pieza, durante el
   traslado y el descenso. Con Z en precaptura tambien corrige X/Y con el joystick
   izquierdo. La pinza permanece abierta hasta el catch.
4. La primera condición entre el umbral automático del encoder y una pulsación
   de X inicia el descenso final. Al confirmar DIN04 se envía la orden de cierre;
   si DIN04 no aparece, el intento se cancela. El umbral automático adelanta
   tanto el descenso final como el tiempo de cierre. X conserva el disparo manual.
5. La línea `ML_SAMPLE` registra si el cierre
    fue disparado por `X` o por `ENCODER`, junto al angulo final y el error
    temporal respecto al umbral automatico. `ML_GRIP_APPLIED` confirma que
    la ESP ordeno el pulso; `ML_CLOSE` es el fin estimado de su recorrido,
    no una medida de agarre fisico.
6. Z sube al mismo tiempo que X/Y transportan la pieza a `X máximo - 10 mm`,
   junto al final derecho. Después vuelve a bajar, abre la garra, sube y queda
   listo para otra pieza.

Ejemplo de etiqueta supervisada:

```text
ML_SAMPLE|mode=ML|ms=...|session=...|n=1|seq=4|class=0|cam_x=...|cam_y=...|auto_x=...|auto_y=0.000|label_x=...|label_y=...|delta_x=...|delta_y=...|label_rot=...|piece_y=...|catch_y=...|close_threshold_y=...|belt_mm_s=...|encoder_ref=...|encoder=...|trigger=X|catch_type=MANUAL|catch_command_ms=...|detection_to_command_ms=...
```

Las columnas `cam_*`, `belt_mm_s`, `piece_y` y `encoder_ref` son entradas para
entrenar el modelo; `label_*` y `catch_y` describen la corrección hecha por el
operador. Si la pieza rebasa el catch, se detiene o pierde una lectura válida,
el intento se cancela, la garra se abre y Z se retira de forma segura. Triángulo
cancela el ciclo y vuelve al menú.

### Enseñanza ML V2

`ENSENANZA ML V2` es una opción nueva del menú. Conserva la detección estable,
la alineación X/Y, la bajada automática a precaptura, la orientación de la
garra y la entrega de `ENSENANZA ML`. En la fase 3 espera exclusivamente el
flanco de **X**: el encoder sigue estimando la posición y el umbral que habría
disparado el catch, pero no ordena el descenso final. Al pulsar X, registra
`ML_BUTTON_CATCH` con tiempo, conteo y posición estimada de la pieza, baja Z
hasta DIN04, ordena cerrar la pinza y hace la entrega completa. Tras subir Z,
se detiene en la fase 14 `ESPERANDO_CONFIRMACION`. Pulse **X** si realmente
agarró la pieza o **cuadrado** si no. Solo entonces se registra `ML_SAMPLE`
con `physical_result=EXITO/FALLO` y el modo se rearma para otra pieza.

El CSV `ml_v2_*.csv` contiene las columnas previas de enseñanza más
`catch_button_ms`, `catch_button_encoder`, `catch_button_piece_y_mm`,
`z_bottom_ms`, `catch_command_ms`, `button_to_grip_ms` y `physical_result`.
La ESP registra `ML_GRIP_APPLIED` con su propio reloj; `session_esp` y
`objective_seq` permiten relacionar los eventos de ambas placas.

Para verificar la velocidad, la ESP calcula en cada objetivo una estimación
independiente a partir del desplazamiento Y de la misma pieza entre consultas
de cámara y el tiempo transcurrido. El evento `CAMERA_SPEED` conserva esa
velocidad, el recorrido y periodo observados, la velocidad del encoder en la
misma ventana y su diferencia. `camera_speed_valid=0` indica que faltó tiempo,
recorrido o tres posiciones Y distintas para una medición útil; las lecturas
repetidas de cámara se conservan con `camera_unique_y`. Por ahora esta medición sirve para
comparar y depurar; el control de movimiento continúa usando el encoder hasta
validar la cámara con los CSV de pruebas.

### Prueba de seguimiento Y

`SEGUIMIENTO Y` usa la detección de cámara y el encoder de Enseñanza ML V2.
Después de preposicionar X y bajar Z a la precaptura, mueve Y hacia la
posición estimada de la pieza. La fase 15 de la OLED indica que está siguiendo.
El joystick derecho horizontal permite ajustar el ángulo de la garra. X inicia
el descenso final; Y continúa siguiendo durante ese descenso y el tiempo
estimado de cierre. Al finalizar se entrega la pieza y se confirma con X
(éxito) o cuadrado (fallo), como en Enseñanza ML V2.

La Portenta limita Y a 6 mm de cada extremo físico y desestima X si no queda
recorrido para completar el descenso y el cierre. Al llegar al límite cancela
el intento, detiene Y y retira Z. Triángulo cancela y sale del modo.
El registro `ml_track_*.csv` incluye `ML_TRACK` cada 200 ms con la posición
estimada de la pieza, la posición real del brazo y su diferencia. La muestra
final conserva el resultado físico y `track_error_button_mm`,
`track_error_bottom_mm` y `track_error_close_mm`. Estos errores dependen de
la posición inicial de cámara y de la escala del encoder; la confirmación
física del operador determina si hubo agarre.

En `MANUAL` se mueven X/Y con el joystick izquierdo y Z con el eje vertical del
derecho. El eje horizontal del joystick derecho mueve la rotación (GPIO25) por
ángulos de 0–180 grados. Cada pulsación de círculo alterna la garra (GPIO26)
entre abierta y cerrada usando los pulsos calibrados de 937 µs y 1816 µs.
Para medir la altura de precaptura, entre en `MANUAL`, deje la pinza abierta y
mueva Z con el eje vertical del joystick derecho hasta que la pieza pueda pasar
sin rozar. Al soltar el joystick, lea `Z-DIN04: Np` en la OLED: `0p` corresponde
al final inferior DIN04 y `N` es la separacion en pasos, no milimetros. Esta
lectura requiere que la calibracion Z haya terminado; de lo contrario la OLED
muestra `Z:CALIBRAR`. El valor medido puede usarse después para reemplazar el
margen actual de 3000 pasos compartido por Automatico V2 y Ensenanza ML.
En `PRUEBA SERVOS` se usa el mismo control: joystick derecho horizontal para
rotación y círculo para alternar la garra.
El OLED muestra ambos ángulos y el pulso de la garra. Fuera de Manual y Prueba
de servos, los mandos no cambian sus ángulos.

## Registro sincronizado en Windows

Cierre los monitores seriales del Arduino IDE y ejecute el registrador antes
de iniciar la calibración:

```powershell
powershell -ExecutionPolicy Bypass -File ".\pruebas de automatico v2\registrar_v2.ps1"
```

El programa muestra los puertos COM disponibles y solicita primero la ESP32
y después la Portenta. La ESP32 usa 460800 baudios y la Portenta 115200. Puede
evitar las preguntas indicando ambos puertos:

```powershell
powershell -ExecutionPolicy Bypass -File ".\pruebas de automatico v2\registrar_v2.ps1" `
  -PuertoESP COM8 -PuertoPortenta COM6
```

El registrador permanece silencioso durante calibración y conserva dos
segundos de prebúfer. Cuando la Portenta entra en V2 crea
`registros_v2/v2_YYYY-MM-DD_HH-mm-ss.csv`; en Registrar Angulo usa el prefijo
`angulo_`. Al salir, cancelar o perder un
puerto vacía y cierra el archivo. Cada fila se escribe inmediatamente. Las
líneas antiguas que no usan `V2LOG` quedan como eventos `RAW`, por lo que no se
pierde el contexto de diagnóstico. El separador del CSV es coma, compatible
con la configuración de Excel usada en la computadora de pruebas. La columna
`description_es` resume cada fila en lenguaje legible.

Mientras la sesión está activa, la terminal mezcla cronológicamente ESP32 y
Portenta con etiquetas de color. Las consultas `SIN_PIEZA` se resumen una vez
por segundo y la telemetría repetida de la Portenta cuatro veces por segundo;
publicaciones, candidatos, cambios de fase, ACK, captura, resultado, errores y
cancelaciones aparecen inmediatamente. El CSV conserva todas las muestras sin
ese filtrado visual.

La unión principal entre placas se hace con `encoder_seq` + `encoder_count`.
`objective_seq` enlaza publicación, aceptación, captura, ACK y resultado. La
hora UTC del PC ordena ambas corrientes sin modificar los paquetes I²C de 32
bytes. Los archivos de `registros_v2` están excluidos de Git.

## Conexión del encoder

El OMRON E6B2-CWZ6C se conecta al encoder 0 de la Portenta Machine Control:

| Encoder | Machine Control |
|---|---|
| OUTA | A0 |
| OUTB | B0 |
| OUTC (índice) | Z0/C0 |
| VCC | alimentación del conector de encoder |
| 0 V | GND del conector de encoder |

Las entradas ABZ de la Machine Control están preparadas para el encoder NPN
open-collector. No conectar las salidas de 24 V del encoder a GPIO de la ESP32.
La librería `Arduino_MachineControl` usa decodificación X2, por lo que un
encoder de 1024 P/R debe entregar aproximadamente 2048 cuentas por vuelta.
OUTC solo se usa como índice de diagnóstico: no pone la distancia en cero.

## Tercera calibración: encoder y banda

No se introduce una distancia por terminal. La rueda configurada en el código
mide 49 mm de diámetro,
gira 1:1 con el encoder y la lectura es X2 (2048 cuentas/vuelta), por lo que la
escala se obtiene directamente de la geometría:

```text
mmPorCuenta = pi * 49 / 2048 = 0.0751651 mm/cuenta
```

Al solicitar Automático V2 sin encoder calibrado, el OLED muestra la
calibración del encoder:

1. El técnico pone manualmente la banda al 50 % y en sentido cámara→brazo.
2. Cuando ya esté avanzando, presiona X.
3. El sistema descarta 2 s de estabilización.
4. Mide durante 5 s y obtiene automáticamente sentido y velocidad al 50 %.
5. Rechaza la medición si hay paro, inversión, pocos pulsos o variación mayor
   al 10 % entre ventanas de 200 ms. X permite repetirla.
6. El OLED pide detener la banda. Cuando el encoder confirma el paro, entra
   automáticamente en V2 si las demás calibraciones ya están listas.

La velocidad máxima matemática se estima como `2 * velocidadAl50`. Esta
referencia se recalibra en cada encendido porque el técnico ajusta físicamente
el variador. Automático V2 sigue usando la velocidad instantánea real del
encoder y rechaza una lectura superior en más de 10 % a la máxima estimada.

El modo V2 puede abrirse con la banda detenida. Una vez dentro permanece en
`V2_ESPERANDO_PIEZA`; la ESP no publica una pieza hasta que haya observado
pulsos válidos. La banda debe avanzar en sentido cámara→brazo antes de aceptar
el objetivo.

Comandos disponibles:

- `ENC ESTADO`: A/B, índice, conteo, distancia, velocidad, escala y signo.
- `ENC VUELTA`: compara únicamente intervalos completos entre índices con las
  2048 cuentas/vuelta esperadas. La fracción anterior al primer índice y la
  posterior al último ya no sesgan el resultado.
- `ENC CERO`: único comando que reinicia manualmente conteo e índice.
- `STOP`: detiene inmediatamente el movimiento.

Estos comandos son solo de diagnóstico; la calibración no depende de la
terminal.

## Diagnóstico de detección V2

Mientras Automático V2 permanece en fase 0, la ESP muestra en el OLED la
causa de bloqueo, el número de detecciones consecutivas (`N`), el recorrido
medido (`D`), la antigüedad del encoder (`AGE`), resultados HUSKYLENS (`R`) y
candidatos válidos (`V`). La lógica de detección no se relaja: siguen vigentes
50 ms de antigüedad máxima, tres detecciones, 6 mm de tolerancia y 2 mm de
desplazamiento mínimo.

En V2 la cámara se consulta cada 50 ms; Automático V1 conserva sus 200 ms. La
ESP toma una muestra de encoder antes y otra después de `getResult()` y asocia
la imagen al conteo medio entre ambas. Esto reduce el error producido por el
tiempo de transferencia UART sin inventar una marca de tiempo que HUSKYLENS
no proporciona.

Cuando HUSKYLENS devuelve varias cajas para una sola pieza, se agrupan si
tienen la misma clase y se solapan al menos 50 % respecto de la caja menor, o
si sus centros son prácticamente coincidentes. En cada grupo se conserva la
caja de mayor confianza y, en empate, la de mayor área. Entre grupos distintos
se priorizan continuidad X/Y, tamaño y clase; nunca se cuentan dos cajas de la
misma consulta como dos detecciones consecutivas.

La relación de sentido entre Y cruda de cámara y el avance del encoder se
elige al obtener la segunda detección coherente (`relY=1` o `relY=-1`) y se
mantiene durante esa trayectoria. Una imagen con coordenadas repetidas aunque
el encoder ya avanzó se marca `CAMARA_REPETIDA`: no incrementa `N`, pero
tampoco borra inmediatamente la trayectoria.

Serial informa inmediatamente cuando cambia la causa y la repite una vez por
segundo mientras no cambie:

```text
[V2][DIAG] bloqueo=ENCODER_ANTIGUO age=... flags=... n=... d=...
[V2][DIAG] bloqueo=TRAYECTORIA x=... y=... yc=... dx=... dy=...
[V2][DIAG] bloqueo=CAMARA_REPETIDA n=... d=... query=... dq=...
[V2][DIAG] bloqueo=DESPLAZAMIENTO n=3 d=...
[V2][DIAG] PUBLICADO ... seq=...
[V2][CAND] total=2 valid=2 unicos=1 duplicados=1 consulta=... enc=.../.../...
[V2][CAND] i=0 grupo=0 id=... conf=... px=... caja=... mm=...
```

Códigos principales:

- `SIN_PIEZA`: HUSKYLENS respondió correctamente y no encontró piezas.
- `HUSKY_ERROR` o `SIN_RESULTADOS`: fallo de consulta o respuesta incoherente.
- `HOMOGRAFIA` y `FUERA_BANDA`: la detección no produjo una coordenada física
  utilizable o quedó fuera del área blanca.
- `ENCODER_*`: muestra ausente, mayor de 50 ms, sin escala/pulsos, saturada o
  con signo inválido.
- `DETECCIONES`: la trayectoria es válida pero todavía faltan muestras.
- `CAMARA_REPETIDA`: la cámara repitió una posición mientras la banda sí
  avanzó; la muestra se ignora y se espera una imagen nueva.
- `TRAYECTORIA`: clase o dispersión X/Y compensada reinició el filtro; `dx` y
  `dy` muestran la dispersión que provocó el rechazo.
- `DESPLAZAMIENTO`: ya existen tres detecciones pero aún no se recorrieron
  2 mm desde la primera.
- `PUBLICADO`: la ESP envió el objetivo y la Portenta debe abandonar fase 0.

Las líneas también incluyen flags, conteo y velocidad del encoder, cantidad
de resultados (`husky`), candidatos originales (`valid`), grupos únicos
(`unique`), duplicados (`dup`), rechazos, clase, X/Y crudas, Y compensada,
dispersión, relación aprendida (`relY`), duración UART (`query`), recorrido
durante la consulta (`dq`) y conteos antes/asociado/después (`encQ`). Esta
instrumentación es local a la ESP: no cambia los paquetes ni la versión del
protocolo I²C.

## Lógica de interceptación

La ESP descarta muestras de encoder recibidas hace más de 50 ms. Con tres
detecciones coherentes publica X/Y y el conteo exacto de referencia. La
Portenta calcula continuamente:

```text
piezaY = -(510 mm + ajusteCatch) + YlocalCamara
         + signo * (conteoActual - conteoReferencia) * mmPorCuenta
```

La calibración manual final produjo tres marcas coherentes en `Y=116.7, 131.9 y
133.2 mm` y una marca aislada en `-98.2 mm`. La mediana robusta indicó que
faltaban aproximadamente `125 mm` de recorrido. Por eso `ajusteCatch` pasa de
`210 a 335 mm` y la distancia efectiva cámara-catch queda en `845 mm`, sin
cambiar la escala del encoder.

Después preposiciona X con la coordenada detectada, lleva Y hacia atrás hasta
la estación de catch (`Y=0 mm`) y baja Z al mismo tiempo hasta la precaptura. El
encoder actualiza la posición estimada de la pieza, pero ya no gobierna la
velocidad del eje Y. La Portenta calcula el tiempo mecánico de descenso Z y lo
convierte en distancia usando la velocidad actual de la banda. Las pruebas del
27 de agosto midieron un descenso muy estable de `1.686 a 1.695 s`; el problema
no era una variación de Z, sino que llegaba abajo prácticamente al mismo tiempo
que la pieza (`-2 a +8 mm` alrededor de `Y=0`) y no dejaba tiempo para pulsar X.

El umbral inicial conserva una reserva de `0.50 s` para completar la
preposicion. Z espera `Z_MARGEN_PRECAPTURA_PASOS = 3000` pasos sobre DIN04;
este valor compartido por Portenta y ESP debe ajustarse con la lectura real
de la OLED en `MANUAL`. A 5000 pasos/s, el tramo final nominal dura
unos 600 ms. La camara permanece bloqueada y el encoder actualiza la posicion
estimada mientras Z espera elevado.

Cuando la pieza se aproxima a `Y=0`, la Portenta adelanta el descenso final por
el tiempo nominal de Z mas 450 ms de garra y 50 ms de margen I²C. Solo al
confirmar DIN04 envia `ACK_OBJ_CERRAR_PINZA`. Si el conteo termina antes del
sensor, hace una busqueda lenta adicional; si DIN04 no aparece, cancela sin
ordenar el cierre. La ESP32 cierra GPIO26 con el pulso calibrado de 1816 us.
Tras completar el tiempo de cierre, Z se retira verticalmente
hasta HOME. No se requiere X. Si se pulsa, queda un evento diagnóstico
`BUTTON_X`, pero se ignora y no modifica la trayectoria.

El intento se cancela si la banda se detiene o invierte antes del disparo, la
pieza rebasa la estación antes de que el brazo esté listo, falta espacio para
bajar Z, se pierde I²C, encoder o control, aparece un final inesperado o vence
un timeout. La cámara solo condiciona la publicación inicial.

Una cancelación propia del intento (llegada, trayectoria o encoder)
retira Z de forma segura y mantiene Automático V2 activo para rearmarse; no
regresa al menú principal. Solamente triángulo, `STOP` o la pérdida del control
salen del modo.

Desde que se publica un objetivo, la ESP bloquea su secuencia, clase,
coordenadas de cámara y conteo de referencia. No consulta HUSKYLENS durante el
movimiento; la Portenta actualiza únicamente Y con el delta del encoder. Las
fases normales son `0 -> 1 -> 2 -> 3 -> 4 -> 6 -> 10 -> 5 -> 7 -> 0`.
La fase `9` aparece únicamente al entrar si Z debe volver primero a la altura
segura de espera.

## Catch físico automático por encoder

En la fase 6 los tres ejes permanecen detenidos mientras el encoder aproxima la
pieza. Al alcanzar el umbral anticipado se pasa a `CERRANDO_PINZA`; X/Y/Z siguen
inmóviles y la garra completa su cierre alrededor de la estación `Y=0`. Luego Z
sube. Cuando vuelve a HOME, la Portenta registra `CATCH_AUTOMATICO`, envía
`ACK_OBJ_COMPLETADO`, la ESP32 vuelve a abrir la garra y se pasa a `COMPLETADO`.
El OLED muestra las fases de cierre y resultado. Triángulo conserva la
cancelación, abre la garra y hace la retirada segura de Z.

La ESP libera el objetivo terminal, mantiene HUSKYLENS bloqueada durante 500 ms
y después permite una publicación nueva.
La comparación de secuencia solo impide repetir el mismo mensaje I²C: la misma
pieza o clase puede volver a procesarse cuando una detección posterior recibe
otra secuencia.

## Orden de puesta en marcha

1. Trabaje primero con los motores sin herramienta y a velocidad baja.
2. Verifique A/B, OUTC y unas 2048 cuentas por vuelta.
3. Seleccione Automático V2 y complete la medición del encoder al 50 % cuando
   el menú la solicite.
4. Valide cámara+encoder observando que la pieza estimada avance desde la
   distancia efectiva configurada hasta la estación `Y=0 mm`.
5. Pruebe la preposición fija X/Y y confirme que Y no se mueve durante la
   espera ni durante el descenso Z.
6. Pruebe Z con la banda detenida y confirme la nueva altura física.
7. Habilite el ciclo completo sin pulsar X. Confirme en el registro la cadena
   `READY_CATCH -> GRIP_COMMAND -> CAPTURE (PINZA CERRADA) -> RESULT
   (CATCH_AUTOMATICO)`. Triángulo debe seguir cancelando y no debe producir un
   resultado positivo.
8. Inyecte individualmente pérdida de I²C, cámara, encoder, banda, control y
   finales de carrera.

## Compilación verificada

FQBN usados:

```text
ESP:      esp32-bluepad32:esp32:esp32
Portenta: arduino:mbed_portenta:envie_m7
```

Los `static_assert` de ambos headers verifican en cada compilación el tamaño
de 32 bytes y que el CRC esté en el byte 31.

### Protección del USB durante el arranque

La Portenta espera como máximo 1.5 s para que Windows enumere el USB CDC antes
de inicializar I²C. Luego comprueba físicamente SDA y SCL. Si cualquiera ya
está en bajo, no entra en una transacción bloqueante: mantiene los motores
detenidos, conserva el puerto USB y muestra `Bus ocupado antes de iniciar`.
Reintenta la inicialización cada segundo y solo habilita la comunicación y el
movimiento después de verificar el expansor de entradas de Machine Control.

Los marcadores `[BOOT][1]` a `[BOOT][5]` identifican la última etapa de
arranque completada. El enlace volvió a la frecuencia estándar de 100 kHz;
los periodos 23/47 ms, CRC y reintentos siguen reduciendo la sensibilidad al
ruido sin depender de una frecuencia I²C no validada.

## Diagnóstico de comunicación con el encoder girando

El enlace funciona a 100 kHz. La Portenta solicita el control cada 23 ms y
publica su estado cada 47 ms; la ESP renueva su instantánea cada 20 ms. Los
periodos 23/47 ms evitan iniciar siempre las transferencias en la misma fase
del ticker STEP de 100 us. Al fallar una operación, la Portenta realiza hasta
dos reintentos no bloqueantes, separados primero 3 ms y después 9 ms. Durante
esa recuperación no intercala otra operación normal del bus.

El timeout de seguridad permanece en 150 ms. Un reintento válido conserva el
enlace, pero ningún paquete corto, con cabecera incorrecta, CRC inválido o
semántica inválida modifica el estado de control. El eje Y arranca con divisor
8 y baja hacia el divisor solicitado cada 75 ms para reducir el transitorio de
conmutación del driver.

Una vez por segundo la Portenta imprime:

```text
[I2C] rxOK=... rxError=...(len=... hdr=... crc=... sem=...) txOK=...
      txError=... ultimoTx=... rxErrPct=... txErrPct=... retry=...
      recRx=... recTx=... burstFail=... pausaLoopMax=...ms estado=...
```

- `len`: lectura incompleta o sin respuesta de la ESP.
- `hdr`: `magic`, versión o longitud declarada no coincide con el protocolo.
- `crc`: paquete recibido con bytes alterados.
- `sem`: paquete íntegro pero con valores fuera del protocolo.
- `ultimoTx`: código devuelto por `Wire.endTransmission()`; cero es correcto.
- `rxErrPct` y `txErrPct`: porcentaje de error de la última ventana de un
  segundo, no el acumulado desde el arranque.
- `retry`: cantidad acumulada de reintentos ejecutados.
- `recRx` y `recTx`: ráfagas recuperadas antes de vencer los 150 ms.
- `burstFail`: operaciones que continuaron fallando después de ambos
  reintentos.
- `pausaLoopMax`: mayor tiempo observado sin ejecutar el `loop()`.
- `encCps`: cuentas X2 por segundo; permite relacionar el fallo con la carga
  real de interrupciones incluso antes de calibrar los mm por cuenta.

Si todavía se pierde el enlace, haga dos pruebas separadas: girar el encoder a
mano con el motor de la banda apagado, y luego hacerlo con el variador/motor
encendido. Si solo falla con el motor, la causa es eléctrica (EMI, masas,
blindaje o tendido de cables), no la frecuencia de paquetes.

Tanto el maestro como el esclavo son recuperables. La Portenta reinicia su
periférico I²C una vez por segundo mientras está detenida y esperando enlace;
la ESP vuelve a iniciar su esclavo si `Wire.begin()` falla o si pasan cinco
segundos sin ninguna solicitud ni escritura. Reiniciar el periférico no cambia
la sesión de arranque de la ESP y nunca permite reanudar motores por sí solo.

## Diagnóstico de reinicios y pérdidas I²C

El protocolo versión 12 conserva ambos paquetes en 32 bytes. La ESP publica en
cada paquete la causa de su último reset y la Portenta distingue un reinicio
real de una recuperación del periférico I²C.

Cuando la Portenta pierde el enlace, detiene los motores y mantiene durante dos
segundos el estado `DIAG I2C PORTENTA`. La espera usa `millis()`; no bloquea el
programa ni detiene los intentos de comunicación. Después exige tres desafíos
de ida y vuelta válidos. En el arranque inicial pasa automáticamente a los cinco
segundos de estabilización; después de una pérdida posterior muestra
`PULSE X PARA ESTABILIZAR 5 S` y permanece detenido hasta recibir ese flanco del
control. Las causas mostradas son:

- `REINICIO ESP`: cambió la sesión de arranque; el detalle indica encendido,
  pin externo, software, watchdog, brownout, panic u otra causa;
- `PAQUETE CORTO`: faltaron bytes o no respondió el esclavo;
- `VERSION/HEADER`: firmware incompatible o cabecera dañada;
- `ERROR CRC`: se alteraron bytes durante la transferencia;
- `DATO INVALIDO`: paquete íntegro con contenido fuera del contrato;
- `SEQ CONGELADA`: siguen llegando bytes, pero la instantánea de la ESP no
  avanza;
- `TIMEOUT RX`: no llegó ningún paquete válido durante 150 ms;
- `NACK DIRECCION`, `NACK DATOS` o `ERROR TX`: fallo persistente al escribir
  desde la Portenta.

La ESP muestra además `PORTENTA AUSENTE`, `ESCLAVO REINIC.` o
`WIRE.BEGIN FALLO`. Esto permite separar un reinicio completo —repite
`[BOOT]`, cambia la sesión y genera `[RESET]`— de un reinicio exclusivo del
periférico I²C, que conserva la misma sesión.

Cada evento produce una línea estructurada `[I2CDBG]`; cada arranque produce
`[RESET]`. `registrar_v2.ps1` guarda desde que abre los puertos un archivo
`registros_v2/i2c_YYYY-MM-DD_HH-mm-ss.log`, aunque nunca se entre a Automático
V2. El CSV de movimiento conserva su funcionamiento anterior.

Para registrar ambas placas:

```powershell
powershell -ExecutionPolicy Bypass -File ".\pruebas de automatico v2\registrar_v2.ps1" `
  -PuertoESP COM8 -PuertoPortenta COM6
```

La ESP usa 460800 baudios y la Portenta 115200. Las dos deben cargarse juntas
con protocolo 9; mezclar versiones debe producir `VERSION/HEADER`.

### Revisión física recomendada

1. Con el sistema apagado, comprobar continuidad de SDA, SCL y GND extremo a
   extremo, además de ausencia de cortos entre esas redes y 3.3 V.
2. Inspeccionar con lupa soldaduras frías, grietas, puentes, vías, resistencias
   pull-up, terminales y los conectores de la Portenta/Machine Control.
3. Con alimentación, verificar que SDA y SCL suban a 3.3 V, nunca a 5 V.
4. Repetir primero sin motores ni servos y reconectar cargas una por una. Si la
   falla aparece al conmutar potencia, revisar EMI, caída de tensión y masas.
5. Con osciloscopio o analizador lógico, buscar una línea retenida en LOW,
   flancos lentos, picos o pausas superiores a 150 ms.

# Pruebas de Automático V2

Esta carpeta es una maqueta independiente. Los sketches finales de `ESP/` y
`PORTENTA/` no forman parte de estas pruebas ni deben sobrescribirse al cargar
las placas.

La escala cartesiana usada después de recorrer los finales es actualmente
X=446 mm y Y=336 mm.

## Contenido

- `ESP/ESP.ino`: HUSKYLENS, Bluepad32, OLED e I²C esclavo `0x40`.
- `PORTENTA/PORTENTA.ino`: calibración X/Y/Z, encoder ABZ, motores y máquina
  de estados de interceptación.
- Los dos archivos `ProtocoloI2C.h` son copias idénticas del protocolo de
  prueba versión 5. Ambos paquetes miden exactamente 32 bytes y terminan con
  CRC-8/ATM.
- `registrar_v2.ps1`: abre simultáneamente los USB de ambas placas y crea un
  CSV sincronizado por cada entrada a Automático V2.

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
`registros_v2/v2_YYYY-MM-DD_HH-mm-ss.csv`; al salir, cancelar o perder un
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

No se introduce una distancia por terminal. La rueda mide 65 mm de diámetro,
gira 1:1 con el encoder y la lectura es X2 (2048 cuentas/vuelta), por lo que la
escala se obtiene directamente de la geometría:

```text
mmPorCuenta = pi * 65 / 2048 = 0.0997088 mm/cuenta
```

El arranque tiene tres etapas obligatorias: cámara, brazo X/Y/Z y encoder. Al
terminar HOME y conectar el control, el OLED muestra `CAL ENCODER 3/3`:

1. El técnico pone manualmente la banda al 50 % y en sentido cámara→brazo.
2. Cuando ya esté avanzando, presiona X.
3. El sistema descarta 2 s de estabilización.
4. Mide durante 5 s y obtiene automáticamente sentido y velocidad al 50 %.
5. Rechaza la medición si hay paro, inversión, pocos pulsos o variación mayor
   al 10 % entre ventanas de 200 ms. X permite repetirla.
6. El OLED pide detener la banda. Cuando el encoder confirma el paro, se
   habilitan el checklist y el menú.

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

La prueba con `ajusteCatch=0` quedó adelantada y la prueba con `420 mm` quedó
atrasada. La siguiente iteración usa el punto medio, `210 mm`, por lo que la
distancia efectiva cámara-catch es `720 mm`. El ajuste se afina por aproximación
sucesiva sin cambiar la escala del encoder.

Después preposiciona X y deja Y inmóvil en la estación de catch (`Y=0 mm`). El
encoder actualiza la posición estimada de la pieza, pero ya no gobierna la
velocidad del eje Y. La Portenta calcula el tiempo mecánico de descenso Z y lo
convierte en distancia usando la velocidad actual de la banda. Cuando la pieza
entra en ese umbral anticipado, dispara Z para que llegue al final inferior al
mismo tiempo que la pieza alcanza `Y=0`. Entonces exige que el final físico
`DIN04` confirme `Z abajo`. Si el conteo termina antes del sensor, realiza una
búsqueda lenta adicional de hasta 600 pasos; solo después cancela. Al confirmar
DIN04 imprime `CAPTURA_VIRTUAL`, congela Y y regresa Z verticalmente a HOME.

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
fases normales son `0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 7 -> 0`; el valor 6 queda
reservado para interpretar registros anteriores.

## Finalización automática

Cuando Z vuelve a HOME, Y se detiene y la Portenta registra
`CATCH_AUTOMATICO`, envía `ACK_OBJ_COMPLETADO` y pasa directamente a
`COMPLETADO`. No se requiere X, círculo ni timeout de confirmación. El OLED
muestra `CATCH AUTOMATICO` durante la fase completada.

La ESP libera el objetivo terminal, mantiene HUSKYLENS bloqueada durante
500 ms y después permite una publicación nueva. La comparación de secuencia
solo impide repetir el mismo mensaje I²C: la misma pieza o clase puede volver a
procesarse cuando una detección posterior recibe otra secuencia.

## Orden de puesta en marcha

1. Trabaje primero con los motores sin herramienta y a velocidad baja.
2. Verifique A/B, OUTC y unas 2048 cuentas por vuelta.
3. Complete en el arranque la medición automática con la banda al 50 %.
4. Valide cámara+encoder observando que la pieza estimada avance desde la
   distancia efectiva configurada hasta la estación `Y=0 mm`.
5. Pruebe la preposición fija X/Y y confirme que Y no se mueve durante la
   espera ni durante el descenso Z.
6. Pruebe Z con la banda detenida y confirme la nueva altura física.
7. Habilite el ciclo completo y confirme el resultado `CATCH_AUTOMATICO` sin
   pulsar X ni círculo; triángulo debe seguir cancelando antes de completarse.
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

## Diagnóstico de comunicación con el encoder girando

El enlace permanece a 100 kHz, pero la Portenta solicita el control cada 10 ms
y publica la telemetría del encoder cada 20 ms. Dos paquetes completos de 32
bytes ocupaban casi el 90 % del bus con los periodos anteriores de 5/10 ms;
los nuevos periodos reducen la ocupación teórica aproximadamente al 45 % y
dejan margen para las interrupciones X2 del encoder.

Una vez por segundo la Portenta imprime:

```text
[I2C] rxOK=... rxError=...(len=... crc=... sem=...) txOK=...
      txError=... ultimoTx=... pausaLoopMax=...ms encVel=...
      encCps=... estado=...
```

- `len`: lectura incompleta o sin respuesta de la ESP.
- `crc`: paquete recibido con bytes alterados.
- `sem`: paquete íntegro pero con valores fuera del protocolo.
- `ultimoTx`: código devuelto por `Wire.endTransmission()`; cero es correcto.
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

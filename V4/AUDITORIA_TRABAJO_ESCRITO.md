# Auditoría del trabajo escrito frente al trabajo realizado

Documento auditado: `Trabajo_de_Graduacion-7.pdf` (78 páginas) y sus fuentes LaTeX
(`Trabajo_de_Graduacion(4).zip`; el PDF coincide con el ZIP). Referencia de formato:
*Guía para la redacción de tesis y trabajos de graduación UVG 2019*.

Evidencia contrastada:

| Fuente | Qué aporta |
|---|---|
| `V4/automatico v2 rs485/` (ESP.ino, PORTENTA.ino y cabeceras) | Firmware vigente; **manda sobre cualquier documento** |
| `V4/DOCUMENTACION_RS485_AUTOMATICO_V2.md`, `CAPTURA_FIJA_V2.md`, `DIAGNOSTICO_RS485_2026-10-08.md` | Documentación técnica de V4 |
| `V3/` (respaldo, pruebas, `registros_v2/*.log`, diagnósticos) | Historia I2C, registros reales de calibración y de agarres manuales |
| `C:\Users\samue\OneDrive\Documents\Universidad\CodexModelos\ENTRENAMIENTOS\` | Entrenamiento y métricas del modelo de visión |

Prioridades: **[C] crítica** (el texto contradice el trabajo o afirma algo no demostrado),
**[I] importante** (omisión relevante o dato que sí existe y no se usa), **[F] formato/guía UVG**.
Las secciones se citan con la numeración del PDF (p. ej. 7.5.3) y el archivo `.tex`.

Nada se modificó en la tesis. Este documento solo propone correcciones.

---

## 1. Resumen ejecutivo

1. **[C] Los parámetros de visión y del modo automático descritos son de una versión anterior
   del firmware.** La tesis dice 382 mm entre filas de AprilTags, coordenadas (±176, ±191),
   `X_b = −X_c + 15`, `Y_b = −Y_c`, 845 mm cámara–captura y `CUSTOM_MODEL_INDEX = 0`. El
   firmware V4 usa 255 mm (±127,5), `X_b = −X_c − 5`, signo Y **+1**, 551 mm y el índice 1
   (algoritmo 129). El desfase de +15 mm no aparece en **ninguna** versión del código.
2. **[C] El modelo de visión descrito no es el instalado.** La tesis describe 800 imágenes
   (2 clases, Mind+) y declara que no hay arquitectura, particiones ni métricas. El modelo 129
   que usa el firmware es un **YOLO11n de segmentación** (5 clases, 2350 imágenes
   1879/471, 100 épocas), con precisión, exhaustividad, mAP y matriz de confusión guardadas.
3. **[C] Resumen, Abstract, Introducción y Alcance afirman resultados que la propia tesis
   niega**: que se evaluaron precisión, repetibilidad y tiempo de respuesta, que se compararon
   posiciones alcanzadas con objetivo, que se reconoce "figura y color" y que se clasifica.
4. **[C] La Metodología sigue describiendo I2C como enlace vigente** (7.4.4, 7.4.5, 7.4.7) y
   la **Fase 6 de Metodología está vacía**, aunque Resultados y Discusión de la Fase 6 existen.
5. **[C] Faltan partes que la guía UVG exige**: capítulo de **Conclusiones** y **Lista de
   cuadros y figuras**. Tampoco hay Recomendaciones (opcional, pero muy justificado aquí).
6. **[I] Omisiones de trabajo real que explican los resultados**: orientación automática de
   la garra estimada por la caja (y su ambigüedad en diagonales), la secuencia completa del
   Automático V2 (estación fija Y = 0, predicción con encoder, confirmación DIN04, **entrega y
   apertura de la pinza**) y la corrección del modelo de posición (error de −69 ± 103 mm a
   −1 ± 24 mm).
7. **[I] Hay datos cuantitativos en el repositorio que la tesis da por inexistentes**:
   repetibilidad del recorrido medido en la referenciación, tasas de error I2C frente a RS485,
   velocidades de banda en mm/s, métricas del modelo y factores pasos/mm.
8. **[F] Citas y bibliografía incoherentes**: en el texto se usa (Autor, año), pero la
   bibliografía sale numerada ([1]…[41]) y `\nocite{*}` incluye obras que nunca se citan.
   Los captions dicen "Tabla 8.x" mientras el texto dice "Cuadro 8.x".

---

## 2. Hallazgos críticos

### C1. Geometría cámara–brazo y parámetros del automático desactualizados

| Dónde | Dice la tesis | Firmware V4 (evidencia) |
|---|---|---|
| 7.5.3 y 8.5.3 (`metodologia.tex` ≈l.262–271; `Resultados.tex` Cuadro 8.8) | Separación longitudinal de 382 mm; tags en (±176, ±191) mm | `TAG_ROWS_DISTANCE_MM = 255.0` (ESP.ino:1200) → tags en (±176, **±127,5**) mm |
| 7.5.5 y 8.5.5 (`metodologia.tex` ≈l.347–356; `Resultados.tex` ≈l.720–726) | `X_b = −X_c + 15`, `Y_b = −Y_c` | `CAMERA_SIGN_X = −1`, `DESFASE_CAMARA_X_MM = −5`, `CAMERA_SIGN_Y = +1` (PORTENTA.ino:63–68) → `X_b = −X_c − 5` |
| 7.5.5 y 8.5.5 | Distancia cámara–captura 845 mm | `CAMARA_A_HOME_Y_MM = 510 + 41 = 551` mm (PORTENTA.ino:72–85) |
| 8.5.2 (`Resultados.tex` ≈l.589–593) | `CUSTOM_MODEL_INDEX = 0` → `ALGORITHM_CUSTOM_BEGIN + 0` | `INDICE_MODELO = 1` → algoritmo **129** (VisionModelo129.h:9) |

Contexto que la tesis debe incorporar (está en `CAPTURA_FIJA_V2.md` y en los comentarios
del código): el valor 845 mm (510 + 335) y el signo −1 compensaban dos errores entre sí y
solo funcionaban para piezas vistas cerca de Y = −170 mm. Con los registros de 142
trayectorias se encontró que la Y de la homografía crece con el avance de la banda (signo
+1) y que la homografía asignaba 0,69 mm/px en X y 0,99 mm/px en Y para la misma pieza
(escala Y inflada ≈1,5×). Por eso se adoptó 255 mm como distancia **efectiva** entre filas
de tags (382/1,5) y se reajustó la distancia cámara–captura con 22 agarres manuales
(564 ± 24 mm) y ensayos en la maqueta (565 − 14 = 551 mm).

**Sugerencia de corrección**

- Reemplazar las coordenadas del Cuadro 8.8 por Y = ±127,5 mm y explicar que 255 mm es una
  distancia efectiva ajustada con el encoder, no una medición con cinta. Indicar la
  distancia física real entre centros de tags si se mide (el código lo deja pendiente).
- Sustituir las ecuaciones de 7.5.5 y 8.5.5 por el modelo implementado
  (`piezaBrazoDesdeCamara()`, PORTENTA.ino:1278):

  ```latex
  X_b = -X_c - 5\ \text{mm}, \qquad
  Y_p(t) = -551 + Y_c + s\,k\,\bigl(N(t) - N_{\mathrm{ref}}\bigr) - 40\,\lvert\cos\theta\rvert\ \text{mm},
  ```

  con $s = \pm 1$ (sentido medido en la calibración de la banda), $k = 0{,}07517$ mm/cuenta,
  $N_{\mathrm{ref}}$ la cuenta del encoder sellada en la imagen y $\theta$ el giro aplicado a
  la garra; $Y_p = 0$ es la estación de captura.
- Cambiar `CUSTOM_MODEL_INDEX = 0` por 1 (algoritmo 129).
- Eliminar los comentarios `% CONFIRMAR: offset físico X = 15 mm` y `procedimiento de
  medición de los 845 mm`: ambos valores ya no aplican.
- En Discusión 9.5.3, agregar el hallazgo de la escala Y inflada como evidencia concreta de
  que "la existencia de una solución numérica no demostró la exactitud de la transformación"
  (el texto ya lo anticipa; ahora hay un caso real que lo confirma).

### C2. El modelo de visión descrito no corresponde al instalado

| Dice la tesis (7.5.1, 8.5.1, 9.5.1) | Evidencia |
|---|---|
| 800 imágenes, 400 por pieza 6 y 7, anotadas con cajas en formato YOLO en Mind+ | `RESUMEN_MODELOS.md` y `REPORTE_VALIDACION.md`: dataset de **2350 imágenes** (1879 entrenamiento / 471 validación), **5 clases** (pieza2, 5, 6, 7, 8), etiquetas de polígono (segmentación); pieza6 = 645 imágenes, pieza7 = 423 |
| Arquitectura, particiones y parámetros "pendientes de documentación" | YOLO11n-seg (`yolo11n-seg.pt`), entrada 320 × 320, 100 épocas (mejor 97), batch 9, AdamW, lr0 = 0,001111, semilla 42, GPU GTX 1650; convertido a kmodel con DFRobot onnx2kmodel (nncase 2.10.0) e instalado como modelo 129 (firmware HUSKYLENS 1.4.1, según `ANALISIS_EJES_2026-10-05.md`) |
| "No se dispuso de matriz de confusión ni de precisión, exhaustividad o mAP" | Validación (best.pt): cajas P = 0,993, R = 0,991, mAP50 = 0,987, mAP50-95 = 0,958; máscaras mAP50-95 = 0,625 (pieza6 0,437; pieza7 0,547). Matriz de confusión normalizada: pieza6 → 0,93 correcta (0,03 como pieza7); pieza7 → 0,98 correcta |

Es posible que las 800 imágenes de Mind+ correspondan a un modelo anterior (el 128, que la
documentación llama "integración original"). Si es así, la tesis describe un antecedente y
omite el modelo con el que se hicieron las pruebas finales.

**Sugerencia de corrección**

- En 7.5.1, describir el modelo 129: tarea (segmentación), base, entrada, clases, partición
  80/20, hiperparámetros, conversión a kmodel y que el firmware acepta solo `pieza6` y
  `pieza7`. Mencionar el modelo Mind+ como etapa previa si efectivamente existió.
- En 8.5.1, reportar las métricas con su alcance: se midieron en computadora con `best.pt`
  sobre 471 imágenes de validación, **no** en la cámara ni con el kmodel cuantizado. Incluir
  la matriz de confusión normalizada como figura.
- Advertir que las imágenes son fotogramas de video del 2026-10-04 (nombres `…_f000…`), así
  que entrenamiento y validación pueden compartir escenas casi idénticas. Las métricas pueden
  estar sobreestimadas.
- En 9.5.1, discutir la diferencia entre la buena separación 6/7 fuera de línea y la
  "diferenciación no consistente" observada en la maqueta (cuantización, iluminación, ángulo
  de cámara, escena distinta a la del dataset). Confirmar si esa observación se hizo con el
  modelo 129 o con el anterior.
- Aclarar que la HUSKYLENS 2 entrega por UART solo nombre, centro y ancho/alto de la caja,
  sin máscara ni ángulo, aunque el modelo sea de segmentación (ver C5).

### C3. Afirmaciones no sustentadas en Resumen, Abstract, Introducción y Alcance

| Dónde | Afirmación | Por qué no se sostiene |
|---|---|---|
| Resumen ¶3 / Abstract ¶3 | "permitieron evaluar … la precisión de posicionamiento, la repetibilidad, el tiempo de respuesta, la capacidad de detección …; se compararon las posiciones alcanzadas con las posiciones objetivo y se analizó la variación entre distintas ejecuciones" | 8.6.4 y 9.6.3 dicen expresamente que no se midieron precisión ni repetibilidad en mm; 9.5.2 dice que los tiempos del firmware no son tiempos de respuesta medidos; no hay métricas de detección en la tesis |
| Resumen ¶4 / Abstract ¶4 / Alcance ¶1 | "movimiento más fluido, estable y repetible" | La mejora mecánica fue cualitativa (8.3.3, 9.3); la repetibilidad no se cuantificó |
| Introducción ¶3 | "se evaluaron la precisión, repetibilidad, tiempo de respuesta y capacidad de detección" | Igual que arriba |
| Resumen ¶2 / Abstract / Alcance ¶2 y ¶5 | Reconocimiento por "figura y color" | El sistema usa un modelo de aprendizaje profundo por forma; no hay segmentación por color en el firmware |
| Resumen ¶2 / Objetivo general / Alcance | "clasificación" | 8.5.1 dice que la clasificación no quedó validada; además todas las piezas se entregan en el mismo punto (X = borde derecho − 10 mm), sin destino por clase |
| Alcance ¶2 | "trasladarlo hacia su posición asignada" | Hay un único punto de entrega |
| Alcance ¶5 | "La validación del sistema se realizó mediante pruebas de precisión, repetibilidad, tiempo de respuesta" | No ocurrió así |

**Sugerencia de corrección**

- Reescribir Resumen y Abstract (ver propuesta en la sección 6.1): máximo ≈250 palabras
  (hoy 433 y 387), con datos concretos y conclusiones, sin métodos detallados.
- En Introducción, reemplazar el último enunciado por lo que efectivamente se evaluó (120
  intentos de agarre) y agregar las conclusiones principales (la guía lo pide).
- En Alcance, cambiar "figura y color" por "forma, mediante un modelo de detección entrenado
  para dos tipos de pieza". Indicar como limitaciones: una sola pieza a la vez (el detector
  rechaza escenas con varias piezas, `MULTIPLES_PIEZAS`), un único punto de entrega, la
  ambigüedad del sentido de las diagonales y que la clase detectada no se usa para clasificar
  el destino.

### C4. Metodología desactualizada respecto del enlace y Fase 6 vacía

| Dónde | Problema |
|---|---|
| 7.4.2 (`metodologia.tex` ≈l.93) | "La comunicación I2C se implementó mediante `Wire`". En V4 `Wire` solo atiende el I2C interno de Machine Control (expansor de entradas); el enlace usa `ArduinoRS485` 1.1.1 |
| 7.4.4 completa | Describe I2C a 100 kHz como "la versión integrada más reciente". La versión final es UART 115200 8N1 sobre RS485 half-duplex con transporte COBS + CRC16 (44 bytes), protocolo v19 |
| 7.4.5 ¶3 | "Las órdenes … se transmitieron hacia la Portenta mediante el enlace I2C" |
| 7.4.7 ¶2 | La cuarta placa aloja "el enlace I2C con la Portenta"; hoy aloja ESP32 + convertidor de niveles + MAX485 |
| 7.6 | Solo comentarios: falta todo el procedimiento de la validación cuyos resultados sí se presentan en 8.6 |

La migración a RS485 aparece únicamente en Resultados (8.4.6–8.4.7). Según la guía, el
**cómo** se hizo va en Metodología y lo **obtenido**, en Resultados.

**Sugerencia de corrección**

- Reorganizar 7.4.4 en dos partes: "Enlace inicial I2C" (lo actual, en pasado y como
  antecedente) y "Migración a UART sobre RS485": transceptores, pines, temporización de
  turno (15 ms, 200 µs, 2000 µs, 3 ms), capa de transporte, recuperación (100 ms / 1000 ms)
  y las pruebas progresivas (Nano/Uno, test ASCII a 115200, banco simulado de 11 000
  intercambios). En 8.4.6–8.4.7 dejar solo lo obtenido (tasas, diagnóstico, corrección).
- Actualizar 7.4.5 y 7.4.7 a RS485 y agregar `ArduinoRS485` a la lista de bibliotecas.
- Redactar 7.6 (ver propuesta en la sección 6.2).

### C5. Falta la orientación automática de la garra (y explica los resultados por orientación)

La tesis solo menciona el giro de la garra con el joystick (7.4.5). En los modos automáticos,
la ESP32 estima el giro con la caja de la cámara (`giroPorCaja()`, `CalibracionAnguloMLV2.h`):

- La caja alineada con la imagen mide `L|cos θ| + W|sin θ|` × `L|sin θ| + W|cos θ|`;
  normalizándola con las cajas calibradas de cada pieza recta se despeja |θ|.
- Servo: θ ≤ 12° → 59° (eje largo en X); θ ≥ 78° → 155° (pieza6) o 166° (pieza7);
  interpolación entre ambos. Requiere ≥2 cajas compatibles con dispersión ≤35°.
- **Limitación estructural:** "/" y "\" producen la misma caja; el firmware elige un sentido
  fijo (`AUTO_V2_SIGNO_DIAGONAL = +1`), así que la diagonal espejo queda mal alineada
  (documentado en `ANALISIS_EJES_2026-10-05.md` y en el código).
- La Portenta retrasa la captura `40·|cos θ|` mm porque en piezas horizontales el dedo de
  aguas arriba caía delante de la pieza (`V2_RETRASO_CATCH_HORIZONTAL_MM`).
- La **clase detectada** elige las dimensiones de referencia para el giro: un error 6/7
  afecta el ángulo aplicado.

**Sugerencia de corrección**

- Agregar en 7.5 una subsección "Orientación automática del efector final" con lo anterior.
- En 9.6.2, usar estos mecanismos como explicaciones **posibles** (no demostradas) de: la
  baja tasa horizontal (45,2 %), la dispersión diagonal (54,4 %) y la diferencia entre
  piezas. Recomendar registrar en futuras pruebas el sentido de la diagonal ("/" o "\") y la
  clase detectada.
- Definir en 7.6 qué se entendió por "vertical" y "horizontal" (en el código, horizontal =
  eje largo transversal a la banda, X; vertical = eje largo a lo largo de la banda, Y).

### C6. La secuencia del Automático V2 está incompleta (y la entrega sí forma parte del ciclo)

7.5.5 y 8.5.5 describen preposicionamiento, Z a 3000 pasos, descenso, DIN04, cierre y
retirada. Además dicen que el traslado a una posición de entrega era una herramienta de
ajuste de "variantes posteriores". En V4, el ciclo autónomo **incluye** el traslado:
`SUBIENDO_Z → MOVIENDO_ENTREGA → BAJANDO_ENTREGA → ABRIENDO_PINZA → SUBIENDO_FINAL → COMPLETADO`.

También faltan los elementos que deciden el éxito de la captura:

- Estación de captura fija en Y = 0; el brazo **no** persigue la pieza en Y
  (`AUTO_V2_SEGUIMIENTO_Y = false`).
- Detector V2: sello del encoder en la imagen, ≥3 detecciones con ±6 mm y ≥2 mm de avance.
- Estimador de velocidad y aceleración por ventanas de 100 ms y predicción
  `y(T) = y + vT + ½aT²` con `T = t_Z + t_contacto` (0,6 s de bajada final y 0,300 s de
  contacto nominal más el ajuste `CATCH`).
- Disparo cuando la predicción cruza Y = 0 dentro de ±5 mm; continuación hasta ±15 mm.
- Nunca cierra sin DIN04 (búsqueda corta de 600 pasos, espera de 40 ms); confirmación de la
  pinza a los 608 ms; *timeout* de 30 s por fase; cancelación segura.
- Modos auxiliares usados para ajustar el sistema: Enseñanza ML / ML V2, AJUSTE CATCH,
  CAMBIOS CATCH, Registro de ángulo, Prueba de encoder; comandos `CATCH` y `CATCHDIAG`.

**Sugerencia de corrección**

Agregar el diagrama de fases (está en la sección 6.6 de `DOCUMENTACION_RS485_AUTOMATICO_V2.md`)
y una tabla de parámetros. Reformular el párrafo final de 7.5.5: el traslado y la apertura
son parte del ciclo autónomo; los modos de enseñanza y ajuste fueron las herramientas de
calibración. Indicar en 8.6.4 si el operador observó o no la entrega en los 120 intentos.

### C7. Falta el capítulo de Conclusiones y la valoración explícita de cada objetivo

La guía UVG lo exige: "se indica si se cumplieron o no los objetivos planteados". Está
comentado en `main.tex` (l.395–403) y `conclusiones.tex` está vacío. Ver propuesta de
contenido en la sección 6.3.

---

## 3. Hallazgos importantes

### I1. Evidencia cuantitativa disponible que la tesis no usa

**a) Repetibilidad de la referenciación (aporta al objetivo 4).** Los registros
`V3/pruebas de automatico v2/registros_v2/i2c_*.log` contienen el recorrido medido entre
finales de carrera en cada calibración (`[CAL][X] Rango=…`):

| Eje | n | Media (pasos) | Desv. est. | Mín–máx | Equivalente |
|---|---:|---:|---:|---|---|
| X | 13 | 19 944,5 | 57,8 | 19 839–20 012 | 44,72 pasos/mm; s = 1,29 mm; rango 3,87 mm |
| Y | 9 | 11 811,9 | 16,1 | 11 790–11 835 | 35,15 pasos/mm; s = 0,46 mm; rango 1,28 mm |
| Z | 9 | 14 687,2 | 10,8 | 14 676–14 704 | (sin escala en mm) |

Sesiones del 23-09 al 06-10-2026. Advertencias para el texto: mide la repetibilidad del
recorrido detectado por los finales de carrera (sensores + posible pérdida de pasos), no la
repetibilidad del efector final según ISO 9283. Puede incluir ajustes mecánicos entre
sesiones (el X bajó ≈130 pasos a partir del 05-10). Con esto se pueden llenar los pendientes
"recorrido de X/Y/Z en pasos" y "factores pasos/mm" de 8.4.4.

**b) Error de posición del modelo cámara–brazo.** 22 agarres manuales exitosos (ML V2): el
error de posición al contacto pasó de −69 ± 103 mm a −1 ± 24 mm con la corrección del
modelo (`CAPTURA_FIJA_V2.md`). Es una medida de exactitud del modelo de predicción en Y, no
del posicionamiento del brazo. Debe presentarse con esa salvedad.

**c) I2C frente a RS485.** La tesis dice que no hubo comparación (8.4.7, 9.4.2). Hay datos
descriptivos de ambos enlaces, en condiciones distintas:

| Enlace | Registro | Indicador |
|---|---|---|
| I2C | `DIAGNOSTICO_I2C_2026-10-06_171641.md` (≈30 s) | 10,7 % lecturas fallidas en Portenta; 15,7 % envíos fallidos; 14,9 % rechazos en la ESP32; durante el movimiento de Y subió a 22–30 % |
| RS485 | `DIAGNOSTICO_RS485_2026-10-08.md` | 1,66 % tramas malformadas; 186 *timeouts* en 10 129 solicitudes (1,84 %), antes de la corrección de 3 ms |

Sugerencia: presentarlos en 8.4 como comparación **descriptiva** (no controlada). El aumento
de errores I2C durante el movimiento del eje Y apoya, sin demostrarla, la hipótesis de
acoplamiento con la potencia que plantea 9.4.5.

**d) Velocidad de la banda en mm/s.** Los registros V2 traen `belt_mm_s` (≈75–80 mm/s el
05-10; 125–132 mm/s en sesiones de V4 según la documentación). Si se conservan los registros
de las sesiones de los 120 intentos, de ahí sale la velocidad lineal de 25 % y 35 % que 8.6.1
declara faltante. Si no se conservan, medirla de nuevo con el modo Prueba de encoder.

**e) Estadística de las 120 pruebas.** La guía pide resultados "representativos
estadísticamente". Verifiqué que todas las sumas y porcentajes de los Cuadros 8.10 y 8.11
son consistentes. Se pueden agregar intervalos de confianza (Wilson, 95 %) y pruebas exactas
de Fisher:

| Grupo | Éxitos | % | IC 95 % |
|---|---:|---:|---|
| Global | 74/120 | 61,7 | 52,7–69,9 |
| 25 % | 44/60 | 73,3 | 61,0–82,9 |
| 35 % | 30/60 | 50,0 | 37,7–62,3 |
| Pieza 6 | 29/60 | 48,3 | 36,2–60,7 |
| Pieza 7 | 45/60 | 75,0 | 62,8–84,2 |
| Vertical | 29/32 | 90,6 | 75,8–96,8 |
| Horizontal | 14/31 | 45,2 | 29,2–62,2 |
| Diagonal | 31/57 | 54,4 | 41,6–66,6 |

Fisher bilateral: 25 % vs 35 %, p = 0,014; pieza 7 vs 6, p = 0,005; vertical vs horizontal,
p = 0,0001; vertical vs diagonal, p = 0,0004; horizontal vs diagonal, p = 0,50 (sin
diferencia). Advertencia: los ensayos no se aleatorizaron y las orientaciones no están
balanceadas, así que son comparaciones descriptivas, no causales.

### I2. Hardware sin describir

La tesis no nombra los drivers, motores, fuentes ni servos. Datos disponibles:

- Drivers **DM542T** (identificado en fotografía; `figuras_fase4/driver_dm542t.png` está en el
  ZIP pero no se usa). Faltan modelo de motores, micropasos, corriente y fuente de 24 V.
- Encoder **Omron E6B2-CWZ6C**, 1024 P/R (PORTENTA.ino:97). Con esto se cierra el pendiente
  "modelo físico definitivo del encoder" de 7.5.4.
- Finales de carrera en lógica **NC** (abierto o desconectado = activo). Cierra el pendiente
  "configuración física de los contactos" de 7.4.3. DIN04 = Z inferior y DIN05 = Z superior
  están confirmados en el código.
- Pinza: pulsos de 937 µs (abierta) y 1816 µs (cerrada); giro 59°/155°/166°.
- Placa de la ESP32: la documentación la llama "ESP32 UWB" (Makerfabs DW3000). Indicar el
  modelo en 7.4.2 y aclarar que la función UWB no se usó (ver I3).
- Interfaz RS485: MAX485 a 5 V con resistencia de terminación de 120 Ω en el módulo,
  convertidor de niveles 3,3/5 V, terminación interna de Machine Control desactivada.

### I3. Marco teórico: desajustes con lo que se usó

| Tema | Problema | Sugerencia |
|---|---|---|
| "ESP32 UWB DW3000" (6.5.1) | Presenta la UWB como útil para el sistema; el proyecto no la usa | Describir la ESP32 por lo que sí se usó: doble núcleo, Bluetooth (Bluepad32), UART, FreeRTOS. Mencionar la UWB solo como característica no utilizada o quitar Qorvo/Makerfabs |
| ADC (6.5.2) | No hay sensores analógicos en el sistema | Eliminar o reducir |
| Faltan | Motores paso a paso y drivers, servomotores, transmisión por faja y tornillo de avance, cámaras con IA embebida (HUSKYLENS 2), YOLO / redes de detección y segmentación, marcadores fiduciales AprilTag, homografía (formal), RTOS y máquinas de estado, ISO 9283 (exactitud y repetibilidad de pose) | Agregar secciones breves; todos se usan en Metodología |
| Repetibilidad (6.3.4) citada con Seborg et al. (control de procesos) | Fuente poco pertinente para robots | Citar ISO 9283:1998 |
| CRC-8 (6.5.3) vs Metodología 7.4.4 | El marco explica que "CRC-8/ATM" es ambiguo, pero la metodología lo usa sin parámetros completos | En 7.4.4 escribir: polinomio 0x07, valor inicial 0x00, sin reflexión, XOR final 0x00 (equivale a CRC-8/SMBUS) |

### I4. Antecedentes y Justificación

- **Antecedentes** no cita el trabajo de graduación previo (autor, año, título), y la guía lo
  exige ("dar crédito y citar a los autores"). Tampoco incluye trabajos similares (robots
  cartesianos con visión en empaque).
- **Justificación** afirma que la lubricación no resolvió las fallas, pero 7.1 y 8.1 no lo
  documentan. Agregarlo a la Fase 1 o quitarlo. Además, la guía indica que la justificación
  no adelanta resultados: "la implementación … permitió disponer de una plataforma" debería
  plantearse como propósito.
- **Objetivo 1** dice "ejes endurecidos"; la tesis solo dice "ejes de acero de 12 mm".
  Confirmar el tipo de eje (p. ej. acero templado y cromado) y escribirlo en 7.2 y 8.2.
- **Objetivo 3** dice "lazo cerrado". La discusión 9.5.4 ya lo matiza bien; falta llevar esa
  conclusión al capítulo de Conclusiones.

### I5. Arquitectura de supervisión no descrita

La tesis no explica el arranque ni la seguridad del sistema, que son parte importante del
trabajo de firmware:

- Arranque seguro (3 s) → espera RS485 → estabilización (5 s) → *checklist* (enlace,
  protocolo, cámara) → menú.
- Menú y submenú de calibraciones (brazo, cámara, encoder), prueba de servos, diagnóstico.
- Requisitos de entrada a cada modo (control, calibración, cámara, encoder, alturas Z).
- Estado de error con reintento seguro; pérdida de enlace > 1000 ms con conservación de
  calibraciones; regreso a HOME en manual (Z primero, luego XY).
- Comandos por terminal (`POS`, `RANGO`, `GOTO`, `STOP`, `ENC …`, `CATCH`).

Sugerencia: una subsección corta en 7.4 o un anexo con el diagrama de estados.

### I6. Pendientes en comentarios que ya se pueden llenar con el repositorio

| Comentario en la tesis | Dato disponible |
|---|---|
| Versiones de bibliotecas (`metodologia.tex` ≈l.99) | Core ESP32 `esp32-bluepad32` 4.1.0; `mbed_portenta` 4.6.0; `Arduino_MachineControl` 1.1.2; `ArduinoRS485` 1.1.1; `DFRobot_HuskylensV2` 1.0.9; **`ESP32Servo` 3.2.1** (el comentario dice 3.1.3) |
| DIN04/DIN05 y contactos | DIN04 = Z abajo, DIN05 = Z arriba, lógica NC |
| Modelo del encoder, 49 mm, 1:1 | E6B2-CWZ6C 1024 P/R; 49 mm y 1:1 están en el código (método de acoplamiento pendiente) |
| 845 mm y offset de 15 mm | Ya no aplican (ver C1) |
| Arquitectura, particiones y métricas del detector | Ver C2 |
| Recorridos y factores pasos/mm | Ver I1-a |

### I7. Otros datos a revisar

- 8.4.7 dice que los bancos de 1000 y 10 000 intercambios fueron simulados. Es correcto: la
  ronda física del banco final sigue pendiente (`tests/RS485_FINAL_MIGRACION/README.md`).
- 8.5.5: las 5 pruebas con banda detenida usaron el Automático original (solo XY, 4
  detecciones estables ±4 mm). Indicarlo para separarlas del Automático V2.
- 8.6.1 dice que no se registró la versión de firmware de los 120 intentos. Se puede
  **inferir**: las diagonales solo se aceptan desde el cambio "giro en cualquier ángulo" de V4
  (antes se rechazaban), así que las 57 pruebas diagonales con 31 éxitos implican firmware V4.
  Confirmarlo y declararlo.
- Ninguno de los modos describe el criterio de "agarre exitoso" (¿pieza retenida al subir?
  ¿entregada?). Definirlo en 7.6.

---

## 4. Formato según la guía UVG

| # | Hallazgo | Corrección sugerida |
|---|---|---|
| F1 | Falta **Lista de cuadros y figuras** (comentada en `main.tex` l.215–221) y hay 18 figuras y 11 cuadros | Descomentar y usar `\listoffigures` y `\listoftables` |
| F2 | Captions salen como "**Tabla** 8.x" (babel `mexico`) y el texto dice "Cuadro 8.x"; en la Fase 6 el texto también dice "Tabla" | En el preámbulo: `\addto\captionsspanish{\renewcommand{\tablename}{Cuadro}\renewcommand{\listtablename}{Índice de cuadros}}` y cambiar "La Tabla~\ref" por "El Cuadro~\ref" en `Resultados.tex` ≈l.756 y l.794 |
| F3 | Falta capítulo de **Conclusiones** (obligatorio) | Ver 6.3 |
| F4 | Sin **Recomendaciones** (opcional) | Muy recomendable: hay muchos pendientes técnicos documentados |
| F5 | Citas (Autor, año) escritas a mano, pero la bibliografía sale numerada (`babplain`); `\nocite{*}` incluye obras no citadas (Bolton, Boyer, Brunton, Bosch Rexroth, cinco de Oriental Motor, una de THK) | Activar APA (`\def\usarAPA` en `datos_estudiante.tex`), citar con `\cite{}` o `\citeA{}` y quitar `\nocite{*}`. Diferenciar las entradas "s. f." de un mismo autor (OMRON a/b, THK a/b) también en el texto |
| F6 | Metodología y Resultados no tienen ni una cita | Citar documentación de HUSKYLENS 2 y su biblioteca, Bluepad32, Arduino Machine Control, MAX485, AprilTag (Olson, 2011; Wang y Olson, 2016), Ultralytics YOLO11, trabajo previo y el origen del modelo de la garra (pendiente en 7.3) |
| F7 | Resumen de 433 palabras y Abstract de 387; sin datos concretos ni conclusiones | ≤250 palabras, informativo, con datos (ver 6.1) |
| F8 | Introducción sin conclusiones principales | Agregar un párrafo final con los resultados clave |
| F9 | Hoja de Vo.Bo. solo con asesora; tribunal y fecha comentados. `datos_estudiante.tex` aún tiene examinadores y fecha de la plantilla ("MSc. Carlos Juárez", "Ing. Luis Pedro Gonzáles", "5 de diciembre de 2018") | Actualizar con los datos reales antes de descomentar |
| F10 | Carátula: "Licenciado en Ingenieria Mecatronica" sin tildes (`main.tex` l.119 y l.148) y `\uvgcarrera` = "Ingeniería Mecatronica" | "Ingeniería Mecatrónica" |
| F11 | El título termina en punto ("…de la UVG.") | La guía pide títulos sin punto final |
| F12 | Glosario activado con las entradas de ejemplo de la plantilla (`latex`, `fórmula`); el PDF termina con una página vacía (p. 78) | Quitar `\def\CAPglosario` o llenarlo con términos reales (COBS, CRC, homografía, half-duplex, HOME, DIN04…) |
| F13 | Prefacio comentado (contiene *lorem ipsum*) | Opcional; si hay agradecimientos, la guía pide ponerlos en el Prefacio |
| F14 | 10 pt y numeración arriba (`\pagestyle{headings}`); la guía pide 11–12 pt y número abajo al centro | Consultar con la asesora si se respeta la plantilla LaTeX oficial; si no, `11pt` y `\pagestyle{plain}` |
| F15 | Errores de escritura en `marco_teorico.tex`: "adopados" (l.25), "entrono", "caracteristicas" (l.34), "manipulacion", "basica", "aproximacion", "elevacion", "colocacion", "liberacion", "mas" (l.62), "retroalimentacion" (l.64) | Corregir ortografía y tildes |

---

## 5. Lo que sí coincide con el código (no requiere cambios)

- Distribución de funciones Portenta/ESP32; STEP/DIR en DO04/05 (X), DO02/03 (Y), DO00/01 (Z).
- `mbed::Ticker` de 100 µs; finales DIN00–DIN05; bloqueo por sentido; detección de finales
  incoherentes.
- Referenciación X→Y→Z: separación de 300 pasos, segunda aproximación lenta, recorrido
  mínimo de 100 pasos, límite de 180 s, origen en el centro, 446/336 mm, margen de 2 mm.
- Zona neutra del joystick de 100; servos en GPIO25/26 a 50 Hz; OLED SH1106 0x3C en
  GPIO21/22.
- HUSKYLENS 2 por UART1 (GPIO32 RX, GPIO33 TX, 115200 8N1); tarea FreeRTOS con prioridad de
  reposo y 2 ms; máquina de estados de la cámara; 25 muestras por tag; homografía por
  Gauss-Jordan con pivote.
- Encoder 1024 P/R, X2, 49 mm, 0,0752 mm/cuenta; calibración con banda al 50 %, 2 s + 5 s,
  ventanas de 200 ms, ≥100 cuentas y ≥20 ventanas.
- Z a 3000 pasos sobre DIN04, confirmación DIN04 antes del cierre.
- RS485: pines GPIO27/14/18, MAX485 + convertidor, 115200 8N1, tramas de 44 bytes (sesión 4 +
  solicitud 4 + 32 + CRC16 2 → COBS + 0x00), 15 ms / 200 µs / 2000 µs / 3 ms / 100 ms /
  1000 ms, 1,66 % y 1,84 %, bancos simulados.
- Toda la aritmética de los Cuadros 8.10 y 8.11.

---

## 6. Textos propuestos

### 6.1 Resumen (propuesta, 261 palabras con los corchetes; 239 sin ellos)

Los elementos entre corchetes dependen de lo que confirmes (sección 7).

> Se realizó la reingeniería mecánica parcial y la automatización de un brazo robótico
> cartesiano existente para la línea de empaque del Laboratorio de Diseño de Procesos de la
> Universidad del Valle de Guatemala. El guiado original, de tubos metálicos y carros
> impresos, presentaba fricción, desalineación y avance desigual del pórtico. En Autodesk
> Inventor se rediseñaron los carros, el conjunto central y el mecanismo vertical para
> incorporar ejes de acero de 12 mm y rodamientos lineales, y se fabricaron 48 piezas en PLA.
> El control se distribuyó entre una Portenta H7 con Machine Control, que acciona los motores
> paso a paso y lee los finales de carrera y el encoder de la banda, y una ESP32, que atiende
> el mando inalámbrico, la pantalla OLED, la pinza y una cámara HUSKYLENS 2. El enlace entre ambos controladores se migró de I2C a UART sobre RS485
> por la corrupción persistente de paquetes. La cámara localiza las piezas con un modelo
> [de segmentación YOLO11n] y una homografía calibrada con cuatro AprilTags; el encoder
> permite predecir el instante de captura con la banda en movimiento. En 120 intentos
> automáticos se obtuvo 61,7 % de agarres exitosos: 73,3 % con la banda al 25 % y 50,0 % al
> 35 %; las piezas verticales alcanzaron 90,6 %, las horizontales 45,2 % y las diagonales
> 54,4 %. [La referenciación mostró una desviación estándar del recorrido medido de 1,3 mm en
> X y 0,5 mm en Y.] No se midieron la exactitud ni la repetibilidad del efector final, por lo
> que el cuarto objetivo se cumplió de forma parcial.

### 6.2 Metodología, Fase 6 (contenido mínimo)

1. Firmware y modelo usados (V4, protocolo 19, modelo 129) y parámetros vigentes (551 mm,
   `CATCH` aplicado, retraso de 40 mm, signo de diagonal +1).
2. Preparación de cada sesión: calibraciones de brazo, cámara y encoder.
3. Condiciones: consignas del variador de 25 % y 35 % [y su velocidad en mm/s, si se puede
   recuperar]; iluminación.
4. Muestras: piezas 6 y 7 (identidad física puesta por el operador, no la clase detectada);
   30 intentos por pieza y condición; definición de vertical, horizontal y diagonal; sentido
   de la diagonal, si se registró; posición de colocación sobre la banda.
5. Criterio de éxito (p. ej. pieza retenida durante la elevación) y quién lo observó.
6. Registro: libro `Pruebas_Agarre_Brazo.xlsx` (hojas Vel 35 %, Vel 25 %, Resumen) y campos.
7. Análisis: proporción de éxito (Ec. 8.1), intervalos de confianza y prueba de Fisher si se
   adopta I1-e.

### 6.3 Conclusiones (estructura sugerida, una por objetivo y una general)

1. **OE1 cumplido.** Se rediseñaron y fabricaron los sistemas de traslación con ejes de 12 mm
   y rodamientos lineales; desaparecieron los atascos y el *racking* visible; la mejora fue
   cualitativa.
2. **OE2 cumplido.** Modo manual por eje con mando inalámbrico, bloqueo por finales y
   neutralización ante pérdida del mando.
3. **OE3 cumplido parcialmente.** Visión, finales de carrera y encoder integrados; el encoder
   cierra el lazo a nivel de supervisión (posición de la pieza), pero la posición de los ejes
   sigue siendo de lazo abierto por conteo de pasos.
4. **OE4 cumplido parcialmente.** 61,7 % de agarres (IC 95 %: 52,7–69,9 %), con mejor
   desempeño a menor velocidad y en piezas verticales; [repetibilidad de la referenciación];
   error del modelo de posición −1 ± 24 mm en 22 agarres; no se midió la exactitud ni la
   repetibilidad de pose del efector.
5. **Comunicación.** La migración a RS485 redujo los errores observados de 10–16 % (I2C) a
   menos de 2 % (RS485 antes de la última corrección), en condiciones no idénticas.
6. **Recomendaciones derivadas:** medir exactitud y repetibilidad según ISO 9283; sensor de
   presencia en la garra; modelo OBB o salida de ángulo para resolver el sentido de las
   diagonales; registrar sistemáticamente las causas de falla; medir la distancia física entre
   tags y validar la homografía con puntos independientes; ronda física del banco RS485 y
   medición con osciloscopio.

---

## 7. Información que necesito que confirmes

1. ¿Las 800 imágenes de Mind+ corresponden a un modelo anterior (128)? ¿Los 120 intentos y la
   observación de "no diferencia 6/7" usaron el modelo 129 (YOLO11n-seg)?
2. ¿Los 120 intentos se hicieron con el firmware V4 (551 mm, giro por caja)?
3. ¿En los intentos se observó la entrega de la pieza o solo el agarre? ¿Qué criterio se usó
   para marcar "Sí"?
4. ¿Se conservan los registros (`V2LOG`, CSV de `registrar_v2.ps1`) de esas sesiones? De ahí
   salen la velocidad en mm/s, el valor de `CATCH` usado y los tiempos de ciclo.
5. ¿Cuál es la distancia física medida entre centros de tags (382 mm u otra)?
6. ¿Los ejes de 12 mm son endurecidos/cromados? ¿Modelo de motores, micropasos y fuente?
7. Autor, año y título del trabajo de graduación previo, y fuente del modelo de la garra.
8. ¿Se intentó lubricar el mecanismo original (lo afirma la Justificación)?

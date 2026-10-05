# Prueba de segmentacion con calibracion por tags

Abra `HUSKYLENS2_SEGMENTACION.ino` en Arduino IDE y carguelo a la ESP32.
Usa la biblioteca `DFRobot_HuskylensV2` instalada en este equipo. El modelo
de segmentacion debe estar instalado en la HUSKYLENS 2: la ESP32 lo selecciona
y recibe sus resultados. Seleccion inicial confirmada: **segundo modelo,
indice 1, algoritmo 129**.

## Conexion y puesta en marcha

- TX de HUSKYLENS -> GPIO32 (RX ESP32).
- RX de HUSKYLENS <- GPIO33 (TX ESP32).
- GND comun y alimentacion habitual de la camara.
- HUSKYLENS configurada en UART a 115200 baudios.
- Monitor serie de la ESP32 a **115200 baudios**.

El sketch es independiente: solo usa camara y terminal. No inicializa
Bluepad32, I2C con Portenta, OLED ni servos. Cargarlo sustituye temporalmente
el firmware de la ESP32; para volver al sistema, cargue nuevamente su sketch
integrado habitual. Esta prueba no requiere la Portenta.

1. Coloque los tags como en la calibracion existente:

   ```text
   0 superior izquierdo      1 superior derecho
   3 inferior izquierdo      2 inferior derecho
   ```

2. Mantenga fijos camara y tags durante toda la prueba. Al encender, intenta
   conectar, abre Tag Recognition y espera 3 segundos.
3. Captura 25 muestras por tag, como maximo una por tag por consulta. Imprime
   el avance y calcula la misma homografia del algoritmo funcional.
4. Abre automaticamente el algoritmo 129, espera 8 segundos y confirma
   que responda. Sigue intentando la primera consulta hasta 30 segundos.
5. Coloque piezas y observe la salida JSON continua, aproximadamente cada
   200 ms mas el tiempo de respuesta de la camara.

Se conservan las medidas existentes: banda blanca 292 mm, ancho total 412 mm,
centros laterales de tags X=+/-176 mm y distancia entre filas 382 mm.
El origen esta en el centro de los tags; X positivo a la derecha e Y positivo
hacia abajo. Si cambio la geometria fisica, ajuste esas constantes antes de
calibrar. La homografia vive en RAM y se recalcula al reiniciar.

## Comandos

Puede enviar cada caracter con o sin salto de linea; acepta minusculas.

| Comando | Accion |
| --- | --- |
| `C` | Borrar calibracion, reconectar si hace falta y repetir tags/modelo. |
| `M` | Reabrir el modelo seleccionado conservando la homografia. |
| `0` / `1` / `2` | Seleccionar algoritmo 128 / 129 / 130; si ya calibro, abrirlo inmediatamente. |
| `P` | Pausar/reanudar los JSON de deteccion; la camara sigue siendo consultada. |
| `H` | Mostrar ayuda e indice seleccionado. |

Sin conexion, reintenta cada 2 segundos despues de terminar el intento
anterior. La biblioteca puede esperar hasta 5 segundos por respuesta;
las consultas se ejecutan en una tarea FreeRTOS exclusiva con prioridad idle
y un solo intento. Las esperas de carga usan `millis()`. Los comandos pueden
quedar en cola hasta que termine una consulta. Si falla la lectura tres veces,
reconecta y vuelve a calibrar. Si faltan tags por 120 segundos, muestra cuales
faltan y espera `C`.

## Como llegan los datos

Para guardar la terminal en CSV desde un solo puerto COM, abra
`registrador_un_com/ABRIR_REGISTRADOR.cmd`. La aplicacion incluye selector de
COM, baudios, carpeta, terminal en vivo y botones C/M/P/H. Consulte el
[README del registrador](registrador_un_com/README.md) para los pasos.

Los mensajes de estado comienzan con `[BOOT]`, `[UART]`, `[CAL]`, `[MODELO]`,
etc. Las lineas que comienzan con `{` son JSON independientes y se pueden
procesar una por una. Ejemplo ilustrativo, no capturado de la camara:

```json
{"tipo":"frame","frame":12,"ms":25000,"algoritmo":129,"resultados":1}
{"tipo":"segmentacion","frame":12,"ms":25000,"algoritmo":129,"indice":0,"id":1,"nombre":"pieza","contenido":"","tipo_resultado_raw":28,"level_raw":0,"u_px":350,"v_px":220,"ancho_px":60,"alto_px":35,"x_mm":20.50,"y_mm":-15.20,"coordenadas_validas":true,"en_calibracion":true,"en_banda":true}
```

| Campo | Significado |
| --- | --- |
| `frame`, `ms` | Numero de consulta valida y tiempo local de recepcion en la ESP32. No son el ID ni timestamp de captura de la camara. |
| `resultados` | Cantidad devuelta por la biblioteca en esa consulta. Cero significa que respondio sin detecciones. |
| `algoritmo` | Modelo consultado: inicialmente 129. |
| `indice` | Posicion del resultado dentro de esta consulta, desde cero. No identifica una pieza entre frames. |
| `id`, `nombre`, `contenido` | Campos `ID`, `name` y `content` recibidos, sin remapear IDs. Un ID por si solo no garantiza seguimiento de una pieza. |
| `tipo_resultado_raw` | Comando del resultado en la biblioteca: 28 = bloque, 29 = flecha. |
| `level_raw` | Segundo byte firmado del resultado (`level/confidence/rfu1`). Se imprime sin interpretarlo como porcentaje. |
| `u_px`, `v_px` | Centro del bloque recibido, en pixeles. |
| `ancho_px`, `alto_px` | Ancho y alto recibidos, en pixeles. |
| `x_mm`, `y_mm` | Centro del bloque transformado con la homografia; `null` si no se puede convertir. |
| `coordenadas_validas` | Bloque con ancho/alto positivos y conversion finita. No implica estar dentro de la banda. |
| `en_calibracion` | Centro dentro del rectangulo fisico de los cuatro tags. |
| `en_banda` | Centro dentro de la banda blanca y del area calibrada. No valida que toda la pieza quede dentro. |

Se imprimen todos los resultados disponibles, incluso los que quedan fuera
de la banda o tienen ID=0, para inspeccionar exactamente que entrega el modelo.
La biblioteca instalada limita la cache a 10 resultados por consulta.

**Alcance de la segmentacion:** el `Result` de la biblioteca instalada expone
ID, nombre, centro, ancho/alto, contenido y el byte adicional. No ofrece una
mascara ni un contorno de segmentacion. Este sketch tampoco calcula area de
mascara, orientacion ni centroide de la silueta. El centro convertido a mm es
el del bloque recibido. El comportamiento real del modelo personalizado se
debe confirmar con las lecturas de esta prueba. La documentacion de
[DFRobot_HuskylensV2](https://github.com/DFRobot/DFRobot_HuskylensV2) describe
los campos habituales de la segmentacion incorporada; no garantiza campos
adicionales de los modelos personalizados.

## Verificacion

Compilacion verificada el 2026-10-05 con `esp32-bluepad32:esp32:esp32`,
core 4.1.0 y la biblioteca instalada: 734041 bytes de programa (56%) y
89524 bytes de variables globales (27%). Se verifico que las siete funciones
copiadas de geometria, lectura del codigo del tag, homografia y conversion
coinciden con las del algoritmo funcional.

No se cargo el firmware ni se realizo una prueba fisica. La calibracion y los
datos reales requieren ejecutar el sketch con la ESP32 y la HUSKYLENS conectadas.

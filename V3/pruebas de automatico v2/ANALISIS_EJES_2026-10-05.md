# Calibracion de los dos ejes y limite del calculo de diagonales

Fuentes sin modificar:

- Verticales: `registros_v2/ml_v2_2026-10-05_16-34-24.csv` y
  `registros_v2/i2c_2026-10-05_16-31-59.log`.
- Horizontales: `registros_v2/ml_v2_2026-10-05_17-13-14.csv` y
  `registros_v2/i2c_2026-10-05_17-10-47.log`.

El operador confirma la orientacion de ambos lotes. Se cruzan las muestras
ML_SAMPLE con sus sugerencias por objetivo y sesion, y se ajustan medianas
de servo usando solo EXITO. Los originales se conservan.

| Pieza | Eje X horizontal | Muestras X exitosas | Eje Y vertical | Muestras Y exitosas |
| --- | --- | --- | --- | --- |
| pieza6 | 59 grados | 1 | 155 grados | 7 |
| pieza7 | 59 grados | 5 | 166 grados | 2 |

Los seis ensayos nuevos completan descenso, ML_GRIP_APPLIED y confirmacion
EXITO. El servo permanece a 59 grados despues de la primera correccion;
los otros cinco exitos apoyan ese ajuste, sin medir la precision ni demostrar
que sea el unico angulo que funciona. Se necesita ampliar especialmente la
muestra horizontal de pieza6 y la vertical de pieza7.

| Objetivo nuevo | Pieza | Caja px | Votos X/Y | Servo al catch | Resultado |
| --- | --- | --- | --- | --- | --- |
| 2 | 7 | 78 x 34 | 3 / 0 | 59 | EXITO |
| 3 | 7 | 78 x 34 | 3 / 0 | 59 | EXITO |
| 4 | 6 | 71 x 34 | 3 / 0 | 59 | EXITO |
| 5 | 7 | 71 x 41 | 0 / 0 | 59 | EXITO |
| 6 | 7 | 78 x 27 | 3 / 0 | 59 | EXITO |
| 7 | 7 | 75 x 27 | 3 / 0 | 59 | EXITO |

El objetivo 5 era horizontal segun el operador, pero su caja no obtuvo
consenso en mm. No se reduce el umbral para forzar una orientacion: no hay
muestras diagonales que validen esa decision. El caso conserva ajuste manual.

## Lo que falta para girar hacia ambos lados

El operador requiere distinguir inclinaciones positivas y negativas. La
interfaz actual del modelo 129 entrega nombre, centro y ancho/alto de caja;
content llega vacio y no se recibe mascara, contorno ni giro de silueta.
La homografia convierte posiciones, pero no recupera la forma que la caja
perdio. En la biblioteca instalada, Result::angle comparte memoria con width;
su uso documentado es Line Tracking, no un angulo adicional de segmentacion.

Por ejemplo, para un rectangulo en el plano de imagen, las extensiones son:
`ancho = L * abs(cos(theta)) + W * abs(sin(theta))` y
`alto = L * abs(sin(theta)) + W * abs(cos(theta))`.
Son iguales para theta y -theta. Si la clase y el centro tambien coinciden,
las entradas actuales son identicas y un calculo no puede elegir el giro
correcto de forma general. Incluso la magnitud exige un modelo de forma
validado; las cajas variables del detector no lo garantizan.

Los dos ejes calibran el montaje, pero no validan interpolacion ni regresion
para diagonales. El firmware usa 59 para consenso X y 155/166 para Y, solo
en ML V2, con fuente `MLV2_EJES_20261005`. Una caja dominante tambien puede
pertenecer a una diagonal: no se presenta el eje como giro continuo medido.

Para resolver ambas direcciones se necesita que el modelo/camara publique
un angulo calculado con la silueta, un contorno o dos puntos del eje largo.
Con dos puntos, se transforman ambos con la homografia y se calcula
`atan2(y2-y1, x2-x1)`; despues se calibra su correspondencia con el servo.
Eso requiere modificar la salida visual del modelo, no solo ajustar el servo.

## Proyecto de entrenamiento localizado

Se localizaron las conversaciones "Investigar modelos YOLO" y
"Entrena y exporta modelos YOLO K230", y se corroboraron sus archivos locales
en `C:/Users/samue/OneDrive/Documents/Universidad/CodexModelos/ENTRENAMIENTOS`.
El proyecto usa Ultralytics YOLO11n, entrada 320 x 320 y las clases
pieza2, pieza5, pieza6, pieza7 y pieza8. Ya existen entrenamientos de
DETECT, OBB y SEGMENTATION. El instalado como 129 es el de segmentacion,
empaquetado con las herramientas de DFRobot.

El modelo `OBB/best.kmodel` ya esta convertido para K230. Su registro
`OBB/CONVERSION_K230/resultado_conversion.json` confirma simulador PASS,
nncase 2.11.0 y `hardware_tested: false`. No hace falta suponer que aun
deba entrenarse un modelo OBB, pero la conversion no confirma su ejecucion
en el firmware actual de HUSKYLENS 2.

`OBB/CANMV_TEST/obb_geometry.py` calcula el eje largo a partir de cuatro
vertices y devuelve el angulo en [-90,90), equivalente modulo 180 grados.
El ejemplo de inferencia depende de CanMV/K230 y `libs.YOLO.YOLO11`;
su salida JSON no es el protocolo UART que recibe actualmente la ESP.
Sus cuatro pruebas geometricas en PC pasaron. Ademas, se verificaron
pares +30/-30, +45/-45 y +60/-60: el helper distingue los signos, mientras
las cajas alineadas con los ejes tienen iguales dimensiones para cada par.
Son comprobaciones matematicas, no inferencias con piezas reales.

La documentacion oficial de DFRobot enumera para segmentacion nombre,
ID, centro, ancho y alto, sin vertices ni angulo:
https://github.com/DFRobot/DFRobot_HuskylensV2#algorithm
CanMV documenta YOLO11 OBB, pero eso no verifica la compatibilidad del
firmware instalado en esta camara:
https://www.kendryte.com/k230_canmv/en/main/example/ai/yolo_battle.html

El operador confirma firmware 1.4.1 y uso del modelo de SEGMENTATION 129.
Se conserva esa seleccion. La biblioteca instalada y los ejemplos oficiales
de UART/I2C solo documentan cajas para segmentacion; no se encontro una API
documentada para extraer su mascara o angulo. La captura actual tampoco
incluye esos datos. Referencia adicional:
https://wiki.dfrobot.com/sen0638/docs/22636/

Pendiente una ruta compatible para extraer la silueta de segmentacion y
transmitir su eje, o evaluar OBB si se decide cambiar el despliegue. Despues
hay que transformar el eje al plano de la homografia, calibrar el servo
y validar ambos sentidos con piezas inclinadas. No se ha cambiado el
firmware de la camara ni se ha habilitado un angulo continuo en la ESP.

Mientras se incorpora esa señal, las diagonales se prueban en ML V2 con giro
manual y confirmacion X/cuadrado. Esas etiquetas sirven para validar el futuro
estimador, no para eliminar la ambiguedad de las entradas actuales.

Verificacion automatizada: `tests/mlv2_rectas_test.py` recalcula medianas de
ambos CSV, las cruza con los logs y ejecuta la funcion C++ real. Prueba tambien
el objetivo horizontal ambiguo, falta de votos, contradicciones y otras clases.
Resultado: PASS. Los parametros nuevos aun requieren probarse en las placas.

## Reporte de giro opuesto en diagonales

El operador reporta que horizontal y vertical funcionan, pero las piezas
inclinadas provocan un giro hacia el lado opuesto. Se reviso la publicacion
ML V2: una sugerencia >= 0 se escribe directamente en el servo, sin que la
camara haya confirmado el angulo de silueta. La relacion 1.35 solo detecta
dominancia de la caja; no confirma que la pieza este alineada con un eje.

`tmp/diagnostico_diagonal_modelo129.cpp` ejecuta los headers C++ reales:
un rectangulo 80 x 20 a +30 y -30 grados produce en ambos casos una caja
79.282 x 57.321. Con tres votos X, ambos reciben servo 59. Por tanto, el
fallback de ejes puede mover una diagonal hacia un valor inadecuado. El
ejemplo demuestra el defecto de informacion, sin reproducir un ensayo
fisico ni asumir que esas sean las dimensiones de las piezas reales.

El operador confirma que el joystick funciona bien y que el modo probado
es AUTOMATICO V2. En ese modo, `orientarGarraAutomatica(..., true)` usaba
90/0 grados de servo segun la dominancia de caja. No usaba la calibracion
59/155/166, que corresponde exclusivamente a ENSENANZA ML V2.

Se agrega `AUTO_V2_APLICAR_GIRO_POR_CAJA=false`: la funcion retorna antes
de cambiar el angulo o escribir el servo cuando la llamada es de V2.
Conserva el angulo ajustado previamente en Manual, registra ese valor y
muestra `ROT FIJA` en pantalla. Esto desactiva tambien el giro por caja
en horizontales y verticales. Para lotes alineados conocidos puede
habilitarse el parametro, pero no resuelve las inclinaciones.

No se invirtio el joystick. Los movimientos XY/Z, encoder, seleccion
pieza6/pieza7 y secuencia de catch conservan su implementacion. El modo
Automatico original y Ensenanza ML V2 conservan su comportamiento anterior.
V2 con giro fijo requiere piezas con una orientacion compatible con ese
ajuste; para ensayar giros distintos, usar Ensenanza ML V2 y corregir por pieza.

`tests/auto_v2_giro_test.py` ejecuta la funcion real con servo simulado:
para todos los ajustes 0..180 y ocho combinaciones de votos, V2 conserva
el ajuste sin escribir el servo. Comprueba ademas la orientacion X/Y y
el fallback previo en Automatico original. Resultado PASS. La correccion
evita ordenar un giro no medido; no implementa el angulo de las diagonales.
La ESP de pruebas compilo correctamente: programa 805825 bytes (61%),
variables globales 104124 bytes (31%). No se cargo a las placas; falta
verificar el comportamiento fisico tras cargar esta ESP.

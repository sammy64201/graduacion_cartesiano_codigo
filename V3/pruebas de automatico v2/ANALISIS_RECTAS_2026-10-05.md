# Calibracion inicial del servo para piezas rectas en Ensenanza ML V2

Fuentes originales, conservadas sin cambios:

- `registros_v2/ml_v2_2026-10-05_16-34-24.csv`.
- `registros_v2/i2c_2026-10-05_16-31-59.log`.

El operador confirma que todas las piezas iban rectas. Cada objetivo tiene
una sugerencia de ESP y una muestra final de Portenta con el mismo numero;
la captura usa una unica sesion. Todas las cajas son altas y la sugerencia
anterior fue 0 grados, correspondiente al consenso Y. El angulo manual final
es un comando de servo del montaje, no una medida del giro de la silueta.

| Objetivo | Pieza | Caja px | Sugerencia anterior | Servo al catch | Resultado indicado por operador |
| --- | --- | --- | --- | --- | --- |
| 1 | 6 | 23 x 75 | 0 | 154 | EXITO |
| 2 | 6 | 30 x 71 | 0 | 152 | EXITO |
| 3 | 6 | 27 x 64 | 0 | 161 | EXITO |
| 4 | 7 | 27 x 75 | 0 | 161 | EXITO |
| 5 | 7 | 27 x 75 | 0 | 171 | EXITO |
| 6 | 6 | 26 x 61 | 0 | 151 | EXITO |
| 7 | 6 | 31 x 62 | 0 | 165 | FALLO |
| 8 | 6 | 31 x 61 | 0 | 156 | EXITO |
| 9 | 6 | 31 x 61 | 0 | 158 | EXITO |
| 10 | 6 | 31 x 62 | 0 | 155 | EXITO |

La mediana de los exitos proporciona el valor inicial de servo por clase:
pieza6 = 155 grados (7 exitos, rango 151..161); pieza7 = 166 grados
(2 exitos, rango 161..171). Se excluye el fallo a 165 del ajuste, sin atribuir
su causa al angulo: tambien pueden intervenir posicion y momento del cierre.
Estos nueve exitos ocurrieron con ajuste manual; no constituyen nueve exitos
de la nueva sugerencia automatica. El valor 166 interpola los dos ensayos de
pieza7 y requiere nuevas pruebas. No se estima una precision ni un giro continuo.

`ESP/CalibracionAnguloMLV2.h` contiene los dos parametros. Solo Ensenanza ML V2
los aplica cuando hay al menos dos votos Y y ninguno X, y la clase es 6/7.
Si falta consenso o se identifica X, publica `suggested_rot=NA` y conserva
el ajuste manual. La fuente se registra como `MLV2_RECTAS_20261005`, junto
con `votes_x`/`votes_y`. El catch por X y la confirmacion exito/fallo se conservan.

Automatico, Automatico V2, Registrar Angulo, Ensenanza ML original y
Seguimiento Y mantienen su estimacion anterior. Esta primera calibracion no
se extrapola a piezas horizontales ni diagonales. Las reglas de seleccion
del modelo 129 y filtrado por nombre pieza6/pieza7 permanecen activas.

En el registro tambien aparecen errores CRC/longitud y recuperaciones I2C.
El objetivo 6 figura como EXITO pese a un error estimado Y de -241.041 mm;
por tanto este lote no basta para ajustar el desfase o tiempo de catch.
Esos parametros no se modifican en esta correccion de giro.

Siguiente prueba: cargar solo la ESP de esta carpeta y repetir Ensenanza ML V2
con piezas rectas del mismo eje, manteniendo la geometria y montaje. Registrar
si la sugerencia 155/166 permite cerrar sin corregir, o guardar el angulo final
cuando se necesite ajuste. En pieza7 conviene ampliar primero las dos muestras.

Verificacion automatizada: `tests/mlv2_rectas_test.py --compiler RUTA_A_g++.exe`
cruza las diez muestras CSV/log, recalcula medianas solo de EXITO, verifica los
parametros del firmware y reproduce la sugerencia en C++. Incluye rechazo de
eje X, falta de votos, votos contradictorios y clases no admitidas.

Prueba automatizada ejecutada: PASS. Compilacion de la ESP actualizada:
`esp32-bluepad32:esp32:esp32`, core 4.1.0, 805649 bytes de programa (61%)
y 104124 bytes de variables globales (31%). No se cargaron placas ni se
probo fisicamente esta nueva sugerencia.

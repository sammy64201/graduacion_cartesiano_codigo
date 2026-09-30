# Prueba de registro de angulos: 29 de septiembre de 2026

Fuentes: `registros_v2/angulo_2026-09-29_17-26-05.csv` y
`registros_v2/i2c_2026-09-29_17-25-59.log`.

| Objetivo | Caja (px) | Votos X/Y | Aproximacion actual | Angulo elegido |
| --- | --- | --- | --- | --- |
| 2 | 75 x 44 | 0 / 0 | 90° | 55° |
| 3 | 47 x 79 | 0 / 4 | 0° | 156° |
| 4 | 78 x 71 | 0 / 0 | 90° | 129° |
| 5 | 82 x 38 | 4 / 0 | 90° | 65° |
| 6 | 82 x 69 | 0 / 0 | 90° | 31° |

Los cinco objetivos se detectaron, movieron XY, ordenaron el cierre de la
pinza y retiraron Z. El encoder permanecio en cero. Los angulos fueron elegidos
manualmente como orientaciones deseadas; **el agarre fisico aun no se evaluo**.

La caja rectangular no identifica el giro continuo. Tres de cinco detecciones
no tuvieron votos de eje y recurrieron al valor predeterminado de 90°. Ademas,
las cajas 78 x 71 y 82 x 69 px se parecen, pero recibieron etiquetas de 129°
y 31°. Un ajuste basado solo en ancho, alto o posicion sobreajustaria estas
cinco muestras. Se conserva la aproximacion actual de los modos automaticos
hasta disponer de una senal visual de orientacion y mas etiquetas verificadas.
De forma experimental, REGISTRAR ANGULO puede sugerir 60° para cajas muy
anchas y 156° para cajas muy altas, conservando el ajuste manual en las
demas. En estos datos esa regla cubre los objetivos 2, 3 y 5; los objetivos
4 y 6 permanecen ambiguos. Esto no valida el exito fisico del catch ni
autoriza trasladar la regla a los modos automaticos.

El campo `confidence=-128` aparece en las cinco detecciones; no debe
interpretarse como porcentaje de confianza. El firmware siguiente marca ese
valor como `NA`. El registro tambien mostro que `ACK_OBJ_CERRAR_PINZA` liberaba
el objetivo antes de que Z terminara la retirada; el firmware ESP32 se corrigio
para mantenerlo reservado hasta `COMPLETADO`, `CANCELADO` o rechazo.

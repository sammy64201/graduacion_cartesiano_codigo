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

# Captura autonoma en una estacion Y fija

Revision de 2026-10-09 para Automatico V2 de esta carpeta y su variante
[RS485](../automatico%20v2%20rs485/README_RS485.md). Esta guia reemplaza para
el ciclo autonomo las instrucciones anteriores que pedian seguimiento Y.
El seguimiento y las evaluaciones humanas se conservan en sus modos de prueba.
No se cargaron placas ni se acciono hardware durante esta revision.

## Comportamiento y limites fisicos

La garra se prepara en **X de la pieza y Y=0**, con Z en precaptura. Mientras
espera, Y permanece fija y el encoder actualiza la posicion de la pieza.
El descenso final y el cierre se programan para que el **contacto efectivo**
ocurra dentro de la ventana fisica de captura. No se baja para esperar
indefinidamente a que llegue la pieza. La entrega y la retirada mantienen
su secuencia propia despues del intento. Antes de mover lateralmente se
retira Z; hasta quedar libre de la banda se limita tambien el avance de
banda que la pieza retenida puede soportar con Y fija.

El sensor actual **DIN04 confirma la altura final Z**. No detecta la llegada
de la pieza ni confirma que quedo agarrada. Una orden aplicada al servo,
un ACK, el tiempo de cierre cumplido o un ciclo entregado tampoco prueban
agarre fisico. La validacion debe observar la pieza retenida y entregada;
esa observacion no se convierte en un requisito manual del ciclo autonomo.

Antes de publicar `CAPTURE` y empezar la retirada, Portenta exige una
respuesta ESP valida posterior a la orden, la misma secuencia de objetivo
reservada y `servoPinza=130`. Si falta, registra `GRIP_NOT_APPLIED` y
cancela. Esa respuesta confirma el estado logico del comando PWM aplicado;
no existe un sensor adicional de cierre o retencion en esta revision.

La captura con Y fija necesita que la deriva lateral de la pieza sea menor
que la tolerancia X y que la variacion de velocidad durante el descenso y
contacto quede dentro de limites medidos. Cambios arbitrarios de velocidad
despues de iniciar el descenso pueden hacer imposible la intercepcion.
Un intento cuya ventana deja de ser viable se cancela y retira de forma
segura; no se fuerza el cierre para terminar la secuencia.

## Posicion de imagen, encoder y anticipacion

La homografia convierte la deteccion a milimetros del plano calibrado y la
transformacion de montaje la lleva al sistema del brazo. Debe poder obtener
la posicion dentro de toda la region calibrada y visible, sin asumir que
todas las piezas fueron vistas en una marca Y unica. Detecciones demasiado
tardias pueden ser validas para vision y rechazadas para captura por falta
de tiempo de preparacion.

En la ruta autonoma se rechazan varias piezas unicas (`MULTIPLES_PIEZAS`),
cajas recortadas por el borde o extrapoladas fuera del area calibrada
(`CAJA_RECORTADA`) y ausencia de consenso axial (`ORIENTACION_AMBIGUA`).
Las cajas duplicadas de una misma pieza siguen agrupandose. Detectar en
cualquier punto significa dentro de esa region y con una pieza atribuible;
no elimina las restricciones de imagen ni de tiempo disponible.

Para una pieza reservada se conserva su identidad y referencia de imagen:

```text
x_actual = x_imagen
y_actual = y_imagen + signo * mm_por_cuenta * (N_actual - N_imagen)
```

Las cuentas incorporan el movimiento ya ocurrido, incluso si la banda
acelero o freno. No se reconstruye ese recorrido multiplicando una velocidad
inicial por el tiempo transcurrido. La velocidad y, cuando es fiable, la
aceleracion se usan solamente para el horizonte mecanico futuro corto:

```text
y_contacto = y_actual + velocidad * t_contacto
             + 0.5 * aceleracion * t_contacto^2
```

`t_contacto` incluye lo que falta de descenso, transporte/aplicacion de la
orden y movimiento de dedos hasta retener la pieza. El instante de primer
contacto efectivo puede ser distinto del instante de servo completamente
cerrado. Un margen de timeout de comunicacion tampoco es una medicion de
latencia mecanica real.

**N_imagen tiene incertidumbre.** El promedio de muestras de encoder antes
y despues de consultar HUSKYLENS aproxima una referencia, pero no demuestra
el instante de exposicion de la imagen. La fecha de recepcion de una muestra
remota no es necesariamente su fecha de medicion. Debe medir el desfase y
variacion de exposicion, inferencia, consulta y transporte; su error forma
parte de la ventana de prediccion. No asumir precision temporal por recibir
un paquete reciente.

`CAMERA_RETARDO_CAPTURA_CALIBRADO_MS=-1` y
`CAMERA_JITTER_CAPTURA_CALIBRADO_MS=-1` indican medidas pendientes. Son
metadatos: el firmware no posee timestamp real de exposicion ni historial
sincronizado para aplicar una correccion temporal demostrada. Los registros
mantienen `capture_timestamp=NA`, `image_age_known=0`,
`image_reference_uncertainty_mm=NA` y `camera_delay_compensated=0`.
Una configuracion de metadatos no convierte la referencia aproximada en
una imagen sincronizada.

La decision usa un intervalo de posiciones posibles, no solo el centro
predicho. La incertidumbre de homografia/referencia, escala, tiempo,
velocidad y deslizamiento debe caber dentro de la ventana fisica util de
los dedos y la pieza. Si no cabe, la accion correcta es rechazar o cancelar.

## Preparacion, orientacion y guardas

- Preparar X, apertura/orientacion y Z de precaptura en paralelo; el tiempo
  disponible debe cubrir el mas lento y su estabilizacion.
- Armar solo con X estable, Y=0 dentro de tolerancia, giro asentado, Z en
  precaptura y datos de camara/encoder validos. La ruta actual cancela
  si la banda se detiene; no conserva un objetivo armado para reanudarlo.
- Recalcular la ventana mientras espera y durante el descenso. En reversa,
  perdida de referencia, cambio de velocidad no acotado o pieza rebasada,
  cancelar; no sustituir una velocidad negativa por cero para continuar.
- Cerrar solo con DIN04 confirmado y una ventana de contacto viable. La
  permanencia abajo tiene un limite independiente; no esperar abajo una
  llegada tardia. Mantener finales, limites, STOP, control y cancelacion.

El objetivo fisico del giro axial es cerrar sobre el **ancho menor** de la
pieza. El codigo conserva el mapeo de montaje de
`CalibracionAnguloMLV2::sugerir`: eje largo X -> 59 para ambas clases;
eje largo Y -> 155 para clase 6 y 166 para clase 7. No intercambiar esos
votos o etiquetas para deducir el eje de cierre por una regla algebraica.
La correspondencia real entre esos angulos, el eje de cierre y el ancho
menor debe comprobarse fisicamente antes de validar el perfil. La caja
del modelo 129 solo permite consenso axial; una pieza diagonal o una caja
casi cuadrada no determina giro continuo. Sin consenso se conserva el
ajuste anterior, se registra la ambiguedad y se rechaza el objetivo
autonomo; no se intenta capturarlo con una nueva orientacion inventada.

En el paquete ESP, `reservadoV2` usa
`OBJ_V2_REFERENCIA_APROXIMADA=1`, `OBJ_V2_ORIENTACION_AXIAL=2` y
`OBJ_V2_GIRO_APLICADO=4` para la ruta autonoma. La combinacion 7 identifica
un objetivo aproximado, axial y con orden de giro aplicada; no prueba que
el servo haya terminado ni que exista agarre. Portenta espera el tiempo
de asentamiento configurado desde la aceptacion. Ensenanza y AJUSTE CATCH
mantienen esos flags en cero. Las versiones son 18 I2C y 19 RS485, con
32 bytes; no mezclar ESP/Portenta de revisiones distintas.

## Variables fisicas que deben medirse

Los valores compilados anteriores son referencias de puesta en marcha,
no mediciones nuevas de esta revision. No corregir escala, distancia y
tiempo a la vez a partir de una sola captura fallida.

La configuracion entregada tiene `V2_CAPTURA_FIJA_VALIDADA=false`.
Automatico V2 rechaza el objetivo antes de iniciar movimiento y publica
`CALIBRATION_REQUIRED` con perfil `PRELIMINAR`. Medir y configurar las
variables siguientes, verificar su coherencia y solo entonces cambiar el
perfil a validado. Este bloqueo es de la captura autonoma fija: no exige
una confirmacion humana por pieza ni bloquea los modos de ensenanza o
AJUSTE CATCH. Sus parametros temporales no calibran la geometria.

La OLED identifica `AUTO V2 Y FIJO` y, mientras falta validar, muestra
`CALIBRAR CAPTURA` / `PERFIL FISICO` / `TRI:SALIR`. El estado de aplicacion
`SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA=9` comunica esa razon solo en el
modo autonomo; no cambia la maquina general a un fallo de enlace ni
impone ese banner a AJUSTE CATCH.

| Variable | Unidad | Comprobacion independiente |
|---|---|---|
| Escala y sentido del encoder | mm/cuenta, signo | Recorrido fisico marcado frente al delta de cuentas; incluir X2 y acoplamiento real. |
| Homografia y transformacion camara-brazo | mm | Cuadricula estatica en varios X/Y, a la altura de la pieza, incluido el centro real de agarre. |
| Distancia al catch Y=0 | mm | Marca fisica de la estacion y coordenadas de camara; separar offset de escala. |
| Tiempo y variacion de referencia de imagen | ms | Imagen/exposicion frente a cuentas y tiempos de consulta/transporte. |
| Preparacion y asentamiento X/giro | ms | Cambio de posicion/angulo hasta quedar listo; el conteo de pasos no mide perdidas mecanicas. |
| Descenso final a DIN04 | ms | Varias repeticiones desde la misma precaptura con carga representativa. |
| Orden hasta contacto y cierre | ms | Tiempo de orden/aplicacion frente a video del contacto y retencion. |
| Ventana util X/Y y deriva lateral | mm | Apertura/dedos, dimensiones y orientacion de ambas clases, altura y trayectoria. |
| Variacion admisible de v/a y deslizamiento | mm/s, mm/s2, mm | Velocidades reales y cambios de banda, comparando encoder con trayectoria fisica. |
| Permanencia maxima abajo | ms | Tiempo limitado compatible con la pieza, banda y retirada segura. |
| Altura libre, tiempo de retirada y arrastre admisible | pasos, s, mm | Altura que libera la pieza de la banda y avance soportable desde contacto hasta esa altura. No confirma fuerza de agarre. |

El perfil se configura en ambos `PORTENTA/PORTENTA.ino` y el modelo
portable compartido esta en las copias identicas de `CapturaFijaV2.h`.
Los parametros principales son:

- `V2_VENTANA_CAPTURA_Y_MM`: semiancho util alrededor de Y=0;
  `V2_ERROR_GEOMETRIA_CAPTURA_MM`: cota de geometria y deslizamiento;
  `V2_ERROR_RELATIVO_ESCALA`: error relativo del recorrido de encoder;
  `V2_ERROR_REFERENCIA_CAMARA_MS`: cota temporal de la referencia de imagen.
- `V2_MARGEN_TIEMPO_Z_S`: variacion del descenso final;
  `V2_TIEMPO_CONTACTO_MIN_S` / `V2_TIEMPO_CONTACTO_MAX_S`: intervalo hasta
  el contacto efectivo desde aplicar la orden. El horizonte agrega
  `V2_LATENCIA_ORDEN_PINZA_MS` como cota de transporte.
- `V2_VENTANA_VELOCIDAD_MS` y `V2_EDAD_MAXIMA_VELOCIDAD_MS`: ventana de
  estimacion y vigencia; `V2_LIMITE_ACELERACION_MM_S2`: limite admisible
  del modelo; `V2_ERROR_MODELO_ACELERACION_MM_S2`: cota medida de su
  variacion futura. `V2_VELOCIDAD_MIN_CAPTURA_MM_S` y
  `V2_VELOCIDAD_MAX_CAPTURA_MM_S` delimitan las velocidades admisibles.
- `V2_ASENTAMIENTO_X_MS`, `V2_ASENTAMIENTO_GIRO_MS` y
  `V2_ESPERA_MAXIMA_ABAJO_MS`: preparacion y permanencia limitada. Son
  tiempos configurados; los sensores actuales no verifican por si solos
  la posicion fisica del servo ni pasos perdidos del brazo.
- `V2_ALTURA_LIBRE_BANDA_PASOS`, `V2_TIEMPO_RETIRADA_BANDA_MAX_S` y
  `V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM`: altura que libera la banda,
  plazo para alcanzarla y avance soportable mientras la garra retiene
  con Y fija. El avance desde la orden es una cota conservadora;
  no existe sensor de contacto que mida cuando empezo la retencion.

El error de posicion crece con el recorrido y la incertidumbre temporal:

```text
error_posicion = error_geometria + recorrido * error_relativo_escala
                + max(v_max_perfil, |v| + error_v) * error_referencia_s
                + 0.5 * max(a_max_perfil, |a| + error_a)
                    * error_referencia_s^2
```

El desfase de imagen usa los maximos del perfil porque la banda pudo
moverse mas rapido antes de desacelerar: la velocidad actual no acota
por si sola el desplazamiento de una imagen anterior. La cota temporal
y los maximos deben validarse fisicamente para toda la operacion.

Las cotas de aceleracion deben cubrir variaciones futuras durante el
horizonte, no solamente el ruido de ventanas anteriores. Superar el
modelo cancela; ni la derivada medida ni ese limite garantizan anticipar
un cambio no acotado antes del siguiente muestreo.

`V2_AJUSTE_DISPARO_CATCH_MS` y `CATCH_ADELANTO_EXTRA_MS` no intervienen
en la nueva ruta autonoma fija. Las etiquetas de AJUSTE CATCH no producen
una medida de estos parametros ni habilitan `V2_CAPTURA_FIJA_VALIDADA`.

## Validacion preparada, pendiente de hardware

Primero ejecutar las regresiones offline sobre los fuentes de ambas
variantes. Estas verifican cuentas, maquina de estados, guardas y comandos;
no acreditan mediciones fisicas ni una tasa de agarre.

La regresion `tests/auto_v2_vision_capture_test.py` ejecuta la deteccion y
reserva reales ESP en I2C y RS485: nueve puntos interiores, proyeccion y
referencia de cuentas, duplicados, multiples piezas, cajas recortadas,
consenso axial, giro conservado ante ambiguedad y compatibilidad de los
modos de prueba. Tambien verifica el estado de perfil pendiente y la
OLED con hardware simulado. `tests/captura_fija_logger_test.ps1` verifica
ambos CSV sin abrir puertos. Estas dos regresiones pasaron en esta revision.

Tambien pasaron `tests/captura_fija_posicion_test.py` sobre funciones
reales Portenta de ambas variantes y `tests/captura_fija_v2_test.cpp`
sobre el modelo portable. Las suites de integracion de I2C y RS485
verifican ciclo y handshake con hardware simulado. Los logs finales
estan en `../tmp/captura-fija-v2/`; el estado de compilacion de los cuatro
sketches se registra en
[INTEGRACION_AUTOMATICO_V2.md](INTEGRACION_AUTOMATICO_V2.md).

Cuando el usuario autorice la prueba en placas:

1. Medir escala/signo contra recorridos conocidos; repetir al inicio, medio
   y final del rango util y comprobar parada/reversa.
2. Con banda detenida, medir una cuadricula en toda la region visible y
   ambas clases. Separar error constante, deformacion de homografia y
   diferencia entre centro de caja y punto de agarre.
3. Medir tiempos de imagen, preparacion, descenso, orden y contacto con
   registros y video. Repetir con distintos X de partida y velocidades;
   registrar dispersion, no solo el promedio.
4. Ajustar una familia de variables por vez. Error constante apunta a
   offset; error proporcional al recorrido apunta a escala; error que
   crece con velocidad apunta a desfase temporal. Son diagnosticos que
   requieren contraste con las medidas, no correcciones automaticas.
5. Comprobar que incertidumbres y cambios de velocidad medidos caben en
   la ventana de captura antes de habilitar la configuracion validada.
6. Ejecutar una matriz que incluya ambas clases, varios X/Y de primera
   deteccion, velocidades admitidas y cambios de velocidad dentro de
   limites. Incluir llegadas tardias, parada, reversa, datos viejos,
   DIN04 ausente y cambio abrupto que deba cancelar.

El objetivo de aceptacion es **al menos 95 agarres fisicos de 100 intentos
ejecutados** dentro de la configuracion validada, con retencion y entrega
observadas. El disparo de descenso final inicia un intento: una
cancelacion posterior cuenta como fallo en esos 100. Registrar aparte
piezas presentadas y rechazos/cancelaciones anteriores al disparo, con
sus causas, y publicar la cobertura `intentos / presentados`. No excluir
un fallo posterior al disparo ni inferir EXITO a partir del evento de ciclo
completo. La matriz y parametros usados deben acompanar el resultado;
esta revision no afirma haber alcanzado esa tasa.

Conservar por pieza: clase/secuencia, coordenadas originales, cuentas de
imagen y actuales, incertidumbre de referencia, v/a, estado X/Y/Z/giro,
ventana predicha y motivo de aceptar/rechazar/cancelar. Para un intento,
guardar listo, descenso, DIN04, orden/aplicacion de cierre, contacto
observado y resultado fisico. Las etiquetas humanas siguen fuera de las
guardas del ciclo autonomo.

`QUERY` distingue `reference_source` aproximada y las medidas temporales
desconocidas. `OBJECTIVE_REFERENCE` registra flags, cuentas, eje de cierre
solicitado y `rotation_physically_verified=0`. `CAMERA_SPEED` compara
trayectoria observada y encoder para diagnostico; sus sugerencias no se
adoptan como calibracion fisica automatica.

from pathlib import Path
root = Path(__file__).resolve().parents[1]
folder = root / 'pruebas de automatico v2'
guide = '''# Ajuste automatico del catch V2

**AJUSTE CATCH V2** ejecuta el mismo seguimiento Y, giro, descenso, cierre con
DIN04 y entrega derecha de Automatico V2. Al terminar y regresar Z a HOME,
detiene los motores y espera tu evaluacion. No empieza otra pieza hasta recibirla.

## Uso

Cargar ambos sketches de esta carpeta, ESP y PORTENTA, con protocolo **16**.
Conservan paquetes de 32 bytes. No mezclar con los sketches de la raiz.
Entrar a AJUSTE CATCH V2 desde el menu principal. Requiere las mismas
calibraciones de brazo, camara y encoder que V2.

1. Presentar una pieza con giro conocido y mantener estable la velocidad de banda.
2. El brazo sigue con Y e inicia el catch automaticamente; X no dispara.
3. Despues de entregar y subir Z aparece RESULTADO CATCH?
4. **X = LA AGARRO**, **cuadrado = ANTES**, **circulo = DESPUES**.
   ANTES/DESPUES describen el catch que observaste. Si el fallo no fue de tiempo,
   **triangulo descarta y sale** sin aprender de ese intento.
5. Al confirmar, guarda la muestra, calcula el ajuste siguiente y rearma para
   otra pieza. La espera de evaluacion no tiene timeout de fase y permite
   detener la banda, que sigue controlada externamente.

Se mantienen las guardas de alineacion X/Y <=5 mm, recorrido restante para
descenso y cierre, encoder, control, I2C, finales y timeout de movimiento.
Perder alineacion reinicia la espera estable. Un ensayo cancelado no cambia el ajuste.

## Correccion entre ensayos

El parametro compartido con Automatico V2 es
`V2_AJUSTE_DISPARO_CATCH_MS`, inicialmente **0**. Corrige la espera de alineacion
continua anterior al descenso: `300 ms + ajuste`.

- **ANTES:** aumenta el ajuste para descender mas tarde.
- **DESPUES:** disminuye el ajuste para descender antes.
- **LA AGARRO:** conserva el valor probado.

El paso inicial es 50 ms. Cuando cambia el sentido de la correccion, reduce
el paso a 25, 12 y finalmente 10 ms. El rango es **-200 a +1000 ms**: conserva
al menos 100 ms de alineacion. Llegar al limite muestra REVISAR MEDIDAS y
conserva las guardas. No ajusta rueda, cuentas, distancia ni giro usando estas etiquetas.

Tres agarres consecutivos al mismo valor muestran REPETIDO 3/3. Un fallo
reinicia la cuenta. Esto registra tu observacion; repetir con ambas clases y
otras velocidades antes de adoptar el valor. La escala incorrecta puede generar
un error que este ajuste de tiempo no resuelva.

Los cambios se aplican automaticamente al siguiente ensayo de **AJUSTE CATCH V2**.
Se conservan al salir y entrar al menu durante el mismo encendido, y se pierden
al reiniciar Portenta. Automatico V2 usa el valor compilado del parametro.
Su seguimiento mantiene los mismos calculos y movimiento Y.

## Menu CAMBIOS CATCH

Esta opcion esta junto a AJUSTE CATCH V2. No mueve el brazo.

- Primera pagina: ultimo valor **PROBADO**, **PROXIMO** ensayo, agarres consecutivos,
  cantidad de ensayos y estado EN PRUEBA, REPETIDO 3/3 o LIMITE.
- **X** cambia pagina: muestra el nombre del parametro y el ultimo valor probado.
- **Triangulo** regresa al menu principal.

Al entrar o cambiar pagina, la terminal Portenta imprime el detalle y la linea
concreta para `PORTENTA/PORTENTA.ino`, por ejemplo:

```cpp
constexpr int32_t V2_AJUSTE_DISPARO_CATCH_MS = -100;
```

Es un ejemplo; copiar el valor que indique tu ensayo. La linea corresponde al
ultimo valor probado, incluso si aun esta EN PRUEBA. El valor PROXIMO puede no
haberse ejecutado y se muestra por separado. Al adoptar el ajuste, recompilar
Portenta; el mismo parametro se usa en Automatico V2 y como inicio del modo de ajuste.
`CATCH_ADELANTO_EXTRA_MS` pertenece a la ruta fija y no sustituye este parametro.
El aprendizaje de este modo corresponde a la configuracion vigente
`AUTO_V2_SEGUIMIENTO_Y=true`.

## Registros y analisis

Iniciar `registrar_v2.ps1` antes de entrar. Genera
`registros_v2/catch_cal_AAAA-MM-DD_HH-mm-ss.csv` y el log continuo I2C.
El CSV permanece abierto entre intentos y se cierra al salir.

CAL_TRIGGER registra el disparo autonomo y la referencia nominal de 300 ms.
Para un adelanto, reference_status=PROYECTADA indica una referencia futura,
no observada. `delta_reference_ms` compara tiempos de decision, no mide
independientemente el error fisico. CAL_GRIP registra DIN04 y orden de cierre;
CAL_GRIP_APPLIED en ESP registra el pulso aplicado, sin confirmar agarre.
CAL_SAMPLE guarda etiqueta, `tested_offset_ms`, `next_offset_ms`,
`adjust_step_ms`, `adjust_trials`, `adjust_success_streak`, `adjust_confirmed`
y `adjust_limit`. Los `trigger_*` conservan encoder, velocidad y posiciones
al disparar; las posiciones generales de CAL_SAMPLE ya son las de entrega.

CAMERA_SPEED sigue comparando recorrido camara/encoder antes de bloquear la
pieza. `scale_camera_suggested_mm_count` es una escala candidata basada en
homografia; no se aplica automaticamente. La rueda sigue en 49 mm / 2048
cuentas X2 y la distancia camara-HOME en 845 mm. Si los resultados alternan
sin converger, revisar esas medidas y el giro con datos fisicos.

```powershell
python ".\\pruebas de automatico v2\\analizar_catch_v2.py" ".\\pruebas de automatico v2\\registros_v2\\catch_cal_AAAA-MM-DD_HH-mm-ss.csv" --salida ".\\pruebas de automatico v2\\registros_v2\\resumen_catch.md"
```

El analizador resume resultados por clase y ajuste probado, muestra el ultimo
valor y proximo ensayo de cada sesion, y excluye duplicados/etiquetas contradictorias.
Conserva compatibilidad con CSV historicos del disparo manual con X.

## Verificacion

Pruebas de software: ciclo normal V2, disparo autonomo ajustado, guardas,
espera y etiqueta unica, persistencia entre piezas, direccion de correccion,
reduccion de paso, tres agarres, limites, resumen I2C, CRC/version, CSV y analizador.
Compilar ESP con `esp32-bluepad32:esp32:esp32` y Portenta con
`arduino:mbed_portenta:envie_m7`. No se han cargado placas ni probado fisicamente
estos ajustes. Builds locales: `../tmp/build-ajuste-catch-esp/` y
`../tmp/build-ajuste-catch-portenta/`.
'''
(folder / 'AJUSTE_CATCH_V2.md').write_text(guide, encoding='utf-8')
p = folder / 'INTEGRACION_AUTOMATICO_V2.md'
s = p.read_text(encoding='utf-8-sig')
start=s.index('Actualizacion posterior')
end=s.index('## Estado de integracion', start)
s=s[:start]+'''Actualizacion posterior del 2026-10-06: **AJUSTE CATCH V2** es una opcion
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

'''+s[end:]
s=s.replace('CSV requieren analizar los ensayos; el firmware no aprende por si solo.',
'''CSV requieren analizar los ensayos. El modo de ajuste aprende exclusivamente
una espera temporal dentro de limites; no infiere escala o distancia fisica.''')
s=s.replace('el nuevo modo usa version 15', 'el nuevo modo usa version 16')
p.write_text(s, encoding='utf-8')
p=folder/'README_PRUEBAS.md'
s=p.read_text(encoding='utf-8-sig')
start=s.index('**Nuevo: AJUSTE CATCH V2**')
end=s.index('La escala cartesiana',start)
s=s[:start]+'''**AJUSTE CATCH V2** ejecuta el catch automaticamente y espera LA AGARRO/ANTES/DESPUES
tras la entrega. Adapta el tiempo del siguiente ensayo, conserva el seguimiento
y registra valores probados/proximos y comparacion camara-encoder.
**CAMBIOS CATCH** muestra el ajuste y la linea concreta para Automatico V2.
Ver `AJUSTE_CATCH_V2.md`. Protocolo vigente **16**; cargar ambos sketches juntos.

'''+s[end:]
s=s.replace('version 15','version 16').replace('protocolo 15','protocolo 16')
s=s.replace('`SEGUIMIENTO Y`, `AJUSTE CATCH V2`,','`SEGUIMIENTO Y`, `AJUSTE CATCH V2`, `CAMBIOS CATCH`,')
p.write_text(s, encoding='utf-8')
p=root/'README.md'
s=p.read_text(encoding='utf-8-sig').replace('Los sketches de prueba usan protocolo 15','Los sketches de prueba usan protocolo 16')
s=s.replace('modo `AJUSTE CATCH V2` permite disparar con X y evaluar el timing con el mismo\nseguimiento, registrando tambien la comparacion camara-encoder.',
'''modo `AJUSTE CATCH V2` ejecuta el catch, espera tu evaluacion y ajusta el tiempo
del siguiente ensayo. `CAMBIOS CATCH` muestra el valor probado y el parametro
que implementar en Automatico V2. Conserva el seguimiento y registra tambien
la comparacion camara-encoder.''')
p.write_text(s, encoding='utf-8')
print('Guias actualizadas')

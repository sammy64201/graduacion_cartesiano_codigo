# Cancelacion del catch manual antes de DIN04

Fuentes originales:

- `registros_v2/i2c_2026-10-05_16-56-13.log`.
- `registros_v2/ml_v2_2026-10-05_16-57-10.csv`.

| Objetivo | Resultado registrado |
| --- | --- |
| 5 | X a ms Portenta 330904; DIN04 y orden de cierre a 331517. La ESP registra ML_GRIP_APPLIED con pulso 1816 us. El operador confirma EXITO. |
| 6 | X inicia descenso; 586 ms despues, cancela por estimacion Y rebasando catch antes de DIN04. No hay orden ni aplicacion de cierre. |
| 7 | X inicia descenso; 577 ms despues, cancela por la misma causa. No hay orden ni aplicacion de cierre. |

Los intervalos de cancelacion se obtienen de pc_utc dentro del registro;
los 613 ms entre X y cierre del objetivo 5 provienen del reloj de Portenta.
La causa explicita en los dos intentos fallidos es:
`[ML] Cancelando: pieza rebaso el catch antes de DIN04 ML`.
La Portenta pasa a CANCELANDO, publica ACK cancelado y la ESP ordena abrir.
Por tanto esos dos intentos no alcanzan la instruccion de cerrar la pinza.

El modo ML V2 espera exclusivamente X y ya permite al operador elegir el
momento de catch durante alineacion. Sin embargo, el descenso final mantenia
una cancelacion por estimacion Y, propia del control automatico. El operador
podria disparar correctamente segun la observacion fisica y quedar bloqueado
por esa estimacion, que tampoco se ha validado como medida de posicion real.

Correccion exclusiva del descenso manual ML V2: despues de X aceptado,
el rebase estimado se registra una vez como ML_MANUAL_OVERRUN y Z continua
hasta confirmar DIN04. Solo entonces se envia la orden de cierre. Si DIN04
no aparece tras la busqueda acotada, el intento se cancela. Tambien conservan
su efecto timeout, perdida de control/encoder, banda detenida y cancelacion
del operador. Otros modos y disparadores mantienen el control anterior.

No se cambian el angulo, los pulsos de la garra, la velocidad, los desfases,
el encoder ni el protocolo I2C. Tampoco se atribuye la cancelacion a I2C: el
registro contiene errores de ese enlace, pero identifica otra causa en estos
dos intentos. Completar el ciclo manual no garantiza un agarre fisico exitoso;
se confirma con X/cuadrado y debe registrarse como antes.

`tests/mlv2_catch_test.py` extrae y ejecuta en C++ el estado real de descenso
con su preambulo de guardas y hardware simulado. Verifica que el cierre solo
ocurra una vez tras DIN04, que el aviso no se repita, y que se mantengan las
cancelaciones por otros modos/disparadores, falta de DIN04, perdida de control,
encoder invalido, banda detenida, prediccion invalida, timeout y recorrido Y.
Cruza ademas que el lote suministrado tenga botones en 5/6/7 y cierre aplicado
solo en 5. Resultado automatizado: PASS. Pendiente la repeticion fisica.

Para aplicar esta correccion, cargar solo `PORTENTA/PORTENTA.ino` de la carpeta
de pruebas. Repetir ENSENANZA ML V2 y buscar ML_CATCH_TRIGGER en Portenta y
ML_GRIP_APPLIED en ESP despues del descenso; confirmar el resultado observado.

Compilacion completada correctamente con `arduino:mbed_portenta:envie_m7`,
core 4.6.0. Se verifico el evento nuevo en el sketch compilado y en el ELF.
No se cargaron placas durante esta correccion.

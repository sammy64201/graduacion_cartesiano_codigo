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
- `ENC VUELTA`: compara cuentas/índice con las 2048 cuentas/vuelta esperadas.
- `ENC CERO`: único comando que reinicia manualmente conteo e índice.
- `STOP`: detiene inmediatamente el movimiento.

Estos comandos son solo de diagnóstico; la calibración no depende de la
terminal.

## Lógica de interceptación

La ESP descarta muestras de encoder recibidas hace más de 50 ms. Con tres
detecciones coherentes publica X/Y y el conteo exacto de referencia. La
Portenta calcula continuamente:

```text
piezaY = -510 mm + YlocalCamara
         + signo * (conteoActual - conteoReferencia) * mmPorCuenta
```

Después preposiciona X y deja Y 20 mm dentro del límite aguas arriba. Espera
por conteo a que la pieza llegue, la sigue con velocidad anticipada y
corrección proporcional, y exige errores X/Y de hasta 5 mm durante 300 ms.
Entonces baja Z hasta `límite inferior + 300 pasos`, imprime
`CAPTURA_VIRTUAL` y regresa Z inmediatamente a HOME sin dejar de seguir Y.

El intento se cancela si la banda se detiene o invierte, cambia más de 10 %,
la pieza sale del recorrido, falta espacio para bajar/subir Z, se pierde I²C,
cámara, encoder o control, aparece un final inesperado o vence un timeout.

## Confirmación del técnico

Cuando Z vuelve a HOME, Y se detiene y tanto Serial como OLED muestran:

```text
X = CATCH / CIRCULO = FALLO
```

- X (`ctl->a()`): `CATCH_CONFIRMADO`.
- Círculo (`ctl->b()`): `CATCH_FALLIDO`.
- Triángulo (`ctl->y()`): cancela sin clasificar.
- Sin pulsación nueva en 10 segundos: `CATCH_SIN_RESPUESTA`.

Los eventos anteriores se limpian y ambos botones deben estar liberados antes
de armar la confirmación. Por eso una pulsación mantenida no puede confirmar
accidentalmente el intento siguiente. Serial muestra los acumulados de
intentos, éxitos, fallos y sin respuesta.

## Orden de puesta en marcha

1. Trabaje primero con los motores sin herramienta y a velocidad baja.
2. Verifique A/B, OUTC y unas 2048 cuentas por vuelta.
3. Complete en el arranque la medición automática con la banda al 50 %.
4. Valide cámara+encoder observando el objetivo desde aproximadamente
   `Y=-510 mm` hasta que entra en el recorrido del brazo.
5. Pruebe preposición y seguimiento X/Y con Z eléctricamente inhibido.
6. Pruebe Z con la banda detenida y confirme la nueva altura física.
7. Habilite el ciclo completo y pruebe X, círculo, triángulo y timeout.
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

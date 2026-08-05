# Pruebas de Automático V2

Esta carpeta es una maqueta independiente. Los sketches finales de `ESP/` y
`PORTENTA/` no forman parte de estas pruebas ni deben sobrescribirse al cargar
las placas.

## Contenido

- `ESP/ESP.ino`: HUSKYLENS, Bluepad32, OLED e I²C esclavo `0x40`.
- `PORTENTA/PORTENTA.ino`: calibración X/Y/Z, encoder ABZ, motores y máquina
  de estados de interceptación.
- Los dos archivos `ProtocoloI2C.h` son copias idénticas del protocolo de
  prueba versión 4. Ambos paquetes miden exactamente 32 bytes y terminan con
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

## Calibrar la escala

La primera carga tiene esta constante en cero y bloquea Automático V2:

```cpp
constexpr float ENCODER_MM_POR_CUENTA = 0.0f;
```

Desde el monitor serial de la Portenta a 115200 baudios:

1. Ejecute `ENC CERO` si desea comenzar la medición en cero.
2. Ejecute `ENC INICIO`.
3. Mueva la banda una distancia larga y medida; se recomiendan 1000 mm.
4. Ejecute `ENC FIN 1000`.
5. Copie en `PORTENTA/PORTENTA.ino` la línea `constexpr` que se imprime.
6. Recompile y cargue únicamente el sketch de prueba de Portenta.
7. Compruebe ida y vuelta con `ENC ESTADO`. El error aceptable es
   `max(2 mm, 1 %)`, y la distancia debe crecer en el sentido cámara→brazo.

La escala calculada funciona durante la sesión actual antes de recompilar. La
constante compilada es lo que hace que sobreviva reinicios. Si la distancia
decrece en el avance normal, cambie `ENCODER_SIGNO_AVANCE` de `1` a `-1`,
recompile y vuelva a comprobarla.

Comandos disponibles:

- `ENC ESTADO`: A/B, índice, conteo, distancia, velocidad, escala y signo.
- `ENC INICIO`: captura el conteo inicial de calibración.
- `ENC FIN <mm>`: calcula mm/cuenta y muestra la constante que debe copiarse.
- `ENC VUELTA`: compara cuentas/índice con las 2048 cuentas/vuelta esperadas.
- `ENC CERO`: único comando que reinicia manualmente conteo e índice.
- `STOP`: detiene inmediatamente el movimiento.

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
2. Verifique A/B, el sentido, OUTC y unas 2048 cuentas por vuelta.
3. Calibre sobre 1000 mm y compile la constante.
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

# Automático V2: puesta en marcha

Automático V2 es la quinta opción del menú. Los modos manual, automático
original y las dos calibraciones conservan sus códigos y comportamiento. V2
permanece bloqueado hasta que el encoder tenga escala válida y haya producido
pulsos.

## Conexión del encoder

El firmware usa el contador PCNT de la ESP32:

- fase A: GPIO34;
- fase B: GPIO35;
- fase Z: no utilizada en esta versión;
- GPIO34 y GPIO35 no tienen pull-up interno;
- las señales deben acondicionarse y elevarse únicamente a 3.3 V;
- nunca se deben aplicar 12 V o 24 V directamente a la ESP32.

En la variante OMRON CWZ6C las salidas son NPN open-collector. Antes de cablear
se debe comprobar en la placa del encoder el modelo, alimentación, resolución
P/R y colores correspondientes. La tierra y el tipo de interfaz deben cumplir
el esquema eléctrico definitivo de la máquina.

## Parámetros que deben calibrarse

`ENCODER_MM_POR_CUENTA`, en las dos copias de `ProtocoloI2C.h`, está
deliberadamente en `0.0f`. Esto impide habilitar V2 con una escala inventada.
Para obtener el valor:

1. marcar una posición inicial de la banda;
2. ejecutar `ENCODER` en la terminal de la Portenta y anotar el conteo;
3. desplazar la banda una distancia larga medida, por ejemplo 1000 mm;
4. volver a ejecutar `ENCODER`;
5. calcular `mmPorCuenta = distancia / abs(conteoFinal-conteoInicial)`;
6. copiar el resultado en ambos headers y mantenerlos idénticos;
7. ajustar `ENCODER_SIGNO_CAMARA_Y` a `1` o `-1` hasta que el movimiento
   compensado coincida con Y de la cámara.

En `PORTENTA/PORTENTA.ino` también se deben medir y configurar:

- `V2_Y_INICIO_SEGUIMIENTO_MM`: punto donde Y espera y empieza a seguir;
- `V2_Z_SEGURO_PASOS`: altura segura absoluta;
- `V2_Z_AGARRE_PASOS`: altura absoluta del cierre virtual.

Los dos valores Z están inicialmente en cero. Por ello la lógica completa se
ejecuta, pero no existe descenso físico hasta introducir una profundidad segura
obtenida mediante pruebas manuales.

## Secuencia implementada

1. La ESP32 confirma la pieza con tres detecciones compensadas por encoder.
2. La Portenta mueve X y preposiciona Y.
3. Y alcanza la pieza y sigue su velocidad mediante alimentación anticipada y
   corrección proporcional.
4. Tras mantener error X/Y menor o igual a 5 mm durante 300 ms, Z desciende.
5. Se imprime `PINZA_CERRADA_VIRTUAL` con posición, error, velocidad y conteo.
6. Z regresa a la altura segura y el detector se rearma.

Un error de encoder solo bloquea o cancela Automático V2. Las pérdidas I2C y
los finales de carrera conservan el tratamiento de seguridad general.

## Orden recomendado de pruebas

1. Verificar el comando `ENCODER` girando lentamente la banda en ambos sentidos.
2. Calibrar milímetros por cuenta y comprobar una distancia de ida y vuelta.
3. Configurar Z seguro y mantener agarre igual a seguro para probar solo X/Y.
4. Ejecutar V2 a velocidad baja y verificar el error mostrado, sin bajar Z.
5. Configurar gradualmente la profundidad Z y repetir con cierre virtual.
6. Probar desconexión de encoder, parada de banda, cámara, I2C, control y
   cancelación con triángulo en cada fase.


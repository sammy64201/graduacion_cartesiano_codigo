# Fase 4: revisión documental del firmware

Consulta: 7 de septiembre de 2026. Base: carpeta `pruebas de automatico v2` del workspace V3, indicada por el autor como la versión más reciente. No se usaron los sketches de las carpetas raíz ESP/PORTENTA como sustitutos.

## Entrega y alcance

- `metodologia_fase4.tex`: reemplazo propuesto solo para el bloque de la cuarta fase de metodologia.tex, sin agregar un capítulo nuevo.
- `resultados_fase4.tex`: sección para añadir después de la Fase 3 de Resultados.tex.
- `discusion_fase4.tex`: sección para añadir después de la Fase 3 de Discusion.tex.

Los archivos se entregaron separados y como borrador para no modificar lo ya aprobado. Se utilizaron comandos disponibles en la plantilla (`amsmath`, `float`, `tabular`) y etiquetas nuevas con el prefijo fase4. Se incorporaron dos fotografías aportadas por el autor, sin inferir conexiones ocultas ni añadir citas bibliográficas inventadas. Los cuadros identificaron el código como fuente; esta matriz proporcionó la ubicación precisa de sus afirmaciones.

Las conexiones físicas en protoboard, el calibre AWG 22 y el orden de desarrollo procedieron de la descripción del autor. La distribución del software, los canales, las interfaces y los parámetros procedieron de la inspección del código. Los parámetros configurados no se presentaron como resultados medidos. No se cargó firmware ni se accionó hardware.

## Matriz de evidencia

Las líneas se refirieron a la copia consultada; podrían cambiar después de editar el código.

| Afirmación | Archivo dentro de pruebas de automatico v2 | Ubicación |
|---|---|---|
| Canales STEP/DIR y constantes X=446/Y=336 mm | PORTENTA/PORTENTA.ino | Líneas 20–45 |
| Finales DIN00–DIN05 e inversión lógica de lectura | PORTENTA/PORTENTA.ino | leerFinalesCarrera(), línea 539; finalesCoherentes(), línea 550 |
| Generación de pulsos y conteo por flanco ascendente | PORTENTA/PORTENTA.ino | generarPulsoMotor(), línea 564 |
| Movimiento continuo y bloqueo por finales | PORTENTA/PORTENTA.ino | Líneas 677–815 |
| Factores pasos/mm y conversión XY | PORTENTA/PORTENTA.ino | calcularEscalaAutomatica(), línea 855; cinematicaInversaCartesiana(), línea 872 |
| Z recibido sin movimiento cartesiano | PORTENTA/PORTENTA.ino | iniciarMovimientoXY(), línea 922 |
| Doble aproximación, 300 pasos, conteo y cero en centro | PORTENTA/PORTENTA.ino | Constantes 38–40 y funciones 1076–1323 |
| Lectura iniciada por Portenta y validación de paquetes | PORTENTA/PORTENTA.ino | leerPaqueteESP32(), línea 1796 |
| Mando ejecutado por dirección, sin invocar cinemática inversa | PORTENTA/PORTENTA.ino | procesarModoManual(), línea 2261 |
| Vigencia y errores de comunicación | PORTENTA/PORTENTA.ino | enlaceI2CVigente(), línea 1379; vigilarSeguridadComunicacion(), línea 2975 |
| Estados generales ejecutados en Portenta | PORTENTA/PORTENTA.ino | procesarMaquinaGeneral(), línea 3095 |
| HOME de terminal solo XY, GOTO condicionado al menú y calibración | PORTENTA/PORTENTA.ino | Líneas 3256–3447 |
| Dos buses I2C, OLED SH1106 128x64 y pines de servos | ESP/ESP.ino | Líneas 49–81 |
| ESP32 esclava y callbacks | ESP/ESP.ino | requestEvent(), línea 493; iniciarI2CEsclavo(), línea 535 |
| Mapeo de joystick y control local de servos | ESP/ESP.ino | processControllers(), línea 610 |
| Inicialización recuperable de OLED | ESP/ESP.ino | intentarInicializarOLEDNoBloqueante(), línea 675 |
| Paquete de control preparado por ESP | ESP/ESP.ino | prepararSnapshotI2C(), línea 823 |
| OLED manual: direcciones, límites X/Y y ángulos ordenados | ESP/ESP.ino | mostrarModoManual(), línea 3144 |
| Pantalla durante referenciación | ESP/ESP.ino | actualizarPantallaESP32(), línea 3384 |
| Servos: frecuencia y pines | ESP/ESP.ino | setup(), líneas 3562–3569 |
| Versión 5, dirección 0x40, paquetes 32 bytes y CRC | ESP/ProtocoloI2C.h y PORTENTA/ProtocoloI2C.h | Constantes iniciales, estructuras, static_assert y validarPaquete() |

## Discrepancias y límites que no se ocultaron

1. La Portenta fue maestra I2C y ejecutó los estados generales. La ESP32 recibió el mando, manejó la HMI y los servos. No se describió a la ESP32 como coordinadora general única de esta versión.
2. El autor aclaró que primero utilizó joysticks y luego coordenadas por terminal. Se documentaron ambas etapas, sin llamar cinemática inversa a la ruta de movimiento continuo, que no invocó esa función.
3. El autor confirmó Y=336 mm como recorrido físico. Se corrigió ese valor en la copia integrada de Resultados.tex y en las notas de metodologia.tex; no se modificó el código.
4. El HOME de la calibración situó los tres ejes en el centro de sus recorridos contados. El comando HOME de terminal actuó únicamente en XY. No se confundieron ambas operaciones ni se llamó HOME a un extremo.
5. La versión V2 incluyó calibración de cámara y encoder como parte del arranque antes de habilitar el menú. No fue un programa manual independiente. La cronología histórica provino del autor y no se dedujo del orden de arranque actual.
6. Los seis finales se leyeron con inversión lógica. El comentario del firmware describió contacto NC abierto/desconectado como condición activa; el cableado físico no se verificó mediante una fotografía o esquema.
7. Los GPIO y canales se extrajeron del código. La segunda foto permitió leer DM542T en un controlador. No se infirió que todos fueran iguales, ni se dedujeron tensiones, fuentes, corrientes, conexiones de masa, resistencias de pull-up o ajustes de micropasos.
8. La OLED manual no mostró coordenadas cartesianas ni los seis límites individualmente: mostró direcciones XYZ, límites X/Y y ángulos ordenados de servos. La pantalla de calibración mostró el campo de límites.

## Confirmaciones y datos todavía no documentados

El autor confirmó Y=336 mm, el uso inicial de joysticks seguido por coordenadas por terminal y la correspondencia de la calibración con el código actual. Las fotografías se incluyeron sin edición como figuras_fase4/montaje_protoboard.png y figuras_fase4/driver_dm542t.png.

Quedaron sin documentar las tensiones y el modelo de las fuentes, los ajustes de corriente y micropasos, el esquema eléctrico completo y los registros numéricos de calibración y desempeño. Estos datos no impidieron describir las funciones implementadas, pero no se sustituyeron por suposiciones. AWG 22 se atribuyó a las interconexiones entre módulos y microcontroladores, no al cableado de potencia.

## Ampliación de la evidencia I2C

- ProtocoloI2C.h, líneas 189–255: estructuras de 32 bytes y posición del CRC; líneas 259–272 y 310–344: cálculo y validación.
- ESP/ESP.ino, líneas 493–553: entrega de copia y almacenamiento de recepción; 782–870: validación diferida y preparación de la copia.
- PORTENTA/PORTENTA.ino, líneas 93–103: intervalos y umbrales; 1379–1384: vigencia; 1796–1835: lectura y validación; 1879–1930: construcción y escritura; 2975–3010: fallos; 3546–3566: prioridad de envío sobre lectura.
- Los 32 bytes incluyeron campos de etapas posteriores. El cuadro agrupó esos campos sin presentar los resultados de control automático.
- La versión 5 identificó el formato de aplicación, no el estándar I2C. La frecuencia del bus se distinguió de los intervalos programados de sondeo y envío.
- El CRC se presentó como comprobación de integridad, no como garantía de detección de todos los errores.

## Integración en Overleaf

La carpeta tesis_fases_1_4_actualizada contuvo los archivos completos: metodologia.tex, Resultados.tex y Discusion.tex se actualizaron con la Fase 4. La única corrección factual en las Fases 1–3 fue Y=336 mm. El marco teórico y los demás archivos originales se conservaron. Las Fases 5–6 permanecieron sin revisión: la antigua descripción del ESP32 como supervisor principal en Fase 5 quedó pendiente y no debió interpretarse como una conclusión de esta entrega. Tampoco se validaron las afirmaciones experimentales de la introducción metodológica y de la Fase 6.

La nota CORRECCIONES_Y_PENDIENTES.md correspondió a la entrega anterior; para esta actualización prevaleció esta matriz en lo relativo al recorrido Y y a la Fase 4.

Se encontraron registros de Automático V2 de agosto, pero una búsqueda dirigida no localizó las líneas de rangos y escala de calibración. Esos registros no se utilizaron para atribuir mediciones a las pruebas manuales.

## Identificación de las fuentes consultadas (SHA-256)

- ESP/ESP.ino: `455A42A88A6531689A4C98AB102306F2F60F31EE44CC792987867BB9C034E327`
- PORTENTA/PORTENTA.ino: `5B5F4C2BBA0D291C80C40C10863D5C642F689E0030EBDD342B88613A271DD590`
- Ambas copias de ProtocoloI2C.h: `B09DFC85E5CD69FC2E0AF02A943980B8F6FA8F7CB42D33082C26BB2BDF1B95C2`

No se compiló un PDF nuevo ni se verificó visualmente la distribución final en Overleaf. Esta entrega documentó la implementación inspeccionada; no certificó el cumplimiento experimental o la seguridad del prototipo.

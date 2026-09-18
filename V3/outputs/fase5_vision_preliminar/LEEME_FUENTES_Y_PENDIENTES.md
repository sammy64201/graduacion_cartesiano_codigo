# Fase 5: visión y prueba previa al encoder — borrador documental

Se conservaron las seis fases. La Fase 5 se dividió en cuatro subetapas iniciales: conjunto de datos y entrenamiento; comunicación y gestión de cámara; calibración geométrica del plano; posicionamiento visual con banda detenida. El encoder y la sincronización dinámica quedaron para una ampliación posterior de la misma fase. No se modificaron las Fases 1–4, el marco teórico ni el firmware.

## Uso en Overleaf

- `metodologia_fase5_vision.tex`: sustitución provisional del bloque de Fase 5. No contiene las actividades del encoder todavía. No sustituir todo metodologia.tex con este fragmento.
- `resultados_fase5_vision.tex` y `discusion_fase5_vision.tex`: secciones para añadir después de la Fase 4 en sus archivos respectivos.
- `referencias_fase5.bib`: añadir sus dos entradas al archivo bibliografia.bib existente. No reemplazar la bibliografía completa. Las claves nuevas se citaron en el texto.
- Se usaron los comandos de LaTeX ya empleados en la plantilla. No se compiló el PDF; la comprobación fue estática. Los textos se entregaron en pasado y tercera persona/impersonal. Los títulos y nombres de funciones se conservaron como denominaciones.

Los pasajes que indicaron información pendiente se dejaron visibles para evitar que el borrador se confundiera con una validación completa. Se requiere completarlos antes de una entrega definitiva. No se añadieron imágenes ficticias de entrenamiento, marcadores o detecciones.

## Actualización con las confirmaciones del autor

- Se utilizó Mind+ para cargar las imágenes y preparar las anotaciones YOLO.
- Se reunieron 400 imágenes de cada tipo de pieza (6 y 7), 800 en total. El total no se atribuyó a una partición de entrenamiento específica.
- El autor confirmó 292 mm de ancho útil, 412 mm de ancho total y 382 mm entre filas de marcadores. El instrumento y el procedimiento de medición no se documentaron.
- En la primera comprobación con banda detenida se colocó una pieza por prueba y solo se ordenó el posicionamiento. Se reportaron cinco desplazamientos correctos en cinco pruebas; no hubo recogida en esta etapa.
- Se incorporó el resultado agregado 5/5 (100 %) sin inventar coordenadas de ensayos, distribución por clase, tolerancia métrica o mediciones de precisión.
- La foto siguió pendiente. El bloque condicional de LaTeX mostró un recuadro y lo sustituyó por la imagen cuando existió `figuras_fase5/apriltags_banda.jpg`. Para incorporarla, subir un JPEG real con ese nombre y recompilar. El texto de presentación y la fuente también se actualizaron automáticamente.

## Criterio de la guía

Se consultó la Guía UVG 2019 adjunta, páginas impresas 9–11 (páginas 17–19 del PDF): metodología como descripción de materiales y procedimientos; resultados objetivos y cuantificados cuando fue posible, cuadros numerados y citados con fuente; discusión de significado, explicaciones y relación con objetivos. Se revisaron el texto y las páginas renderizadas. La guía permitió presentar resultados y discusión separados. El pasado y la tercera persona correspondieron al criterio explícito del autor.

## Fuentes y alcance de la evidencia

La cronología de entrenamiento, etiquetado YOLO, uso inicial de I2C, incidencias de integración, abandono de clasificación, conteo global y prueba estática procedió de la descripción del autor. El código actual documentó la implementación, pero no probó por sí solo esa historia ni sus resultados cuantitativos. La búsqueda dirigida de archivos de entrenamiento, modelos y configuraciones de conjunto de datos en el workspace, excluyendo compilaciones y salidas, no localizó esos materiales; no se concluyó que no existieran en otras carpetas.

Se inspeccionó `pruebas de automatico v2`, no se utilizó el nombre de una carpeta o un comentario como prueba de que todas las funciones se hubieran validado físicamente.

| Evidencia | Ubicación en la carpeta de pruebas |
|---|---|
| Biblioteca y UART2, RX32/TX33, 115200 | ESP/ESP.ino, líneas 24, 61–65 y 3584–3589 |
| Intervalos de conexión, carga y adquisición; cuatro detecciones y tolerancia de 4 mm | ESP/ESP.ino, líneas 98–118 |
| Dimensiones 292/412/382 mm, 25 muestras/tag, cuatro tags e índice personalizado 1 | ESP/ESP.ino, líneas 871–885 |
| Códigos 0–3, matriz y coordenadas métricas | ESP/ESP.ino, líneas 1148–1179 |
| Identificación y reinicio de datos | ESP/ESP.ino, líneas 1181–1245 |
| Solución lineal y homografía | ESP/ESP.ino, líneas 1247–1366 |
| Transformación de puntos y filtro espacial | ESP/ESP.ino, líneas 1367–1403 |
| Salida de matriz por terminal | ESP/ESP.ino, printHomography(), línea 1405 |
| Estado compartido de cámara | ESP/ESP.ino, publicarEstadoCamara(), línea 1505 |
| Adquisición de centros de tags | ESP/ESP.ino, leerTagsUnaVez(), línea 1634 |
| Identidad de clase en el filtro, acumulación y objetivo en décimas de mm | ESP/ESP.ino, líneas 1714–1800 |
| Lectura automática básica y rama separada V2 | ESP/ESP.ino, leerPiezasUnaVez(), línea 2446 |
| Secuencia y aceptación de comandos | ESP/ESP.ino, procesarComandoCamara(), línea 2605 |
| Conexión, carga, calibración, recuperación | ESP/ESP.ino, líneas 2715–2983 |
| Tarea dedicada y cesión de 2 ms | ESP/ESP.ino, tareaCamara(), línea 2985 |
| Transformación entre referencias: sin intercambio, signos -1, offsets 0 | PORTENTA/PORTENTA.ino, líneas 46–51 y transformarCamaraABrazo(), línea 903 |
| Ruta automática XY sin movimiento Z y sin compensación por encoder | PORTENTA/PORTENTA.ino, procesarModoAutomatico(), línea 2301 |
| Contadores identificados como intentos y catches automáticos en V2, no en la prueba XY básica | PORTENTA/PORTENTA.ino, imprimirContadoresV2(), línea 2579 |

Las líneas se refirieron a la copia consultada el 7 de septiembre de 2026. Se verificaron las huellas:

- ESP/ESP.ino: SHA-256 `455A42A88A6531689A4C98AB102306F2F60F31EE44CC792987867BB9C034E327`.
- PORTENTA/PORTENTA.ino: SHA-256 `5B5F4C2BBA0D291C80C40C10863D5C642F689E0030EBDD342B88613A271DD590`.

## Límites que deben conservarse

1. YOLO identificó el formato de anotación descrito, no una arquitectura de red confirmada. Mind+ y el total de 800 imágenes fueron confirmados por el autor. No se inventaron particiones, épocas, precisión, recall, mAP o matriz de confusión.
2. La migración I2C → UART fue una decisión reportada. El firmware corroboró UART actual, no una causa raíz demostrada de los fallos previos ni una mejora medida de latencia. No se describió UART como modificación del enlace Portenta–ESP32, que siguió siendo I2C.
3. La calibración empleó centros de cuatro tags; 25 lecturas por tag no equivalieron a 100 correspondencias geométricas distintas. Se resolvieron ocho parámetros por Gauss-Jordan, no mediante OpenCV, RANSAC ni calibración intrínseca.
4. El ancho entre centros de tags de 352 mm se calculó desde las constantes: 292 + (412−292)/2. El autor confirmó las dimensiones base 292/412/382 mm. Los ±176 y ±191 mm fueron coordenadas configuradas, no mediciones adicionales independientes de los centros físicos.
5. La homografía produjo posiciones en el sistema del rectángulo de tags. La coincidencia de su origen con el origen central del brazo necesitó comprobación física aparte; los offsets cero no la demostraron.
6. Los campos de clase siguieron interviniendo en el filtro, aunque se abandonó la clasificación como resultado validado. No se eliminó esa dependencia del código.
7. La ruta EST_AUTOMATICO no accionó Z. No se usó como prueba de recogida completa. El conteo informado por el autor debe asociarse a su prueba y procedimiento real, sin equiparar detecciones, ciclos finalizados y recogidas exitosas.
8. El programa integrado actual incluyó encoder en su arranque. Que la ruta básica no compensara por encoder no implicó que la versión actual pudiera iniciarse sin él. Se distinguió la secuencia histórica del autor de la dependencia de arranque actual.
9. No se documentó esta etapa como servo visual en lazo cerrado. Se documentó una localización visual seguida por posicionamiento XY. La integración del encoder requiere identificar qué variable realimentó, qué error se corrigió y qué acciones se actualizaron.

## Información solicitada al autor

1. Ruta del conjunto de datos, etiquetas y entrenamiento; modelo/versión y parámetros; divisiones de entrenamiento/validación/prueba y métricas disponibles. Si no se conservaron, indicarlo. Fecha y lugar de adquisición. Mind+ y 400 imágenes por clase ya quedaron confirmados.
2. Fotografía con tags 0–3; posición/altura de cámara y tags; comprobación métrica de origen y signos respecto del brazo. Las dimensiones 292/412/382 mm ya quedaron confirmadas.
3. Si se conservaron, posiciones y registros de las cinco pruebas estáticas. El posicionamiento sin recogida y el resultado 5/5 ya quedaron confirmados. La medición instrumentada y el conteo de recogidas de etapas posteriores siguieron pendientes, sin impedir documentar esta comprobación funcional.

Para ilustrar resultados se solicitaron evidencias reales: una imagen anotada, ejemplos de detección/confusión de clases y la disposición de los cuatro tags. No se generaron sustitutos gráficos de esas evidencias.

## Referencias conceptuales

- [AprilRobotics: AprilTag](https://github.com/AprilRobotics/apriltag): terminología de marcador fiducial. No se atribuyó al proyecto una implementación de pose 6D por utilizar AprilTags.
- [OpenCV: homografía](https://docs.opencv.org/4.10.0/d9/dab/tutorial_homography.html): correspondencia entre planos y matriz proyectiva. Se utilizó como respaldo conceptual, no como evidencia del montaje ni como biblioteca del firmware.

La revisión de la guía influyó en mantener el procedimiento fuera del relato de resultados, agrupar las funciones relevantes en cuadros y conservar las limitaciones y su relación con los objetivos en la discusión.

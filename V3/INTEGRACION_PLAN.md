# Plan de integracion ESP32 - Portenta H7 - HUSKYLENS 2

## 1. Alcance y criterio de preservacion

- Crear dos sketches nuevos e independientes: `ESP/ESP.ino` y `PORTENTA/PORTENTA.ino`.
- Mantener intactos como referencia los sketches funcionales originales:
  `ESP_SLAVE/ESP_SLAVE.ino`, `MAster/MAster.ino` y
  `algoritmo_funcionando/algoritmo_funcionando.ino`.
- Conservar todos los pines, sentidos, escalas, logica NC de finales de carrera,
  cinematica cartesiana, comandos de terminal, Bluepad32, servos y OLED existentes.
- Mantener la Portenta como unico maestro del bus I2C de control y la ESP32 como
  esclavo `0x40`; el bus independiente de la OLED seguira bajo control de la ESP32.
- Mantener HUSKYLENS 2 por UART2 a 115200 baudios: GPIO32 de la ESP32 como RX y
  GPIO33 como TX.

## 2. Estructura de los entregables

- `ESP/ESP.ino`: coordinacion local de Bluepad32, servos, OLED, I2C esclavo y tarea
  de vision.
- `ESP/ProtocoloI2C.h`: copia versionada del protocolo comun.
- `PORTENTA/PORTENTA.ino`: coordinador general, motores, calibracion, checklist,
  menu y movimiento automatico XY.
- `PORTENTA/ProtocoloI2C.h`: copia identica del protocolo comun.
- `INFORME_INTEGRACION.md`: arquitectura, conexiones, estados, protocolo, pruebas,
  parametros ajustables y riesgos fisicos.

Las dos copias de `ProtocoloI2C.h` se usan porque Arduino compila cada carpeta de
sketch de forma independiente. Se verificara que sean byte a byte identicas.

## 3. Protocolo I2C versionado

Redisenar ambos sentidos con estructuras `packed`, `magic`, version, longitud y
CRC-8/ATM, sin enviar `float`.

### ESP32 hacia Portenta

Paquete menor de 32 bytes con:

- secuencia de estado;
- joystick X/Y/Z normalizado a `-1`, `0` o `1`;
- Bluetooth y botones;
- angulos de ambos servos;
- estado/error/flags de camara;
- avance de muestras de tags 0, 1, 2 y 3;
- clase, X/Y en decimas de milimetro y secuencia del objetivo estable.

### Portenta hacia ESP32

Paquete de exactamente 32 bytes con:

- estado general y opcion de menu;
- fase/error de calibracion del brazo;
- comando de camara y secuencia de comando;
- flags de calibracion, modo automatico, brazo ocupado, HOME y error critico;
- finales de carrera y movimiento compactado;
- acuse de secuencia del objetivo;
- cuatro valores de presentacion para la OLED.

Los callbacks I2C de la ESP32 solo copiaran/escribiran instantaneas ya preparadas.
La validacion, la camara, los mensajes Serial y la OLED se ejecutaran fuera de los
callbacks.

## 4. Integracion no bloqueante de HUSKYLENS

- Iniciar UART2 sin esperar a que la camara responda.
- Ejecutar todas las llamadas de `DFRobot_HuskylensV2` en una tarea FreeRTOS de
  baja prioridad, nunca en `loop()` ni en un callback I2C.
- Configurar un solo reintento de la libreria. La version instalada usa esperas
  internas de hasta 5 s; la tarea aislada evita que esas esperas congelen
  Bluetooth, servos, OLED o I2C.
- Usar estados publicados equivalentes a: `OFFLINE`, `CONNECTING`, `STANDBY`,
  `OPENING_TAGS`, `CALIBRATING`, `CALCULATING`, `OPENING_MODEL`, `READY` y `ERROR`.
- Sustituir las esperas de carga de 3 s, 2 s y 8 s del ejemplo por transiciones
  temporizadas con `millis()` dentro de la tarea.
- Reintentar conexion periodicamente y limpiar objetivos antiguos al desconectar o
  cambiar de modo.
- Conservar sin simplificar: tags 0..3, 25 muestras por tag, geometria 292/412/382
  mm, centros laterales a +/-176 mm, solucion Gauss-Jordan 8x8, homografia,
  conversion pixel-mm y validaciones de area/banda.

## 5. Filtro y handshake de objetivos

- Exigir cuatro detecciones consecutivas de la misma clase.
- Exigir dispersion maxima configurable de 4 mm en X/Y.
- Aceptar solo puntos dentro del cuadrilatero calibrado y sobre la banda blanca.
- Publicar un objetivo latched con secuencia no nula y coordenadas `int16_t x10`.
- No crear objetivos cuando el modo automatico no este activo o el brazo este
  ocupado.
- Mantener el objetivo hasta recibir el acuse de la Portenta.
- Tras el acuse, exigir ausencia de la pieza durante un tiempo configurable antes
  de rearmar el filtro, evitando ordenes repetidas para la misma pieza.

## 6. Maquina general de la Portenta

Implementar el flujo autonomo:

1. `BOOT_SAFE`: salidas STEP en LOW y motores detenidos.
2. `WAIT_I2C`: aceptar solo un paquete completo y valido de la ESP32.
3. `I2C_SETTLE`: contar 5 s desde ese primer paquete valido; reiniciar si se pierde
   la comunicacion.
4. `CAMERA_CALIBRATION`: enviar comando versionado de calibracion y esperar camara,
   homografia y modelo listos.
5. `ARM_CALIBRATION`: ejecutar X/Y/Z y HOME directamente desde `loop()`, sin
   depender de Bluetooth.
6. `WAIT_CONTROLLER`: esperar el mando con motores detenidos.
7. `FINAL_CHECKLIST`: comprobar protocolo, camara, homografia, modelo, XY, Z, HOME,
   motores, Bluetooth, error de calibracion y coherencia de finales.
8. `MAIN_MENU`: habilitar exactamente las cuatro opciones solicitadas.
9. `SYSTEM_ERROR`: detener motores, informar la causa y permitir un reintento
   seguro con el boton principal cuando exista control.

## 7. Modos de usuario

1. **Modo manual**: conservar joystick X/Y/Z, limites, servos y retorno con
   triangulo.
2. **Modo automatico**: recibir un objetivo nuevo, transformar camara a brazo,
   validar rangos, mover solo X/Y, confirmar secuencia y quedar detenido en el
   objetivo. Triangulo cancela y regresa al menu.
3. **Calibracion de brazo**: iniciar X/Y/Z inmediatamente, llegar a HOME y volver al
   menu; no exige Bluetooth durante el movimiento.
4. **Calibracion de camara**: invalidar la calibracion anterior, ordenar tags,
   homografia y modelo, y regresar al menu al quedar lista.

Los comandos `GOTO`, `HOME`, `POS`, `RANGO`, `STOP` y `AYUDA` seguiran disponibles
por terminal. `GOTO`/`HOME` usaran la misma validacion segura sin convertirlos en
una opcion adicional del menu.

## 8. Transformacion camara-brazo y seguridad

- Aislar `swap`, signos y offsets en una funcion configurable.
- Rechazar valores no finitos, fuera del rango calibrado o dentro del margen de
  seguridad.
- No mover Z en automatico ni implementar agarre/pinza automatica.
- Detener todos los motores ante timeout I2C, paquete invalido persistente, final
  inesperado, timeout de movimiento o error critico.
- Invalidar la posicion XY si un final interrumpe un movimiento posicionado.
- No ejecutar objetivos viejos tras reinicio/reconexion de la ESP32.
- Detectar como incoherente que ambos finales del mismo eje esten activos a la vez.

## 9. Pantalla y diagnostico

- Actualizar la OLED fuera de callbacks, como maximo a 10 Hz.
- Mostrar arranque, camara, calibracion, espera de control, checklist, cuatro
  opciones, manual, automatico y errores.
- Mantener inicializacion recuperable de OLED: un fallo de pantalla no bloquea los
  demas subsistemas.
- Usar prefijos `[BOOT]`, `[I2C]`, `[CAM]`, `[CAL]`, `[AUTO]` y `[ERROR]`.

## 10. Verificacion

- Comparar automaticamente las dos copias del protocolo.
- Comprobar tamanos, offsets basicos y checksum con una prueba C++ de host.
- Buscar esperas largas y confirmar que no existan en `loop()`/callbacks ESP32.
- Verificar por inspeccion que pines, sentidos, finales, escalas y comandos se
  conservaron.
- Compilar ambos sketches si las herramientas y cores Arduino locales lo permiten;
  si no, ejecutar comprobaciones estaticas y documentar exactamente la limitacion.
- Documentar las pruebas fisicas pendientes: polaridad real, cableado cruzado UART,
  posicion de tags, offsets camara-brazo y parada de emergencia.

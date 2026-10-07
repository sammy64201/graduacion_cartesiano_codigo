# Registrador CSV de un solo COM

Aplicacion independiente para Windows, con ventana, para capturar la salida
del sketch de segmentacion de la ESP32. Usa Windows PowerShell 5.1 y .NET que
vienen con Windows: no requiere instalar Python ni paquetes.

## Abrir y capturar

1. Ejecute **ABRIR_REGISTRADOR.cmd** con doble clic. Mantenga los tres archivos
   `.cmd`, `.ps1` y `.psm1` juntos en esta carpeta.
2. Cierre el monitor serie de Arduino u otra aplicacion que tenga abierto el
   COM de la ESP32. Un puerto no se puede abrir desde ambas aplicaciones a la vez.
3. Elija **un COM**, deje **115200** baudios y seleccione una carpeta de salida.
4. Pulse **Iniciar captura**. Desde ese momento cada linea recibida se escribe
   inmediatamente en un CSV nuevo y se muestra en la terminal de la ventana.
5. Pulse **Calibrar (C)** si necesita repetir la calibracion desde el inicio.
   El programa no envia comandos automaticamente al conectar.
6. Termine con **Detener** o cierre la ventana. Se cierra el archivo y se libera
   el puerto. **Abrir carpeta** muestra la carpeta donde estan los CSV.

Tambien puede usar **Abrir modelo (M)**, **Pausa salida (P)** y **Ayuda (H)**.
El cuadro de envio permite `0`, `1` o `2` para cambiar el modelo, o cualquiera
de los comandos anteriores. P pausa la salida del sketch: lo que ya esta en
el CSV permanece guardado. **Limpiar pantalla** solo limpia la vista.

Los archivos se llaman `captura_COM5_20261005_123456_abcdef.csv` (ejemplo).
Cada inicio crea un archivo diferente, sin sobrescribir capturas anteriores.
Por defecto se guardan en la subcarpeta `registros`, junto a la aplicacion.

## Contenido del CSV

Todas las lineas de la ESP32 quedan en **un mismo CSV**:

- `tipo=segmentacion`: una fila por objeto, con ID, nombre, centro/tamano en
  pixeles, X/Y en milimetros y los indicadores de validez/ubicacion. El sketch
  actualizado solo emite pieza6/pieza7 y agrega clase interna, eje aproximado,
  angulo del eje y sugerencia de servo. Los angulos ambiguos quedan vacios.
- `tipo=frame`: una fila por consulta, incluyendo `resultados=0` cuando no hay
  detecciones. Permite distinguir ausencia de piezas de ausencia de datos.
- `tipo=mensaje`: mensajes de arranque, calibracion y errores de la ESP32.
- `tipo=error_json`: JSON que no pudo interpretarse; conserva la linea y el error.
- `tipo=fragmento`: una ultima linea sin salto al detener, o un bloque demasiado
  largo sin salto de linea. Se conserva aunque no se interprete como JSON.

`pc_utc` contiene fecha/hora UTC de procesamiento en la computadora,
`puerto_com` y `baud` identifican la conexion, y `raw` conserva la linea original
sin el terminador CR/LF. Los campos desconocidos de JSON siguen disponibles en
`raw`. Los mensajes locales `[APP]` que muestra la ventana no son datos de la
ESP32 y no se agregan al CSV.

Columnas:

```text
pc_utc,puerto_com,baud,tipo,frame,ms,algoritmo,resultados,indice,id,nombre,
contenido,tipo_resultado_raw,level_raw,u_px,v_px,ancho_px,alto_px,x_mm,y_mm,
coordenadas_validas,en_calibracion,en_banda,permitidos,ignorados,clase_pieza,
recogible,eje_aprox,orientacion_valida,orientacion_aprox_deg,servo_sugerido_deg,
metodo_angulo,error_parseo,raw
```

El archivo usa UTF-8 con BOM, coma como separador, comillas escapadas y punto
decimal. Los valores JSON `null` quedan vacios; `0` y `false` se conservan.
En Excel, si aparece todo en una columna, importe con **Datos > Desde texto/CSV**
y elija coma como delimitador. Para analizar solo piezas, filtre la columna
`tipo` por `segmentacion`.

## Comportamiento de la conexion

Lee fragmentos sin bloquear esperando un salto de linea. Reconstruye los JSON
que llegan en varias lecturas y mantiene limitada la terminal visible; el CSV
conserva toda la captura. Ante un error de lectura/escritura, detiene la captura
y muestra el motivo. Para volver a conectar, inicie otra captura, que genera
otro CSV. La aplicacion no abre un segundo COM ni conecta con la Portenta.

DTR y RTS permanecen deshabilitados: no se genera una secuencia intencional de
reset. Segun el adaptador USB, abrir el puerto puede reiniciar la placa. Use
`C` desde la ventana para comenzar otra calibracion mientras se esta grabando.
La aplicacion guarda lo recibido desde Iniciar; no puede recuperar mensajes
anteriores ni datos que se pierdan en USB durante una desconexion.

## Verificacion

Se verificaron la construccion de la ventana y la conversion a CSV con datos
simulados: JSON dividido entre lecturas, CR/LF, texto de calibracion, frames
vacios, caracteres especiales, comillas/comas/saltos dentro de campos, nulos,
decimales con configuracion regional espanola y fragmento al cerrar. No se
abrio un puerto real durante estas pruebas.

La prueba de conversion puede ejecutarse desde V3:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tests\registrador_un_com_test.ps1
```

Si prefiere abrir desde una terminal:

```powershell
powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File .\registrar_un_com.ps1 -Puerto COM5
```

El parametro `-Puerto` preselecciona el puerto si esta disponible; no inicia
la captura automaticamente. `ExecutionPolicy Bypass` se aplica solo al proceso
que ejecuta esta aplicacion y no cambia la configuracion permanente de Windows.

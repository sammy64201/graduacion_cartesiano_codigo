from pathlib import Path

path = Path('pruebas de automatico v2/README_PRUEBAS.md')
text = path.read_text(encoding='utf-8-sig')
start = text.index('Después preposiciona X con la coordenada detectada')
end = text.index('## Orden de puesta en marcha', start)
text = text[:start] + '''Despues preposiciona X con la coordenada detectada y Y en la estacion
de catch, y baja Z en paralelo a la precaptura de 3000 pasos sobre DIN04.
Con `AUTO_V2_SEGUIMIENTO_Y=true`, el brazo sigue la posicion estimada por
encoder. El destino Y usa un margen total de 6 mm en cada extremo fisico.
Tras mantener X/Y dentro de 5 mm durante 300 ms, inicia el descenso final
automaticamente si queda recorrido para descenso, cierre y reserva de 200 ms.
Y continua siguiendo durante el descenso y el cierre. Solo con DIN04 y Y
alineada envia `ACK_OBJ_CERRAR_PINZA`; si falta DIN04, busca lentamente una
distancia limitada y cancela si no aparece. X no dispara el catch.

Con `AUTO_V2_SEGUIMIENTO_Y=false`, se conserva la ruta fija: Y espera en 0
y el umbral anticipa el descenso final mas 450 ms de garra y 50 ms de I2C.
`CATCH_ADELANTO_EXTRA_MS` permite ajustar esa anticipacion. Ambas rutas usan
el contador vivo, la misma precaptura, DIN04 y la entrega completa.

Despues del cierre, se retira Z y se traslada X/Y en paralelo al destino
derecho compartido con ML: X=maximo-10 mm, Y=0. Cuando los tres ejes terminan
y Z esta en HOME, baja a DIN04, envia `ACK_OBJ_ABRIR_PINZA`, espera 450 ms
mas 50 ms de transporte y vuelve a subir Z. La orden de apertura mantiene
reservado el objetivo; `ACK_OBJ_COMPLETADO` solo se publica tras la retirada
final. La camara permanece bloqueada durante todo el ciclo.

Se cancela el intento si falta recorrido Y, la pieza no queda alineada al
cerrar, falla DIN04, se detiene la banda antes de terminar la captura, falta
encoder/control/I2C, aparece un final inesperado o vence un timeout. La banda
no condiciona la entrega de una pieza ya capturada. Una cancelacion propia del
intento retira Z y mantiene V2 activo; triangulo, STOP o perdida del control
salen del modo. La perdida de I2C aplica la parada y recuperacion general.

Las fases normales con seguimiento son
`0 -> 1 -> 4 -> 16 -> 11 -> 10 -> 12 -> 13 -> 14 -> 15 -> 7 -> 0`.
La fase 9 prepara Z al entrar y 8 corresponde a cancelacion. Las fases 12..15
son traslado, bajada de entrega, apertura y retirada final; 16 es seguimiento.
El registrador y la OLED reconocen estos estados. Las fases previas conservan
sus numeros para leer registros historicos.

## Catch y entrega automaticos

El resultado final es `CICLO_ENTREGADO_NO_VERIFICADO`: confirma que termino
la secuencia de software. No hay sensor que compruebe agarre fisico ni
realimentacion de la posicion del brazo; Y se estima con los pasos ordenados.
El evento CAPTURE describe el cierre ordenado y RELEASE la apertura en entrega.
La sugerencia de giro, su fuente y el angulo aplicado quedan en el CSV.

La ESP conserva el objetivo durante captura, apertura y retirada final. Tras
COMPLETADO, mantiene HUSKYLENS bloqueada durante 500 ms y luego permite otra
publicacion. Una deteccion posterior recibe otra secuencia y puede procesarse
aunque tenga la misma clase.

''' + text[end:]
text = text.replace('5. Pruebe la preposición fija X/Y y confirme que Y no se mueve durante la\n   espera ni durante el descenso Z.',
                    '5. Pruebe el seguimiento Y a baja velocidad: confirme alineacion antes\n   del descenso y durante el cierre, margen de recorrido y cancelacion segura.')
text = text.replace('   `READY_CATCH -> GRIP_COMMAND -> CAPTURE (PINZA CERRADA) -> RESULT\n   (CATCH_AUTOMATICO)`. Triángulo debe seguir cancelando y no debe producir un\n   resultado positivo.',
                    '   `READY_CATCH -> AUTO_TRACK -> GRIP_COMMAND -> CAPTURE -> RELEASE\n   -> RESULT (CICLO_ENTREGADO_NO_VERIFICADO)`. Confirmar entrega derecha,\n   apertura y retirada final. Triangulo debe cancelar sin completar el ciclo.')
text = text.replace('El protocolo versión 12 conserva', 'El protocolo version 14 conserva')
text = text.replace('con protocolo 9; mezclar versiones', 'con protocolo 14; mezclar versiones')
path.write_text(text, encoding='utf-8')

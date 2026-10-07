# Modo integrador del proyecto

Por instruccion del usuario, **Automatico V2** es el modo completo para probar
la integracion del proyecto. Su firmware vigente esta en
`pruebas de automatico v2/ESP/ESP.ino` y
`pruebas de automatico v2/PORTENTA/PORTENTA.ino`.

Al modificar cualquier otro modo, revisar e incorporar en Automatico V2 las
mejoras aplicables en la misma tarea: deteccion, calibraciones, orientacion,
encoder, coordenadas, motores, catch, entrega, seguridad, OLED y registros.
Preferir funciones y parametros compartidos para impedir que diverjan.
Actualizar `pruebas de automatico v2/INTEGRACION_AUTOMATICO_V2.md` indicando
que se integro, que es exclusivo del modo de origen y como se verifico.

Los gestos de ensenanza (correccion por joystick, catch manual con X y etiqueta
humana EXITO/FALLO) no deben convertirse automaticamente en requisitos del
ciclo autonomo ni en excepciones a sus guardas. La caja del modelo 129 no mide
giro diagonal continuo: conservar el ajuste previo cuando no haya consenso.
Un ciclo ejecutado no demuestra agarre fisico.

Mantener identicas las dos copias del protocolo de la maqueta. Ante una nueva
orden o semantica incompatible, incrementar su version conservando los paquetes
de 32 bytes. Verificar ambos sketches y las regresiones pertinentes. No cargar
placas ni accionar hardware salvo que el usuario lo solicite.

`ESP/` y `PORTENTA/` de la raiz contienen otra version del firmware; no
sobrescribirlos como si fueran los sketches de prueba V2. Los documentos
historicos deben distinguirse de las instrucciones vigentes de la maqueta.

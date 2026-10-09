# Prueba funcional anterior: RS485 ASCII

Estos sketches se conservan tal como los entrego el usuario: ESP32
RX14/TX27/DE18, RS485 a 9600 baudios, 1000 mensajes Q/ACK con CRC8.
Son diagnostico de enlace, sin paquetes completos de Automatico V2.

El test final de migracion vigente esta en
`../RS485_FINAL_MIGRACION/README.md`. Conserva este cableado y usa el protocolo
17 del firmware completo (32 bytes de aplicacion, tramas COBS de 44 bytes,
115200 baudios). No mezclar los sketches de estas dos pruebas.

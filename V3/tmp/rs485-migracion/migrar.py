from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / 'pruebas de automatico v2'
NEW = ROOT / 'automatico v2 rs485'

def transport_names(s):
    s = s.replace('I2C_Pantalla', 'BUS_OLED_PROTEGIDO').replace('OLED_I2C_HZ', 'HZ_OLED_PROTEGIDO')
    s = s.replace('I2C', 'RS485').replace('i2c', 'rs485')
    return s.replace('BUS_OLED_PROTEGIDO', 'I2C_Pantalla').replace('HZ_OLED_PROTEGIDO', 'OLED_I2C_HZ')

def function(s, signature, replacement):
    start = s.index(signature)
    opening = s.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (s[end] == '{') - (s[end] == '}')
        end += 1
    return s[:start] + replacement.strip() + s[end:]

protocol = transport_names((OLD / 'ESP/ProtocoloI2C.h').read_text(encoding='utf-8-sig'))
protocol = protocol.replace('VERSION_PROTOCOLO = 16', 'VERSION_PROTOCOLO = 17')
protocol = protocol.replace('// - Los callbacks RS485 solo copian paquetes; la logica se ejecuta en loop().',
    '// - EnlaceRS485.h transporta los 32 bytes con COBS, sesion, solicitud y CRC16.\n'
    '// - Portenta coordina el par half duplex; ESP responde a cada solicitud valida.')
for board in ['ESP', 'PORTENTA']:
    (NEW / board / 'ProtocoloRS485.h').write_text(protocol, encoding='utf-8')
    (NEW / board / 'ProtocoloI2C.h').unlink(missing_ok=True)
(NEW / 'PORTENTA/EnlaceRS485.h').write_bytes((NEW / 'ESP/EnlaceRS485.h').read_bytes())

esp = transport_names((OLD / 'ESP/ESP.ino').read_text(encoding='utf-8-sig'))
esp = esp.replace('maestro del bus RS485 independiente de la pantalla', 'maestro del bus I2C independiente de la pantalla')
esp = esp.replace('#include "ProtocoloRS485.h"', '#include "ProtocoloRS485.h"\n#include "EnlaceRS485.h"')
esp = esp.replace('  const EstadoCamaraPublicado camara = copiarEstadoCamara();\n'
                  '  const EstadoEncoderCompartido encoder = copiarEstadoEncoder();\n'
                  '  PaqueteESPAPortenta paquete = {};',
                  '  const EstadoCamaraPublicado camara = copiarEstadoCamara();\n'
                  '  PaqueteESPAPortenta paquete = {};')
esp = esp.replace('constexpr int RS485_PORTENTA_SDA = 27;\nconstexpr int RS485_PORTENTA_SCL = 14;\nconstexpr uint32_t RS485_PORTENTA_HZ = 100000;',
    'constexpr int RS485_RX = 27;\nconstexpr int RS485_TX = 14;\nconstexpr int RS485_DIRECCION = 18;\n'
    'constexpr bool RS485_DIRECCION_MANUAL = true;\nHardwareSerial PuertoRS485(2);')
start = esp.index('struct RxPendiente {')
end = esp.index('PaquetePortentaAESP estadoPortenta', start)
esp = esp[:start] + '''EnlaceRS485::Receptor receptorRS485;
EnlaceRS485::Servidor servidorRS485;
EnlaceRS485::Mensaje solicitudRS485 = {};
bool respuestaRS485Pendiente = false;
uint32_t txRS485Ok = 0, txRS485Error = 0, solicitudesRS485Antiguas = 0;
uint32_t ultimaSesionPortentaRS485 = 0;

''' + esp[end:]
esp = function(esp, 'void requestEvent()', '')
esp = function(esp, 'void receiveEvent(int cantidadBytes)', '')
esp = function(esp, 'bool iniciarRS485Esclavo(bool reinicio)', '''
bool iniciarRS485Esclavo(bool reinicio) {
  if (rs485EsclavoIniciado) PuertoRS485.end();
  if (RS485_DIRECCION_MANUAL) {
    digitalWrite(RS485_DIRECCION, LOW);
    pinMode(RS485_DIRECCION, OUTPUT);
  }
  PuertoRS485.setRxBufferSize(256);
  PuertoRS485.begin(EnlaceRS485::BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  receptorRS485.reiniciar();
  respuestaRS485Pendiente = false;
  rs485EsclavoIniciado = true;
  ultimoIntentoRS485 = millis();
  inicioInstanciaRS485 = ultimoIntentoRS485;
  ultimaActividadRS485 = 0;
  if (reinicio) ++reiniciosRS485Esclavo;
  Serial.println(reinicio ? F("[RS485][RECUPERACION] UART2 reiniciada") :
    F("[BOOT] RS485 UART2 RX27/TX14 DE18; 115200 8N1; protocolo 17"));
  return true;
}''')
esp = function(esp, 'void procesarRecepcionRS485()', '''
void procesarRecepcionRS485() {
  EnlaceRS485::Mensaje mensaje;
  PaquetePortentaAESP paquete;
  bool hayNuevo = false;
  // Vaciar la UART antes de aplicar: si hubo atraso, conservar la solicitud
  // mas reciente, nunca reproducir una cola de ordenes de garra antiguas.
  while (PuertoRS485.available()) {
    if (!receptorRS485.agregar(static_cast<uint8_t>(PuertoRS485.read()), millis(), mensaje))
      continue;
    ++rxRS485Total;
    ultimoTamanoRecibido = sizeof(paquete);
    memcpy(&paquete, mensaje.datos, sizeof(paquete));
    if (!validarPaquete(paquete) || !paquetePortentaSemanticamenteValido(paquete)) {
      ++rxRS485ProtocoloIncorrecto;
      continue;
    }
    const EnlaceRS485::TipoSolicitud tipo = servidorRS485.aceptar(mensaje);
    if (tipo == EnlaceRS485::DESCARTADA) { ++solicitudesRS485Antiguas; continue; }
    solicitudRS485 = mensaje;
    respuestaRS485Pendiente = true;
    if (tipo == EnlaceRS485::DUPLICADA) continue;
    if (ultimaSesionPortentaRS485 && ultimaSesionPortentaRS485 != mensaje.sesion) {
      estadoPortentaValido = false;
      invalidarControlCamaraPorTimeout();
    }
    ultimaSesionPortentaRS485 = mensaje.sesion;
    estadoPortenta = paquete;
    hayNuevo = true;
    ++rxRS485Ok;
    ultimaActividadRS485 = millis();
  }
  receptorRS485.vencerFragmento(millis());
  if (hayNuevo) {
    estadoPortentaValido = true;
    ultimoEstadoPortenta = millis();
    actualizarEncoderDesdePortenta(estadoPortenta);
    publicarControlCamaraDesdePortenta(estadoPortenta);
  }
  if (estadoPortentaValido && millis() - ultimoEstadoPortenta > TIMEOUT_PORTENTA_MS) {
    estadoPortentaValido = false;
    invalidarControlCamaraPorTimeout();
    Serial.println(F("[RS485] Timeout de la Portenta"));
  }
}''')
pos = esp.index('// Geometria y homografia HUSKYLENS.')
pos = esp.rfind('// =============================================================================', 0, pos)
esp = esp[:pos] + '''void responderPortentaRS485() {
  if (!respuestaRS485Pendiente) return;
  // Snapshot actual despues de aplicar control y pinza; la correlacion de
  // transporte no sustituye los ACK de camara/objetivo ni confirma agarre.
  ultimaPublicacionRS485 = millis() - PERIODO_PUBLICACION_RS485_MS;
  prepararSnapshotRS485();
  EnlaceRS485::Mensaje respuesta = solicitudRS485;
  portENTER_CRITICAL(&txRS485Mux);
  memcpy(respuesta.datos, &paqueteTxSnapshot, sizeof(paqueteTxSnapshot));
  portEXIT_CRITICAL(&txRS485Mux);
  uint8_t trama[EnlaceRS485::TRAMA];
  const size_t n = EnlaceRS485::codificar(respuesta, trama);
  delayMicroseconds(EnlaceRS485::GIRO_BUS_US);
  if (RS485_DIRECCION_MANUAL) {
    digitalWrite(RS485_DIRECCION, HIGH);
    delayMicroseconds(50);
  }
  const size_t escritos = PuertoRS485.write(trama, n);
  PuertoRS485.flush();
  if (RS485_DIRECCION_MANUAL) {
    delayMicroseconds(50);
    digitalWrite(RS485_DIRECCION, LOW);
  }
  respuestaRS485Pendiente = false;
  ++solicitudesLecturaRS485;
  if (escritos == n) ++txRS485Ok; else ++txRS485Error;
}

''' + esp[pos:]
esp = esp.replace('  procesarPinzaAutomaticaV2();\n',
    '  procesarPinzaAutomaticaV2();\n  responderPortentaRS485();\n')
esp = esp.replace('    Serial.println(reiniciosRS485Esclavo);', '''    Serial.print(reiniciosRS485Esclavo);
    Serial.print(F(" txOK=")); Serial.print(txRS485Ok);
    Serial.print(F(" txError=")); Serial.print(txRS485Error);
    Serial.print(F(" crcTrama=")); Serial.print(receptorRS485.crcIncorrecto);
    Serial.print(F(" lenTrama=")); Serial.print(receptorRS485.longitudIncorrecta);
    Serial.print(F(" fragmentos=")); Serial.print(receptorRS485.fragmentos);
    Serial.print(F(" antigua=")); Serial.print(solicitudesRS485Antiguas);
    Serial.print(F(" rxBytes=")); Serial.println(receptorRS485.bytes);''')
esp = esp.replace('// Callbacks RS485: no validan, no calculan CRC, no imprimen y no usan la camara.',
    '// UART2 dedicada al enlace RS485. La recepcion se procesa en loop().')
esp = esp.replace('ni el otro RS485.', 'ni el enlace RS485.')
esp = esp.replace('// El bus de control se activa al final para que los callbacks nunca observen\n'
                  '  // perifericos a medio inicializar. Si el primer begin coincide con el\n'
                  '  // arranque de la Portenta, loop() vuelve a levantarlo automaticamente.',
    '// La UART de control se activa al final, con la base inicializada.')
(NEW / 'ESP/ESP.ino').write_text(esp, encoding='utf-8')

p = transport_names((OLD / 'PORTENTA/PORTENTA.ino').read_text(encoding='utf-8-sig'))
p = p.replace('// Cada paquete ocupa aproximadamente 3 ms a 100 kHz. La combinacion anterior',
    '// El enlace RS485 realiza solicitud+respuesta con correlacion y sin esperas. La combinacion anterior')
p = p.replace('// Un callback RS485 todavia podria responder aunque el loop de la ESP32 se',
    '// Una respuesta antigua no demuestra que el loop de la ESP32 este vivo. Si se')
p = p.replace('#include <Wire.h>', '#include "hal/trng_api.h"')
p = p.replace('#include "ProtocoloRS485.h"', '#include "ProtocoloRS485.h"\n#include "EnlaceRS485.h"')
p = p.replace('const unsigned long PERIODO_ESTADO_ESP_MS = 20UL;',
    'const unsigned long PERIODO_ESTADO_ESP_MS = 10UL; // Solicitud+respuesta cada 10 ms si el bus esta libre.')
pos = p.index('bool comunicacionRS485Habilitada = false;')
p = p[:pos] + '''EnlaceRS485::Receptor receptorRS485;
EnlaceRS485::Cliente clienteRS485;
bool envioRS485Urgente = false;
uint32_t respuestasRS485Ajenas = 0, timeoutsRS485 = 0;
uint32_t ultimaLatenciaRS485Ms = 0, maximaLatenciaRS485Ms = 0;

''' + p[pos:]
p = function(p, 'bool leerPaqueteESP32()', '''
bool leerPaqueteESP32() {
    EnlaceRS485::Mensaje mensaje;
    bool aceptado = false;
    while (comm_protocols.rs485.available()) {
        if (!receptorRS485.agregar(static_cast<uint8_t>(comm_protocols.rs485.read()), millis(), mensaje))
            continue;
        if (!clienteRS485.coincide(mensaje, millis())) { ++respuestasRS485Ajenas; continue; }
        PaqueteESPAPortenta recibido;
        memcpy(&recibido, mensaje.datos, sizeof(recibido));
        if (!validarPaquete(recibido) || !paqueteSemanticamenteValido(recibido)) {
            ++lecturasRS485Error;
            ++erroresRS485Semantica;
            if (fallosPaqueteConsecutivos < 255) ++fallosPaqueteConsecutivos;
            continue;
        }
        ultimaLatenciaRS485Ms = millis() - clienteRS485.enviadaMs;
        if (ultimaLatenciaRS485Ms > maximaLatenciaRS485Ms)
            maximaLatenciaRS485Ms = ultimaLatenciaRS485Ms;
        clienteRS485.confirmar();
        registrarPaqueteValido(recibido);
        ++lecturasRS485Ok;
        aceptado = true;
    }
    receptorRS485.vencerFragmento(millis());
    if (clienteRS485.vencer(millis())) {
        ++timeoutsRS485;
        ++lecturasRS485Error;
        if (fallosPaqueteConsecutivos < 255) ++fallosPaqueteConsecutivos;
    }
    return aceptado;
}''')
p = function(p, 'bool enviarPaquetePortenta()', '''
bool enviarPaquetePortenta() {
    if (!clienteRS485.sesion) return false;
    if (clienteRS485.pendiente) {
        // Las ordenes inmediatas de cierre/apertura no se pierden: el siguiente
        // envio construye el estado actual al quedar libre el bus.
        envioRS485Urgente = true;
        return false;
    }
    PaquetePortentaAESP salida = {};
    construirPaquetePortenta(salida);
    clienteRS485.iniciar(millis());
    EnlaceRS485::Mensaje mensaje = {};
    mensaje.sesion = clienteRS485.sesion;
    mensaje.solicitud = clienteRS485.solicitud;
    memcpy(mensaje.datos, &salida, sizeof(salida));
    uint8_t trama[EnlaceRS485::TRAMA];
    const size_t n = EnlaceRS485::codificar(mensaje, trama);
    comm_protocols.rs485.noReceive();
    comm_protocols.rs485.beginTransmission();
    const size_t escritos = comm_protocols.rs485.write(trama, n);
    comm_protocols.rs485.endTransmission();
    comm_protocols.rs485.receive();
    clienteRS485.enviadaMs = millis();
    envioRS485Urgente = false;
    if (escritos == n) {
        ++enviosRS485Ok;
        ultimoCodigoErrorEnvioRS485 = 0;
        return true; // TX local completo; la respuesta se valida por separado.
    }
    clienteRS485.confirmar();
    ++enviosRS485Error;
    ultimoCodigoErrorEnvioRS485 = 1;
    return false;
}''')
pos = p.index('void mantenerBusRS485MaestroRecuperable()')
p = p[:pos] + '''bool iniciarRS485Maestro() {
    if (!clienteRS485.sesion) {
        trng_t rng;
        trng_init(&rng);
        size_t cantidad = 0;
        uint32_t sesion = 0;
        const int error = trng_get_bytes(&rng, reinterpret_cast<uint8_t *>(&sesion), sizeof(sesion), &cantidad);
        trng_free(&rng);
        if (error != 0 || cantidad != sizeof(sesion) || !sesion) {
            Serial.println(F("[RS485][ERROR] No se obtuvo sesion aleatoria; enlace detenido"));
            return false;
        }
        clienteRS485.sesion = sesion;
    }
    comm_protocols.init();
    comm_protocols.rs485ModeRS232(false);
    comm_protocols.rs485FullDuplex(false);
    comm_protocols.rs485ABTerm(true);
    comm_protocols.rs485Enable(true);
    comm_protocols.rs485.begin(EnlaceRS485::BAUD, SERIAL_8N1, 50, 50);
    comm_protocols.rs485.receive();
    clienteRS485.confirmar();
    receptorRS485.reiniciar();
    Serial.print(F("[RS485] Half duplex TX P/TX N; 115200 8N1; protocolo 17; sesion="));
    Serial.println(clienteRS485.sesion);
    return true;
}

''' + p[pos:]
p = function(p, 'void mantenerBusRS485MaestroRecuperable()', '''
void mantenerBusRS485MaestroRecuperable() {
    if (!comunicacionRS485Habilitada || enlaceRS485Vigente()) return;
    const bool estadoSeguro = estadoGeneral == EST_BOOT_SAFE ||
        estadoGeneral == EST_WAIT_RS485 || estadoGeneral == EST_RS485_SETTLE ||
        estadoGeneral == EST_SYSTEM_ERROR;
    if (!estadoSeguro || motoresEnMovimiento() || movimientoPosicionadoActivo) return;
    const unsigned long ahora = millis();
    if (ahora - ultimoReinicioBusRS485Ms < 1000UL || clienteRS485.pendiente) return;
    ultimoReinicioBusRS485Ms = ahora;
    comm_protocols.rs485.end();
    iniciarRS485Maestro();
    ++reiniciosBusRS485Maestro;
    Serial.print(F("[RS485][RECUPERACION] Maestro reiniciado, intento="));
    Serial.println(reiniciosBusRS485Maestro);
}''')
p = p.replace('    Wire.begin();\n    Wire.setClock(100000);', '    iniciarRS485Maestro();')
start = p.index('    // if (comunicacionRS485Habilitada) {')
end = p.index('    if (!finalesCoherentes()', start)
p = p[:start] + '''    if (comunicacionRS485Habilitada) {
        // RX/timeout sin esperar. Solo una solicitud pendiente permite que
        // los dos extremos alternen DE sin colisiones en el par compartido.
        leerPaqueteESP32();
        if (!clienteRS485.pendiente &&
            (envioRS485Urgente || ahora - tAnteriorEstadoESP >= PERIODO_ESTADO_ESP_MS)) {
            if (enviarPaquetePortenta()) tAnteriorEstadoESP = millis();
        }
    }

''' + p[end:]
p = p.replace('[BOOT] RS485 maestro 0x40 a 100 kHz; retencion inicial 3000 ms',
    '[BOOT] RS485 integrado, 115200 8N1; retencion inicial 3000 ms')
p = p.replace('[BOOT] RS485: control 10 ms, telemetria encoder 20 ms (~45 % de bus)',
    '[BOOT] RS485: intercambio nominal 10 ms, una solicitud pendiente; OLED conserva I2C')
p = p.replace('        Serial.print(erroresRS485Longitud);',
    '        Serial.print(receptorRS485.longitudIncorrecta);')
p = p.replace('        Serial.print(erroresRS485CRC);',
    '        Serial.print(receptorRS485.crcIncorrecto);')
p = p.replace('        Serial.println(estadoGeneralWire());', '''        Serial.print(estadoGeneralWire());
        Serial.print(F(" timeout=")); Serial.print(timeoutsRS485);
        Serial.print(F(" ajena=")); Serial.print(respuestasRS485Ajenas);
        Serial.print(F(" fragmentos=")); Serial.print(receptorRS485.fragmentos);
        Serial.print(F(" rtt=")); Serial.print(ultimaLatenciaRS485Ms);
        Serial.print(F(" rttMax=")); Serial.print(maximaLatenciaRS485Ms);
        Serial.print(F(" rxBytes=")); Serial.println(receptorRS485.bytes);''')
(NEW / 'PORTENTA/PORTENTA.ino').write_text(p, encoding='utf-8')

registrar = transport_names((OLD / 'registrar_v2.ps1').read_text(encoding='utf-8-sig'))
registrar = registrar.replace('(RS485DBG|RESET)', '(RS485|RS485DBG|RESET)')
(NEW / 'registrar_v2.ps1').write_text(registrar, encoding='utf-8-sig')
assert not re.search(r'\bWire\.\w+\s*\(', esp + p), 'Quedo una llamada del enlace I2C antiguo'
assert 'I2C_Pantalla.begin(' in esp and 'OLED_I2C_HZ = 100000' in esp
print('Migracion escrita en:', NEW)

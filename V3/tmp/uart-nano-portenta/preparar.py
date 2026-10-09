from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
origen = root / 'tests' / 'RS485_NANO_PORTENTA'
destino = root / 'tests' / 'UART_NANO_PORTENTA'
if destino.exists():
    raise SystemExit('No sobrescribir automaticamente una prueba existente.')
for placa in ('NANO', 'PORTENTA'):
    (destino / placa).mkdir(parents=True)

proto = (origen / 'NANO' / 'PruebaRS485.h').read_text(encoding='utf-8')
proto = proto.replace('PruebaRS485', 'PruebaUART').replace('VERSION = 3;', 'VERSION = 1;')
proto = proto.replace('"R485"', '"UTTL"')
proto = proto.replace('// Protocolo exclusivo de esta prueba.', '// Diagnostico UART TTL, firma UTTL. Protocolo exclusivo de esta prueba.')

nano = (origen / 'NANO' / 'NANO.ino').read_text(encoding='utf-8')
nano = nano.replace('PruebaRS485', 'PruebaUART').replace('[RS485', '[UART')
nano = nano.replace('RS485_RX', 'UART_RX').replace('RS485_TX', 'UART_TX')
nano = nano.replace('// Nano clasico + MAX485: solo comunicacion:', '// Nano clasico + UART TTL: solo comunicacion:')
nano = nano.replace('constexpr int RS485_DIRECCION = 4;\n// true: DE y /RE unidos a D4. false: modulo de direccion automatica.\nconstexpr bool DIRECCION_MANUAL = true;\n', '')
nano = nano.replace('  // La Portenta mantiene DE 500 us tras TX. Dar margen antes de responder.', '  // Margen antes de responder para SoftwareSerial del Nano.')
nano = nano.replace('  if (DIRECCION_MANUAL) {\n    digitalWrite(RS485_DIRECCION, HIGH);\n    delayMicroseconds(50);\n  }\n', '')
nano = nano.replace('  if (DIRECCION_MANUAL) {\n    delayMicroseconds(50);\n    digitalWrite(RS485_DIRECCION, LOW);\n  }\n', '')
nano = nano.replace('  if (DIRECCION_MANUAL) {\n    digitalWrite(RS485_DIRECCION, LOW);\n    pinMode(RS485_DIRECCION, OUTPUT);\n  }\n', '')
nano = nano.replace('BIDIRECCIONAL v3', 'BIDIRECCIONAL v1')
nano = nano.replace('  Serial.println(DIRECCION_MANUAL ? "[UART NANO] DE+/RE=D4" :\n                                 "[UART NANO] Direccion automatica");', '  Serial.println(F("[UART NANO] TTL con adaptacion de nivel; D4/D5 sin uso."));')
assert 'DIRECCION_MANUAL' not in nano and 'RS485_DIRECCION' not in nano

portenta = (origen / 'PORTENTA' / 'PORTENTA.ino').read_text(encoding='utf-8')
portenta = portenta.replace('PruebaRS485', 'PruebaUART').replace('[RS485', '[UART')
portenta = portenta.replace('// Solo prueba RS485 integrada de Portenta H7 + Machine Control.', '// UART TTL de Machine Control: TX TP78/PA0 y RX TP83/PI9. NO usar TX P/TX N.')
portenta = portenta.replace('constexpr bool TERMINACION_120_OHM = true;', 'arduino::UART& enlace = comm_protocols._UART4_;')
portenta = portenta.replace('  // Permitir que el Nano libere DE antes de responder por el mismo par.', '  // Margen para SoftwareSerial del Nano; UART tiene TX/RX separados.')
portenta = portenta.replace('  comm_protocols.rs485.noReceive();\n  comm_protocols.rs485.beginTransmission();\n  comm_protocols.rs485.write(paquete, sizeof(paquete));\n  comm_protocols.rs485.endTransmission();\n  comm_protocols.rs485.receive();', '  enlace.write(paquete, sizeof(paquete));\n  enlace.flush();')
viejo = '''  comm_protocols.init();
  comm_protocols.rs485ModeRS232(false);
  comm_protocols.rs485FullDuplex(false);
  comm_protocols.rs485ABTerm(TERMINACION_120_OHM);
  comm_protocols.rs485Enable(true);
  comm_protocols.rs485.begin(PruebaUART::BAUD, SERIAL_8N1, 50, 500);
  comm_protocols.rs485.receive();'''
nuevo = '''  // Modo RS485, /RE=HIGH, DE=LOW y SHDN=LOW: SP335 sin manejar RX/TX.
  // No llamar comm_protocols.init(): toca tambien las lineas UART como GPIO.
  digitalWrite(PinNameToIndex(PG_9), LOW);
  pinMode(PinNameToIndex(PG_9), OUTPUT);
  comm_protocols.rs485Enable(false);
  digitalWrite(PinNameToIndex(PA_10), HIGH);
  pinMode(PinNameToIndex(PA_10), OUTPUT);
  comm_protocols.rs485ModeRS232(false);
  digitalWrite(PinNameToIndex(PI_10), HIGH);
  pinMode(PinNameToIndex(PI_10), OUTPUT);
  digitalWrite(PinNameToIndex(PI_13), LOW);
  pinMode(PinNameToIndex(PI_13), OUTPUT);
  delayMicroseconds(5);
  enlace.begin(PruebaUART::BAUD, SERIAL_8N1);'''
assert viejo in portenta
portenta = portenta.replace(viejo, nuevo)
portenta = portenta.replace('comm_protocols.rs485.available()', 'enlace.available()')
portenta = portenta.replace('comm_protocols.rs485.read()', 'enlace.read()')
portenta = portenta.replace('BIDIRECCIONAL v3; TX P/TX N;', 'BIDIRECCIONAL v1; TX=TP78 RX=TP83;')
assert 'rs485Enable(true)' not in portenta
assert 'comm_protocols.rs485.' not in portenta
for placa, sketch in [('NANO', nano), ('PORTENTA', portenta)]:
    (destino / placa / (placa + '.ino')).write_text(sketch, encoding='utf-8')
    (destino / placa / 'PruebaUART.h').write_text(proto, encoding='utf-8')
print('Creada UART TTL Nano/Portenta: TX TP78, RX TP83, UTTL v1, 9600, monitores 115200.')

from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
origen = root / 'tests' / 'RS485_COMUNICACION'
destino = root / 'tests' / 'RS485_NANO_PORTENTA'
if destino.exists():
    raise SystemExit('La carpeta de destino ya existe; no sobrescribir automaticamente.')
(destino / 'NANO').mkdir(parents=True)
(destino / 'PORTENTA').mkdir()

protocolo = (origen / 'ESP' / 'PruebaRS485.h').read_text(encoding='utf-8')
assert 'constexpr uint8_t VERSION = 2;' in protocolo
assert 'constexpr uint32_t BAUD = 115200;' in protocolo
protocolo = protocolo.replace('VERSION = 2;', 'VERSION = 3;')
protocolo = protocolo.replace('BAUD = 115200;', 'BAUD = 9600;')
protocolo = protocolo.replace('_ESP', '_NANO')

nano = (origen / 'ESP' / 'ESP.ino').read_text(encoding='utf-8')
nano = nano.replace('ESP', 'NANO').replace('_esp', '_nano').replace('esp_ms=', 'nano_ms=')
nano = nano.replace('BIDIRECCIONAL v2', 'BIDIRECCIONAL v3').replace('la NANO', 'el Nano')
nano = nano.replace('#include <Arduino.h>', '#include <Arduino.h>\n#include <SoftwareSerial.h>\n\n#if !defined(__AVR_ATmega328P__)\n#error "Esta prueba requiere Nano clasico ATmega328P de 5 V."\n#endif')
nano = nano.replace('RS485_RX = 27;', 'RS485_RX = 2;')
nano = nano.replace('RS485_TX = 14;', 'RS485_TX = 3;')
nano = nano.replace('RS485_DIRECCION = 18;', 'RS485_DIRECCION = 4;')
nano = nano.replace('GPIO18', 'D4')
nano = nano.replace('HardwareSerial enlace(2);', 'SoftwareSerial enlace(RS485_RX, RS485_TX);')
nano = nano.replace('enlace.begin(PruebaRS485::BAUD, SERIAL_8N1, RS485_RX, RS485_TX);', 'enlace.begin(PruebaRS485::BAUD);\n  enlace.listen();')
nano = nano.replace('  enlace.flush(); // Esperar el ultimo bit antes de liberar el bus.', '  // SoftwareSerial::write transmite sin buffer y espera el bit de parada.')
nano = nano.replace('RX=27 TX=14; 115200; 32 bytes', 'RX=D2 TX=D3; bus=9600; monitor=115200; 32 bytes')
nano = re.sub(r'Serial\.(print|println)\(("(?:[^"\\]|\\.)*")\);', r'Serial.\1(F(\2));', nano)
nano = nano.replace('// Solo prueba de comunicacion:', '// Nano clasico + MAX485: solo comunicacion:')
assert 'HardwareSerial' not in nano
assert 'enlace.flush()' not in nano
assert 'RS485_RX = 2;' in nano and 'RS485_TX = 3;' in nano
assert 'RS485_DIRECCION = 4;' in nano

portenta = (origen / 'PORTENTA' / 'PORTENTA.ino').read_text(encoding='utf-8')
portenta = portenta.replace('ESP', 'NANO').replace('_esp', '_nano').replace('esp_ms=', 'nano_ms=')
portenta = portenta.replace('NANOERA', 'ESPERA').replace('BIDIRECCIONAL v2', 'BIDIRECCIONAL v3').replace('la NANO', 'el Nano')
portenta = portenta.replace('115200; 32 bytes', 'bus=9600; monitor=115200; 32 bytes')
assert 'Serial.begin(115200);' in portenta
assert 'Serial.begin(115200);' in nano
for placa, sketch in [('NANO', nano), ('PORTENTA', portenta)]:
    (destino / placa / (placa + '.ino')).write_text(sketch, encoding='utf-8')
    (destino / placa / 'PruebaRS485.h').write_text(protocolo, encoding='utf-8')
print('Creada prueba bidireccional Nano/Portenta: protocolo 3, bus 9600, monitores 115200.')

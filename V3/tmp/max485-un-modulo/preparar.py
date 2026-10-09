from pathlib import Path
import re

root = Path.cwd()
src = root / 'tests/RS485_NANO_PORTENTA'
dst = root / 'tests/MAX485_UN_MODULO_NANO_UNO'
for board in ('NANO', 'UNO'):
    (dst / board).mkdir(parents=True, exist_ok=True)

def roles(s):
    return (s.replace('PORTENTA', 'UNO').replace('Portenta', 'Uno')
            .replace('portenta', 'uno'))

header = roles((src / 'NANO/PruebaRS485.h').read_text(encoding='utf-8'))
header = header.replace('VERSION = 3', 'VERSION = 4')
header = header.replace('"R485"', '"M485"')
header = header.replace('PruebaRS485', 'PruebaMAX485')
header = header.replace('// Protocolo exclusivo de esta prueba. No contiene ordenes de la maqueta.',
    '// Diagnostico local con un MAX485: Nano->DI->A/B->RO->Uno; retorno UART directo.\n'
    '// Version 4 y firma M485: incompatible con pruebas RS485 anteriores; 32 bytes.')
for board in ('NANO', 'UNO'):
    (dst / board / 'PruebaMAX485.h').write_text(header, encoding='utf-8')

nano = roles((src / 'NANO/NANO.ino').read_text(encoding='utf-8'))
nano = nano.replace('// Nano clasico + MAX485: solo comunicacion: sin servos, camara, OLED, control ni motores.',
    '// Nano clasico 5 V: TX D3->DI del unico MAX485; RX D2<-TX D3 del Uno.\n'
    '// MAX485 DE a 5 V y /RE a GND, separados. A/B sin cables externos. Solo diagnostico.')
nano = nano.replace('PruebaRS485', 'PruebaMAX485')
nano = nano.replace('constexpr int RS485_DIRECCION = 4;\n// true: DE y /RE unidos a D4. false: modulo de direccion automatica.\nconstexpr bool DIRECCION_MANUAL = true;\n', '')
start = nano.index('  // La Uno mantiene')
end = nano.index('\n}\n', start)
nano = nano[:start] + '''  // Margen entre turnos para que el receptor procese la trama anterior.
  delayMicroseconds(2000);
  // SoftwareSerial::write espera el bit de parada. DE y /RE son fijos.
  enlace.write(paquete, sizeof(paquete));''' + nano[end:]
start = nano.index('  if (DIRECCION_MANUAL) {', nano.index('void setup()'))
end = nano.index('  enlace.begin', start)
nano = nano[:start] + nano[end:]
start = nano.index('  Serial.println(F("[RS485 NANO] Prueba')
end = nano.index('\n}', start)
nano = nano[:start] + '''  Serial.println(F("[MAX485 NANO] UN MODULO v4; bus=9600; monitor=115200; 32 bytes"));
  Serial.println(F("[MAX485 NANO] TX D3->DI; RX D2<-Uno D3; DE=5V /RE=GND"));''' + nano[end:]
nano = nano.replace('[RS485 NANO]', '[MAX485 NANO]').replace('de el Nano', 'del Nano')
(dst / 'NANO/NANO.ino').write_text(nano, encoding='utf-8')

uno = roles((src / 'PORTENTA/PORTENTA.ino').read_text(encoding='utf-8'))
start = uno.index('PruebaRS485::Receptor')
uno = '''// Uno R3 5 V: RX D2<-RO del unico MAX485; TX D3->Nano D2 directo.
// Solo diagnostico local: no es un enlace RS485 entre dos extremos.
#include <Arduino.h>
#include <SoftwareSerial.h>
#if !defined(__AVR_ATmega328P__)
#error "Esta prueba requiere Uno R3 ATmega328P de 5 V."
#endif
#include "PruebaMAX485.h"
constexpr int RS485_RX = 2;
constexpr int RS485_TX = 3;
SoftwareSerial enlace(RS485_RX, RS485_TX);

''' + uno[start:]
uno = uno.replace('PruebaRS485', 'PruebaMAX485')
start = uno.index('  // Permitir')
end = uno.index('\n}\n', start)
uno = uno[:start] + '''  delayMicroseconds(2000);
  // Retorno directo al Nano, sin MAX485; escritura completa y bloqueante.
  enlace.write(paquete, sizeof(paquete));''' + uno[end:]
start = uno.index('  const uint32_t inicioUSB')
end = uno.index('  ultimoEnvio = millis();', start)
uno = uno[:start] + '''  enlace.begin(PruebaMAX485::BAUD);
  enlace.listen();
''' + uno[end:]
uno = uno.replace('comm_protocols.rs485.available()', 'enlace.available()')
uno = uno.replace('comm_protocols.rs485.read()', 'enlace.read()')
uno = uno.replace('[RS485 UNO]', '[MAX485 UNO]')
uno = uno.replace('Prueba BIDIRECCIONAL v3; TX P/TX N; bus=9600; monitor=115200; 32 bytes',
    'UN MODULO v4; RX D2<-RO; TX D3->Nano D2; bus=9600; monitor=115200')
uno = re.sub(r'(Serial\.(?:print|println)\() (?!x)', r'\1', uno)  # Sin cambios de contenido.
uno = re.sub(r'(Serial\.(?:print|println)\()("[^"\n]*")(\))', r'\1F(\2)\3', uno)
(dst / 'UNO/UNO.ino').write_text(uno, encoding='utf-8')

test = (root / 'tests/rs485_nano_portenta_test.cpp').read_text(encoding='utf-8')
test = roles(test).replace('RS485_NANO_UNO', 'MAX485_UN_MODULO_NANO_UNO')
test = test.replace('PruebaRS485', 'PruebaMAX485')
test = test.replace('VERSION == 3', 'VERSION == 4').replace('diagnostico 3', 'diagnostico 4')
test = test.replace('bool driverNano = false, driverUno = false;\nbool receptorUno = false, escuchaNano = false, conectado = true;',
    'bool escuchaUno = false, escuchaNano = false, conectado = true;')
start = test.index('void pinMode(')
end = test.index('struct Consola', start)
test = test[:start] + test[end:]
start = test.index('  assert(desdeNano ?')
end = test.index('  std::array', start)
test = test[:start] + '  assert(desdeNano ? escuchaUno : escuchaNano);\n' + test[end:]
start = test.index('class SoftwareSerial')
end = test.index('#define __AVR', start)
test = test[:start] + '''int instanciasUART = 0;
class SoftwareSerial {
  bool esNano;
 public:
  SoftwareSerial(int rx, int tx) : esNano(instanciasUART++ == 0) {
    assert(rx == 2 && tx == 3);
  }
  void begin(uint32_t baud) { assert(baud == 9600); }
  void listen() { (esNano ? escuchaNano : escuchaUno) = true; }
  size_t write(const uint8_t* datos, size_t n) { return transmitir(esNano, datos, n); }
  int available() const { return static_cast<int>((esNano ? haciaNano : haciaUno).size()); }
  int read() { return leer(esNano ? haciaNano : haciaUno); }
};

''' + test[end:]
test = test.replace('  assert(!driverNano && !driverUno && receptorUno);',
    '  assert(escuchaNano && escuchaUno);')
test = test.replace('versionAnterior[4] = 2', 'versionAnterior[4] = 3')
test = test.replace('reconexion, version incompatible y control half duplex.',
    'reconexion y version incompatible. Modelo logico, sin validacion electrica.')
(root / 'tests/max485_un_modulo_nano_uno_test.cpp').write_text(test, encoding='utf-8')
print('Creados dos sketches, protocolo identico y regresion en MAX485_UN_MODULO_NANO_UNO.')

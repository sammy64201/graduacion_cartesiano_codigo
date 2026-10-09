from pathlib import Path

root = Path(__file__).resolve().parents[2]
texto = (root / 'tests/rs485_nano_portenta_test.cpp').read_text(encoding='utf-8')
texto = texto.replace('RS485_NANO_PORTENTA', 'UART_NANO_PORTENTA')
texto = texto.replace('PruebaRS485.h', 'PruebaUART.h').replace('PruebaRS485', 'PruebaUART')
texto = texto.replace('VERSION == 3', 'VERSION == 1').replace('Protocolo diagnostico 3', 'Protocolo diagnostico UART 1')
texto = texto.replace('bool driverNano = false, driverPortenta = false;', '''bool driverNano = false, driverPortenta = false;
constexpr int PG_9 = 209, PA_10 = 210, PI_10 = 310, PI_13 = 313;
int PinNameToIndex(int pin) { return pin; }
int nivelSHDN = HIGH, nivelModo = LOW, nivelRE = LOW, nivelDE = HIGH;''')
viejo = '''void pinMode(int pin, int modo) { assert(pin == 4 && modo == OUTPUT); }
void digitalWrite(int pin, int nivel) {
  assert(pin == 4 && (nivel == HIGH || nivel == LOW));
  driverNano = nivel == HIGH;
}'''
nuevo = '''void pinMode(int pin, int modo) {
  assert((pin == PG_9 || pin == PA_10 || pin == PI_10 || pin == PI_13) && modo == OUTPUT);
}
void digitalWrite(int pin, int nivel) {
  assert(nivel == HIGH || nivel == LOW);
  if (pin == PG_9) nivelSHDN = nivel;
  else if (pin == PA_10) nivelModo = nivel;
  else if (pin == PI_10) nivelRE = nivel;
  else if (pin == PI_13) nivelDE = nivel;
  else assert(false);
}'''
assert viejo in texto
texto = texto.replace(viejo, nuevo)
viejo = '''  assert(desdeNano ? (driverNano && !driverPortenta && receptorPortenta) :
                     (driverPortenta && !driverNano && escuchaNano));'''
nuevo = '''  assert(escuchaNano && receptorPortenta);
  assert(nivelSHDN == LOW && nivelModo == HIGH && nivelRE == HIGH && nivelDE == LOW);'''
assert viejo in texto
texto = texto.replace(viejo, nuevo)
texto = texto.replace('''  void begin(uint32_t baud, int config, int previo, int posterior) {
    assert(baud == 9600 && config == SERIAL_8N1 && previo == 50 && posterior == 500);
  }''', '''  void begin(uint32_t baud, int config) {
    assert(baud == 9600 && config == SERIAL_8N1);
    assert(nivelSHDN == LOW && nivelModo == HIGH && nivelRE == HIGH && nivelDE == LOW);
    receptorPortenta = true;
  }
  void flush() {}''')
texto = texto.replace('  UART rs485;', '  UART _UART4_;')
texto = texto.replace('  void init() {}', '  void init() { assert(false); } // No tocar RX TTL como GPIO.')
texto = texto.replace('  void rs485Enable(bool activo) { assert(activo); }', '  void rs485Enable(bool activo) { assert(!activo && nivelSHDN == LOW); }')
texto = texto.replace('#define __AVR_ATmega328P__ 1', 'namespace arduino { using UART = machinecontrol::UART; }\n#define __AVR_ATmega328P__ 1')
texto = texto.replace('control half duplex.', 'deshabilitacion SP335 y UART TTL.')
texto = texto.replace('// No reproduce temporizacion', '// No reproduce temporizacion')
assert 'void begin(uint32_t baud, int config, int previo' not in texto
assert 'UART _UART4_;' in texto
(root / 'tests/uart_nano_portenta_test.cpp').write_text(texto, encoding='utf-8')
print('Regresion UART TTL creada a partir de las maquinas de estados reales.')

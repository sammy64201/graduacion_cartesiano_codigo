// Ejecuta las maquinas de estados de los DOS sketches reales con UART simulada.
// No reproduce temporizacion de interrupciones, niveles electricos ni hardware.
#include <array>
#include <deque>
#include <cassert>
#include <iostream>
#include <string>
#include "UART_NANO_PORTENTA/NANO/PruebaUART.h"
#include "rs485_nano_portenta_mocks/Arduino.h"

static_assert(PruebaUART::TAMANO == 32, "Paquetes de 32 bytes");
static_assert(PruebaUART::VERSION == 1, "Protocolo diagnostico UART 1");
static_assert(PruebaUART::BAUD == 9600, "Ambos UART a 9600");
uint64_t relojUS = 0;
bool driverNano = false, driverPortenta = false;
constexpr int PG_9 = 209, PA_10 = 210, PI_10 = 310, PI_13 = 313;
int PinNameToIndex(int pin) { return pin; }
int nivelSHDN = HIGH, nivelModo = LOW, nivelRE = LOW, nivelDE = HIGH;
bool receptorPortenta = false, escuchaNano = false, conectado = true;
std::deque<uint8_t> haciaNano, haciaPortenta;
enum class Fallo { NINGUNO, CRC_ACK_NANO, ACK_NANO_AJENO, PERDER_ACK_NANO,
                   PERDER_DATOS_NANO, PERDER_ACK_PORTENTA };
Fallo siguienteFallo = Fallo::NINGUNO;

uint32_t millis() { return static_cast<uint32_t>(relojUS / 1000); }
void delay(uint32_t ms) { relojUS += uint64_t(ms) * 1000; }
void delayMicroseconds(uint32_t us) { relojUS += us; }
void pinMode(int pin, int modo) {
  assert((pin == PG_9 || pin == PA_10 || pin == PI_10 || pin == PI_13) && modo == OUTPUT);
}
void digitalWrite(int pin, int nivel) {
  assert(nivel == HIGH || nivel == LOW);
  if (pin == PG_9) nivelSHDN = nivel;
  else if (pin == PA_10) nivelModo = nivel;
  else if (pin == PI_10) nivelRE = nivel;
  else if (pin == PI_13) nivelDE = nivel;
  else assert(false);
}
struct Consola {
  void begin(uint32_t baud) { assert(baud == 115200); }
  explicit operator bool() const { return true; }
  template<class T> void print(const T&) {}
  template<class T> void println(const T&) {}
} Serial;

size_t transmitir(bool desdeNano, const uint8_t* datos, size_t longitud) {
  assert(longitud == 32);
  assert(escuchaNano && receptorPortenta);
  assert(nivelSHDN == LOW && nivelModo == HIGH && nivelRE == HIGH && nivelDE == LOW);
  std::array<uint8_t, 32> trama{};
  for (size_t i = 0; i < longitud; ++i) trama[i] = datos[i];
  bool perder = !conectado;
  if (desdeNano && trama[5] == PruebaUART::ACK_NANO) {
    if (siguienteFallo == Fallo::CRC_ACK_NANO) {
      trama[10] ^= 0x40;
      siguienteFallo = Fallo::NINGUNO;
    } else if (siguienteFallo == Fallo::ACK_NANO_AJENO) {
      PruebaUART::crear(PruebaUART::ACK_NANO,
                        PruebaUART::secuencia(datos) + 1,
                        PruebaUART::valor(datos), PruebaUART::turno(datos), trama.data());
      siguienteFallo = Fallo::NINGUNO;
    } else if (siguienteFallo == Fallo::PERDER_ACK_NANO) {
      perder = true;
      siguienteFallo = Fallo::NINGUNO;
    }
  } else if (desdeNano && trama[5] == PruebaUART::DATOS_NANO &&
             siguienteFallo == Fallo::PERDER_DATOS_NANO) {
    perder = true;
    siguienteFallo = Fallo::NINGUNO;
  } else if (!desdeNano && trama[5] == PruebaUART::ACK_PORTENTA &&
             siguienteFallo == Fallo::PERDER_ACK_PORTENTA) {
    perder = true;
    siguienteFallo = Fallo::NINGUNO;
  }
  // Escritura completa y bloqueante, 32 bytes a 9600 8N1.
  relojUS += (uint64_t(longitud) * 10 * 1000000 + 9599) / 9600;
  if (!perder) {
    auto& destino = desdeNano ? haciaPortenta : haciaNano;
    for (uint8_t byte : trama) destino.push_back(byte);
  }
  return longitud;
}
int leer(std::deque<uint8_t>& cola) {
  if (cola.empty()) return -1;
  const int byte = cola.front();
  cola.pop_front();
  return byte;
}

class SoftwareSerial {
 public:
  SoftwareSerial(int rx, int tx) { assert(rx == 2 && tx == 3); }
  void begin(uint32_t baud) { assert(baud == 9600); }
  void listen() { escuchaNano = true; }
  size_t write(const uint8_t* datos, size_t n) { return transmitir(true, datos, n); }
  int available() const { return static_cast<int>(haciaNano.size()); }
  int read() { return leer(haciaNano); }
};
namespace machinecontrol {
struct UART {
  void begin(uint32_t baud, int config) {
    assert(baud == 9600 && config == SERIAL_8N1);
    assert(nivelSHDN == LOW && nivelModo == HIGH && nivelRE == HIGH && nivelDE == LOW);
    receptorPortenta = true;
  }
  void flush() {}
  void noReceive() { receptorPortenta = false; }
  void receive() { receptorPortenta = true; }
  void beginTransmission() { driverPortenta = true; }
  void endTransmission() { driverPortenta = false; }
  size_t write(const uint8_t* datos, size_t n) { return transmitir(false, datos, n); }
  int available() const { return static_cast<int>(haciaPortenta.size()); }
  int read() { return leer(haciaPortenta); }
};
struct Protocolos {
  UART _UART4_;
  void init() { assert(false); } // No tocar RX TTL como GPIO.
  void rs485ModeRS232(bool activo) { assert(!activo); }
  void rs485FullDuplex(bool activo) { assert(!activo); }
  void rs485ABTerm(bool activo) { assert(activo); }
  void rs485Enable(bool activo) { assert(!activo && nivelSHDN == LOW); }
} comm_protocols;
}

namespace arduino { using UART = machinecontrol::UART; }
#define __AVR_ATmega328P__ 1
namespace nano {
#include "UART_NANO_PORTENTA/NANO/NANO.ino"
}
namespace portenta {
#include "UART_NANO_PORTENTA/PORTENTA/PORTENTA.ino"
}

template<class Condicion> void esperar(Condicion condicion, uint32_t limiteMS = 5000) {
  const uint32_t inicio = millis();
  while (!condicion() && millis() - inicio < limiteMS) {
    portenta::loop();
    nano::loop();
    delay(2);
  }
  assert(condicion());
}
void cicloCompletoNuevo() {
  const uint32_t previo = nano::confirmadosNANO;
  esperar([&] { return nano::confirmadosNANO > previo; });
  assert(portenta::fase == portenta::REPOSO && !nano::esperandoACK);
  assert(!driverNano && !driverPortenta && receptorPortenta);
}

int main() {
  nano::setup();
  portenta::setup();
  esperar([] { return nano::confirmadosNANO >= 100; }, 150000);
  assert(nano::confirmadosNANO == 100 && nano::enviadosNANO == 100);
  assert(nano::recibidosPortenta == 100 && portenta::recibidosNANO == 100);
  assert(portenta::confirmadosPortenta == 100 && portenta::enviados == 100);
  assert(nano::timeoutsNANO == 0 && portenta::timeoutsPortenta == 0);
  assert(portenta::timeoutsTurnoNANO == 0 && nano::inesperados == 0);
  assert(nano::receptor.invalidos == 0 && portenta::receptor.invalidos == 0);

  for (Fallo fallo : {Fallo::CRC_ACK_NANO, Fallo::ACK_NANO_AJENO, Fallo::PERDER_ACK_NANO}) {
    const auto previos = portenta::timeoutsPortenta;
    const auto invalidos = portenta::receptor.invalidos;
    const auto inesperados = portenta::inesperados;
    siguienteFallo = fallo;
    esperar([&] { return portenta::timeoutsPortenta > previos; });
    assert(siguienteFallo == Fallo::NINGUNO);
    if (fallo == Fallo::CRC_ACK_NANO) assert(portenta::receptor.invalidos > invalidos);
    if (fallo == Fallo::ACK_NANO_AJENO) assert(portenta::inesperados > inesperados);
    cicloCompletoNuevo();
  }
  auto previoTurno = portenta::timeoutsTurnoNANO;
  siguienteFallo = Fallo::PERDER_DATOS_NANO;
  esperar([&] { return portenta::timeoutsTurnoNANO > previoTurno; });
  cicloCompletoNuevo();

  auto previoNano = nano::timeoutsNANO;
  siguienteFallo = Fallo::PERDER_ACK_PORTENTA;
  esperar([&] { return nano::timeoutsNANO > previoNano; });
  cicloCompletoNuevo();

  conectado = false;
  auto previoPortenta = portenta::timeoutsPortenta;
  esperar([&] { return portenta::timeoutsPortenta > previoPortenta; });
  conectado = true;
  cicloCompletoNuevo();

  uint8_t versionAnterior[32];
  PruebaUART::crear(PruebaUART::DATOS_PORTENTA, 1, 2, 1, versionAnterior);
  versionAnterior[4] = 2;
  const uint16_t crc = PruebaUART::crc16(versionAnterior, 30);
  versionAnterior[30] = crc & 0xFF;
  versionAnterior[31] = crc >> 8;
  assert(!PruebaUART::valido(versionAnterior));
  std::cout << "PASS: 100 ciclos bidireccionales, CRC/ACK ajeno/perdidas, "
               "reconexion, version incompatible y deshabilitacion SP335 y UART TTL.\n";
}

// Ejecuta las maquinas de estados de los DOS sketches reales con UART simulada.
// No reproduce temporizacion de interrupciones, niveles electricos ni hardware.
#include <array>
#include <deque>
#include <cassert>
#include <iostream>
#include <string>
#include "MAX485_UN_MODULO_NANO_UNO/NANO/PruebaMAX485.h"
#include "rs485_nano_portenta_mocks/Arduino.h"

static_assert(PruebaMAX485::TAMANO == 32, "Paquetes de 32 bytes");
static_assert(PruebaMAX485::VERSION == 4, "Protocolo diagnostico 4");
static_assert(PruebaMAX485::BAUD == 9600, "Ambos UART a 9600");
uint64_t relojUS = 0;
bool escuchaUno = false, escuchaNano = false, conectado = true;
std::deque<uint8_t> haciaNano, haciaUno;
enum class Fallo { NINGUNO, CRC_ACK_NANO, ACK_NANO_AJENO, PERDER_ACK_NANO,
                   PERDER_DATOS_NANO, PERDER_ACK_UNO };
Fallo siguienteFallo = Fallo::NINGUNO;

uint32_t millis() { return static_cast<uint32_t>(relojUS / 1000); }
void delay(uint32_t ms) { relojUS += uint64_t(ms) * 1000; }
void delayMicroseconds(uint32_t us) { relojUS += us; }
struct Consola {
  void begin(uint32_t baud) { assert(baud == 115200); }
  explicit operator bool() const { return true; }
  template<class T> void print(const T&) {}
  template<class T> void println(const T&) {}
} Serial;

size_t transmitir(bool desdeNano, const uint8_t* datos, size_t longitud) {
  assert(longitud == 32);
  assert(desdeNano ? escuchaUno : escuchaNano);
  std::array<uint8_t, 32> trama{};
  for (size_t i = 0; i < longitud; ++i) trama[i] = datos[i];
  bool perder = !conectado;
  if (desdeNano && trama[5] == PruebaMAX485::ACK_NANO) {
    if (siguienteFallo == Fallo::CRC_ACK_NANO) {
      trama[10] ^= 0x40;
      siguienteFallo = Fallo::NINGUNO;
    } else if (siguienteFallo == Fallo::ACK_NANO_AJENO) {
      PruebaMAX485::crear(PruebaMAX485::ACK_NANO,
                        PruebaMAX485::secuencia(datos) + 1,
                        PruebaMAX485::valor(datos), PruebaMAX485::turno(datos), trama.data());
      siguienteFallo = Fallo::NINGUNO;
    } else if (siguienteFallo == Fallo::PERDER_ACK_NANO) {
      perder = true;
      siguienteFallo = Fallo::NINGUNO;
    }
  } else if (desdeNano && trama[5] == PruebaMAX485::DATOS_NANO &&
             siguienteFallo == Fallo::PERDER_DATOS_NANO) {
    perder = true;
    siguienteFallo = Fallo::NINGUNO;
  } else if (!desdeNano && trama[5] == PruebaMAX485::ACK_UNO &&
             siguienteFallo == Fallo::PERDER_ACK_UNO) {
    perder = true;
    siguienteFallo = Fallo::NINGUNO;
  }
  // Escritura completa y bloqueante, 32 bytes a 9600 8N1.
  relojUS += (uint64_t(longitud) * 10 * 1000000 + 9599) / 9600;
  if (!perder) {
    auto& destino = desdeNano ? haciaUno : haciaNano;
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

int instanciasUART = 0;
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

#define __AVR_ATmega328P__ 1
namespace nano {
#include "MAX485_UN_MODULO_NANO_UNO/NANO/NANO.ino"
}
namespace uno {
#include "MAX485_UN_MODULO_NANO_UNO/UNO/UNO.ino"
}

template<class Condicion> void esperar(Condicion condicion, uint32_t limiteMS = 5000) {
  const uint32_t inicio = millis();
  while (!condicion() && millis() - inicio < limiteMS) {
    uno::loop();
    nano::loop();
    delay(2);
  }
  assert(condicion());
}
void cicloCompletoNuevo() {
  const uint32_t previo = nano::confirmadosNANO;
  esperar([&] { return nano::confirmadosNANO > previo; });
  assert(uno::fase == uno::REPOSO && !nano::esperandoACK);
  assert(escuchaNano && escuchaUno);
}

int main() {
  nano::setup();
  uno::setup();
  esperar([] { return nano::confirmadosNANO >= 100; }, 150000);
  assert(nano::confirmadosNANO == 100 && nano::enviadosNANO == 100);
  assert(nano::recibidosUno == 100 && uno::recibidosNANO == 100);
  assert(uno::confirmadosUno == 100 && uno::enviados == 100);
  assert(nano::timeoutsNANO == 0 && uno::timeoutsUno == 0);
  assert(uno::timeoutsTurnoNANO == 0 && nano::inesperados == 0);
  assert(nano::receptor.invalidos == 0 && uno::receptor.invalidos == 0);

  for (Fallo fallo : {Fallo::CRC_ACK_NANO, Fallo::ACK_NANO_AJENO, Fallo::PERDER_ACK_NANO}) {
    const auto previos = uno::timeoutsUno;
    const auto invalidos = uno::receptor.invalidos;
    const auto inesperados = uno::inesperados;
    siguienteFallo = fallo;
    esperar([&] { return uno::timeoutsUno > previos; });
    assert(siguienteFallo == Fallo::NINGUNO);
    if (fallo == Fallo::CRC_ACK_NANO) assert(uno::receptor.invalidos > invalidos);
    if (fallo == Fallo::ACK_NANO_AJENO) assert(uno::inesperados > inesperados);
    cicloCompletoNuevo();
  }
  auto previoTurno = uno::timeoutsTurnoNANO;
  siguienteFallo = Fallo::PERDER_DATOS_NANO;
  esperar([&] { return uno::timeoutsTurnoNANO > previoTurno; });
  cicloCompletoNuevo();

  auto previoNano = nano::timeoutsNANO;
  siguienteFallo = Fallo::PERDER_ACK_UNO;
  esperar([&] { return nano::timeoutsNANO > previoNano; });
  cicloCompletoNuevo();

  conectado = false;
  auto previoUno = uno::timeoutsUno;
  esperar([&] { return uno::timeoutsUno > previoUno; });
  conectado = true;
  cicloCompletoNuevo();

  uint8_t versionAnterior[32];
  PruebaMAX485::crear(PruebaMAX485::DATOS_UNO, 1, 2, 1, versionAnterior);
  versionAnterior[4] = 3;
  const uint16_t crc = PruebaMAX485::crc16(versionAnterior, 30);
  versionAnterior[30] = crc & 0xFF;
  versionAnterior[31] = crc >> 8;
  assert(!PruebaMAX485::valido(versionAnterior));
  std::cout << "PASS: 100 ciclos bidireccionales, CRC/ACK ajeno/perdidas, "
               "reconexion y version incompatible. Modelo logico, sin validacion electrica.\n";
}

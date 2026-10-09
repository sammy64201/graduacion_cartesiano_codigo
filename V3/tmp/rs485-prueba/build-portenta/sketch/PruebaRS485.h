#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_COMUNICACION\\PORTENTA\\PruebaRS485.h"
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Protocolo exclusivo de esta prueba. No contiene ordenes de la maqueta.
namespace PruebaRS485 {
constexpr size_t TAMANO = 32;
constexpr uint8_t VERSION = 2;
constexpr uint8_t DATOS_PORTENTA = 1;
constexpr uint8_t ACK_ESP = 2;
constexpr uint8_t TURNO_ESP = 3;
constexpr uint8_t DATOS_ESP = 4;
constexpr uint8_t ACK_PORTENTA = 5;
constexpr uint32_t BAUD = 115200;
constexpr uint32_t INTERVALO_MS = 1000;
constexpr uint32_t TIMEOUT_MS = 500;

inline uint16_t crc16(const uint8_t *datos, size_t longitud) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < longitud; ++i) {
    crc ^= static_cast<uint16_t>(datos[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

inline uint32_t leer32(const uint8_t *datos, uint8_t offset) {
  uint32_t resultado = 0;
  for (uint8_t i = 0; i < 4; ++i)
    resultado |= static_cast<uint32_t>(datos[offset + i]) << (8 * i);
  return resultado;
}

inline void escribir32(uint8_t *datos, uint8_t offset, uint32_t valor) {
  for (uint8_t i = 0; i < 4; ++i) datos[offset + i] = valor >> (8 * i);
}
inline uint32_t secuencia(const uint8_t *datos) { return leer32(datos, 6); }
inline uint32_t valor(const uint8_t *datos) { return leer32(datos, 10); }
inline uint32_t turno(const uint8_t *datos) { return leer32(datos, 14); }

inline void crear(uint8_t tipo, uint32_t numero, uint32_t valorLocal,
                  uint32_t turnoActual, uint8_t *datos) {
  memcpy(datos, "R485", 4);
  datos[4] = VERSION;
  datos[5] = tipo;
  escribir32(datos, 6, numero);
  escribir32(datos, 10, valorLocal);
  escribir32(datos, 14, turnoActual);
  for (uint8_t i = 18; i < 30; ++i)
    datos[i] = static_cast<uint8_t>(numero + valorLocal + turnoActual + i * 17 + tipo);
  const uint16_t crc = crc16(datos, 30);
  datos[30] = crc & 0xFF;
  datos[31] = crc >> 8;
}

inline bool valido(const uint8_t *datos) {
  if (memcmp(datos, "R485", 4) != 0 || datos[4] != VERSION ||
      datos[5] < DATOS_PORTENTA || datos[5] > ACK_PORTENTA) return false;
  const uint16_t recibido = datos[30] | (static_cast<uint16_t>(datos[31]) << 8);
  if (crc16(datos, 30) != recibido) return false;
  const uint32_t numero = secuencia(datos);
  for (uint8_t i = 18; i < 30; ++i)
    if (datos[i] != static_cast<uint8_t>(numero + valor(datos) + turno(datos) +
                                       i * 17 + datos[5])) return false;
  return true;
}

// Ventana deslizante: recupera sincronizacion despues de ruido/bytes perdidos.
class Receptor {
 public:
  uint32_t invalidos = 0;
  uint32_t bytes = 0;
  uint32_t paquetes = 0;
  bool agregar(uint8_t dato, uint8_t *paquete) {
    ++bytes;
    if (usados == TAMANO) {
      memmove(buffer, buffer + 1, TAMANO - 1);
      --usados;
    }
    buffer[usados++] = dato;
    if (usados != TAMANO) return false;
    if (valido(buffer)) {
      memcpy(paquete, buffer, TAMANO);
      usados = 0;
      ++paquetes;
      return true;
    }
    if (memcmp(buffer, "R485", 4) == 0) ++invalidos;
    return false;
  }
 private:
  uint8_t buffer[TAMANO] = {};
  size_t usados = 0;
};
} // namespace PruebaRS485

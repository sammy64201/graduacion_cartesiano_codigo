#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\automatico v2 rs485\\PORTENTA\\RecuperacionEnlace.h"
#pragma once
#include <stdint.h>

// Politica comun de Automatico V2 I2C y RS485. El plazo de cada intercambio
// pertenece al transporte y no dispara por si solo la recuperacion general.
namespace RecuperacionEnlace {
constexpr uint32_t SIN_RESPUESTA_MS = 1000;

inline bool vigente(bool existePaqueteValido, uint32_t ahora, uint32_t ultimoValido) {
    return existePaqueteValido && ahora - ultimoValido <= SIN_RESPUESTA_MS;
}
} // namespace RecuperacionEnlace

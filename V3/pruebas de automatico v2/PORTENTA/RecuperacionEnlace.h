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

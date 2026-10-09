#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\automatico v2 rs485\\PORTENTA\\AjusteTemporalCapturaV2.h"
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>

namespace AjusteTemporalCapturaV2 {
constexpr int32_t MIN_MS = -500;
constexpr int32_t MAX_MS = 500;

// Positivo adelanta; negativo retrasa. El resultado es un horizonte,
// no una espera fija desde detectar la pieza ni un cambio de geometria.
inline float horizonte(float baseS, int32_t desfaseMs) {
    return baseS + static_cast<float>(desfaseMs) * 0.001f;
}

inline bool analizar(const char *texto, int32_t &desfaseMs) {
    if (texto == nullptr) return false;
    char *fin = nullptr;
    errno = 0;
    const long valor = strtol(texto, &fin, 10);
    if (fin == texto || errno == ERANGE) return false;
    while (*fin != '\0' && isspace(static_cast<unsigned char>(*fin))) ++fin;
    if (*fin != '\0' || valor < MIN_MS || valor > MAX_MS) return false;
    desfaseMs = static_cast<int32_t>(valor);
    return true;
}
} // namespace AjusteTemporalCapturaV2

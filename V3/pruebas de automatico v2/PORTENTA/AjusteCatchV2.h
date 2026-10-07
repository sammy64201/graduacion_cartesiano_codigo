#pragma once
#include <stdint.h>

namespace AjusteCatchV2 {
// Se conserva al menos 100 ms de alineacion continua (base: 300 ms).
constexpr int32_t MIN_OFFSET_MS = -200;
constexpr int32_t MAX_OFFSET_MS = 1000;
enum Resultado : uint8_t { SIN_RESULTADO, AGARRO, ANTES, DESPUES };
inline int32_t limitar(int32_t valor) {
    return valor < MIN_OFFSET_MS ? MIN_OFFSET_MS :
        (valor > MAX_OFFSET_MS ? MAX_OFFSET_MS : valor);
}
struct Sesion {
    int32_t offsetMs;
    int32_t ultimoProbadoMs = 0;
    uint16_t pasoMs = 50;
    uint16_t ensayos = 0;
    uint8_t aciertosConsecutivos = 0;
    int8_t direccionAnterior = 0;
    Resultado ultimoResultado = SIN_RESULTADO;
    bool limiteAlcanzado = false;

    explicit Sesion(int32_t inicial = 0) : offsetMs(limitar(inicial)) {}
    bool confirmado() const { return aciertosConsecutivos >= 3; }
    void evaluar(Resultado resultado, int32_t probado) {
        const bool mismoValor = ensayos != 0 && ultimoProbadoMs == probado;
        ultimoProbadoMs = probado;
        ultimoResultado = resultado;
        if (ensayos < UINT16_MAX) ++ensayos;
        limiteAlcanzado = false;
        if (resultado == AGARRO) {
            aciertosConsecutivos = mismoValor ? aciertosConsecutivos : 0;
            if (aciertosConsecutivos < 3) ++aciertosConsecutivos;
            offsetMs = limitar(probado);
            return;
        }
        aciertosConsecutivos = 0;
        const int8_t direccion = resultado == ANTES ? 1 : -1;
        if (direccionAnterior != 0 && direccion != direccionAnterior)
            pasoMs = pasoMs > 20 ? pasoMs / 2 : 10;
        direccionAnterior = direccion;
        const int32_t candidato = probado + direccion * static_cast<int32_t>(pasoMs);
        offsetMs = limitar(candidato);
        limiteAlcanzado = offsetMs != candidato;
    }
};
} // namespace AjusteCatchV2

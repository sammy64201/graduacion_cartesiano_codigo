#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\automatico v2 rs485\\PORTENTA\\CapturaFijaV2.h"
#pragma once
#include <stdint.h>
#include <math.h>

// Modelo portable. Las tolerancias y tiempos vienen del perfil fisico del
// sketch: este archivo no declara ninguna medida mecanica como validada.
namespace CapturaFijaV2 {

inline int32_t diferencia(int32_t actual, int32_t referencia) {
    const uint32_t delta = static_cast<uint32_t>(actual) -
                           static_cast<uint32_t>(referencia);
    return delta <= 0x7fffffffUL ? static_cast<int32_t>(delta) :
        static_cast<int32_t>(static_cast<int64_t>(delta) - 4294967296LL);
}

struct Movimiento {
    bool valido;
    float velocidad;
    float aceleracion;
    float errorVelocidad;
    float errorAceleracion;
    uint32_t muestraMs;
};

class Estimador {
public:
    Estimador() : iniciado_(false), ventanas_(0), referencia_(0), inicio_(0),
        periodoAnteriorS_(0.0f), velocidadAnterior_(0.0f), signo_(0), escala_(0.0f),
        movimiento_{} {}

    void reiniciar(int32_t conteo, uint32_t ahora, float escala, int8_t signo) {
        iniciado_ = true;
        ventanas_ = 0;
        referencia_ = conteo;
        inicio_ = ahora;
        escala_ = escala;
        signo_ = signo;
        periodoAnteriorS_ = 0.0f;
        velocidadAnterior_ = 0.0f;
        movimiento_ = Movimiento{};
    }

    // Cada ventana incluye todos los intervalos sin pulsos. Usa tiempo real,
    // tambien cuando el loop se retrasa; no divide cada pulso por 10 ms.
    bool actualizar(int32_t conteo, uint32_t ahora, float escala, int8_t signo,
                    uint32_t ventanaMs = 100UL) {
        if (!iniciado_ || escala != escala_ || signo != signo_) {
            reiniciar(conteo, ahora, escala, signo);
            return false;
        }
        const uint32_t dtMs = ahora - inicio_;
        if (dtMs < ventanaMs || dtMs == 0) return false;
        const float dt = static_cast<float>(dtMs) * 0.001f;
        const float velocidad = static_cast<float>(signo) * escala *
            static_cast<float>(diferencia(conteo, referencia_)) / dt;
        const float entreCentros = 0.5f * (dt + periodoAnteriorS_);
        const float aceleracionCruda = ventanas_ > 0 && entreCentros > 0.0f
            ? (velocidad - velocidadAnterior_) / entreCentros : 0.0f;
        const float anteriorA = movimiento_.aceleracion;
        const float aceleracion = ventanas_ > 1
            ? 0.5f * anteriorA + 0.5f * aceleracionCruda : aceleracionCruda;
        const float cuantizacionV = fabsf(escala) / dt;
        const float cuantizacionA = entreCentros > 0.0f
            ? 2.0f * cuantizacionV / entreCentros : 0.0f;
        movimiento_.velocidad = velocidad;
        movimiento_.aceleracion = aceleracion;
        movimiento_.errorVelocidad = cuantizacionV +
            0.5f * fabsf(aceleracion) * dt;
        movimiento_.errorAceleracion = cuantizacionA +
            fabsf(aceleracionCruda - aceleracion);
        movimiento_.muestraMs = ahora;
        if (ventanas_ < 255U) ++ventanas_;
        movimiento_.valido = ventanas_ >= 3U && isfinite(velocidad) &&
            isfinite(aceleracion) && escala > 0.0f &&
            (signo == 1 || signo == -1);
        velocidadAnterior_ = velocidad;
        periodoAnteriorS_ = dt;
        referencia_ = conteo;
        inicio_ = ahora;
        return true;
    }

    Movimiento movimiento() const { return movimiento_; }

private:
    bool iniciado_;
    uint8_t ventanas_;
    int32_t referencia_;
    uint32_t inicio_;
    float periodoAnteriorS_, velocidadAnterior_;
    int8_t signo_;
    float escala_;
    Movimiento movimiento_;
};

struct Intervalo {
    bool valido;
    float minimo;
    float maximo;
};

inline Intervalo predecir(float y, const Movimiento &m, float tMinS,
                         float tMaxS, float errorPosicionMm,
                         float limiteAceleracionMmS2) {
    Intervalo resultado = {};
    if (!m.valido || !isfinite(y) || !isfinite(tMinS) || !isfinite(tMaxS) ||
        !isfinite(errorPosicionMm) || tMinS < 0.0f || tMaxS < tMinS ||
        errorPosicionMm < 0.0f || !isfinite(limiteAceleracionMmS2) ||
        limiteAceleracionMmS2 <= 0.0f ||
        !isfinite(m.velocidad) || !isfinite(m.aceleracion) ||
        !isfinite(m.errorVelocidad) || !isfinite(m.errorAceleracion) ||
        m.errorVelocidad < 0.0f || m.errorAceleracion < 0.0f ||
        fabsf(m.aceleracion) + m.errorAceleracion > limiteAceleracionMmS2)
        return resultado;
    const float vMin = m.velocidad - m.errorVelocidad;
    const float vMax = m.velocidad + m.errorVelocidad;
    const float aMin = m.aceleracion - m.errorAceleracion;
    const float aMax = m.aceleracion + m.errorAceleracion;
    // La envolvente solo se admite si todos los escenarios siguen avanzando.
    // Un paro/inversion no se extrapola como si la banda mantuviera su avance.
    if (vMin <= 0.0f || vMin + fminf(0.0f, aMin) * tMaxS <= 0.0f)
        return resultado;
    resultado.minimo = y + vMin * tMinS +
        0.5f * aMin * tMinS * tMinS - errorPosicionMm;
    resultado.maximo = y + vMax * tMaxS +
        0.5f * aMax * tMaxS * tMaxS + errorPosicionMm;
    resultado.valido = isfinite(resultado.minimo) && isfinite(resultado.maximo) &&
        resultado.minimo <= resultado.maximo;
    return resultado;
}

enum Decision : uint8_t { INVALIDA, ANTES, DENTRO, DESPUES, INCIERTA };

inline Decision evaluar(const Intervalo &p, float yCatch, float toleranciaMm) {
    if (!p.valido || !isfinite(yCatch) || !isfinite(toleranciaMm) ||
        toleranciaMm <= 0.0f) return INVALIDA;
    if (p.maximo - p.minimo > 2.0f * toleranciaMm) return INCIERTA;
    if (p.minimo < yCatch - toleranciaMm) return ANTES;
    if (p.maximo > yCatch + toleranciaMm) return DESPUES;
    return DENTRO;
}

}  // namespace CapturaFijaV2

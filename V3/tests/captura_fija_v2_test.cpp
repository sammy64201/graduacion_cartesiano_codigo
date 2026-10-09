#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include "../automatico v2 rs485/PORTENTA/CapturaFijaV2.h"

using namespace CapturaFijaV2;

static bool cerca(float a, float b, float margen = 0.001f) {
    return fabsf(a - b) <= margen;
}

int main() {
    assert(diferencia(INT32_MIN + 9, INT32_MAX - 10) == 20);
    assert(diferencia(INT32_MAX - 10, INT32_MIN + 9) == -20);
    assert(diferencia(-4, -14) == 10);

    Estimador lento;
    lento.reiniciar(0, 0, 0.1f, 1);
    // Un pulso cada 50 ms: los otros intervalos de 10 ms deben aportar cero.
    for (uint32_t t = 10; t <= 600; t += 10)
        lento.actualizar(static_cast<int32_t>(t / 50), t, 0.1f, 1, 100);
    assert(lento.movimiento().valido);
    assert(cerca(lento.movimiento().velocidad, 2.0f));
    assert(cerca(lento.movimiento().aceleracion, 0.0f));
    lento.actualizar(12, 700, 0.1f, 1, 100);
    assert(cerca(lento.movimiento().velocidad, 0.0f));
    assert(lento.movimiento().aceleracion < 0.0f);

    Estimador irregular;
    irregular.reiniciar(0, 0, 0.1f, 1);
    assert(!irregular.actualizar(0, 0, 0.1f, 1));
    assert(irregular.actualizar(30, 100, 0.1f, 1));
    assert(irregular.actualizar(90, 300, 0.1f, 1));
    assert(irregular.actualizar(135, 450, 0.1f, 1));
    assert(irregular.movimiento().valido);
    assert(cerca(irregular.movimiento().velocidad, 30.0f));
    assert(cerca(irregular.movimiento().aceleracion, 0.0f));

    Estimador acelerado;
    acelerado.reiniciar(0, 0, 0.01f, 1);
    acelerado.actualizar(100, 100, 0.01f, 1);
    acelerado.actualizar(220, 200, 0.01f, 1);
    acelerado.actualizar(360, 300, 0.01f, 1);
    assert(cerca(acelerado.movimiento().velocidad, 14.0f));
    assert(cerca(acelerado.movimiento().aceleracion, 20.0f, 0.01f));
    // Cambiar escala/sentido invalida el historial anterior.
    assert(!acelerado.actualizar(360, 310, 0.02f, -1));
    assert(!acelerado.movimiento().valido);

    Estimador conWrap;
    const uint32_t inicio = UINT32_MAX - 49;
    conWrap.reiniciar(INT32_MAX - 5, inicio, 0.1f, 1);
    conWrap.actualizar(INT32_MIN + 4, 50, 0.1f, 1);
    assert(cerca(conWrap.movimiento().velocidad, 10.0f));

    Movimiento m = {true, 40.0f, 20.0f, 1.0f, 2.0f, 300};
    Intervalo p = predecir(-20.0f, m, 0.4f, 0.5f, 2.0f, 200.0f);
    assert(p.valido);
    assert(cerca(p.minimo, -4.96f));
    assert(cerca(p.maximo, 5.25f));
    assert(evaluar(p, 0.0f, 5.0f) == INCIERTA);
    assert(evaluar(Intervalo{true, -20.0f, -15.0f}, 0.0f, 5.0f) == ANTES);
    assert(evaluar(Intervalo{true, -4.0f, 4.0f}, 0.0f, 5.0f) == DENTRO);
    assert(evaluar(Intervalo{true, -2.0f, 6.0f}, 0.0f, 5.0f) == DESPUES);

    // Una incertidumbre futura mayor ensancha la envolvente incluso cuando
    // la aceleracion observada hoy sea cero.
    m = Movimiento{true, 30.0f, 0.0f, 0.5f, 2.0f, 300};
    const Intervalo estrecha = predecir(-15.0f, m, 0.5f, 0.5f, 1.0f, 200.0f);
    m.errorAceleracion += 30.0f;
    const Intervalo futura = predecir(-15.0f, m, 0.5f, 0.5f, 1.0f, 200.0f);
    assert(estrecha.valido && futura.valido);
    assert(futura.minimo < estrecha.minimo && futura.maximo > estrecha.maximo);
    assert(evaluar(estrecha, 0.0f, 5.0f) == DENTRO);
    assert(evaluar(futura, 0.0f, 5.0f) == INCIERTA);

    // Paro/inversion posible y aceleracion fuera del perfil no se extrapolan.
    m.velocidad = 1.0f;
    assert(!predecir(0.0f, m, 0.0f, 0.5f, 1.0f, 200.0f).valido);
    m.velocidad = 40.0f;
    m.aceleracion = 190.0f;
    assert(!predecir(0.0f, m, 0.0f, 0.5f, 1.0f, 200.0f).valido);
    m.aceleracion = 0.0f;
    assert(!predecir(0.0f, m, 0.5f, 0.4f, 1.0f, 200.0f).valido);
    assert(!predecir(0.0f, m, 0.0f, 0.1f, 1.0f, NAN).valido);
    puts("CapturaFijaV2: wrap, ventanas cero, tiempo real, aceleracion e intervalos OK");
}

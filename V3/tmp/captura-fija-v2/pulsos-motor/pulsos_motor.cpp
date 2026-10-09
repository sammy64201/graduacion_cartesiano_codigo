#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>
#define HIGH true
#define LOW false
const int pP_X = 4;
const int pP_Y = 2;
const int pP_Z = 0;
const uint16_t DIV_MANUAL = 1;
volatile int8_t movX = 0;
volatile int8_t movY = 0;
volatile int8_t movZ = 0;
volatile bool pulsoX = false;
volatile bool pulsoY = false;
volatile bool pulsoZ = false;
volatile uint16_t cuentaX = 0;
volatile uint16_t cuentaY = 0;
volatile uint16_t cuentaZ = 0;
volatile uint16_t divisorX = DIV_MANUAL;
volatile uint16_t divisorY = DIV_MANUAL;
volatile uint16_t divisorZ = DIV_MANUAL;
volatile long pasosX = 0;
volatile long pasosY = 0;
volatile long pasosZ = 0;
volatile long objetivoX = 0;
volatile bool objetivoXActivo = false;
volatile long objetivoY = 0;
volatile bool objetivoYActivo = false;
volatile long objetivoZ = 0;
volatile bool objetivoZActivo = false;

uint32_t tiempoUs = 0;
int bloqueoInterrupciones = 0;
void noInterrupts() { ++bloqueoInterrupciones; }
void interrupts() { assert(bloqueoInterrupciones > 0); --bloqueoInterrupciones; }
struct Cambio { int pin; bool high; uint32_t tiempoUs; };
struct Salidas {
    bool nivel[8] = {};
    std::vector<Cambio> cambios;
    void set(int pin, bool high) {
        assert(pin >= 0 && pin < 8);
        if (nivel[pin] != high) cambios.push_back({pin, high, tiempoUs});
        nivel[pin] = high;
    }
} digital_outputs;
void generarPulsoMotor() {
    if (movX != 0) {
        cuentaX++;
        if (cuentaX >= divisorX) {
            cuentaX = 0;
            pulsoX = !pulsoX;
            digital_outputs.set(pP_X, pulsoX ? HIGH : LOW);
            if (pulsoX) {
                pasosX += movX;
                if (objetivoXActivo &&
                    ((movX > 0 && pasosX >= objetivoX) ||
                     (movX < 0 && pasosX <= objetivoX))) {
                    pasosX = objetivoX;
                    movX = 0;
                    objetivoXActivo = false;
                    // Conservar el ultimo HIGH hasta el siguiente tick;
                    // contarlo no permite acortar su ancho a una sola ISR.
                }
            }
        }
    } else {
        cuentaX = 0;
        pulsoX = false;
        digital_outputs.set(pP_X, LOW);
    }

    if (movY != 0) {
        cuentaY++;
        if (cuentaY >= divisorY) {
            cuentaY = 0;
            pulsoY = !pulsoY;
            digital_outputs.set(pP_Y, pulsoY ? HIGH : LOW);
            if (pulsoY) {
                pasosY += movY;
                if (objetivoYActivo &&
                    ((movY > 0 && pasosY >= objetivoY) ||
                     (movY < 0 && pasosY <= objetivoY))) {
                    pasosY = objetivoY;
                    movY = 0;
                    objetivoYActivo = false;
                }
            }
        }
    } else {
        cuentaY = 0;
        pulsoY = false;
        digital_outputs.set(pP_Y, LOW);
    }

    if (movZ != 0) {
        cuentaZ++;
        if (cuentaZ >= divisorZ) {
            cuentaZ = 0;
            pulsoZ = !pulsoZ;
            digital_outputs.set(pP_Z, pulsoZ ? HIGH : LOW);
            if (pulsoZ) {
                pasosZ += movZ;
                if (objetivoZActivo &&
                    ((movZ > 0 && pasosZ >= objetivoZ) ||
                     (movZ < 0 && pasosZ <= objetivoZ))) {
                    pasosZ = objetivoZ;
                    movZ = 0;
                    objetivoZActivo = false;
                }
            }
        }
    } else {
        cuentaZ = 0;
        pulsoZ = false;
        digital_outputs.set(pP_Z, LOW);
    }
}
void detenerX() {
    noInterrupts();
    movX = 0;
    objetivoXActivo = false;
    cuentaX = 0;
    interrupts();
    // Si ya hubo flanco ascendente, el ticker completa su semiperiodo.
    // mov=0 impide cualquier paso nuevo, tambien al cancelar por finales.
    if (!pulsoX) digital_outputs.set(pP_X, LOW);
}
void detenerY() {
    noInterrupts();
    movY = 0;
    objetivoYActivo = false;
    cuentaY = 0;
    interrupts();
    if (!pulsoY) digital_outputs.set(pP_Y, LOW);
}
void detenerZ() {
    noInterrupts();
    movZ = 0;
    objetivoZActivo = false;
    cuentaZ = 0;
    interrupts();
    if (!pulsoZ) digital_outputs.set(pP_Z, LOW);
}

struct Eje {
    char nombre;
    int pin;
    volatile int8_t &mov;
    volatile bool &pulso;
    volatile uint16_t &cuenta;
    volatile uint16_t &divisor;
    volatile long &pasos;
    volatile bool &activo;
    volatile long &objetivo;
    void (*detener)();
};
Eje ejes[] = {
    {'X', pP_X, movX, pulsoX, cuentaX, divisorX, pasosX, objetivoXActivo, objetivoX, detenerX},
    {'Y', pP_Y, movY, pulsoY, cuentaY, divisorY, pasosY, objetivoYActivo, objetivoY, detenerY},
    {'Z', pP_Z, movZ, pulsoZ, cuentaZ, divisorZ, pasosZ, objetivoZActivo, objetivoZ, detenerZ},
};
void reset() {
    assert(bloqueoInterrupciones == 0);
    tiempoUs = 0;
    digital_outputs = {};
    for (auto &e : ejes) {
        e.mov = 0; e.pulso = false; e.cuenta = 0; e.divisor = 1;
        e.pasos = 17; e.activo = false; e.objetivo = 17;
    }
}
void tick() {
    assert(bloqueoInterrupciones == 0);
    tiempoUs += 100;
    generarPulsoMotor();
}
void iniciar(Eje &e, int direccion, uint16_t divisor, int cantidad) {
    e.divisor = divisor;
    e.mov = static_cast<int8_t>(direccion);
    e.activo = true;
    e.objetivo = e.pasos + direccion * cantidad;
}
void comprobarOtrosEjes(const Eje &actual) {
    for (const auto &e : ejes) {
        if (e.nombre == actual.nombre) continue;
        assert(e.pasos == 17 && e.mov == 0 && !e.activo);
        assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    }
}
void comprobarPulsos(Eje &e, unsigned esperados) {
    unsigned subidas = 0, bajadas = 0;
    bool high = false;
    uint32_t inicio = 0;
    for (const auto &c : digital_outputs.cambios) {
        assert(c.pin == e.pin);
        if (c.high) {
            assert(!high);
            high = true; inicio = c.tiempoUs; ++subidas;
        } else {
            assert(high);
            // El ticker mas rapido dispone de un semiperiodo de 100 us.
            // Al completar una orden tampoco debe haber un pulso de 0 us.
            assert(c.tiempoUs - inicio >= 100);
            high = false; ++bajadas;
        }
    }
    assert(!high && subidas == esperados && bajadas == esperados);
}
void completarObjetivo(Eje &e, int direccion, uint16_t divisor, int cantidad) {
    reset(); iniciar(e, direccion, divisor, cantidad);
    const long destino = e.objetivo;
    unsigned ticks = 0;
    while (e.mov != 0) {
        tick(); assert(++ticks <= 2U * divisor * static_cast<unsigned>(cantidad));
    }
    assert(e.pasos == destino && !e.activo);
    // El contador y el movimiento ya terminaron, pero el ultimo HIGH sigue
    // presente hasta la siguiente ISR. No hay otro flanco de subida.
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    const uint32_t ultimoHigh = tiempoUs;
    assert(digital_outputs.cambios.back().high);
    assert(digital_outputs.cambios.back().tiempoUs == ultimoHigh);
    tiempoUs += 37;
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    tiempoUs = ultimoHigh;
    tick();
    assert(tiempoUs - ultimoHigh == 100);
    assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    assert(e.pasos == destino && e.mov == 0 && !e.activo);
    for (int i = 0; i < 20; ++i) tick();
    assert(e.pasos == destino);
    comprobarPulsos(e, static_cast<unsigned>(cantidad));
    comprobarOtrosEjes(e);
}
void detenerEntreISR(Eje &e, int direccion, uint16_t divisor, bool despuesDelObjetivo) {
    reset(); iniciar(e, direccion, divisor, despuesDelObjetivo ? 1 : 4);
    for (uint16_t i = 0; i < divisor; ++i) tick();
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    assert(e.pasos == 17 + direccion);
    const uint32_t flancoUs = tiempoUs;
    tiempoUs += 37; // Parada de loop, entre el flanco HIGH y el proximo ticker.
    e.detener();
    assert(bloqueoInterrupciones == 0);
    assert(e.mov == 0 && !e.activo);
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    // Paradas repetidas no convierten el HIGH pendiente en un pulso corto.
    e.detener();
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    tiempoUs = flancoUs;
    tick();
    assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    assert(e.pasos == 17 + direccion);
    for (int i = 0; i < 20; ++i) tick();
    assert(e.pasos == 17 + direccion);
    comprobarPulsos(e, 1);
    comprobarOtrosEjes(e);
}
void detenerEnLOW(Eje &e, int direccion, uint16_t divisor) {
    reset(); iniciar(e, direccion, divisor, 4);
    for (uint16_t i = 0; i < 2 * divisor; ++i) tick();
    assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    e.detener();
    assert(e.mov == 0 && !e.activo && !e.pulso);
    for (int i = 0; i < 20; ++i) tick();
    assert(e.pasos == 17 + direccion);
    comprobarPulsos(e, 1);
    comprobarOtrosEjes(e);
}
int main() {
    unsigned casos = 0;
    for (auto &e : ejes) {
        for (int direccion : {-1, 1}) {
            for (uint16_t divisor : {uint16_t(1), uint16_t(8)}) {
                for (int pasos : {1, 4}) {
                    completarObjetivo(e, direccion, divisor, pasos); ++casos;
                }
                detenerEntreISR(e, direccion, divisor, false); ++casos;
                detenerEntreISR(e, direccion, divisor, true); ++casos;
                detenerEnLOW(e, direccion, divisor); ++casos;
            }
        }
    }
    std::printf("PASS: %u casos ISR/paradas reales; X/Y/Z, +/- y divisor 1/8; ultimo HIGH >=100 us, sin pasos extra\n", casos);
}

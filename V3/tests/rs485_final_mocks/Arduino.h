#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cassert>
#include <deque>
#include <sstream>
#include <string>
constexpr int LOW = 0, HIGH = 1, OUTPUT = 1, SERIAL_8N1 = 6;
extern uint32_t ahoraSimulado;
extern uint32_t fraccionMicrosegundos;
extern bool deESP;
extern uint64_t inicioTxESP, finTxESP;
extern bool flushESP;
extern std::deque<uint8_t> entradaESP, entradaPortenta;
typedef void (*EventoTramaESP)();
inline EventoTramaESP &eventoFinTramaESP() {
    static EventoTramaESP evento = nullptr;
    return evento;
}
inline uint32_t millis() { return ahoraSimulado; }
inline uint64_t tiempoMicrosegundos() { return uint64_t(ahoraSimulado) * 1000 + fraccionMicrosegundos; }
inline void delayMicroseconds(unsigned us) {
    const uint32_t total = fraccionMicrosegundos + us;
    ahoraSimulado += total / 1000;
    fraccionMicrosegundos = total % 1000;
}
inline uint32_t esp_random() { return 12345; }
inline void digitalWrite(int pin, int valor) {
    assert(pin == 18);
    if (valor == HIGH) {
        assert(!deESP);
        inicioTxESP = tiempoMicrosegundos(); flushESP = false;
    } else if (deESP) {
        assert(flushESP && tiempoMicrosegundos() - finTxESP == 200);
    }
    deESP = valor == HIGH;
}
inline void pinMode(int pin, int modo) { assert(pin == 18 && modo == OUTPUT); }
struct Monitor {
    std::deque<char> entrada;
    std::ostringstream salida;
    void begin(unsigned) {}
    int available() { return static_cast<int>(entrada.size()); }
    int read() { auto c = entrada.front(); entrada.pop_front(); return c; }
    template<class T> void print(T valor) { salida << +valor; }
    void print(const char *valor) { salida << valor; }
    template<class T> void println(T valor) { print(valor); salida << '\n'; }
};
struct HardwareSerial {
    unsigned baud=0; int config=0, rx=-1, tx=-1; size_t enviados=0;
    explicit HardwareSerial(int uart) { assert(uart == 2); }
    void end() { baud=0; }
    void setRxBufferSize(size_t capacidad) { assert(capacidad == 256); }
    void begin(unsigned velocidad, int formato, int pinRX, int pinTX) {
        baud=velocidad; config=formato; rx=pinRX; tx=pinTX;
        assert(baud == 115200 && config == SERIAL_8N1 && rx == 14 && tx == 27);
    }
    int available() { return static_cast<int>(entradaESP.size()); }
    int read() { auto b = entradaESP.front(); entradaESP.pop_front(); return b; }
    size_t write(const uint8_t *p, size_t n) {
        if (!deESP) return 0;
        assert(tiempoMicrosegundos() - inicioTxESP == 200 && !flushESP);
        enviados=n;
        entradaPortenta.insert(entradaPortenta.end(), p, p + n); return n;
    }
    void flush() {
        assert(deESP);
        delayMicroseconds((enviados * 10UL * 1000000UL + baud - 1) / baud);
        enviados=0; finTxESP=tiempoMicrosegundos(); flushESP=true;
        // La otra CPU puede procesar el cero final antes de que ESP termine
        // su POST_TX_US. La simulacion secuencial sola ocultaba este intervalo.
        if (eventoFinTramaESP()) eventoFinTramaESP()();
    }
};

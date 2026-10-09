#pragma once
#include "Arduino.h"
#include <cstring>
struct trng_t {};
inline void trng_init(trng_t *) {}
inline void trng_free(trng_t *) {}
inline int trng_get_bytes(trng_t *, uint8_t *p, size_t n, size_t *cantidad) {
    std::memset(p, 0x12, n); *cantidad = n; return 0;
}
namespace machinecontrol {
struct UART {
    unsigned baud=0, pre=0, post=0; int config=0; size_t enviados=0;
    bool tx = false, rx = true;
    // Overloads de ArduinoRS485: con tres parametros, 0 es predelay,
    // no la configuracion UART. La libreria selecciona SERIAL_8N1.
    void begin(unsigned velocidad, int antes, int despues) { begin(velocidad, SERIAL_8N1, antes, despues); }
    void begin(unsigned velocidad, int formato, int antes, int despues) {
        baud=velocidad; config=formato; pre=antes; post=despues;
        assert(baud == 115200 && config == SERIAL_8N1 && pre == 0 && post == 2000);
    }
    void receive() { assert(!tx); rx = true; }
    void noReceive() { assert(!tx); rx = false; }
    void beginTransmission() { assert(!rx && !tx && !deESP); tx = true; delayMicroseconds(pre); }
    void endTransmission() {
        assert(tx && !rx);
        delayMicroseconds((enviados * 10UL * 1000000UL + baud - 1) / baud);
        delayMicroseconds(post); enviados=0; tx = false;
    }
    int available() { return rx ? static_cast<int>(entradaPortenta.size()) : 0; }
    int read() { auto b = entradaPortenta.front(); entradaPortenta.pop_front(); return b; }
    size_t write(const uint8_t *p, size_t n) {
        assert(!rx);
        if (!tx || deESP) return 0;
        enviados=n;
        entradaESP.insert(entradaESP.end(), p, p + n); return n;
    }
};
struct Comunicacion {
    UART rs485;
    bool iniciada=false, rs232=true, fullDuplex=true, terminacion=true, habilitada=false;
    void init() { iniciada=true; rs232=false; fullDuplex=false; terminacion=false; }
    void rs485ModeRS232(bool valor) { rs232=valor; }
    void rs485FullDuplex(bool valor) { fullDuplex=valor; }
    void rs485ABTerm(bool valor) { terminacion=valor; }
    void rs485Enable(bool valor) { habilitada=valor; }
};
extern Comunicacion comm_protocols;
}

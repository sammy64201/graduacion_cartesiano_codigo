#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\automatico v2 rs485\\PORTENTA\\EnlaceRS485.h"
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Transporte comun, sin dependencias Arduino. La carga de aplicacion sigue
// midiendo 32 bytes. COBS agrega sesion, solicitud y CRC16: 44 bytes en el bus.
namespace EnlaceRS485 {
// Perfil de los sketches ESP32_RS485_115200 y PORTENTA_RS485_115200
// comprobados por el usuario. Portenta usa retardos distintos del MAX485 ESP.
constexpr uint32_t BAUD = 115200;
constexpr uint32_t TIMEOUT_RESPUESTA_MS = 100; // Desde el ultimo bit de solicitud.
constexpr uint32_t TIMEOUT_FRAGMENTO_MS = 80;
constexpr uint32_t GIRO_BUS_US = 15000; // Espera del respondedor en el test funcional.
constexpr uint32_t PRE_TX_US = 200;
constexpr uint32_t POST_TX_US = 200;
constexpr uint32_t PORTENTA_PRE_TX_US = 0;
constexpr uint32_t PORTENTA_POST_TX_US = 2000;
// ESP retiene DE despues del ultimo byte. Portenta debe ceder el bus antes
// de iniciar otra solicitud, incluso si la respuesta ya fue validada.
constexpr uint32_t GIRO_SOLICITUD_MS = 3;
constexpr bool PORTENTA_TERMINACION = false; // Mismo valor que comm_protocols.init().
constexpr size_t CARGA = 32;
constexpr size_t CRUDO = 42;
constexpr size_t CODIFICADO = 43;
constexpr size_t TRAMA = 44;
constexpr uint32_t TIEMPO_TRAMA_MS = (TRAMA * 10UL * 1000UL + BAUD - 1) / BAUD;
static_assert(GIRO_BUS_US > PORTENTA_POST_TX_US,
              "ESP debe responder despues de que Portenta libere DE");
static_assert(GIRO_SOLICITUD_MS * 1000UL > POST_TX_US + 1000UL,
              "Portenta debe esperar la liberacion de DE y el redondeo de millis");
static_assert(TIMEOUT_RESPUESTA_MS > TIEMPO_TRAMA_MS +
              (GIRO_BUS_US + PRE_TX_US + POST_TX_US + 999UL) / 1000UL,
              "El plazo debe permitir recibir una respuesta completa");

struct Mensaje {
    uint32_t sesion;
    uint32_t solicitud;
    uint8_t datos[CARGA];
};

inline uint16_t crc16(const uint8_t *p, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t>(p[i]) << 8;
        for (uint8_t b = 0; b < 8; ++b)
            crc = static_cast<uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}
inline void escribir32(uint8_t *p, uint32_t v) {
    for (uint8_t i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline uint32_t leer32(const uint8_t *p) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < 4; ++i) v |= static_cast<uint32_t>(p[i]) << (8 * i);
    return v;
}
inline size_t codificar(const Mensaje &m, uint8_t *salida) {
    uint8_t crudo[CRUDO];
    escribir32(crudo, m.sesion);
    escribir32(crudo + 4, m.solicitud);
    memcpy(crudo + 8, m.datos, CARGA);
    const uint16_t crc = crc16(crudo, CRUDO - 2);
    crudo[40] = static_cast<uint8_t>(crc);
    crudo[41] = static_cast<uint8_t>(crc >> 8);
    size_t codigo = 0, escrito = 1;
    uint8_t cuenta = 1;
    for (size_t i = 0; i < CRUDO; ++i) {
        if (crudo[i] == 0) {
            salida[codigo] = cuenta;
            codigo = escrito++;
            cuenta = 1;
        } else {
            salida[escrito++] = crudo[i];
            ++cuenta;
        }
    }
    salida[codigo] = cuenta;
    salida[escrito++] = 0;
    return escrito;
}

// Un cero termina la trama. Tras ruido o desborde se recupera en el siguiente
// delimitador. No actualiza ninguna guarda de aplicacion por recibir bytes.
class Receptor {
 public:
    uint32_t bytes = 0, tramas = 0, crcIncorrecto = 0;
    uint32_t delimitadores = 0;
    uint32_t longitudIncorrecta = 0, fragmentos = 0;
    void reiniciar() { usados = 0; descartando = false; }
    void vencerFragmento(uint32_t ahora) {
        if (usados && ahora - ultimoByte >= TIMEOUT_FRAGMENTO_MS) {
            ++fragmentos;
            usados = 0;
            descartando = true;
        }
    }
    bool agregar(uint8_t dato, uint32_t ahora, Mensaje &m) {
        ++bytes;
        if (dato == 0) ++delimitadores;
        ultimoByte = ahora;
        if (dato != 0) {
            if (descartando) return false;
            if (usados == CODIFICADO) {
                ++longitudIncorrecta;
                usados = 0;
                descartando = true;
                return false;
            }
            buffer[usados++] = dato;
            return false;
        }
        if (descartando) { reiniciar(); return false; }
        if (!usados) return false;
        const size_t n = usados;
        usados = 0;
        uint8_t crudo[CRUDO];
        size_t leido = 0, escrito = 0;
        while (leido < n) {
            const uint8_t cuenta = buffer[leido++];
            if (!cuenta || leido + cuenta - 1 > n || escrito + cuenta - 1 > CRUDO) {
                ++longitudIncorrecta; return false;
            }
            for (uint8_t i = 1; i < cuenta; ++i) crudo[escrito++] = buffer[leido++];
            if (cuenta != 255 && leido < n) {
                if (escrito == CRUDO) { ++longitudIncorrecta; return false; }
                crudo[escrito++] = 0;
            }
        }
        if (escrito != CRUDO) { ++longitudIncorrecta; return false; }
        const uint16_t recibido = crudo[40] | (static_cast<uint16_t>(crudo[41]) << 8);
        if (crc16(crudo, CRUDO - 2) != recibido) { ++crcIncorrecto; return false; }
        m.sesion = leer32(crudo);
        m.solicitud = leer32(crudo + 4);
        memcpy(m.datos, crudo + 8, CARGA);
        ++tramas;
        return true;
    }
 private:
    uint8_t buffer[CODIFICADO] = {};
    size_t usados = 0;
    uint32_t ultimoByte = 0;
    bool descartando = false;
};

// Portenta: una unica solicitud pendiente. Solo una respuesta de la misma
// sesion/solicitud dentro de plazo puede renovar el enlace de seguridad.
class Cliente {
 public:
    bool pendiente = false;
    uint32_t sesion = 0, solicitud = 0, enviadaMs = 0;
    // Todo byte, incluido uno ajeno o corrupto, indica actividad del bus.
    // Esto no renueva ninguna guarda de seguridad ni confirma una respuesta.
    void registrarRecepcion(uint32_t ahora) {
        recepcionConocida = true;
        ultimoByteRecibidoMs = ahora;
    }
    bool puedeIniciar(uint32_t ahora) const {
        return !pendiente && (!recepcionConocida ||
            ahora - ultimoByteRecibidoMs >= GIRO_SOLICITUD_MS);
    }
    bool iniciar(uint32_t ahora) {
        if (!puedeIniciar(ahora)) return false;
        if (++solicitud == 0) ++solicitud;
        enviadaMs = ahora;
        pendiente = true;
        return true;
    }
    bool coincide(const Mensaje &m, uint32_t ahora) const {
        return pendiente && m.sesion == sesion && m.solicitud == solicitud &&
            ahora - enviadaMs < TIMEOUT_RESPUESTA_MS;
    }
    bool vencer(uint32_t ahora) {
        if (!pendiente || ahora - enviadaMs < TIMEOUT_RESPUESTA_MS) return false;
        pendiente = false;
        return true;
    }
    void confirmar() { pendiente = false; }
 private:
    bool recepcionConocida = false;
    uint32_t ultimoByteRecibidoMs = 0;
};

enum TipoSolicitud : uint8_t { DESCARTADA, NUEVA, DUPLICADA };
class Servidor {
 public:
    TipoSolicitud aceptar(const Mensaje &m) {
        if (!m.sesion || !m.solicitud) return DESCARTADA;
        if (!conocida || m.sesion != ultimaSesion ||
            static_cast<int32_t>(m.solicitud - ultimaSolicitud) > 0) {
            conocida = true;
            ultimaSesion = m.sesion;
            ultimaSolicitud = m.solicitud;
            memcpy(ultimosDatos, m.datos, CARGA);
            return NUEVA;
        }
        if (m.solicitud == ultimaSolicitud && memcmp(ultimosDatos, m.datos, CARGA) == 0)
            return DUPLICADA;
        return DESCARTADA;
    }
 private:
    bool conocida = false;
    uint32_t ultimaSesion = 0, ultimaSolicitud = 0;
    uint8_t ultimosDatos[CARGA] = {};
};
} // namespace EnlaceRS485

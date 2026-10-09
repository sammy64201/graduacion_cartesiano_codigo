#pragma once
#include "ProtocoloRS485.h"
#include "EnlaceRS485.h"
#include "ConfiguracionRS485.h"

// Fixtures exclusivos del banco: nunca cargar una sola placa con firmware real.
// Se usan los paquetes v19 de produccion, sin agregar ordenes del banco.
namespace FinalRS485 {
using namespace ProtocoloRS485;
constexpr int RX = ConfiguracionRS485::RX, TX = ConfiguracionRS485::TX, DE = ConfiguracionRS485::DE;
constexpr uint32_t LENTOS = 1000, RAPIDOS = 10000;
constexpr uint32_t VENTANA_FALLO_MS = 300;
constexpr uint32_t PERIODO_LENTOS_MS = 150, PERIODO_RAPIDOS_MS = 10;
constexpr uint32_t LIMITE_LENTOS_MS = 200000, LIMITE_RAPIDOS_MS = 1500000;
enum Caso : uint8_t { NORMAL, CRC16_MALO, VERSION_VIEJA, CRC8_MALO,
    TRUNCADO, RUIDO, DUPLICADO, MODIFICADO, ANTIGUO, AJENO, TARDIO, PERDIDO, NUEVA_SESION };
constexpr Caso CASOS[] = { NORMAL, CRC16_MALO, NORMAL, VERSION_VIEJA, NORMAL,
    CRC8_MALO, NORMAL, TRUNCADO, NORMAL, RUIDO, DUPLICADO, MODIFICADO,
    ANTIGUO, AJENO, NORMAL, TARDIO, NORMAL, PERDIDO, NORMAL, NUEVA_SESION };
constexpr size_t NUM_CASOS = sizeof(CASOS) / sizeof(CASOS[0]);
inline uint32_t numero(size_t paso) {
    if (paso == 10 || paso == 11) return 20010;
    if (paso == 12) return 20009;
    return 20001 + paso;
}
inline bool esperaRespuesta(Caso c) {
    return c == NORMAL || c == RUIDO || c == DUPLICADO || c == NUEVA_SESION;
}
inline PaquetePortentaAESP solicitud(uint32_t n) {
    PaquetePortentaAESP p = {};
    p.estadoSistema = SISTEMA_MODO_AUTOMATICO_V2;
    p.opcionMenu = MENU_MODO_AUTOMATICO_V2;
    p.flagsLimites = LIM_FLAG_COHERENTES;
    p.secuenciaComandoCamara = static_cast<uint8_t>(n);
    p.ackSecuenciaObjetivo = static_cast<uint16_t>(n);
    // Valores sinteticos: contador con signo, escala+Z y secuencia con wrap.
    p.conteoEncoder = (n & 1) ? -static_cast<int32_t>(n * 123) : n * 123;
    p.velocidadEncoderUmS = (n & 1) ? -125000 : 125000;
    p.nmPorCuentaEncoder = empacarEscalaEncoderYZ(50000, n % 16384);
    p.secuenciaEncoder = static_cast<uint16_t>(n * 13);
    p.estadoEncoder = ENC_FLAG_HW_LISTO | ENC_FLAG_ESCALA_VALIDA;
    p.signoEncoder = (n & 1) ? -1 : 1;
    prepararPaquete(p);
    return p;
}
inline PaqueteESPAPortenta respuesta(uint32_t n, uint16_t arranque) {
    PaqueteESPAPortenta p = {};
    p.secuenciaPaquete = static_cast<uint8_t>(n);
    p.sesionArranque = arranque;
    p.flags = ESP_FLAG_BASE_LISTA;
    p.joystickX = static_cast<int8_t>(static_cast<int>(n % 201) - 100);
    p.joystickY = -p.joystickX;
    p.joystickZ = (n & 1) ? -100 : 100;
    p.botones = n % 16;
    p.servoRotacion = n % 181;
    p.servoPinza = (n & 1) ? 60 : 120;
    p.estadoCamara = CAMARA_STANDBY;
    p.ackSecuenciaComandoCamara = static_cast<uint8_t>(n);
    uint8_t tags[4] = {static_cast<uint8_t>(n % 32), 0, 15, 31};
    empacarMuestrasTags(tags, p.muestrasTagEmpacadas);
    p.claseObjetivo = (n & 1) ? 6 : 7;
    p.objetivoX10 = static_cast<int16_t>(static_cast<int>(n % 6001) - 3000);
    p.objetivoY10 = -p.objetivoX10;
    p.secuenciaObjetivo = static_cast<uint16_t>(n);
    p.conteoReferenciaObjetivo = solicitud(n).conteoEncoder;
    prepararPaquete(p);
    return p;
}
inline size_t crearTrama(const EnlaceRS485::Mensaje &original, Caso c, uint8_t *trama) {
    EnlaceRS485::Mensaje m = original;
    PaquetePortentaAESP p;
    memcpy(&p, m.datos, sizeof(p));
    if (c == VERSION_VIEJA) { p.version = VERSION_PROTOCOLO - 1; p.checksum = calcularChecksumPaquete(p); }
    if (c == CRC8_MALO) p.checksum ^= 1;
    if (c == MODIFICADO) { ++p.conteoEncoder; prepararPaquete(p); }
    memcpy(m.datos, &p, sizeof(p));
    size_t n = EnlaceRS485::codificar(m, trama);
    if (c == CRC16_MALO) trama[n - 2] ^= 0x80; // CRC externo, longitud intacta.
    if (c == TRUNCADO) n = 12; // Sin delimitador: vence el fragmento.
    return n;
}
class BancoESP {
 public:
    EnlaceRS485::Servidor servidor;
    uint32_t aplicadas = 0, duplicadas = 0, descartadas = 0, invalidas = 0;
    bool aceptar(const EnlaceRS485::Mensaje &m) {
        PaquetePortentaAESP p; memcpy(&p, m.datos, sizeof(p));
        if (!validarPaquete(p) || !movimientosValidos(p.movimientos)) { ++invalidas; return false; }
        const auto tipo = servidor.aceptar(m);
        if (tipo == EnlaceRS485::DESCARTADA) { ++descartadas; return false; }
        if (tipo == EnlaceRS485::DUPLICADA) ++duplicadas;
        else ++aplicadas;
        return true;
    }
};
} // namespace FinalRS485

#pragma once

#include <stddef.h>
#include <stdint.h>

// Protocolo binario compartido por la Portenta H7 (maestra) y la ESP32
// (esclava). Esta cabecera debe mantenerse identica en ambos proyectos.
//
// Reglas de transporte:
// - Todos los enteros multibyte se transmiten en little-endian.
// - No se transmiten bool, enum, float, punteros ni padding del compilador.
// - El checksum es CRC-8/ATM: poly=0x07, init=0x00, refin/refout=false,
//   xorout=0x00. Cubre todos los bytes salvo el ultimo campo checksum.
// - EnlaceRS485.h transporta los 32 bytes con COBS, sesion, solicitud y CRC16.
// - Portenta coordina el par half duplex; ESP responde a cada solicitud valida.

namespace ProtocoloRS485 {

constexpr uint8_t VERSION_PROTOCOLO = 17;
constexpr uint8_t MAGIC_ESP_A_PORTENTA = 0xE3;
constexpr uint8_t MAGIC_PORTENTA_A_ESP = 0xA7;
constexpr size_t MAX_BYTES_WIRE = 32;
// Separacion de Z sobre DIN04 mientras la pieza se aproxima al catch.
constexpr long Z_MARGEN_PRECAPTURA_PASOS = 3000L;
// El paquete conserva su formato de 32 bytes: los 18 bits bajos conservan la
// escala del encoder (nm/cuenta) y los 14 altos llevan Z desde DIN04 (pasos).
constexpr uint32_t MASCARA_ESCALA_ENCODER_NM = 0x0003FFFFUL;
constexpr uint16_t MAX_Z_DESDE_DIN04_PASOS = 0x3FFFU;
constexpr uint8_t DESPLAZAMIENTO_Z_DESDE_DIN04 = 18;

inline uint32_t empacarEscalaEncoderYZ(uint32_t nmPorCuenta, uint16_t pasosZ) {
    return (nmPorCuenta & MASCARA_ESCALA_ENCODER_NM) |
        (static_cast<uint32_t>(pasosZ & MAX_Z_DESDE_DIN04_PASOS)
         << DESPLAZAMIENTO_Z_DESDE_DIN04);
}

inline uint32_t extraerEscalaEncoderNm(uint32_t campo) {
    return campo & MASCARA_ESCALA_ENCODER_NM;
}

inline uint16_t extraerZDesdeDin04Pasos(uint32_t campo) {
    return static_cast<uint16_t>(campo >> DESPLAZAMIENTO_Z_DESDE_DIN04);
}

enum EstadoSistemaWire : uint8_t {
    SISTEMA_ARRANQUE_SEGURO = 0,
    SISTEMA_ESPERANDO_RS485 = 1,
    SISTEMA_ESPERA_5S = 2,
    SISTEMA_CALIBRANDO_CAMARA = 3,
    SISTEMA_CALIBRANDO_BRAZO = 4,
    SISTEMA_ESPERANDO_CONTROL = 5,
    SISTEMA_CHECKLIST = 6,
    SISTEMA_MENU_PRINCIPAL = 7,
    SISTEMA_MODO_MANUAL = 8,
    SISTEMA_MODO_AUTOMATICO = 9,
    SISTEMA_ERROR = 10,
    SISTEMA_MODO_AUTOMATICO_V2 = 11,
    SISTEMA_CALIBRANDO_ENCODER = 12,
    SISTEMA_MENU_CALIBRACIONES = 13,
    SISTEMA_PRUEBA_SERVOS = 14,
    SISTEMA_DIAGNOSTICO = 15,
    SISTEMA_ENTRENAMIENTO_ML = 16,
    SISTEMA_PRUEBA_ENCODER = 17,
    SISTEMA_CAMBIOS_CATCH = 18
};

enum OpcionMenuWire : uint8_t {
    MENU_MODO_MANUAL = 0,
    MENU_MODO_AUTOMATICO = 1,
    MENU_CALIBRACION_BRAZO = 2,
    MENU_CALIBRACION_CAMARA = 3,
    MENU_MODO_AUTOMATICO_V2 = 4,
    MENU_CALIBRACIONES = 5,
    MENU_PRUEBA_SERVOS = 6,
    MENU_DIAGNOSTICO = 7,
    MENU_ENTRENAMIENTO_ML = 8,
    MENU_PRUEBA_ENCODER = 9,
    MENU_REGISTRO_ANGULO = 10,
    MENU_ENTRENAMIENTO_ML_V2 = 11,
    MENU_PRUEBA_SEGUIMIENTO = 12,
    MENU_AJUSTE_CATCH_V2 = 13,
    MENU_CAMBIOS_CATCH = 14
};

enum FlagsEncoder : uint8_t {
    ENC_FLAG_HW_LISTO = 1U << 0,
    ENC_FLAG_ESCALA_VALIDA = 1U << 1,
    ENC_FLAG_PULSOS_VISTOS = 1U << 2,
    ENC_FLAG_EN_MOVIMIENTO = 1U << 3,
    ENC_FLAG_DIRECCION_POSITIVA = 1U << 4,
    ENC_FLAG_SATURADO = 1U << 5
};

enum EstadoCamara : uint8_t {
    CAMARA_OFFLINE = 0,
    CAMARA_CONECTANDO = 1,
    CAMARA_STANDBY = 2,
    CAMARA_ABRIENDO_TAGS = 3,
    CAMARA_CALIBRANDO = 4,
    CAMARA_CALCULANDO = 5,
    CAMARA_ABRIENDO_MODELO = 6,
    CAMARA_LISTA = 7,
    CAMARA_ERROR = 8
};

enum ComandoCamara : uint8_t {
    CAM_CMD_NINGUNO = 0,
    CAM_CMD_STANDBY = 1,
    CAM_CMD_CALIBRAR = 2,
    CAM_CMD_ABRIR_MODELO = 3,
    CAM_CMD_REINICIAR_ERROR = 4
};

enum ErrorCamaraWire : uint8_t {
    CAM_ERROR_NINGUNO = 0,
    CAM_ERROR_SIN_RESPUESTA = 1,
    CAM_ERROR_ABRIR_TAGS = 2,
    CAM_ERROR_TIMEOUT_TAGS = 3,
    CAM_ERROR_HOMOGRAFIA = 4,
    CAM_ERROR_ABRIR_MODELO = 5,
    CAM_ERROR_LECTURA = 6,
    CAM_ERROR_COMANDO_INVALIDO = 7
};

enum ErrorSistemaWire : uint8_t {
    SISTEMA_ERROR_NINGUNO = 0,
    SISTEMA_ERROR_RS485 = 1,
    SISTEMA_ERROR_CAMARA = 2,
    SISTEMA_ERROR_CALIBRACION_BRAZO = 3,
    SISTEMA_ERROR_CHECKLIST = 4,
    SISTEMA_ERROR_OBJETIVO_FUERA_RANGO = 5,
    SISTEMA_ERROR_TIMEOUT_MOVIMIENTO = 6,
    SISTEMA_ERROR_FINALES_INCOHERENTES = 7,
    SISTEMA_ERROR_CANCELADO = 8
};

enum FaseCalibracionBrazoWire : uint8_t {
    BRAZO_CAL_ESPERA = 0,
    BRAZO_CAL_X_MIN_1 = 1,
    BRAZO_CAL_X_MIN_LIBERAR = 2,
    BRAZO_CAL_X_MIN_SEPARAR = 3,
    BRAZO_CAL_X_MIN_2 = 4,
    BRAZO_CAL_X_MAX_1 = 5,
    BRAZO_CAL_X_MAX_LIBERAR = 6,
    BRAZO_CAL_X_MAX_SEPARAR = 7,
    BRAZO_CAL_X_MAX_2 = 8,
    BRAZO_CAL_Y_MIN_1 = 9,
    BRAZO_CAL_Y_MIN_LIBERAR = 10,
    BRAZO_CAL_Y_MIN_SEPARAR = 11,
    BRAZO_CAL_Y_MIN_2 = 12,
    BRAZO_CAL_Y_MAX_1 = 13,
    BRAZO_CAL_Y_MAX_LIBERAR = 14,
    BRAZO_CAL_Y_MAX_SEPARAR = 15,
    BRAZO_CAL_Y_MAX_2 = 16,
    BRAZO_CAL_Z_MIN_1 = 17,
    BRAZO_CAL_Z_MIN_LIBERAR = 18,
    BRAZO_CAL_Z_MIN_SEPARAR = 19,
    BRAZO_CAL_Z_MIN_2 = 20,
    BRAZO_CAL_Z_MAX_1 = 21,
    BRAZO_CAL_Z_MAX_LIBERAR = 22,
    BRAZO_CAL_Z_MAX_SEPARAR = 23,
    BRAZO_CAL_Z_MAX_2 = 24,
    BRAZO_CAL_HOME = 25,
    BRAZO_CAL_COMPLETA = 26,
    BRAZO_CAL_ERROR = 27
};

enum FlagsESP : uint8_t {
    ESP_FLAG_BASE_LISTA = 1U << 0,
    ESP_FLAG_BT_CONECTADO = 1U << 1,
    ESP_FLAG_OLED_LISTA = 1U << 2,
    ESP_FLAG_CAMARA_CONECTADA = 1U << 3,
    ESP_FLAG_HOMOGRAFIA_VALIDA = 1U << 4,
    ESP_FLAG_MODELO_LISTO = 1U << 5,
    ESP_FLAG_OBJETIVO_VALIDO = 1U << 6,
    ESP_FLAG_CAMARA_OCUPADA = 1U << 7
};

enum FlagsBotones : uint8_t {
    BOTON_X = 1U << 0,
    BOTON_TRIANGULO = 1U << 1,
    BOTON_CIRCULO = 1U << 2,
    BOTON_CUADRADO = 1U << 3
};

enum FlagsSistema : uint8_t {
    SIS_FLAG_XY_CALIBRADO = 1U << 0,
    SIS_FLAG_Z_CALIBRADO = 1U << 1,
    SIS_FLAG_EN_HOME = 1U << 2,
    SIS_FLAG_AUTO_ACTIVO = 1U << 3,
    SIS_FLAG_BRAZO_OCUPADO = 1U << 4,
    SIS_FLAG_ERROR_CRITICO = 1U << 5,
    SIS_FLAG_ENCODER_CALIBRADO = 1U << 6,
    SIS_FLAG_MOTORES_HABILITADOS = 1U << 7
};

enum FlagsLimites : uint8_t {
    LIM_FLAG_X_MAS = 1U << 0,
    LIM_FLAG_X_MENOS = 1U << 1,
    LIM_FLAG_Y_MAS = 1U << 2,
    LIM_FLAG_Y_MENOS = 1U << 3,
    LIM_FLAG_Z_ARRIBA = 1U << 4,
    LIM_FLAG_Z_ABAJO = 1U << 5,
    LIM_FLAG_COHERENTES = 1U << 6
};

enum CodigoAckObjetivo : uint8_t {
    ACK_OBJ_NINGUNO = 0,
    ACK_OBJ_ACEPTADO = 1,
    ACK_OBJ_RECHAZADO_RANGO = 2,
    ACK_OBJ_CANCELADO = 3,
    ACK_OBJ_COMPLETADO = 4,
    // En Automatico V2 esta orden conserva la misma secuencia del objetivo y
    // manda cerrar fisicamente la pinza manteniendo reservado el objetivo.
    ACK_OBJ_CERRAR_PINZA = 5,
    // Apertura de entrega sin liberar el objetivo antes de la retirada final.
    ACK_OBJ_ABRIR_PINZA = 6
};

enum CodigoMovimientoEje : uint8_t {
    MOVIMIENTO_DETENIDO = 0,
    MOVIMIENTO_POSITIVO = 1,
    MOVIMIENTO_NEGATIVO = 2,
    MOVIMIENTO_INVALIDO = 3
};

constexpr uint8_t MOV_SHIFT_X = 0;
constexpr uint8_t MOV_SHIFT_Y = 2;
constexpr uint8_t MOV_SHIFT_Z = 4;
constexpr uint8_t MOV_MASK_EJE = 0x03;


struct __attribute__((packed)) PaqueteESPAPortenta {
    uint8_t magic;
    uint8_t version;
    uint8_t longitud;
    uint8_t secuenciaPaquete;
    uint16_t sesionArranque;
    uint8_t flags;
    int8_t joystickX;
    int8_t joystickY;
    int8_t joystickZ;
    uint8_t botones;
    uint8_t servoRotacion;
    uint8_t servoPinza;
    uint8_t estadoCamara;
    uint8_t errorCamara;
    uint8_t ackSecuenciaComandoCamara;
    uint8_t muestrasTagEmpacadas[3];
    uint8_t reservadoV2;
    uint8_t claseObjetivo;
    int16_t objetivoX10;
    int16_t objetivoY10;
    uint16_t secuenciaObjetivo;
    int32_t conteoReferenciaObjetivo;
    uint8_t checksum;
};

struct __attribute__((packed)) PaquetePortentaAESP {
    uint8_t magic;
    uint8_t version;
    uint8_t longitud;
    uint8_t estadoSistema;
    uint8_t opcionMenu;
    uint8_t faseCalibracionBrazo;
    uint8_t flagsSistema;
    uint8_t flagsLimites;
    uint8_t movimientos;
    uint8_t errorSistema;
    uint8_t comandoCamara;
    uint8_t secuenciaComandoCamara;
    uint16_t ackSecuenciaObjetivo;
    uint8_t codigoAckObjetivo;
    // SISTEMA_CAMBIOS_CATCH: conteo=proximo offset ms, velocidad=ultimo probado ms,
    // nmPorCuenta=paso ms, secuencia=ensayos, estado=confirmado bit0/limite bit1/
    // agarres bits2..3, fase=pagina. No publicar estos campos como encoder real.
    int32_t conteoEncoder;
    int32_t velocidadEncoderUmS;
    uint32_t nmPorCuentaEncoder; // escala 18 bits + Z desde DIN04 14 bits
    uint16_t secuenciaEncoder;
    uint8_t estadoEncoder;
    int8_t signoEncoder;
    uint8_t checksum;
};

constexpr size_t TAMANO_ESP_A_PORTENTA = sizeof(PaqueteESPAPortenta);
constexpr size_t TAMANO_PORTENTA_A_ESP = sizeof(PaquetePortentaAESP);

static_assert(sizeof(uint8_t) == 1, "El protocolo requiere uint8_t de 1 byte");
static_assert(sizeof(uint16_t) == 2, "El protocolo requiere uint16_t de 2 bytes");
static_assert(sizeof(int16_t) == 2, "El protocolo requiere int16_t de 2 bytes");
static_assert(sizeof(int32_t) == 4, "El protocolo requiere int32_t de 4 bytes");
static_assert(sizeof(PaqueteESPAPortenta) == 32,
              "PaqueteESPAPortenta debe medir exactamente 32 bytes");
static_assert(sizeof(PaquetePortentaAESP) == 32,
              "PaquetePortentaAESP debe medir exactamente 32 bytes");
static_assert(sizeof(PaqueteESPAPortenta) <= MAX_BYTES_WIRE,
              "PaqueteESPAPortenta excede el limite Wire");
static_assert(sizeof(PaquetePortentaAESP) <= MAX_BYTES_WIRE,
              "PaquetePortentaAESP excede el limite Wire");
static_assert(offsetof(PaqueteESPAPortenta, checksum) == 31,
              "checksum ESP->Portenta debe ser el ultimo byte");
static_assert(offsetof(PaquetePortentaAESP, checksum) == 31,
              "checksum Portenta->ESP debe ser el ultimo byte");

inline uint8_t calcularCRC8ATM(const uint8_t *datos, size_t longitud) {
    uint8_t crc = 0x00;

    for (size_t i = 0; i < longitud; ++i) {
        crc ^= datos[i];

        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U)
                ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
                : static_cast<uint8_t>(crc << 1U);
        }
    }

    return crc;
}

inline void empacarMuestrasTags(
    const uint8_t muestras[4],
    uint8_t destino[3]
) {
    uint32_t valor = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint8_t muestra = muestras[i] > 31U ? 31U : muestras[i];
        valor |= static_cast<uint32_t>(muestra) << (i * 5U);
    }
    destino[0] = static_cast<uint8_t>(valor & 0xFFU);
    destino[1] = static_cast<uint8_t>((valor >> 8U) & 0xFFU);
    destino[2] = static_cast<uint8_t>((valor >> 16U) & 0x0FU);
}

inline void desempacarMuestrasTags(
    const uint8_t origen[3],
    uint8_t muestras[4]
) {
    const uint32_t valor = static_cast<uint32_t>(origen[0]) |
        (static_cast<uint32_t>(origen[1]) << 8U) |
        (static_cast<uint32_t>(origen[2] & 0x0FU) << 16U);
    for (uint8_t i = 0; i < 4; ++i) {
        muestras[i] = static_cast<uint8_t>((valor >> (i * 5U)) & 0x1FU);
    }
}

inline int32_t diferenciaConteosConWrap(
    int32_t actual,
    int32_t referencia
) {
    return static_cast<int32_t>(
        static_cast<uint32_t>(actual) - static_cast<uint32_t>(referencia)
    );
}

template <typename Paquete>
inline uint8_t calcularChecksumPaquete(const Paquete &paquete) {
    return calcularCRC8ATM(
        reinterpret_cast<const uint8_t *>(&paquete),
        sizeof(Paquete) - 1U
    );
}

inline void prepararPaquete(PaqueteESPAPortenta &paquete) {
    paquete.magic = MAGIC_ESP_A_PORTENTA;
    paquete.version = VERSION_PROTOCOLO;
    paquete.longitud = static_cast<uint8_t>(sizeof(paquete));
    paquete.checksum = calcularChecksumPaquete(paquete);
}

inline void prepararPaquete(PaquetePortentaAESP &paquete) {
    paquete.magic = MAGIC_PORTENTA_A_ESP;
    paquete.version = VERSION_PROTOCOLO;
    paquete.longitud = static_cast<uint8_t>(sizeof(paquete));
    paquete.checksum = calcularChecksumPaquete(paquete);
}

inline bool validarPaquete(const PaqueteESPAPortenta &paquete) {
    return paquete.magic == MAGIC_ESP_A_PORTENTA &&
           paquete.version == VERSION_PROTOCOLO &&
           paquete.longitud == sizeof(paquete) &&
           paquete.checksum == calcularChecksumPaquete(paquete);
}

inline bool validarPaquete(const PaquetePortentaAESP &paquete) {
    return paquete.magic == MAGIC_PORTENTA_A_ESP &&
           paquete.version == VERSION_PROTOCOLO &&
           paquete.longitud == sizeof(paquete) &&
           paquete.checksum == calcularChecksumPaquete(paquete);
}

inline uint8_t codificarDireccion(int8_t direccion) {
    if (direccion > 0) return MOVIMIENTO_POSITIVO;
    if (direccion < 0) return MOVIMIENTO_NEGATIVO;
    return MOVIMIENTO_DETENIDO;
}

inline uint8_t codificarMovimientos(int8_t x, int8_t y, int8_t z) {
    return static_cast<uint8_t>(
        (codificarDireccion(x) << MOV_SHIFT_X) |
        (codificarDireccion(y) << MOV_SHIFT_Y) |
        (codificarDireccion(z) << MOV_SHIFT_Z)
    );
}

inline int8_t decodificarMovimiento(uint8_t movimientos, uint8_t shift) {
    const uint8_t codigo =
        static_cast<uint8_t>((movimientos >> shift) & MOV_MASK_EJE);
    if (codigo == MOVIMIENTO_POSITIVO) return 1;
    if (codigo == MOVIMIENTO_NEGATIVO) return -1;
    return 0;
}

inline int8_t decodificarMovimientoX(uint8_t movimientos) {
    return decodificarMovimiento(movimientos, MOV_SHIFT_X);
}

inline int8_t decodificarMovimientoY(uint8_t movimientos) {
    return decodificarMovimiento(movimientos, MOV_SHIFT_Y);
}

inline int8_t decodificarMovimientoZ(uint8_t movimientos) {
    return decodificarMovimiento(movimientos, MOV_SHIFT_Z);
}

inline bool movimientosValidos(uint8_t movimientos) {
    const uint8_t x = (movimientos >> MOV_SHIFT_X) & MOV_MASK_EJE;
    const uint8_t y = (movimientos >> MOV_SHIFT_Y) & MOV_MASK_EJE;
    const uint8_t z = (movimientos >> MOV_SHIFT_Z) & MOV_MASK_EJE;
    return (movimientos & 0xC0U) == 0U &&
           x != MOVIMIENTO_INVALIDO &&
           y != MOVIMIENTO_INVALIDO &&
           z != MOVIMIENTO_INVALIDO;
}

}  // namespace ProtocoloRS485

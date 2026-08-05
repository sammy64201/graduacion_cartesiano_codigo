#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../ESP/ProtocoloI2C.h"

using namespace ProtocoloI2C;

static void probarVectorCRCConocido() {
    static const uint8_t datos[] = {
        '1', '2', '3', '4', '5', '6', '7', '8', '9'
    };

    // Check value canonico de CRC-8/ATM (CRC-8/SMBUS).
    assert(calcularCRC8ATM(datos, sizeof(datos)) == 0xF4);
}

static void probarPaqueteESPAPortenta() {
    PaqueteESPAPortenta paquete;
    std::memset(&paquete, 0, sizeof(paquete));

    paquete.secuenciaPaquete = 7;
    paquete.sesionArranque = 0x1234;
    paquete.flags = ESP_FLAG_BASE_LISTA |
                    ESP_FLAG_CAMARA_CONECTADA |
                    ESP_FLAG_OBJETIVO_VALIDO;
    paquete.joystickX = -1;
    paquete.joystickY = 0;
    paquete.joystickZ = 1;
    paquete.botones = BOTON_X;
    paquete.servoRotacion = 90;
    paquete.servoPinza = 45;
    paquete.estadoCamara = CAMARA_LISTA;
    const uint8_t muestras[4] = {25, 24, 23, 22};
    empacarMuestrasTags(muestras, paquete.muestrasTagEmpacadas);
    paquete.estadoEncoder = ENC_FLAG_HW_LISTO |
                            ENC_FLAG_ESCALA_VALIDA |
                            ENC_FLAG_PULSOS_VISTOS;
    paquete.claseObjetivo = 6;
    paquete.objetivoX10 = 234;
    paquete.objetivoY10 = -158;
    paquete.secuenciaObjetivo = 42;
    paquete.conteoEncoder = 123456;

    prepararPaquete(paquete);

    assert(paquete.magic == MAGIC_ESP_A_PORTENTA);
    assert(paquete.version == VERSION_PROTOCOLO);
    assert(paquete.longitud == 32);
    assert(validarPaquete(paquete));

    uint8_t recuperadas[4] = {};
    desempacarMuestrasTags(paquete.muestrasTagEmpacadas, recuperadas);
    assert(std::memcmp(muestras, recuperadas, sizeof(muestras)) == 0);

    paquete.objetivoX10 ^= 1;
    assert(!validarPaquete(paquete));
}

static void probarPaquetePortentaAESP() {
    PaquetePortentaAESP paquete;
    std::memset(&paquete, 0, sizeof(paquete));

    paquete.estadoSistema = SISTEMA_MODO_AUTOMATICO;
    paquete.opcionMenu = MENU_MODO_AUTOMATICO;
    paquete.flagsSistema = SIS_FLAG_XY_CALIBRADO |
                           SIS_FLAG_Z_CALIBRADO |
                           SIS_FLAG_AUTO_ACTIVO;
    paquete.flagsLimites = LIM_FLAG_COHERENTES;
    paquete.movimientos = codificarMovimientos(-1, 0, 1);
    paquete.comandoCamara = CAM_CMD_NINGUNO;
    paquete.ackSecuenciaObjetivo = 42;
    paquete.codigoAckObjetivo = ACK_OBJ_ACEPTADO;
    paquete.valorPantalla1 = -1234;
    paquete.valorPantalla2 = 5678;

    prepararPaquete(paquete);

    assert(paquete.magic == MAGIC_PORTENTA_A_ESP);
    assert(paquete.version == VERSION_PROTOCOLO);
    assert(paquete.longitud == 32);
    assert(validarPaquete(paquete));
    assert(movimientosValidos(paquete.movimientos));
    assert(decodificarMovimientoX(paquete.movimientos) == -1);
    assert(decodificarMovimientoY(paquete.movimientos) == 0);
    assert(decodificarMovimientoZ(paquete.movimientos) == 1);

    paquete.valorPantalla2 ^= 1;
    assert(!validarPaquete(paquete));
}

static void probarCompatibilidadAutomaticoV2() {
    static_assert(MENU_MODO_MANUAL == 0, "menu manual preservado");
    static_assert(MENU_MODO_AUTOMATICO == 1, "menu automatico preservado");
    static_assert(MENU_CALIBRACION_BRAZO == 2, "menu brazo preservado");
    static_assert(MENU_CALIBRACION_CAMARA == 3, "menu camara preservado");
    static_assert(MENU_MODO_AUTOMATICO_V2 == 4, "V2 se agrega al final");
    static_assert(SISTEMA_ERROR == 10, "codigo de error preservado");
    static_assert(SISTEMA_MODO_AUTOMATICO_V2 == 11, "estado V2 nuevo");

    assert(diferenciaConteosConWrap(120, 100) == 20);
    assert(diferenciaConteosConWrap(-100, -120) == 20);
    assert(diferenciaConteosConWrap(INT32_MIN, INT32_MAX) == 1);
    assert(diferenciaConteosConWrap(INT32_MAX, INT32_MIN) == -1);
}

int main() {
    static_assert(sizeof(PaqueteESPAPortenta) == 32, "layout ESP->Portenta");
    static_assert(sizeof(PaquetePortentaAESP) == 32, "layout Portenta->ESP");
    static_assert(offsetof(PaqueteESPAPortenta, checksum) == 31,
                  "checksum final ESP->Portenta");
    static_assert(offsetof(PaquetePortentaAESP, checksum) == 31,
                  "checksum final Portenta->ESP");

    probarVectorCRCConocido();
    probarPaqueteESPAPortenta();
    probarPaquetePortentaAESP();
    probarCompatibilidadAutomaticoV2();
    return 0;
}

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
    paquete.muestrasTag[0] = 25;
    paquete.muestrasTag[1] = 25;
    paquete.muestrasTag[2] = 25;
    paquete.muestrasTag[3] = 25;
    paquete.claseObjetivo = 6;
    paquete.objetivoX10 = 234;
    paquete.objetivoY10 = -158;
    paquete.secuenciaObjetivo = 42;

    prepararPaquete(paquete);

    assert(paquete.magic == MAGIC_ESP_A_PORTENTA);
    assert(paquete.version == VERSION_PROTOCOLO);
    assert(paquete.longitud == 28);
    assert(validarPaquete(paquete));

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

int main() {
    static_assert(sizeof(PaqueteESPAPortenta) == 28, "layout ESP->Portenta");
    static_assert(sizeof(PaquetePortentaAESP) == 32, "layout Portenta->ESP");
    static_assert(offsetof(PaqueteESPAPortenta, checksum) == 27,
                  "checksum final ESP->Portenta");
    static_assert(offsetof(PaquetePortentaAESP, checksum) == 31,
                  "checksum final Portenta->ESP");

    probarVectorCRCConocido();
    probarPaqueteESPAPortenta();
    probarPaquetePortentaAESP();
    return 0;
}


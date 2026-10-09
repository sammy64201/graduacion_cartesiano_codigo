// Ejecuta ambos sketches completos del banco con UART y reloj simulados.
#include "rs485_final_mocks/Arduino_MachineControl.h"
#include "RS485_FINAL_MIGRACION/ESP32/FinalRS485.h"
#include <cassert>
#include <iostream>
uint32_t ahoraSimulado = 0;
uint32_t fraccionMicrosegundos = 0;
bool deESP = false;
uint64_t inicioTxESP = 0, finTxESP = 0;
bool flushESP = false;
std::deque<uint8_t> entradaESP, entradaPortenta;
namespace machinecontrol { Comunicacion comm_protocols; }
namespace ESPFixture {
Monitor Serial;
#include "RS485_FINAL_MIGRACION/ESP32/ESP32.ino"
}
namespace PortentaFixture {
Monitor Serial;
#include "RS485_FINAL_MIGRACION/PORTENTA/PORTENTA.ino"
}
unsigned recepcionesConcurrentes=0, confirmacionesConcurrentes=0;
void intercalarPortentaDurantePostTX() {
    assert(deESP && flushESP && tiempoMicrosegundos()==finTxESP);
    PortentaFixture::loop();
    ++recepcionesConcurrentes;
    if (!PortentaFixture::cliente.pendiente) ++confirmacionesConcurrentes;
    assert(deESP && machinecontrol::comm_protocols.rs485.rx);
    assert(!machinecontrol::comm_protocols.rs485.tx);
    assert(!PortentaFixture::cliente.puedeIniciar(millis()));
}
int main(int argc, char **) {
    const bool desconectado = argc > 1;
    ESPFixture::setup(); PortentaFixture::setup();
    eventoFinTramaESP()=intercalarPortentaDurantePostTX;
    assert(ESPFixture::enlace.baud == 115200);
    assert(machinecontrol::comm_protocols.rs485.baud == 115200);
    assert(machinecontrol::comm_protocols.rs485.pre == 0 && machinecontrol::comm_protocols.rs485.post == 2000);
    assert(machinecontrol::comm_protocols.rs485.config == SERIAL_8N1);
    assert(machinecontrol::comm_protocols.iniciada && machinecontrol::comm_protocols.habilitada);
    assert(!machinecontrol::comm_protocols.rs232 && !machinecontrol::comm_protocols.fullDuplex);
    assert(!machinecontrol::comm_protocols.terminacion);
    // El banco no transmite hasta T.
    for (unsigned i = 0; i < 100; ++i) { ++ahoraSimulado; ESPFixture::loop(); PortentaFixture::loop(); }
    assert(PortentaFixture::intentos == 0 && ESPFixture::txOK == 0 && !deESP);
    PortentaFixture::Serial.entrada.push_back('T');
    while (!PortentaFixture::terminado && ahoraSimulado < 2500000) {
        PortentaFixture::loop();
        if (!desconectado) ESPFixture::loop();
        else entradaESP.clear();
        assert(!deESP && !machinecontrol::comm_protocols.rs485.tx && machinecontrol::comm_protocols.rs485.rx);
        ++ahoraSimulado;
    }
    assert(PortentaFixture::terminado);
    const std::string log = PortentaFixture::Serial.salida.str();
    if (desconectado) {
        assert(PortentaFixture::fallos > 0 && log.find("RESULTADO=FAIL") != std::string::npos);
        std::cout << "PASS: bus desconectado nunca obtiene aprobacion\n";
        return 0;
    }
    if (PortentaFixture::fallos) std::cerr << log;
    assert(PortentaFixture::fallos == 0);
    assert(log.find("RESULTADO=PASS") != std::string::npos);
    assert(ESPFixture::banco.aplicadas == 11013);
    assert(ESPFixture::banco.duplicadas == 1 && ESPFixture::banco.descartadas == 2);
    assert(ESPFixture::banco.invalidas == 2);
    assert(ESPFixture::receptor.crcIncorrecto == 1);
    assert(ESPFixture::receptor.longitudIncorrecta == 1);
    assert(ESPFixture::receptor.fragmentos == 1);
    assert(ESPFixture::txError == 0);
    assert(recepcionesConcurrentes>=11000 && confirmacionesConcurrentes>=11000);
    std::cout << "PASS: ambos sketches reales a 115200, 11000 intercambios con RX durante DE ESP activo, 20 casos, duplicado sin reaplicar, recuperacion y DE 200/200 us\n";
}

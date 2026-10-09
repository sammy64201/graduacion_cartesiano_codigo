#include <cassert>
#include <cstdio>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/tests/rs485_final_mocks/Arduino_MachineControl.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/ESP/ProtocoloRS485.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/ESP/EnlaceRS485.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/ESP/ConfiguracionRS485.h"

using namespace ProtocoloRS485;
uint32_t ahoraSimulado=1000, fraccionMicrosegundos=0;
bool deESP=false, flushESP=false;
uint64_t inicioTxESP=0, finTxESP=0;
std::deque<uint8_t> entradaESP, entradaPortenta;
namespace machinecontrol { Comunicacion comm_protocols; }
using namespace machinecontrol;
Monitor Serial;
EnlaceRS485::Cliente clienteRS485;
EnlaceRS485::Receptor receptorRS485;
bool iniciarRS485Maestro() {
    if (!clienteRS485.sesion) {
        trng_t rng;
        trng_init(&rng);
        size_t cantidad = 0;
        uint32_t sesion = 0;
        const int error = trng_get_bytes(&rng, reinterpret_cast<uint8_t *>(&sesion), sizeof(sesion), &cantidad);
        trng_free(&rng);
        if (error != 0 || cantidad != sizeof(sesion) || !sesion) {
            Serial.println(F("[RS485][ERROR] No se obtuvo sesion aleatoria; enlace detenido"));
            return false;
        }
        clienteRS485.sesion = sesion;
    }
    comm_protocols.init();
    comm_protocols.rs485ModeRS232(false);
    comm_protocols.rs485FullDuplex(false);
    comm_protocols.rs485ABTerm(EnlaceRS485::PORTENTA_TERMINACION);
    comm_protocols.rs485Enable(true);
    // Sobrecarga (baud, pre, post), 8N1 implicito: igual al test a 115200.
    comm_protocols.rs485.begin(EnlaceRS485::BAUD,
                              EnlaceRS485::PORTENTA_PRE_TX_US,
                              EnlaceRS485::PORTENTA_POST_TX_US);
    comm_protocols.rs485.receive();
    clienteRS485.confirmar();
    receptorRS485.reiniciar();
    Serial.print(F("[RS485] Half duplex TX P/TX N; "));
    Serial.print(EnlaceRS485::BAUD);
    Serial.print(F(" 8N1; protocolo 17; sesion="));
    Serial.println(clienteRS485.sesion);
    return true;
}

namespace Esclavo {
HardwareSerial PuertoRS485(2);
EnlaceRS485::Receptor receptorRS485;
EnlaceRS485::Mensaje solicitudRS485;
PaqueteESPAPortenta paqueteTxSnapshot;
bool rs485EsclavoIniciado=false, respuestaRS485Pendiente=false;
uint32_t ultimoIntentoRS485=0, inicioInstanciaRS485=0, ultimaActividadRS485=0;
uint32_t reiniciosRS485Esclavo=0, ultimaPublicacionRS485=0;
uint32_t solicitudesLecturaRS485=0, txRS485Ok=0, txRS485Error=0;
constexpr uint32_t PERIODO_PUBLICACION_RS485_MS=10;
int txRS485Mux=0;
void portENTER_CRITICAL(int *) {} void portEXIT_CRITICAL(int *) {}
void prepararSnapshotRS485() { prepararPaquete(paqueteTxSnapshot); }
constexpr int RS485_RX = ConfiguracionRS485::RX;
constexpr int RS485_TX = ConfiguracionRS485::TX;
constexpr int RS485_DIRECCION = ConfiguracionRS485::DE;
constexpr bool RS485_DIRECCION_MANUAL = true;
bool iniciarRS485Esclavo(bool reinicio) {
  if (rs485EsclavoIniciado) PuertoRS485.end();
  if (RS485_DIRECCION_MANUAL) {
    digitalWrite(RS485_DIRECCION, LOW);
    pinMode(RS485_DIRECCION, OUTPUT);
  }
  PuertoRS485.setRxBufferSize(256);
  PuertoRS485.begin(EnlaceRS485::BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  receptorRS485.reiniciar();
  respuestaRS485Pendiente = false;
  rs485EsclavoIniciado = true;
  ultimoIntentoRS485 = millis();
  inicioInstanciaRS485 = ultimoIntentoRS485;
  ultimaActividadRS485 = 0;
  if (reinicio) ++reiniciosRS485Esclavo;
  if (reinicio) {
    Serial.println(F("[RS485][RECUPERACION] UART2 reiniciada"));
  } else {
    Serial.print(F("[BOOT] RS485 UART2 RX"));
    Serial.print(RS485_RX);
    Serial.print(F("/TX"));
    Serial.print(RS485_TX);
    Serial.print(F(" DE"));
    Serial.print(RS485_DIRECCION);
    Serial.print(F("; "));
    Serial.print(EnlaceRS485::BAUD);
    Serial.println(F(" 8N1; protocolo 17"));
  }
  return true;
}
void responderPortentaRS485() {
  if (!respuestaRS485Pendiente) return;
  // Snapshot actual despues de aplicar control y pinza; la correlacion de
  // transporte no sustituye los ACK de camara/objetivo ni confirma agarre.
  ultimaPublicacionRS485 = millis() - PERIODO_PUBLICACION_RS485_MS;
  prepararSnapshotRS485();
  EnlaceRS485::Mensaje respuesta = solicitudRS485;
  portENTER_CRITICAL(&txRS485Mux);
  memcpy(respuesta.datos, &paqueteTxSnapshot, sizeof(paqueteTxSnapshot));
  portEXIT_CRITICAL(&txRS485Mux);
  uint8_t trama[EnlaceRS485::TRAMA];
  const size_t n = EnlaceRS485::codificar(respuesta, trama);
  delayMicroseconds(EnlaceRS485::GIRO_BUS_US);
  if (RS485_DIRECCION_MANUAL) {
    digitalWrite(RS485_DIRECCION, HIGH);
    delayMicroseconds(EnlaceRS485::PRE_TX_US);
  }
  const size_t escritos = PuertoRS485.write(trama, n);
  PuertoRS485.flush();
  if (RS485_DIRECCION_MANUAL) {
    delayMicroseconds(EnlaceRS485::POST_TX_US);
    digitalWrite(RS485_DIRECCION, LOW);
  }
  respuestaRS485Pendiente = false;
  ++solicitudesLecturaRS485;
  if (escritos == n) ++txRS485Ok; else ++txRS485Error;
}
}

void comprobarMaestro() {
  const auto &c=comm_protocols;
  assert(c.iniciada && c.habilitada && !c.rs232 && !c.fullDuplex && !c.terminacion);
  assert(c.rs485.baud==115200 && c.rs485.config==SERIAL_8N1);
  assert(c.rs485.pre==0 && c.rs485.post==2000 && c.rs485.rx && !c.rs485.tx);
  assert(clienteRS485.sesion && !clienteRS485.pendiente);
}
int main() {
  assert(iniciarRS485Maestro()); comprobarMaestro();
  const uint32_t sesion=clienteRS485.sesion;
  clienteRS485.iniciar(millis());
  comm_protocols.rs485ABTerm(true); // Una recuperacion debe restablecer el perfil.
  assert(iniciarRS485Maestro()); comprobarMaestro();
  assert(clienteRS485.sesion==sesion);
  assert(Esclavo::iniciarRS485Esclavo(false));
  assert(Esclavo::PuertoRS485.baud==115200 && !deESP);
  assert(Esclavo::iniciarRS485Esclavo(true));
  assert(Esclavo::reiniciosRS485Esclavo==1 && !deESP);
  Esclavo::solicitudRS485.sesion=sesion; Esclavo::solicitudRS485.solicitud=42;
  const uint64_t inicio=tiempoMicrosegundos();
  Esclavo::respuestaRS485Pendiente=true; Esclavo::responderPortentaRS485();
  assert(inicioTxESP-inicio==15000 && !deESP);
  assert(!Esclavo::respuestaRS485Pendiente && Esclavo::txRS485Ok==1);
  EnlaceRS485::Receptor r; EnlaceRS485::Mensaje m; bool recibido=false;
  for(auto b:entradaPortenta) recibido |= r.agregar(b,millis(),m);
  assert(recibido && m.sesion==sesion && m.solicitud==42);
  PaqueteESPAPortenta p; memcpy(&p,m.datos,sizeof(p));
  assert(validarPaquete(p));
  std::puts("PASS: perfil real 115200, inicio/recuperacion, Portenta 0/2000 us, ESP DE 200/flush/200 us y respuesta 15 ms");
}

#include <cassert>
#include <cstdio>
#include <vector>
#include <deque>
#include <cstring>
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/ProtocoloRS485.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/EnlaceRS485.h"

using namespace ProtocoloRS485;
uint32_t now=1000;
unsigned long millis() { return now; }
bool deESPTodaviaActivo=false;
struct UARTSimulada {
  bool rx=true, tx=false, corto=false;
  unsigned transmisiones=0, deshabilitacionesRX=0;
  std::vector<uint8_t> bytes;
  std::deque<uint8_t> entrada;
  int available() { return rx ? entrada.size() : 0; }
  int read() { auto b=entrada.front(); entrada.pop_front(); return b; }
  void noReceive() { assert(!deESPTodaviaActivo); ++deshabilitacionesRX; rx=false; }
  void beginTransmission() { assert(!rx && !deESPTodaviaActivo); ++transmisiones; tx=true; }
  size_t write(const uint8_t *p, size_t n) {
    assert(tx && !rx);
    if(corto) return 0;
    bytes.assign(p,p+n); return n;
  }
  void endTransmission() { tx=false; }
  void receive() { rx=true; }
};
struct Comunicaciones { UARTSimulada rs485; } comm_protocols;
EnlaceRS485::Cliente clienteRS485;
EnlaceRS485::Receptor receptorRS485;
bool envioRS485Urgente=false;
uint32_t enviosRS485Ok=0, enviosRS485Error=0;
uint32_t respuestasRS485Ajenas=0, lecturasRS485Error=0, erroresRS485Semantica=0;
uint32_t ultimaLatenciaRS485Ms=0, maximaLatenciaRS485Ms=0, lecturasRS485Ok=0, timeoutsRS485=0;
uint8_t fallosPaqueteConsecutivos=0;
unsigned paquetesValidos=0;
uint8_t ultimoCodigoErrorEnvioRS485=0, ackActual=ACK_OBJ_NINGUNO;
bool paqueteSemanticamenteValido(const PaqueteESPAPortenta &) { return true; }
void registrarPaqueteValido(const PaqueteESPAPortenta &) { ++paquetesValidos; }
void construirPaquetePortenta(PaquetePortentaAESP &p) {
  p.codigoAckObjetivo=ackActual; p.ackSecuenciaObjetivo=42; prepararPaquete(p);
}
bool leerPaqueteESP32() {
    EnlaceRS485::Mensaje mensaje;
    bool aceptado = false;
    while (comm_protocols.rs485.available()) {
        const uint8_t dato = static_cast<uint8_t>(comm_protocols.rs485.read());
        const uint32_t recibidoMs = millis();
        clienteRS485.registrarRecepcion(recibidoMs);
        if (!receptorRS485.agregar(dato, recibidoMs, mensaje))
            continue;
        if (!clienteRS485.coincide(mensaje, millis())) { ++respuestasRS485Ajenas; continue; }
        PaqueteESPAPortenta recibido;
        memcpy(&recibido, mensaje.datos, sizeof(recibido));
        if (!validarPaquete(recibido) || !paqueteSemanticamenteValido(recibido)) {
            ++lecturasRS485Error;
            ++erroresRS485Semantica;
            if (fallosPaqueteConsecutivos < 255) ++fallosPaqueteConsecutivos;
            continue;
        }
        ultimaLatenciaRS485Ms = millis() - clienteRS485.enviadaMs;
        if (ultimaLatenciaRS485Ms > maximaLatenciaRS485Ms)
            maximaLatenciaRS485Ms = ultimaLatenciaRS485Ms;
        clienteRS485.confirmar();
        registrarPaqueteValido(recibido);
        ++lecturasRS485Ok;
        aceptado = true;
    }
    receptorRS485.vencerFragmento(millis());
    if (clienteRS485.vencer(millis())) {
        ++timeoutsRS485;
        ++lecturasRS485Error;
        if (fallosPaqueteConsecutivos < 255) ++fallosPaqueteConsecutivos;
    }
    return aceptado;
}
bool enviarPaquetePortenta() {
    if (!clienteRS485.sesion) return false;
    if (!clienteRS485.iniciar(millis())) {
        // Las ordenes inmediatas de cierre/apertura no se pierden: el siguiente
        // envio construye el estado actual al quedar libre el bus. Validar el
        // silencio tambien aqui evita saltarlo desde una orden urgente.
        envioRS485Urgente = true;
        return false;
    }
    PaquetePortentaAESP salida = {};
    construirPaquetePortenta(salida);
    EnlaceRS485::Mensaje mensaje = {};
    mensaje.sesion = clienteRS485.sesion;
    mensaje.solicitud = clienteRS485.solicitud;
    memcpy(mensaje.datos, &salida, sizeof(salida));
    uint8_t trama[EnlaceRS485::TRAMA];
    const size_t n = EnlaceRS485::codificar(mensaje, trama);
    comm_protocols.rs485.noReceive();
    comm_protocols.rs485.beginTransmission();
    const size_t escritos = comm_protocols.rs485.write(trama, n);
    comm_protocols.rs485.endTransmission();
    comm_protocols.rs485.receive();
    clienteRS485.enviadaMs = millis();
    envioRS485Urgente = false;
    if (escritos == n) {
        ++enviosRS485Ok;
        ultimoCodigoErrorEnvioRS485 = 0;
        return true; // TX local completo; la respuesta se valida por separado.
    }
    clienteRS485.confirmar();
    ++enviosRS485Error;
    ultimoCodigoErrorEnvioRS485 = 1;
    return false;
}
void verificar(uint8_t ack) {
  EnlaceRS485::Receptor r; EnlaceRS485::Mensaje m; bool recibido=false;
  for(auto b:comm_protocols.rs485.bytes) recibido |= r.agregar(b,now,m);
  assert(recibido && m.sesion==clienteRS485.sesion && m.solicitud==clienteRS485.solicitud);
  PaquetePortentaAESP p; memcpy(&p,m.datos,sizeof(p));
  assert(validarPaquete(p) && p.codigoAckObjetivo==ack && p.ackSecuenciaObjetivo==42);
}
int main() {
  assert(!enviarPaquetePortenta()); // Sesion no inicializada: no transmitir.
  clienteRS485.sesion=123;
  assert(enviarPaquetePortenta()); verificar(ACK_OBJ_NINGUNO);
  auto anterior=comm_protocols.rs485.bytes;
  ackActual=ACK_OBJ_CERRAR_PINZA;
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  assert(comm_protocols.rs485.bytes==anterior && enviosRS485Ok==1);

  // Reproducir la carrera que el test maestro con delay(100) no ejercitaba:
  // el cero final ya llego, pero ESP conserva DE hasta flush + POST_TX_US.
  // Se ejecuta la funcion de envio real, tambien para la orden urgente.
  now+=24; deESPTodaviaActivo=true;
  PaqueteESPAPortenta respuesta={}; prepararPaquete(respuesta);
  EnlaceRS485::Mensaje mensaje={};
  mensaje.sesion=clienteRS485.sesion; mensaje.solicitud=clienteRS485.solicitud;
  memcpy(mensaje.datos,&respuesta,sizeof(respuesta));
  uint8_t trama[EnlaceRS485::TRAMA]; EnlaceRS485::codificar(mensaje,trama);
  comm_protocols.rs485.entrada.insert(comm_protocols.rs485.entrada.end(),trama,trama+sizeof(trama));
  assert(leerPaqueteESP32() && paquetesValidos==1 && !clienteRS485.pendiente);
  const uint32_t solicitudAnterior=clienteRS485.solicitud;
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  assert(comm_protocols.rs485.bytes==anterior && enviosRS485Ok==1 && enviosRS485Error==0);
  now+=2; deESPTodaviaActivo=false;
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  assert(clienteRS485.solicitud==solicitudAnterior && !clienteRS485.pendiente);
  assert(comm_protocols.rs485.transmisiones==1 && comm_protocols.rs485.deshabilitacionesRX==1);
  assert(comm_protocols.rs485.rx && !comm_protocols.rs485.tx);
  ++now;
  assert(enviarPaquetePortenta()); verificar(ACK_OBJ_CERRAR_PINZA);
  assert(!envioRS485Urgente && comm_protocols.rs485.rx && !comm_protocols.rs485.tx);
  clienteRS485.confirmar(); comm_protocols.rs485.corto=true;
  assert(!enviarPaquetePortenta() && !clienteRS485.pendiente && enviosRS485Error==1);
  assert(comm_protocols.rs485.rx && !comm_protocols.rs485.tx);
  comm_protocols.rs485.corto=false; ackActual=ACK_OBJ_ABRIR_PINZA;
  assert(enviarPaquetePortenta()); verificar(ACK_OBJ_ABRIR_PINZA);

  // RX parcial no valida el enlace y tampoco prolonga el plazo de 100 ms.
  // Si coincide con el timeout, el siguiente intento sigue esperando silencio.
  now+=99; comm_protocols.rs485.entrada.push_back(0x55);
  assert(!leerPaqueteESP32() && clienteRS485.pendiente && paquetesValidos==1);
  ++now; assert(!leerPaqueteESP32() && timeoutsRS485==1 && !clienteRS485.pendiente);
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  ++now; assert(!enviarPaquetePortenta());
  ++now; assert(enviarPaquetePortenta()); verificar(ACK_OBJ_ABRIR_PINZA);
  assert(paquetesValidos==1 && lecturasRS485Ok==1);
  std::puts("PASS: RX/envio reales, DE sin solapamiento, urgente diferida 3 ms, ruido sin renovar plazo y TX corto recuperable");
}

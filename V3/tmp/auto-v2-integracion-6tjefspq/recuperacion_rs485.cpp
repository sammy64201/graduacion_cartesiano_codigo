#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/ProtocoloRS485.h"
using namespace ProtocoloRS485;
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/EnlaceRS485.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/RecuperacionEnlace.h"
enum EstadoGeneral : uint8_t {
    EST_BOOT_SAFE = 0,
    EST_WAIT_RS485,
    EST_RS485_SETTLE,
    EST_CAMERA_CALIBRATION,
    EST_ARM_CALIBRATION,
    EST_WAIT_CONTROLLER,
    EST_FINAL_CHECKLIST,
    EST_MAIN_MENU,
    EST_MANUAL,
    EST_AUTOMATICO,
    EST_USER_ARM_CALIBRATION,
    EST_USER_CAMERA_CALIBRATION,
    EST_SYSTEM_ERROR,
    EST_AUTOMATICO_V2,
    EST_ENCODER_CALIBRATION,
    EST_CALIBRACIONES_MENU,
    EST_PRUEBA_SERVOS,
    EST_DIAGNOSTICO,
    EST_ENTRENAMIENTO_ML,
    EST_PRUEBA_ENCODER,
    EST_CAMBIOS_CATCH
};
const unsigned long TIMEOUT_RS485_MS = RecuperacionEnlace::SIN_RESPUESTA_MS;
const unsigned long TIMEOUT_SECUENCIA_ESP_MS = RecuperacionEnlace::SIN_RESPUESTA_MS;
const uint8_t MAX_PAQUETES_INVALIDOS_CONSECUTIVOS = 10;

struct Terminal {
  template<class T> void print(T) {}
  template<class T> void println(T) {}
} Serial;
uint32_t now=1000;
uint32_t millis() { return now; }
EstadoGeneral estadoGeneral=EST_AUTOMATICO_V2, modoPendiente=EST_AUTOMATICO_V2;
bool existePaqueteValido=true, protocoloValido=true, sesionESPConocida=true;
bool secuenciaPaqueteESPConocida=true, cambioSesionESPPendiente=false;
bool movimientoPosicionadoActivo=true, calibracionXYValida=true, calibracionZValida=true;
bool moviendo=true;
uint32_t ultimoPaqueteValidoMs=1000, ultimoCambioSecuenciaESPMs=1000;
uint8_t fallosPaqueteConsecutivos=0, ultimaSecuenciaPaqueteESP=7;
uint16_t sesionArranqueESP=1, secuenciaObjetivoEnMovimiento=42, ackSecuenciaObjetivo=0;
uint8_t codigoAckObjetivo=ACK_OBJ_NINGUNO;
long rangoXPasos=28400, rangoYPasos=16200, rangoZPasos=7300;
float pasosPorMmX=100, pasosPorMmY=99.5f, escalaEncoderMmPorCuenta=0.075f;
unsigned paros=0, cancelacionesPosicionado=0;
bool motoresEnMovimiento() { return moviendo; }
void detenerTodos() { ++paros; moviendo=false; movimientoPosicionadoActivo=false; }
void cambiarEstadoGeneral(EstadoGeneral e) { estadoGeneral=e; }
void cancelarMovimientoPosicionado(const char *, bool invalidar) {
  ++cancelacionesPosicionado; detenerTodos();
  if(invalidar) calibracionXYValida=calibracionZValida=false;
}
PaqueteESPAPortenta paqueteESP={};
int8_t joystickX=0, joystickY=0, joystickZ=0;
bool btConectado=false;
uint8_t posServoRot=90, posServoPin=90;
bool botonX=false, botonCirculo=false, botonTriangulo=false, botonCuadrado=false;
bool botonXAnterior=false, botonCirculoAnterior=false, botonTrianguloAnterior=false, botonCuadradoAnterior=false;
bool eventoBotonX=false, eventoBotonCirculo=false, eventoBotonTriangulo=false, eventoBotonCuadrado=false;
enum { V2_EVALUANDO_CATCH, ML_ESPERANDO_CONFIRMACION };
struct { int fase=V2_EVALUANDO_CATCH; } automaticoV2;
struct { int fase=ML_ESPERANDO_CONFIRMACION; } entrenamientoML;
bool ajusteCatchV2Seleccionado() { return false; }
bool entrenamientoConResultadoSeleccionado() { return false; }
void registrarEventoPortentaV2(const char *, const char *) {}
uint8_t estadoCamara=0, errorCamara=0, flagsCamara=0, flagsObjetivoV2=0, muestrasTag[4]={}, claseObjetivo=0;
int16_t objetivoCamaraX10=0, objetivoCamaraY10=0;
uint16_t secuenciaObjetivoRecibida=0;
int32_t conteoReferenciaObjetivoRecibido=0;

struct UART {
  std::deque<uint8_t> entrada;
  int available() { return entrada.size(); }
  int read() { const uint8_t b=entrada.front(); entrada.pop_front(); return b; }
} ;
struct { UART rs485; } comm_protocols;
EnlaceRS485::Cliente clienteRS485;
EnlaceRS485::Receptor receptorRS485;
uint32_t respuestasRS485Ajenas=0, lecturasRS485Error=0, erroresRS485Semantica=0;
uint32_t ultimaLatenciaRS485Ms=0, maximaLatenciaRS485Ms=0, lecturasRS485Ok=0, timeoutsRS485=0;
bool paqueteSemanticamenteValido(const PaqueteESPAPortenta &p) {
    const bool joystickValido =
        p.joystickX >= -1 && p.joystickX <= 1 &&
        p.joystickY >= -1 && p.joystickY <= 1 &&
        p.joystickZ >= -1 && p.joystickZ <= 1;
    const bool botonesValidos =
        (p.botones & static_cast<uint8_t>(~(
            BOTON_X | BOTON_TRIANGULO | BOTON_CIRCULO | BOTON_CUADRADO
        ))) == 0;
    const bool servosValidos = p.servoRotacion <= 180 && p.servoPinza <= 180;
    const bool camaraValida = p.estadoCamara <= CAMARA_ERROR;
    const bool reservadoValido = (p.reservadoV2 &
        static_cast<uint8_t>(~MASCARA_FLAGS_OBJETIVO_V2)) == 0;
    return joystickValido && botonesValidos && servosValidos &&
           camaraValida && reservadoValido;
}
void registrarPaqueteValido(const PaqueteESPAPortenta &nuevo) {
    if (!sesionESPConocida) {
        sesionArranqueESP = nuevo.sesionArranque;
        sesionESPConocida = true;
        secuenciaPaqueteESPConocida = false;
        ackSecuenciaObjetivo = 0;
        codigoAckObjetivo = ACK_OBJ_NINGUNO;
        secuenciaObjetivoEnMovimiento = 0;
        Serial.print(F("[RS485] Sesion ESP32 inicial="));
        Serial.println(sesionArranqueESP);
    } else if (nuevo.sesionArranque != sesionArranqueESP) {
        Serial.print(F("[RS485] Reinicio ESP32: sesion "));
        Serial.print(sesionArranqueESP);
        Serial.print(F(" -> "));
        Serial.println(nuevo.sesionArranque);
        sesionArranqueESP = nuevo.sesionArranque;
        secuenciaPaqueteESPConocida = false;
        cambioSesionESPPendiente = true;
        ackSecuenciaObjetivo = 0;
        codigoAckObjetivo = ACK_OBJ_NINGUNO;
        secuenciaObjetivoEnMovimiento = 0;
        if (movimientoPosicionadoActivo) {
            cancelarMovimientoPosicionado("reinicio de ESP32", false);
        }
    }

    if (!secuenciaPaqueteESPConocida ||
        nuevo.secuenciaPaquete != ultimaSecuenciaPaqueteESP) {
        ultimaSecuenciaPaqueteESP = nuevo.secuenciaPaquete;
        secuenciaPaqueteESPConocida = true;
        ultimoCambioSecuenciaESPMs = millis();
    }

    paqueteESP = nuevo;
    joystickX = nuevo.joystickX;
    joystickY = nuevo.joystickY;
    joystickZ = nuevo.joystickZ;
    btConectado = (nuevo.flags & ESP_FLAG_BT_CONECTADO) != 0;
    posServoRot = nuevo.servoRotacion;
    posServoPin = nuevo.servoPinza;

    botonX = (nuevo.botones & BOTON_X) != 0;
    botonCirculo = (nuevo.botones & BOTON_CIRCULO) != 0;
    botonTriangulo = (nuevo.botones & BOTON_TRIANGULO) != 0;
    botonCuadrado = (nuevo.botones & BOTON_CUADRADO) != 0;
    if (botonX && !botonXAnterior) {
        eventoBotonX = true;
        if (estadoGeneral == EST_AUTOMATICO_V2) {
            registrarEventoPortentaV2("BUTTON_X", "flanco X recibido");
        }
    }
    if (botonCirculo && !botonCirculoAnterior) eventoBotonCirculo = true;
    if (botonTriangulo && !botonTrianguloAnterior) eventoBotonTriangulo = true;
    if ((estadoGeneral == EST_MANUAL ||
         (estadoGeneral == EST_AUTOMATICO_V2 && ajusteCatchV2Seleccionado() &&
          automaticoV2.fase == V2_EVALUANDO_CATCH) ||
         (estadoGeneral == EST_ENTRENAMIENTO_ML &&
           entrenamientoConResultadoSeleccionado() &&
          entrenamientoML.fase == ML_ESPERANDO_CONFIRMACION)) &&
        botonCuadrado &&
        !botonCuadradoAnterior) eventoBotonCuadrado = true;
    botonXAnterior = botonX;
    botonCirculoAnterior = botonCirculo;
    botonTrianguloAnterior = botonTriangulo;
    botonCuadradoAnterior = botonCuadrado;

    estadoCamara = nuevo.estadoCamara;
    errorCamara = nuevo.errorCamara;
    flagsCamara = nuevo.flags;
    flagsObjetivoV2 = nuevo.reservadoV2;
    desempacarMuestrasTags(nuevo.muestrasTagEmpacadas, muestrasTag);
    claseObjetivo = nuevo.claseObjetivo;
    objetivoCamaraX10 = nuevo.objetivoX10;
    objetivoCamaraY10 = nuevo.objetivoY10;
    secuenciaObjetivoRecibida = nuevo.secuenciaObjetivo;
    conteoReferenciaObjetivoRecibido = nuevo.conteoReferenciaObjetivo;

    existePaqueteValido = true;
    protocoloValido = true;
    ultimoPaqueteValidoMs = millis();
    fallosPaqueteConsecutivos = 0;
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
bool enlaceRS485Vigente() {
    return RecuperacionEnlace::vigente(existePaqueteValido,
                                      millis(), ultimoPaqueteValidoMs);
}
void volverAEsperaRS485(const char *motivo) {
    const bool movimientoInterrumpido =
        motoresEnMovimiento() || movimientoPosicionadoActivo;
    detenerTodos();
    if (movimientoInterrumpido) {
        // Politica solicitada: conservar las referencias de la sesion aunque
        // la posicion fisica pueda dejar de coincidir con el conteo de pasos.
        Serial.println(F("[RS485] Movimiento cancelado; calibracion y escalas conservadas"));
    }
    if (secuenciaObjetivoEnMovimiento != 0) {
        ackSecuenciaObjetivo = secuenciaObjetivoEnMovimiento;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
        secuenciaObjetivoEnMovimiento = 0;
    }
    if (estadoGeneral == EST_MANUAL || estadoGeneral == EST_AUTOMATICO ||
        estadoGeneral == EST_AUTOMATICO_V2 ||
        estadoGeneral == EST_ENTRENAMIENTO_ML) {
        modoPendiente = EST_MAIN_MENU;
    }
    Serial.println(motivo);
    cambiarEstadoGeneral(EST_WAIT_RS485);
}
void vigilarSeguridadComunicacion() {
    if (cambioSesionESPPendiente) {
        cambioSesionESPPendiente = false;
        if (estadoGeneral == EST_SYSTEM_ERROR) return;
        volverAEsperaRS485("[RS485] Nueva sesion ESP32; esperando enlace estable");
        return;
    }

    if (estadoGeneral == EST_BOOT_SAFE || estadoGeneral == EST_WAIT_RS485 ||
        estadoGeneral == EST_SYSTEM_ERROR) {
        return;
    }

    if (!enlaceRS485Vigente()) {
        volverAEsperaRS485("[RS485] Enlace perdido; reintentando sin reinicio fisico");
    }
}

void transmitir(PaqueteESPAPortenta p, unsigned rechazo=0) {
  clienteRS485.pendiente=true; clienteRS485.sesion=123;
  ++clienteRS485.solicitud; clienteRS485.enviadaMs=now-20;
  if(rechazo==4) clienteRS485.enviadaMs=now-EnlaceRS485::TIMEOUT_RESPUESTA_MS;
  EnlaceRS485::Mensaje m={}; m.sesion=clienteRS485.sesion; m.solicitud=clienteRS485.solicitud;
  if(rechazo==2) ++m.sesion;
  if(rechazo==3) ++m.solicitud;
  memcpy(m.datos,&p,sizeof(p));
  uint8_t trama[EnlaceRS485::TRAMA]; EnlaceRS485::codificar(m,trama);
  if(rechazo==5) { comm_protocols.rs485.entrada.push_back(0x55); return; }
  if(rechazo==6) trama[8]^=1; // CRC16/COBS corrupto, sin tocar la carga valida.
  comm_protocols.rs485.entrada.insert(comm_protocols.rs485.entrada.end(),trama,trama+sizeof(trama));
}
void resetTransporte() { clienteRS485={}; receptorRS485={}; comm_protocols.rs485.entrada.clear(); }

void preparar(uint32_t inicio=1000) {
  resetTransporte(); now=inicio; ultimoPaqueteValidoMs=ultimoCambioSecuenciaESPMs=inicio;
  existePaqueteValido=protocoloValido=sesionESPConocida=secuenciaPaqueteESPConocida=true;
  cambioSesionESPPendiente=false; fallosPaqueteConsecutivos=0;
  sesionArranqueESP=1; ultimaSecuenciaPaqueteESP=7;
  estadoGeneral=EST_AUTOMATICO_V2; modoPendiente=EST_AUTOMATICO_V2;
  moviendo=movimientoPosicionadoActivo=calibracionXYValida=calibracionZValida=true;
  secuenciaObjetivoEnMovimiento=42; ackSecuenciaObjetivo=0; codigoAckObjetivo=ACK_OBJ_NINGUNO;
  paros=cancelacionesPosicionado=0;
}
PaqueteESPAPortenta respuesta() {
  PaqueteESPAPortenta p={}; p.sesionArranque=1; p.secuenciaPaquete=7;
  prepararPaquete(p); return p;
}
void comprobarReferencias() {
  assert(calibracionXYValida && calibracionZValida);
  assert(rangoXPasos==28400 && rangoYPasos==16200 && rangoZPasos==7300);
  assert(pasosPorMmX==100 && pasosPorMmY==99.5f && escalaEncoderMmPorCuenta==0.075f);
}
void comprobarActivo() {
  assert(estadoGeneral==EST_AUTOMATICO_V2 && moviendo && movimientoPosicionadoActivo && !paros);
  assert(secuenciaObjetivoEnMovimiento==42 && codigoAckObjetivo==ACK_OBJ_NINGUNO);
  comprobarReferencias();
}
void comprobarParo() {
  assert(estadoGeneral==EST_WAIT_RS485 && !moviendo && !movimientoPosicionadoActivo);
  assert(modoPendiente==EST_MAIN_MENU && paros==1);
  assert(ackSecuenciaObjetivo==42 && codigoAckObjetivo==ACK_OBJ_CANCELADO && !secuenciaObjetivoEnMovimiento);
  comprobarReferencias();
}
int main() {
  // Fronteras solicitadas: el umbral pertenece al sistema, no a cada sondeo.
  for(uint32_t edad : {149U,150U,999U,1000U}) {
    preparar(); now+=edad; vigilarSeguridadComunicacion(); comprobarActivo();
  }
  preparar(); now+=1001; vigilarSeguridadComunicacion(); comprobarParo();
  vigilarSeguridadComunicacion(); comprobarParo(); // Recuperacion no se repite.
  // Recuperar una respuesta conserva el paro y la cancelacion; no inicia pulsos.
  transmitir(respuesta()); assert(leerPaqueteESP32()); vigilarSeguridadComunicacion(); comprobarParo();
  // La secuencia del snapshot puede congelarse con respuestas correctas frescas.
  preparar();
  for(uint32_t edad : {149U,150U,999U,1000U,1001U,5000U}) {
    now=1000+edad; transmitir(respuesta()); assert(leerPaqueteESP32());
    assert(ultimoPaqueteValidoMs==now && ultimoCambioSecuenciaESPMs==1000);
    vigilarSeguridadComunicacion(); comprobarActivo();
  }
  // Ninguna rafaga de errores anticipa la perdida si existe respuesta reciente.
  for(uint32_t edad : {149U,150U,999U,1000U}) {
    preparar(); now+=edad; fallosPaqueteConsecutivos=255;
    vigilarSeguridadComunicacion(); comprobarActivo();
  }
  preparar(); fallosPaqueteConsecutivos=255; now+=1001;
  vigilarSeguridadComunicacion(); comprobarParo();
  // Una respuesta a los 999 ms renueva la vida durante otros 1000 ms completos.
  preparar(); now+=999; transmitir(respuesta()); assert(leerPaqueteESP32());
  now+=1000; vigilarSeguridadComunicacion(); comprobarActivo();
  ++now; vigilarSeguridadComunicacion(); comprobarParo();
  // CRC de aplicacion, semantica y longitud/ruido no renuevan ultimo valido.
  for(unsigned fallo : {0U,1U,5U,7U,8U}) {
    preparar(); now+=999; auto p=respuesta();
    if(fallo==0) p.checksum^=1;
    if(fallo==1) { p.joystickX=2; prepararPaquete(p); }
    if(fallo==7) { p.reservadoV2=0x80; prepararPaquete(p); }
    if(fallo==8) {
      p.version=VERSION_PROTOCOLO-1;
      p.checksum=calcularCRC8ATM(reinterpret_cast<const uint8_t*>(&p),sizeof(p)-1);
    }
    transmitir(p,fallo); assert(!leerPaqueteESP32()); assert(ultimoPaqueteValidoMs==1000);
    vigilarSeguridadComunicacion(); comprobarActivo();
    now=2001; vigilarSeguridadComunicacion(); comprobarParo();
  }
  // La resta uint32_t mantiene la frontera incluso cuando millis() desborda.
  preparar(UINT32_MAX-500); now+=1000; vigilarSeguridadComunicacion(); comprobarActivo();
  ++now; vigilarSeguridadComunicacion(); comprobarParo();
  // Un cambio real de sesion sigue cancelando: la CPU ESP si arranco de nuevo.
  // Esa ruta separada tambien conserva referencias y no reanuda el movimiento.
  preparar(); now+=20; auto nuevaSesion=respuesta(); nuevaSesion.sesionArranque=2;
  prepararPaquete(nuevaSesion); transmitir(nuevaSesion); assert(leerPaqueteESP32());
  vigilarSeguridadComunicacion();
  assert(estadoGeneral==EST_WAIT_RS485 && !moviendo && !movimientoPosicionadoActivo);
  assert(cancelacionesPosicionado==1 && modoPendiente==EST_MAIN_MENU);
  comprobarReferencias();
  transmitir(nuevaSesion); assert(leerPaqueteESP32()); vigilarSeguridadComunicacion();
  assert(estadoGeneral==EST_WAIT_RS485 && !moviendo && !movimientoPosicionadoActivo);
  comprobarReferencias();

  for(unsigned fallo : {2U,3U,4U,6U}) {
    preparar(); now+=999; transmitir(respuesta(),fallo);
    assert(!leerPaqueteESP32() && ultimoPaqueteValidoMs==1000);
    vigilarSeguridadComunicacion(); comprobarActivo();
    now=2001; vigilarSeguridadComunicacion(); comprobarParo();
  }
  // 100 ms vence solo el intercambio: deja sondear sin recuperar el sistema.
  preparar(); clienteRS485.sesion=123; assert(clienteRS485.iniciar(now));
  now+=100; assert(!leerPaqueteESP32() && !clienteRS485.pendiente);
  assert(timeoutsRS485>0); vigilarSeguridadComunicacion(); comprobarActivo();
  assert(clienteRS485.iniciar(now));
  std::puts("PASS: RS485: RX real, umbral >1000 ms, secuencia/invalidos sin recuperacion temprana, referencias y cancelacion persistentes");
}

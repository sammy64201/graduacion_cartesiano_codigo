#include <cassert>
#include <cstdio>
#include <stdint.h>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/ProtocoloRS485.h"
using namespace ProtocoloRS485;
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
const unsigned long TIMEOUT_RS485_MS = 150UL;
const unsigned long TIMEOUT_SECUENCIA_ESP_MS = 150UL;
const uint8_t MAX_PAQUETES_INVALIDOS_CONSECUTIVOS = 10;

struct Terminal { template<class T> void print(T) {} template<class T> void println(T) {} } Serial;
uint32_t now=1000;
unsigned long millis() { return now; }
EstadoGeneral estadoGeneral=EST_MANUAL, modoPendiente=EST_AUTOMATICO_V2;
bool existePaqueteValido=true, secuenciaPaqueteESPConocida=true;
unsigned long ultimoPaqueteValidoMs=1000, ultimoCambioSecuenciaESPMs=1000;
bool cambioSesionESPPendiente=false, movimientoPosicionadoActivo=false;
bool calibracionXYValida=true, calibracionZValida=true, moviendo=true;
uint8_t fallosPaqueteConsecutivos=0, codigoAckObjetivo=ACK_OBJ_NINGUNO;
uint16_t secuenciaObjetivoEnMovimiento=42, ackSecuenciaObjetivo=0;
unsigned paros=0;
bool motoresEnMovimiento() { return moviendo; }
void detenerTodos() { ++paros; moviendo=false; movimientoPosicionadoActivo=false; }
void cambiarEstadoGeneral(EstadoGeneral estado) { estadoGeneral=estado; }
bool enlaceRS485Vigente() {
    return existePaqueteValido &&
           secuenciaPaqueteESPConocida &&
           millis() - ultimoPaqueteValidoMs <= TIMEOUT_RS485_MS &&
           millis() - ultimoCambioSecuenciaESPMs <= TIMEOUT_SECUENCIA_ESP_MS;
}
void volverAEsperaRS485(const char *motivo) {
    const bool movimientoInterrumpido =
        motoresEnMovimiento() || movimientoPosicionadoActivo;
    detenerTodos();
    if (movimientoInterrumpido) {
        calibracionXYValida = false;
        calibracionZValida = false;
        Serial.println(F("[RS485] Movimiento interrumpido; recalibrar brazo antes de mover"));
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

    if (fallosPaqueteConsecutivos >= MAX_PAQUETES_INVALIDOS_CONSECUTIVOS &&
        estadoGeneral != EST_BOOT_SAFE && estadoGeneral != EST_WAIT_RS485 &&
        estadoGeneral != EST_SYSTEM_ERROR) {
        volverAEsperaRS485("[RS485] Paquetes invalidos; reintentando enlace");
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

void preparar() {
  now=1000; ultimoPaqueteValidoMs=ultimoCambioSecuenciaESPMs=1000;
  existePaqueteValido=secuenciaPaqueteESPConocida=true;
  cambioSesionESPPendiente=false; fallosPaqueteConsecutivos=0;
  estadoGeneral=EST_AUTOMATICO_V2; moviendo=true; movimientoPosicionadoActivo=true;
  calibracionXYValida=calibracionZValida=true;
  secuenciaObjetivoEnMovimiento=42; codigoAckObjetivo=ACK_OBJ_NINGUNO;
}
void comprobarParo() {
  assert(!moviendo && !movimientoPosicionadoActivo);
  assert(estadoGeneral==EST_WAIT_RS485);
  assert(!calibracionXYValida && !calibracionZValida);
  assert(ackSecuenciaObjetivo==42 && codigoAckObjetivo==ACK_OBJ_CANCELADO);
}
int main() {
  preparar(); vigilarSeguridadComunicacion(); assert(moviendo);
  now=1151; vigilarSeguridadComunicacion(); comprobarParo();
  preparar(); now=1151; ultimoPaqueteValidoMs=now;
  vigilarSeguridadComunicacion(); comprobarParo(); // Secuencia ESP congelada.
  preparar(); cambioSesionESPPendiente=true;
  vigilarSeguridadComunicacion(); comprobarParo();
  preparar(); fallosPaqueteConsecutivos=MAX_PAQUETES_INVALIDOS_CONSECUTIVOS;
  vigilarSeguridadComunicacion(); comprobarParo();
  preparar(); existePaqueteValido=false;
  vigilarSeguridadComunicacion(); comprobarParo();
  assert(paros==5);
  std::puts("PASS: guardas reales: perdida, secuencia congelada, reinicio ESP, invalidos y cancelacion con recalibracion");
}

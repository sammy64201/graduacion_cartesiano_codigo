#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/PORTENTA/ProtocoloI2C.h"
using namespace ProtocoloI2C;
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/PORTENTA/RecuperacionEnlace.h"
enum EstadoGeneral : uint8_t {
    EST_BOOT_SAFE = 0,
    EST_WAIT_I2C,
    EST_I2C_SETTLE,
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
const unsigned long TIMEOUT_I2C_MS = RecuperacionEnlace::SIN_RESPUESTA_MS;
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

struct I2C {
  std::deque<uint8_t> entrada;
  int requestFrom(uint8_t, uint8_t) { return entrada.size(); }
  int available() { return entrada.size(); }
  int read() { const uint8_t b=entrada.front(); entrada.pop_front(); return b; }
} Wire;
uint32_t lecturasI2CError=0, erroresI2CLongitud=0, erroresI2CCRC=0, erroresI2CSemantica=0, lecturasI2COk=0;
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
        Serial.print(F("[I2C] Sesion ESP32 inicial="));
        Serial.println(sesionArranqueESP);
    } else if (nuevo.sesionArranque != sesionArranqueESP) {
        Serial.print(F("[I2C] Reinicio ESP32: sesion "));
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
    const int recibidos = Wire.requestFrom(
        static_cast<uint8_t>(DIRECCION_ESP32),
        static_cast<uint8_t>(sizeof(PaqueteESPAPortenta))
    );

    if (recibidos != static_cast<int>(sizeof(PaqueteESPAPortenta)) ||
        Wire.available() < static_cast<int>(sizeof(PaqueteESPAPortenta))) {
        while (Wire.available()) Wire.read();
        lecturasI2CError++;
        erroresI2CLongitud++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }

    PaqueteESPAPortenta recibido = {};
    uint8_t *destino = reinterpret_cast<uint8_t *>(&recibido);
    for (size_t i = 0; i < sizeof(recibido); ++i) {
        destino[i] = static_cast<uint8_t>(Wire.read());
    }
    while (Wire.available()) Wire.read();

    if (!validarPaquete(recibido)) {
        lecturasI2CError++;
        erroresI2CCRC++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }
    if (!paqueteSemanticamenteValido(recibido)) {
        lecturasI2CError++;
        erroresI2CSemantica++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }

    registrarPaqueteValido(recibido);
    lecturasI2COk++;
    return true;
}
bool enlaceI2CVigente() {
    return RecuperacionEnlace::vigente(existePaqueteValido,
                                      millis(), ultimoPaqueteValidoMs);
}
void volverAEsperaI2C(const char *motivo) {
    const bool movimientoInterrumpido =
        motoresEnMovimiento() || movimientoPosicionadoActivo;
    detenerTodos();
    if (movimientoInterrumpido) {
        // Politica solicitada: conservar las referencias de la sesion aunque
        // la posicion fisica pueda dejar de coincidir con el conteo de pasos.
        Serial.println(F("[I2C] Movimiento cancelado; calibracion y escalas conservadas"));
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
    cambiarEstadoGeneral(EST_WAIT_I2C);
}
void vigilarSeguridadComunicacion() {
    if (cambioSesionESPPendiente) {
        cambioSesionESPPendiente = false;
        if (estadoGeneral == EST_SYSTEM_ERROR) return;
        volverAEsperaI2C("[I2C] Nueva sesion ESP32; esperando enlace estable");
        return;
    }

    if (estadoGeneral == EST_BOOT_SAFE || estadoGeneral == EST_WAIT_I2C ||
        estadoGeneral == EST_SYSTEM_ERROR) {
        return;
    }

    if (!enlaceI2CVigente()) {
        volverAEsperaI2C("[I2C] Enlace perdido; reintentando sin reinicio fisico");
    }
}

void transmitir(PaqueteESPAPortenta p, unsigned rechazo=0) {
  const auto b=reinterpret_cast<const uint8_t*>(&p);
  Wire.entrada.insert(Wire.entrada.end(),b,b+(rechazo==5 ? 7 : sizeof(p)));
}
void resetTransporte() { Wire.entrada.clear(); }

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
  assert(estadoGeneral==EST_WAIT_I2C && !moviendo && !movimientoPosicionadoActivo);
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
  assert(estadoGeneral==EST_WAIT_I2C && !moviendo && !movimientoPosicionadoActivo);
  assert(cancelacionesPosicionado==1 && modoPendiente==EST_MAIN_MENU);
  comprobarReferencias();
  transmitir(nuevaSesion); assert(leerPaqueteESP32()); vigilarSeguridadComunicacion();
  assert(estadoGeneral==EST_WAIT_I2C && !moviendo && !movimientoPosicionadoActivo);
  comprobarReferencias();
  std::puts("PASS: I2C: RX real, umbral >1000 ms, secuencia/invalidos sin recuperacion temprana, referencias y cancelacion persistentes");
}

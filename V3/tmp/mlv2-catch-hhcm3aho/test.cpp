#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define F(x) x
enum FaseEntrenamientoML : uint8_t {
    ML_ESPERANDO_PIEZA = 0,
    ML_PREPOSICIONANDO = 1,
    ML_BAJANDO_CAPTURA = 2,
    ML_ALINEACION_MANUAL = 3,
    ML_CERRANDO_PINZA = 4,
    ML_SUBIENDO_CON_PIEZA = 5,
    ML_MOVIENDO_ENTREGA = 6,
    ML_BAJANDO_ENTREGA = 7,
    ML_ABRIENDO_PINZA = 8,
    ML_SUBIENDO_FINAL = 9,
    ML_LISTO = 10,
    ML_CANCELANDO = 11,
    ML_PREPARANDO_ESPERA = 12,
    ML_BAJANDO_CATCH = 13,
    ML_ESPERANDO_CONFIRMACION = 14,
    ML_SEGUIMIENTO = 15
};
struct ContextoEntrenamientoML {
    FaseEntrenamientoML fase;
    unsigned long inicioFase;
    uint16_t secuencia;
    uint8_t clase;
    float camX;
    float camY;
    float xInicial;
    float yInicial;
    float xCorregida;
    float yCorregida;
    float piezaYEstimada;
    float catchYConfirmada;
    float umbralCierreY;
    int32_t conteoReferencia;
    uint8_t rotacionCorregida;
    bool busquedaFinalZActiva;
    bool rebaseManualRegistrado;
    bool salidaAlMenu;
    bool salidaAEsperaControl;
    float velocidadDisparo;
    float piezaYDisparo;
    float diferenciaDisparoMm;
    unsigned long instanteOrdenCierre;
    unsigned long instanteDeteccion;
    unsigned long instanteBotonCatch;
    unsigned long instanteDin04Catch;
    int32_t conteoBotonCatch;
    int32_t conteoOrdenCierre;
    float piezaYBotonCatch;
    const char *disparadorCatch;
    float errorSeguimientoBoton;
    float errorSeguimientoDin04;
    float errorSeguimientoCierre;
    float brazoYBoton;
    float brazoYDin04;
    float brazoYCierre;
    unsigned long inicioSeguimiento;
    unsigned long ultimoLogSeguimiento;
};
ContextoEntrenamientoML entrenamientoML;
enum { MENU_ENTRENAMIENTO_ML=0, MENU_ENTRENAMIENTO_ML_V2=1, MENU_PRUEBA_SEGUIMIENTO=2 };
int opcionMenu;
bool ensenanzaMLV2Seleccionada() {
    return opcionMenu == MENU_ENTRENAMIENTO_ML_V2;
}
bool pruebaSeguimientoSeleccionada() {
    return opcionMenu == MENU_PRUEBA_SEGUIMIENTO;
}
bool entrenamientoConResultadoSeleccionado() {
    return ensenanzaMLV2Seleccionada() || pruebaSeguimientoSeleccionada();
}
struct Terminal { template<class T> void print(const T&, int=0) {};
template<class T> void println(const T&, int=0) {}; void println() {};} Serial;
bool btConectado, eventoBotonTriangulo, eventoBotonX, limiteZabajo;
bool encoderOK, bandaOK, predictionOK, followOK, zEnCurso;
int movZ, cierres, cancelaciones, avisos, busquedas;
uint32_t tiempo, sesionArranqueESP=10672; bool sesionESPConocida=true;
int32_t conteoEncoderBanda=100; int posServoRot=54;
float velocidadBandaMmS=78;
constexpr float V2_ERROR_ESTABLE_MM=5;
constexpr unsigned long V2_TIMEOUT_FASE_MS=30000;
constexpr long V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS=600;
constexpr int V2_DIV_BUSQUEDA_FINAL_Z=8, ERROR_TIMEOUT_MOVIMIENTO=1;
unsigned long millis() { return tiempo; }
bool objetivoZEnCurso() { return zEnCurso; }
void detenerX() {} void detenerY() {}
void detenerZ() { movZ=0; zEnCurso=false; }
void detenerTodos() { detenerZ(); }
void cancelarEntrenamientoML(const char*, bool, bool) {
  ++cancelaciones; entrenamientoML.fase=ML_CANCELANDO; detenerZ();
}
void terminarCancelacionEntrenamientoML() {}
void entrarErrorSistema(int, const char*) { ++cancelaciones; }
bool encoderListoAutomaticoV2() { return encoderOK; }
bool bandaEnMovimientoAutomaticoV2() { return bandaOK; }
bool actualizarPiezaEntrenamientoML() { return predictionOK; }
bool seguirPiezaYEntrenamientoML() { return followOK; }
float posicionYmm() { return 0; }
float anticipacionCierreCatchSegundos() { return .5f; }
long limiteMinimoZPasos() { return -7300; }
void fijarPasosZ(long) {}
void moverZHasta(long, int) { ++busquedas; zEnCurso=true; movZ=-1; }
void enviarOrdenCierreCatchAhora() { assert(limiteZabajo); ++cierres; }
void registrarMuestraEntrenamientoML(const char*) {}
void registrarEventoPortentaV2(const char* evento, const char*) {
  if (!strcmp(evento, "ML_MANUAL_OVERRUN")) ++avisos;
}
void reset() {
  entrenamientoML={}; entrenamientoML.fase=ML_BAJANDO_CATCH;
  entrenamientoML.disparadorCatch="X"; entrenamientoML.piezaYEstimada=6;
  opcionMenu=MENU_ENTRENAMIENTO_ML_V2;
  btConectado=encoderOK=bandaOK=predictionOK=followOK=zEnCurso=true;
  eventoBotonTriangulo=eventoBotonX=limiteZabajo=false;
  movZ=-1; cierres=cancelaciones=avisos=busquedas=0; tiempo=1000;
}
void procesarEntrenamientoML() {
    if (entrenamientoML.fase == ML_CANCELANDO) {
        if (!objetivoZEnCurso() && movZ == 0) {
            terminarCancelacionEntrenamientoML();
        } else if (millis() - entrenamientoML.inicioFase > V2_TIMEOUT_FASE_MS) {
            entrarErrorSistema(
                ERROR_TIMEOUT_MOVIMIENTO,
                "Timeout retirando Z en entrenamiento ML"
            );
        }
        return;
    }

    if (!btConectado) {
        cancelarEntrenamientoML("control Bluetooth desconectado", true, false);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        cancelarEntrenamientoML("cancelado por usuario", false, true);
        return;
    }
    const bool seguimientoMovilActivo =
        entrenamientoML.fase == ML_PREPOSICIONANDO ||
        entrenamientoML.fase == ML_BAJANDO_CAPTURA ||
        entrenamientoML.fase == ML_ALINEACION_MANUAL ||
        entrenamientoML.fase == ML_SEGUIMIENTO ||
        entrenamientoML.fase == ML_BAJANDO_CATCH ||
        entrenamientoML.fase == ML_CERRANDO_PINZA;
    if (!encoderListoAutomaticoV2()) {
        if (entrenamientoML.fase == ML_ESPERANDO_PIEZA) {
            detenerTodos();
            return;
        }
        if (seguimientoMovilActivo) {
            cancelarEntrenamientoML("encoder no valido", false, false);
            return;
        }
    }
    if (seguimientoMovilActivo && !bandaEnMovimientoAutomaticoV2()) {
        cancelarEntrenamientoML("la banda se detuvo", false, false);
        return;
    }
    if (entrenamientoML.fase != ML_ESPERANDO_PIEZA &&
        entrenamientoML.fase != ML_ALINEACION_MANUAL &&
        entrenamientoML.fase != ML_SEGUIMIENTO &&
        entrenamientoML.fase != ML_ESPERANDO_CONFIRMACION &&
        entrenamientoML.fase != ML_LISTO &&
        millis() - entrenamientoML.inicioFase > V2_TIMEOUT_FASE_MS) {
        cancelarEntrenamientoML("timeout de fase", false, false);
        return;
    }

switch (entrenamientoML.fase) {
        case ML_BAJANDO_CATCH: {
            detenerX();
            if (!pruebaSeguimientoSeleccionada()) detenerY();
            eventoBotonX = false;
            if (!actualizarPiezaEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "prediccion invalida durante descenso final ML", false, false);
                return;
            }
            if (pruebaSeguimientoSeleccionada() &&
                !seguirPiezaYEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "fin del recorrido Y durante descenso final", false, false);
                return;
            }
            const bool catchManualMLV2 = ensenanzaMLV2Seleccionada() &&
                entrenamientoML.disparadorCatch != nullptr &&
                strcmp(entrenamientoML.disparadorCatch, "X") == 0;
            if (!pruebaSeguimientoSeleccionada() &&
                entrenamientoML.piezaYEstimada >
                entrenamientoML.catchYConfirmada + V2_ERROR_ESTABLE_MM) {
                if (!catchManualMLV2) {
                    cancelarEntrenamientoML(
                        "pieza rebaso el catch antes de DIN04 ML", false, false);
                    return;
                }
                // ML V2 aprende del catch que el operador confirma con X.
                // Tras aceptarlo, la prediccion Y se registra como error,
                // sin impedir alcanzar DIN04 y ordenar el cierre manual.
                // Se mantienen timeout, finales, encoder, control y STOP.
                if (!entrenamientoML.rebaseManualRegistrado) {
                    entrenamientoML.rebaseManualRegistrado = true;
                    registrarEventoPortentaV2("ML_MANUAL_OVERRUN",
                        "estimacion Y rebaso catch; X confirmado, continuar a DIN04");
                }
            }
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                entrenamientoML.busquedaFinalZActiva = false;
                entrenamientoML.instanteDin04Catch = millis();
                if (pruebaSeguimientoSeleccionada()) {
                    entrenamientoML.brazoYDin04 = posicionYmm();
                    entrenamientoML.errorSeguimientoDin04 =
                        entrenamientoML.piezaYEstimada -
                        entrenamientoML.brazoYDin04;
                }
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!entrenamientoML.busquedaFinalZActiva) {
                    entrenamientoML.busquedaFinalZActiva = true;
                    moverZHasta(limiteMinimoZPasos() -
                        V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                        V2_DIV_BUSQUEDA_FINAL_Z);
                    registrarEventoPortentaV2(
                        "ML_Z_SEARCH", "busqueda DIN04 durante catch ML");
                    break;
                }
                cancelarEntrenamientoML(
                    "DIN04 no aparecio durante catch ML", false, false);
                return;
            }
            const char *disparador = entrenamientoML.disparadorCatch;
            const bool disparoPorX = strcmp(disparador, "X") == 0;
            if (pruebaSeguimientoSeleccionada()) {
                entrenamientoML.catchYConfirmada = posicionYmm();
            }
            entrenamientoML.umbralCierreY =
                entrenamientoML.catchYConfirmada -
                fmaxf(0.0f, velocidadBandaMmS) *
                anticipacionCierreCatchSegundos();
            entrenamientoML.velocidadDisparo = velocidadBandaMmS;
            entrenamientoML.piezaYDisparo = entrenamientoML.piezaYEstimada;
            entrenamientoML.diferenciaDisparoMm =
                entrenamientoML.piezaYEstimada -
                (pruebaSeguimientoSeleccionada()
                    ? entrenamientoML.catchYConfirmada
                    : entrenamientoML.umbralCierreY);
            entrenamientoML.instanteOrdenCierre = millis();
            entrenamientoML.conteoOrdenCierre = conteoEncoderBanda;
            entrenamientoML.fase = ML_CERRANDO_PINZA;
            entrenamientoML.inicioFase = entrenamientoML.instanteOrdenCierre;
            enviarOrdenCierreCatchAhora();
            Serial.print(pruebaSeguimientoSeleccionada()
                ? F("V2LOG|P|mode=ML_TRACK|event=ML_CATCH_TRIGGER|session=")
                : (ensenanzaMLV2Seleccionada()
                    ? F("V2LOG|P|mode=ML_V2|event=ML_CATCH_TRIGGER|session=")
                    : F("V2LOG|P|mode=ML|event=ML_CATCH_TRIGGER|session=")));
            Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
            Serial.print(F("|ms=")); Serial.print(entrenamientoML.instanteOrdenCierre);
            Serial.print(F("|obj=")); Serial.print(entrenamientoML.secuencia);
            Serial.print(F("|trigger=")); Serial.print(disparador);
            Serial.print(F("|catch_type="));
            Serial.print(disparoPorX ? F("MANUAL") : F("AUTOMATICO"));
            Serial.print(F("|enc=")); Serial.print(conteoEncoderBanda);
            if (entrenamientoConResultadoSeleccionado()) {
                Serial.print(F("|catch_button_ms="));
                Serial.print(entrenamientoML.instanteBotonCatch);
                Serial.print(F("|z_bottom_ms="));
                Serial.print(entrenamientoML.instanteDin04Catch);
                if (pruebaSeguimientoSeleccionada()) {
                    Serial.print(F("|arm_y="));
                    Serial.print(posicionYmm(), 3);
                    Serial.print(F("|error_y="));
                    Serial.print(entrenamientoML.piezaYDisparo - posicionYmm(), 3);
                }
            }
            Serial.print(F("|piece_y=")); Serial.print(entrenamientoML.piezaYDisparo, 3);
            Serial.print(F("|catch_y=")); Serial.print(entrenamientoML.catchYConfirmada, 3);
            Serial.print(F("|close_threshold_y="));
            Serial.print(entrenamientoML.umbralCierreY, 3);
            Serial.print(F("|servo_rot_deg=")); Serial.print(posServoRot);
            Serial.print(F("|error_disparo_mm="));
            Serial.print(entrenamientoML.diferenciaDisparoMm, 3);
            Serial.print(F("|error_disparo_ms="));
            Serial.println(entrenamientoML.velocidadDisparo > 0.0f
                ? 1000.0f * entrenamientoML.diferenciaDisparoMm /
                  entrenamientoML.velocidadDisparo : NAN, 3);
            if (!entrenamientoConResultadoSeleccionado()) {
                registrarMuestraEntrenamientoML(disparador);
            }
            Serial.print(F("[ML] ORDEN CERRAR disparador="));
            Serial.print(disparador);
            Serial.print(F(" piezaY="));
            Serial.print(entrenamientoML.piezaYEstimada, 2);
            Serial.print(F(" umbral_cierre="));
            Serial.print(entrenamientoML.umbralCierreY, 2);
            Serial.print(F(" catchY="));
            Serial.println(entrenamientoML.catchYConfirmada, 2);
            Serial.println(F("[ML] Fase -> CERRANDO PINZA"));
            break;
        }
default: break;
}
}
int main() {
// Cruce estimado durante descenso manual: continuar, sin cerrar antes de DIN04.
reset(); procesarEntrenamientoML(); procesarEntrenamientoML();
assert(cancelaciones==0 && cierres==0 && avisos==1);
limiteZabajo=true; procesarEntrenamientoML(); procesarEntrenamientoML();
assert(cierres==1 && entrenamientoML.fase==ML_CERRANDO_PINZA);
// Otros modos y disparadores conservan la cancelacion por rebase.
reset(); opcionMenu=MENU_ENTRENAMIENTO_ML; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); entrenamientoML.disparadorCatch="ENCODER"; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); entrenamientoML.disparadorCatch=nullptr; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
// Las guardas reales del preambulo y del descenso siguen impidiendo el cierre.
reset(); btConectado=false; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); eventoBotonTriangulo=true; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); encoderOK=false; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); bandaOK=false; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); predictionOK=false; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); tiempo=V2_TIMEOUT_FASE_MS+1; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
reset(); opcionMenu=MENU_PRUEBA_SEGUIMIENTO; followOK=false; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
// Sin DIN04: una busqueda extra y despues cancelacion, nunca una orden de pinza.
reset(); movZ=0; zEnCurso=false; procesarEntrenamientoML(); assert(busquedas==1 && !cierres);
movZ=0; zEnCurso=false; procesarEntrenamientoML(); assert(cancelaciones==1 && !cierres);
// Replay de los dos objetivos que el registro cancelo antes de DIN04.
int seq; while (scanf("%d", &seq)==1) {
  reset(); entrenamientoML.secuencia=seq;
  procesarEntrenamientoML(); assert(!cancelaciones && !cierres);
  limiteZabajo=true; procesarEntrenamientoML(); assert(cierres==1);
  printf("%d\n", seq);
} }

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/PORTENTA/ProtocoloI2C.h"
using namespace ProtocoloI2C;
enum FaseAutomaticoV2 : uint8_t {
    V2_ESPERANDO_PIEZA = 0,
    V2_PREPOSICIONANDO = 1,
    V2_ESPERANDO_LLEGADA = 2,
    V2_DISPARANDO_CATCH = 3,
    V2_BAJANDO_Z = 4,
    V2_SUBIENDO_Z = 5,
    // Se conserva el numero 6 para no romper la interpretacion de CSV previos.
    V2_ESPERANDO_CATCH_AUTOMATICO = 6,
    V2_COMPLETADO = 7,
    V2_CANCELANDO = 8,
    V2_PREPARANDO_ESPERA = 9,
    V2_CERRANDO_PINZA = 10,
    V2_BAJANDO_CATCH = 11,
    V2_MOVIENDO_ENTREGA = 12,
    V2_BAJANDO_ENTREGA = 13,
    V2_ABRIENDO_PINZA = 14,
    V2_SUBIENDO_FINAL = 15,
    V2_SIGUIENDO_PIEZA = 16
};
struct ContextoAutomaticoV2 {
    FaseAutomaticoV2 fase;
    unsigned long inicioFase;
    unsigned long inicioEstable;
    bool salidaAEsperaControl;
    bool salidaAlMenu;
    uint16_t secuencia;
    uint8_t clase;
    float camXReferencia;
    float camYReferencia;
    int32_t conteoReferencia;
    float objetivoBrazoX;
    float objetivoBrazoY;
    float objetivoBrazoYAnterior;
    int32_t ultimoConteoProcesado;
    unsigned long instanteObjetivoAnterior;
    float velocidadObjetivoY;
    float velocidadInicialY;
    bool busquedaFinalZActiva;
    float umbralDisparoY;
    unsigned long instanteListoCatch;
    int32_t conteoListoCatch;
    bool catchAutomaticoDisparado;
    uint8_t anguloCatch;
    float umbralCierrePinzaY;
    float ultimoErrorY;
    float ultimoErrorX;
    unsigned long ultimoLogSeguimiento;
};
const float RANGO_FISICO_Y_MM = 336.0f;
const float MARGEN_SEGURIDAD_MM = 2.0f;
const long PASOS_SEPARACION = 300;
const uint16_t DIV_CAL_LENTA = 8;
const uint16_t DIV_POSICION = 1;
constexpr float V2_POSICION_CATCH_Y_MM = 0.0f;
constexpr long V2_Z_SEGURO_PASOS = 0;
constexpr float V2_ERROR_ESTABLE_MM = 5.0f;
constexpr unsigned long V2_TIMEOUT_FASE_MS = 30000UL;
constexpr unsigned long V2_TIEMPO_COMPLETADO_MS = 500UL;
constexpr unsigned long V2_TIEMPO_CIERRE_PINZA_MS = 450UL;
constexpr unsigned long V2_LATENCIA_ORDEN_PINZA_MS = 50UL;
constexpr long V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS = 2L * PASOS_SEPARACION;
constexpr uint16_t V2_DIV_BUSQUEDA_FINAL_Z = DIV_CAL_LENTA;
constexpr unsigned long ML_TIEMPO_SERVO_MS = 450UL;
constexpr float ML_MARGEN_FINAL_DERECHO_MM = 10.0f;
constexpr float ML_SEGUIMIENTO_MARGEN_Y_MM = 4.0f;
constexpr float ML_SEGUIMIENTO_RESERVA_S = 0.20f;
constexpr unsigned long ML_SEGUIMIENTO_LOG_MS = 200UL;
constexpr unsigned long V2_SEGUIMIENTO_ESTABLE_MS = 300UL;

// Alternar la ruta solo en el simulador permite verificar ambas configuraciones.
bool AUTO_V2_SEGUIMIENTO_Y=true;
ContextoAutomaticoV2 automaticoV2;
struct Terminal { template<class T> void print(const T&, int=0) {};
template<class T> void println(const T&, int=0) {}; } Serial;
unsigned long now=1000, tAnteriorEstadoESP;
unsigned long millis() { return now; }
bool btConectado, eventoBotonTriangulo, eventoBotonX;
bool limiteZarriba, limiteZabajo, limiteXmas, limiteXmenos, limiteYmas, limiteYmenos;
bool encoderOK, bandaOK, camaraOK, predictionOK, xyOK, comunicacionI2CHabilitada;
bool movimientoPosicionadoActivo, objetivoYActivo, xActive, zActive;
int movX, movY, movZ, divisorY=DIV_POSICION, posServoRot=59;
long objetivoY, ySteps, zSteps, zTarget;
float pasosPorMmY=100, armX, pieceY, velocidadBandaMmS;
int32_t conteoEncoderBanda;
uint16_t secuenciaObjetivoRecibida, ackSecuenciaObjetivo, secuenciaObjetivoEnMovimiento;
uint8_t codigoAckObjetivo;
int errores, cancelaciones, cierres, resultados, aceptados, busquedas, yStarts, tx;
uint32_t intentosV2, exitosV2;
enum { ERROR_FINAL_INESPERADO, ERROR_TIMEOUT_MOVIMIENTO, MOV_AUTOMATICO_V2 };
void noInterrupts() {} void interrupts() {}
float posicionYmm() { return ySteps/pasosPorMmY; }
float posicionXmm() { return armX; }
float rangoXmm() { return 446; }
long leerPasosY() { return ySteps; }
long leerPasosZ() { return zSteps; }
long limiteMinimoZPasos() { return -7300; }
long posicionCapturaZV2() { return limiteMinimoZPasos(); }
long posicionPrecapturaZV2() { return -4300; }
float posicionCatchYV2() { return 0; }
float tiempoDescensoZV2Segundos() { return 1.46f; }
float tiempoDescensoFinalZV2Segundos() { return .6f; }
float anticipacionCierreCatchSegundos() { return .5f; }
float calcularUmbralDisparoYV2() { return -150; }
float calcularUmbralCierrePinzaYV2() { return -velocidadBandaMmS*1.1f; }
bool objetivoXEnCurso() { return xActive; }
bool objetivoYEnCurso() { return objetivoYActivo; }
bool objetivoZEnCurso() { return zActive; }
void detenerX() { movX=0; xActive=false; }
void detenerY() { movY=0; objetivoYActivo=false; }
void detenerZ() { movZ=0; zActive=false; }
void detenerTodos() { detenerX(); detenerY(); detenerZ(); }
void fijarPasosZ(long z) { zSteps=z; }
void moverZHasta(long z, int) { zTarget=z; zActive=true; movZ=z>zSteps?1:-1;
  if (z<limiteMinimoZPasos()) ++busquedas; }
void moverYHasta(long y, int div) { objetivoY=y; objetivoYActivo=true;
  divisorY=div; movY=y>ySteps?1:-1; ++yStarts; }
bool iniciarMovimientoXY(float x, float y, float, int) {
  if (!xyOK) return false;
  armX=x; movimientoPosicionadoActivo=true; xActive=true; movX=1;
  moverYHasta(lroundf(y*pasosPorMmY), DIV_POSICION); return true;
}
void entrarErrorSistema(int, const char*) { ++errores; detenerTodos(); }
void iniciarCancelacionAutomaticoV2(const char*, bool, bool=false) {
  ++cancelaciones; automaticoV2.fase=V2_CANCELANDO; detenerTodos(); }
void finalizarCancelacionAutomaticoV2() { automaticoV2.fase=V2_ESPERANDO_PIEZA; }
bool camaraListaCompleta() { return camaraOK; }
bool encoderListoAutomaticoV2() { return encoderOK; }
bool bandaEnMovimientoAutomaticoV2() { return bandaOK; }
bool objetivoCamaraValido() { return true; }
void aceptarObjetivoAutomaticoV2() { ++aceptados; }
bool actualizarObjetivoMovilV2() { automaticoV2.objetivoBrazoY=pieceY; return predictionOK; }
bool posicionZSeguraV2(long z) { return z>=-7300 && z<=0; }
void registrarEventoPortentaV2(const char*, const char*) {}
void registrarAckPortentaV2(const char*) {}
bool enviarPaquetePortenta() { ++tx; return true; }
void enviarOrdenCierreCatchAhora() { assert(limiteZabajo); ++cierres;
  codigoAckObjetivo=ACK_OBJ_CERRAR_PINZA; }
void cambiarFaseAutomaticoV2(FaseAutomaticoV2 f) {
  automaticoV2.fase=f; automaticoV2.inicioFase=now; automaticoV2.inicioEstable=0; }
void completarResultadoV2(const char* r) {
  assert(!strcmp(r,"CICLO_ENTREGADO_NO_VERIFICADO")); assert(zSteps==0);
  ++resultados; codigoAckObjetivo=ACK_OBJ_COMPLETADO;
  cambiarFaseAutomaticoV2(V2_COMPLETADO); }
void reiniciarAutomaticoV2() { automaticoV2={}; automaticoV2.fase=V2_ESPERANDO_PIEZA; }
void reset(FaseAutomaticoV2 phase=V2_SIGUIENDO_PIEZA) {
  automaticoV2={}; automaticoV2.fase=phase; automaticoV2.secuencia=42;
  automaticoV2.objetivoBrazoX=armX=10;
  now=1000; automaticoV2.inicioFase=now;
  btConectado=encoderOK=bandaOK=camaraOK=predictionOK=xyOK=comunicacionI2CHabilitada=true;
  eventoBotonTriangulo=eventoBotonX=limiteZarriba=limiteZabajo=false;
  limiteXmas=limiteXmenos=limiteYmas=limiteYmenos=false;
  movimientoPosicionadoActivo=xActive=zActive=objetivoYActivo=false;
  movX=movY=movZ=0; pieceY=0; ySteps=0; zSteps=-4300; velocidadBandaMmS=50;
  errores=cancelaciones=cierres=resultados=aceptados=busquedas=yStarts=tx=0;
  intentosV2=exitosV2=0; codigoAckObjetivo=ACK_OBJ_ACEPTADO;
  secuenciaObjetivoRecibida=ackSecuenciaObjetivo=42;
}
void finishXYZ() { detenerTodos(); movimientoPosicionadoActivo=false; zSteps=zTarget; }
bool seguirPiezaY(float piezaY) {
    const float yMin = -RANGO_FISICO_Y_MM * 0.5f +
        MARGEN_SEGURIDAD_MM + ML_SEGUIMIENTO_MARGEN_Y_MM;
    const float yMax = RANGO_FISICO_Y_MM * 0.5f -
        MARGEN_SEGURIDAD_MM - ML_SEGUIMIENTO_MARGEN_Y_MM;
    if (!isfinite(piezaY) || piezaY > yMax || limiteYmas || limiteYmenos) {
        detenerY();
        return false;
    }
    const float destinoMm = fmaxf(yMin, fminf(yMax, piezaY));
    const long destino = lroundf(destinoMm * pasosPorMmY);
    const long actual = leerPasosY();
    const long tolerancia = lroundf(1.5f * pasosPorMmY);
    const int8_t direccion = destino > actual ? 1 : -1;
    if (labs(destino - actual) <= tolerancia) {
        detenerY();
    } else {
        noInterrupts();
        const bool continuar = objetivoYActivo && movY == direccion &&
            divisorY == DIV_POSICION;
        if (continuar) objetivoY = destino;
        interrupts();
        if (!continuar) moverYHasta(destino, DIV_POSICION);
    }
    return true;
}
bool seguirPiezaYAutomaticoV2() {
    if (!seguirPiezaY(automaticoV2.objetivoBrazoY)) return false;
    automaticoV2.ultimoErrorY = automaticoV2.objetivoBrazoY - posicionYmm();
    if (millis() - automaticoV2.ultimoLogSeguimiento >= ML_SEGUIMIENTO_LOG_MS) {
        automaticoV2.ultimoLogSeguimiento = millis();
        registrarEventoPortentaV2("AUTO_TRACK", "Y sigue pieza por encoder");
    }
    return true;
}
bool iniciarTrasladoEntrega() {
    const float xEntrega = rangoXmm() * 0.5f - ML_MARGEN_FINAL_DERECHO_MM;
    if (!iniciarMovimientoXY(xEntrega, posicionCatchYV2(), 0.0f,
                             MOV_AUTOMATICO_V2)) return false;
    moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
    return true;
}
void procesarModoAutomaticoV2() {
    if (automaticoV2.fase == V2_CANCELANDO) {
        if ((movZ > 0 && limiteZarriba) || (movZ < 0 && limiteZabajo)) {
            detenerTodos();
            entrarErrorSistema(ERROR_FINAL_INESPERADO,
                               "Final de Z durante retirada V2");
        } else if (!objetivoZEnCurso() && movZ == 0) {
            finalizarCancelacionAutomaticoV2();
        } else if (millis() - automaticoV2.inicioFase > V2_TIMEOUT_FASE_MS) {
            detenerTodos();
            entrarErrorSistema(ERROR_TIMEOUT_MOVIMIENTO,
                               "Timeout retirando Z en Automatico V2");
        }
        return;
    }

    if (!btConectado) {
        iniciarCancelacionAutomaticoV2("control Bluetooth desconectado", true);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        iniciarCancelacionAutomaticoV2("cancelado por usuario", false, true);
        return;
    }
    const bool finalZAbajoEsperado =
        (automaticoV2.fase == V2_BAJANDO_CATCH ||
         automaticoV2.fase == V2_BAJANDO_ENTREGA) &&
        movZ < 0 && limiteZabajo;
    const bool finalInesperado =
        (movX > 0 && limiteXmas) || (movX < 0 && limiteXmenos) ||
        (movY > 0 && limiteYmas) || (movY < 0 && limiteYmenos) ||
        (movZ > 0 && limiteZarriba) ||
        (movZ < 0 && limiteZabajo && !finalZAbajoEsperado);
    if (finalInesperado) {
        detenerTodos();
        entrarErrorSistema(ERROR_FINAL_INESPERADO,
                           "Final de carrera durante Automatico V2");
        return;
    }
    const bool prediccionLlegadaActivaV2 =
        (automaticoV2.fase >= V2_PREPOSICIONANDO &&
         automaticoV2.fase <= V2_DISPARANDO_CATCH) ||
        automaticoV2.fase == V2_BAJANDO_Z ||
        automaticoV2.fase == V2_BAJANDO_CATCH ||
        automaticoV2.fase == V2_ESPERANDO_CATCH_AUTOMATICO ||
        automaticoV2.fase == V2_SIGUIENDO_PIEZA ||
        automaticoV2.fase == V2_CERRANDO_PINZA;
    if (!camaraListaCompleta()) {
        if (automaticoV2.fase == V2_ESPERANDO_PIEZA) {
            detenerTodos();
            return;
        }
        // Despues de publicar, las coordenadas de camara estan congeladas.
        // La llegada depende solo del conteo del encoder.
    }
    if (!encoderListoAutomaticoV2()) {
        if (automaticoV2.fase == V2_ESPERANDO_PIEZA) {
            detenerTodos();
            return;
        }
        if (prediccionLlegadaActivaV2) {
            iniciarCancelacionAutomaticoV2("encoder no valido", false);
            return;
        }
    }
    if (prediccionLlegadaActivaV2 && !bandaEnMovimientoAutomaticoV2()) {
        iniciarCancelacionAutomaticoV2("la banda se detuvo", false);
        return;
    }
    if (automaticoV2.fase != V2_ESPERANDO_PIEZA &&
        automaticoV2.fase != V2_COMPLETADO &&
        millis() - automaticoV2.inicioFase > V2_TIMEOUT_FASE_MS) {
        iniciarCancelacionAutomaticoV2("timeout de fase", false);
        return;
    }

    switch (automaticoV2.fase) {
        case V2_PREPARANDO_ESPERA:
            detenerX();
            detenerY();
            if (!objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    entrarErrorSistema(
                        ERROR_TIMEOUT_MOVIMIENTO,
                        "Z no alcanzo la posicion segura al entrar a V2"
                    );
                    return;
                }
                cambiarFaseAutomaticoV2(V2_ESPERANDO_PIEZA);
            }
            break;

        case V2_ESPERANDO_PIEZA:
            detenerTodos();
            // La igualdad solo bloquea la repeticion del mismo mensaje I2C.
            // Una deteccion posterior, incluso de la misma pieza o clase,
            // genera otra secuencia y se acepta normalmente.
            if (!objetivoCamaraValido() || secuenciaObjetivoRecibida == 0 ||
                secuenciaObjetivoRecibida == ackSecuenciaObjetivo) {
                return;
            }
            aceptarObjetivoAutomaticoV2();
            break;

        case V2_PREPOSICIONANDO:
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion de pieza no valida", false);
                return;
            }
            if (automaticoV2.objetivoBrazoY >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2(
                    "pieza rebaso el catch antes de terminar la preposicion", false);
                return;
            }
            if (!movimientoPosicionadoActivo && !objetivoXEnCurso() &&
                !objetivoYEnCurso() && movX == 0 && movY == 0) {
                // Z se inicio junto con XY y termina en la altura de espera.
                cambiarFaseAutomaticoV2(V2_BAJANDO_Z);
            }
            break;

        case V2_ESPERANDO_LLEGADA:
            detenerY();
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion de pieza no valida preparando descenso", false);
                return;
            }
            automaticoV2.ultimoErrorY =
                automaticoV2.objetivoBrazoY - posicionCatchYV2();
            automaticoV2.umbralDisparoY = calcularUmbralDisparoYV2();
            if (automaticoV2.objetivoBrazoY >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2(
                    "pieza rebaso la estacion de catch", false);
                return;
            }
            // Si esta ruta de preparacion se usa, Z baja solo hasta
            // precaptura y espera elevado hasta el disparo final.
            if (automaticoV2.objetivoBrazoY >
                automaticoV2.umbralDisparoY) {
                iniciarCancelacionAutomaticoV2(
                    "pieza demasiado cerca para prebajar Z con seguridad", false);
                return;
            }
            Serial.print(F("[AUTO V2] Precaptura Z inmediata piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 2);
            Serial.print(F(" limite_seguro="));
            Serial.print(automaticoV2.umbralDisparoY, 2);
            Serial.print(F(" vel="));
            Serial.print(velocidadBandaMmS, 2);
            Serial.print(F(" tZ="));
            Serial.print(tiempoDescensoZV2Segundos(), 3);
            Serial.print(F(" reservaX="));
            Serial.println(V2_RESERVA_MANUAL_CATCH_S, 3);
            registrarEventoPortentaV2(
                "TRIGGER", "preposicion lista; Z espera sobre DIN04"
            );
            cambiarFaseAutomaticoV2(V2_DISPARANDO_CATCH);
            break;

        case V2_DISPARANDO_CATCH: {
            detenerY();
            const long precapturaZ = posicionPrecapturaZV2();
            if (!posicionZSeguraV2(precapturaZ)) {
                iniciarCancelacionAutomaticoV2(
                    "altura Z de precaptura fuera de rango", false);
                return;
            }
            moverZHasta(precapturaZ, DIV_POSICION);
            cambiarFaseAutomaticoV2(V2_BAJANDO_Z);
            break;
        }

        case V2_BAJANDO_Z:
            detenerY();
            if (limiteZabajo) {
                iniciarCancelacionAutomaticoV2(
                    "DIN04 activo antes del catch", false);
                return;
            }
            if (!objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != posicionPrecapturaZV2()) {
                    iniciarCancelacionAutomaticoV2(
                        "Z no alcanzo la precaptura", false);
                    return;
                }
                automaticoV2.ultimoErrorX =
                    automaticoV2.objetivoBrazoX - posicionXmm();
                // El encoder seguira la pieza mientras Z espera sobre DIN04.
                if (encoderListoAutomaticoV2()) {
                    actualizarObjetivoMovilV2();
                }
                automaticoV2.ultimoErrorY =
                    automaticoV2.objetivoBrazoY - posicionCatchYV2();
                automaticoV2.instanteListoCatch = millis();
                automaticoV2.conteoListoCatch = conteoEncoderBanda;
                automaticoV2.catchAutomaticoDisparado = false;
                // X ya no gobierna el catch. Se limpia cualquier evento viejo
                // para que no afecte otro modo al terminar este ciclo.
                eventoBotonX = false;
                detenerY();
                detenerZ();
                Serial.print(F("[AUTO V2] LISTO PARA CATCH AUTOMATICO X="));
                Serial.print(posicionXmm(), 2);
                Serial.print(F(" Y="));
                Serial.print(posicionYmm(), 2);
                Serial.print(F(" errorX="));
                Serial.print(automaticoV2.ultimoErrorX, 2);
                Serial.print(F(" errorY="));
                Serial.print(automaticoV2.ultimoErrorY, 2);
                Serial.print(F(" velocidad="));
                Serial.print(velocidadBandaMmS, 2);
                Serial.print(F(" encoder="));
                Serial.print(conteoEncoderBanda);
                Serial.print(F(" z_espera="));
                Serial.print(leerPasosZ());
                Serial.print(F(" margen_DIN04="));
                Serial.println(Z_MARGEN_PRECAPTURA_PASOS);
                registrarEventoPortentaV2(
                    "READY_CATCH",
                    "Z en precaptura; esperando descenso final por encoder"
                );
                cambiarFaseAutomaticoV2(AUTO_V2_SEGUIMIENTO_Y
                    ? V2_SIGUIENDO_PIEZA : V2_ESPERANDO_CATCH_AUTOMATICO);
            }
            break;

        case V2_SUBIENDO_Z:
            detenerY();
            if (!objetivoZEnCurso() && movZ == 0) {
                if (!automaticoV2.catchAutomaticoDisparado) {
                    iniciarCancelacionAutomaticoV2(
                        "Z subio sin disparo de catch automatico", false);
                    return;
                }
                if (!iniciarTrasladoEntrega()) {
                    iniciarCancelacionAutomaticoV2("entrega fuera de rango", false);
                    return;
                }
                cambiarFaseAutomaticoV2(V2_MOVIENDO_ENTREGA);
            }
            break;

        case V2_SIGUIENDO_PIEZA: {
            detenerX();
            detenerZ();
            eventoBotonX = false;
            if (limiteZabajo || leerPasosZ() != posicionPrecapturaZV2()) {
                iniciarCancelacionAutomaticoV2("Z salio de precaptura siguiendo", false);
                return;
            }
            if (!actualizarObjetivoMovilV2() || !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("seguimiento Y fuera de recorrido", false);
                return;
            }
            const float yMax = RANGO_FISICO_Y_MM * 0.5f -
                MARGEN_SEGURIDAD_MM - ML_SEGUIMIENTO_MARGEN_Y_MM;
            const float finCatchY = automaticoV2.objetivoBrazoY +
                fmaxf(0.0f, velocidadBandaMmS) *
                (tiempoDescensoFinalZV2Segundos() + anticipacionCierreCatchSegundos() +
                 ML_SEGUIMIENTO_RESERVA_S);
            if (!isfinite(finCatchY) || finCatchY > yMax) {
                iniciarCancelacionAutomaticoV2("sin recorrido Y para completar catch", false);
                return;
            }
            const float yMin = -RANGO_FISICO_Y_MM * 0.5f +
                MARGEN_SEGURIDAD_MM + ML_SEGUIMIENTO_MARGEN_Y_MM;
            if (automaticoV2.objetivoBrazoY < yMin ||
                fabsf(automaticoV2.ultimoErrorY) > V2_ERROR_ESTABLE_MM ||
                fabsf(automaticoV2.objetivoBrazoX - posicionXmm()) > V2_ERROR_ESTABLE_MM) {
                automaticoV2.inicioEstable = 0;
                break;
            }
            if (automaticoV2.inicioEstable == 0) automaticoV2.inicioEstable = millis();
            if (millis() - automaticoV2.inicioEstable < V2_SEGUIMIENTO_ESTABLE_MS) break;
            automaticoV2.busquedaFinalZActiva = false;
            moverZHasta(posicionCapturaZV2(), DIV_POSICION);
            registrarEventoPortentaV2("TRIGGER", "seguimiento estable; descenso automatico");
            cambiarFaseAutomaticoV2(V2_BAJANDO_CATCH);
            break;
        }

        case V2_ESPERANDO_CATCH_AUTOMATICO: {
            // Z espera sobre DIN04; el disparo incluye el tiempo del ultimo
            // descenso y del cierre para alcanzar la pieza en Y=0.
            detenerX();
            detenerY();
            detenerZ();
            // X queda ignorada: ya no es confirmacion ni cambia la trayectoria.
            eventoBotonX = false;
            if (limiteZabajo || leerPasosZ() != posicionPrecapturaZV2()) {
                iniciarCancelacionAutomaticoV2(
                    "Z salio de la altura de precaptura", false);
                return;
            }
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion invalida esperando catch automatico", false);
                return;
            }
            automaticoV2.ultimoErrorY =
                automaticoV2.objetivoBrazoY - posicionCatchYV2();
            automaticoV2.umbralCierrePinzaY = calcularUmbralCierrePinzaYV2();
            if (automaticoV2.objetivoBrazoY <
                automaticoV2.umbralCierrePinzaY) break;

            automaticoV2.busquedaFinalZActiva = false;
            moverZHasta(posicionCapturaZV2(), DIV_POSICION);
            Serial.print(F("[AUTO V2] Descenso final Z por encoder piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 3);
            Serial.print(F(" umbral_cierre="));
            Serial.print(automaticoV2.umbralCierrePinzaY, 3);
            Serial.print(F(" tZ="));
            Serial.println(tiempoDescensoFinalZV2Segundos(), 3);
            cambiarFaseAutomaticoV2(V2_BAJANDO_CATCH);
            break;
        }

        case V2_BAJANDO_CATCH: {
            detenerX();
            if (!AUTO_V2_SEGUIMIENTO_Y) detenerY();
            eventoBotonX = false;
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion invalida durante descenso final", false);
                return;
            }
            if (AUTO_V2_SEGUIMIENTO_Y && !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("fin recorrido Y bajando catch", false);
                return;
            }
            if (!AUTO_V2_SEGUIMIENTO_Y && automaticoV2.objetivoBrazoY >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2(
                    "pieza rebaso el catch antes de DIN04", false);
                return;
            }
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                automaticoV2.busquedaFinalZActiva = false;
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!automaticoV2.busquedaFinalZActiva) {
                    automaticoV2.busquedaFinalZActiva = true;
                    moverZHasta(limiteMinimoZPasos() -
                        V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                        V2_DIV_BUSQUEDA_FINAL_Z);
                    registrarEventoPortentaV2(
                        "Z_SEARCH", "busqueda DIN04 durante catch");
                    break;
                }
                iniciarCancelacionAutomaticoV2(
                    "DIN04 no aparecio durante catch", false);
                return;
            }
            if (AUTO_V2_SEGUIMIENTO_Y &&
                fabsf(automaticoV2.ultimoErrorY) > V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2("Y no alineada al confirmar DIN04", false);
                return;
            }
            automaticoV2.catchAutomaticoDisparado = true;
            automaticoV2.anguloCatch = posServoRot;
            ++intentosV2;
            automaticoV2.ultimoErrorY =
                automaticoV2.objetivoBrazoY -
                (AUTO_V2_SEGUIMIENTO_Y ? posicionYmm() : posicionCatchYV2());
            automaticoV2.fase = V2_CERRANDO_PINZA;
            automaticoV2.inicioFase = millis();
            enviarOrdenCierreCatchAhora();
            Serial.print(F("[AUTO V2] DIN04; ORDEN CERRAR piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 3);
            Serial.print(F(" espera_ms="));
            Serial.println(millis() - automaticoV2.instanteListoCatch);
            registrarEventoPortentaV2(
                "GRIP_COMMAND", "DIN04 confirmado; cerrar pinza");
            registrarAckPortentaV2("orden cerrar pinza");
            break;
        }

        case V2_CERRANDO_PINZA:
            detenerX();
            if (!AUTO_V2_SEGUIMIENTO_Y) detenerY();
            detenerZ();
            eventoBotonX = false;
            if (!limiteZabajo) {
                iniciarCancelacionAutomaticoV2(
                    "se perdio DIN04 mientras cerraba la pinza", false);
                return;
            }
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion invalida mientras cerraba la pinza", false);
                return;
            }
            if (AUTO_V2_SEGUIMIENTO_Y && !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("fin recorrido Y cerrando pinza", false);
                return;
            }
            automaticoV2.ultimoErrorY = automaticoV2.objetivoBrazoY -
                (AUTO_V2_SEGUIMIENTO_Y ? posicionYmm() : posicionCatchYV2());
            if (millis() - automaticoV2.inicioFase <
                V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) break;

            Serial.print(F("[AUTO V2] PINZA CERRADA piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 3);
            Serial.print(F(" errorY="));
            Serial.println(automaticoV2.ultimoErrorY, 3);
            registrarEventoPortentaV2(
                "CAPTURE", "PINZA CERRADA POR PREDICCION DE ENCODER");
            // La garra ya termino su recorrido de cierre. Desde aqui Z puede
            // retirarse verticalmente sin intervencion del operador.
            detenerY();
            if (!iniciarTrasladoEntrega()) {
                iniciarCancelacionAutomaticoV2("destino de entrega fuera de rango", false);
                return;
            }
            cambiarFaseAutomaticoV2(V2_MOVIENDO_ENTREGA);
            break;

        case V2_MOVIENDO_ENTREGA:
            if (!movimientoPosicionadoActivo && !objetivoXEnCurso() &&
                !objetivoYEnCurso() && movX == 0 && movY == 0 &&
                !objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    iniciarCancelacionAutomaticoV2("Z no alcanzo altura de entrega", false);
                    return;
                }
                automaticoV2.busquedaFinalZActiva = false;
                moverZHasta(posicionCapturaZV2(), DIV_POSICION);
                cambiarFaseAutomaticoV2(V2_BAJANDO_ENTREGA);
            }
            break;

        case V2_BAJANDO_ENTREGA:
            detenerX();
            detenerY();
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!automaticoV2.busquedaFinalZActiva) {
                    automaticoV2.busquedaFinalZActiva = true;
                    moverZHasta(limiteMinimoZPasos() - V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                                V2_DIV_BUSQUEDA_FINAL_Z);
                    break;
                }
                iniciarCancelacionAutomaticoV2("DIN04 ausente durante entrega", false);
                return;
            }
            codigoAckObjetivo = ACK_OBJ_ABRIR_PINZA;
            cambiarFaseAutomaticoV2(V2_ABRIENDO_PINZA);
            if (comunicacionI2CHabilitada && enviarPaquetePortenta())
                tAnteriorEstadoESP = millis();
            registrarEventoPortentaV2("RELEASE", "abrir pinza en entrega derecha");
            break;

        case V2_ABRIENDO_PINZA:
            detenerTodos();
            if (millis() - automaticoV2.inicioFase <
                ML_TIEMPO_SERVO_MS + V2_LATENCIA_ORDEN_PINZA_MS) break;
            moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
            cambiarFaseAutomaticoV2(V2_SUBIENDO_FINAL);
            break;

        case V2_SUBIENDO_FINAL:
            if (!objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    iniciarCancelacionAutomaticoV2("Z no regreso a HOME tras entrega", false);
                    return;
                }
                ++exitosV2;
                completarResultadoV2("CICLO_ENTREGADO_NO_VERIFICADO");
            }
            break;

        case V2_COMPLETADO:
            detenerTodos();
            if (millis() - automaticoV2.inicioFase >= V2_TIEMPO_COMPLETADO_MS) {
                reiniciarAutomaticoV2();
            }
            break;

        case V2_CANCELANDO:
            break;
    }
}
int main() {
static_assert(sizeof(PaquetePortentaAESP)==32 && sizeof(PaqueteESPAPortenta)==32,"Paquetes");
// Seguimiento real: actualiza objetivo sin reiniciar pulsos y limita el recorrido.
reset(); pieceY=10; assert(seguirPiezaY(pieceY)); assert(yStarts==1);
assert(seguirPiezaY(11)); assert(yStarts==1 && objetivoY==1100);
assert(!seguirPiezaY(163) && !movY); assert(!seguirPiezaY(NAN));
// Un ciclo completo: alineacion estable, DIN04, una orden, entrega y Z seguro.
reset(); eventoBotonX=true; procesarModoAutomaticoV2();
assert(!cierres && !eventoBotonX && automaticoV2.fase==V2_SIGUIENDO_PIEZA);
now+=299; procesarModoAutomaticoV2(); assert(!zActive);
now+=1; procesarModoAutomaticoV2(); assert(zActive && automaticoV2.fase==V2_BAJANDO_CATCH);
pieceY=3; ySteps=300; procesarModoAutomaticoV2(); assert(movZ<0 && !cierres);
limiteZabajo=true; procesarModoAutomaticoV2(); procesarModoAutomaticoV2();
assert(cierres==1 && automaticoV2.fase==V2_CERRANDO_PINZA && !resultados);
now+=500; procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_MOVIENDO_ENTREGA && codigoAckObjetivo==ACK_OBJ_CERRAR_PINZA);
assert(armX==213 && objetivoY==0 && zTarget==0);
// La banda puede detenerse durante entrega sin cancelar la pieza ya capturada.
bandaOK=encoderOK=false; limiteZabajo=false; finishXYZ();
procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_BAJANDO_ENTREGA && zActive);
procesarModoAutomaticoV2(); assert(codigoAckObjetivo==ACK_OBJ_CERRAR_PINZA);
limiteZabajo=true; procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_ABRIENDO_PINZA && codigoAckObjetivo==ACK_OBJ_ABRIR_PINZA && !resultados);
now+=500; procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_SUBIENDO_FINAL);
limiteZabajo=false; finishXYZ(); procesarModoAutomaticoV2();
assert(resultados==1 && exitosV2==1 && !errores && !cancelaciones);
now+=500; procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_ESPERANDO_PIEZA);
// Fallos impiden el cierre y la entrega no confirma exito anticipadamente.
reset(); pieceY=160; procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(); ySteps=1000; now+=301; procesarModoAutomaticoV2(); assert(!zActive);
reset(); btConectado=false; procesarModoAutomaticoV2(); assert(cancelaciones==1);
reset(); eventoBotonTriangulo=true; procesarModoAutomaticoV2(); assert(cancelaciones==1);
reset(); encoderOK=false; procesarModoAutomaticoV2(); assert(cancelaciones==1);
reset(); bandaOK=false; procesarModoAutomaticoV2(); assert(cancelaciones==1);
reset(); predictionOK=false; procesarModoAutomaticoV2(); assert(cancelaciones==1);
reset(); now+=30001; procesarModoAutomaticoV2(); assert(cancelaciones==1);
reset(); movY=1; limiteYmas=true; procesarModoAutomaticoV2(); assert(errores==1);
reset(V2_BAJANDO_CATCH); procesarModoAutomaticoV2(); assert(busquedas==1 && !cierres);
detenerZ(); procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_BAJANDO_CATCH); limiteZabajo=true; ySteps=1000;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_MOVIENDO_ENTREGA); zSteps=-100;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !resultados);
reset(V2_BAJANDO_ENTREGA); procesarModoAutomaticoV2(); assert(busquedas==1);
detenerZ(); procesarModoAutomaticoV2(); assert(cancelaciones==1 && !resultados);
reset(V2_CERRANDO_PINZA); limiteZabajo=true; now+=500; xyOK=false;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !resultados);
// Ruta fija sigue disponible; X no dispara ni habilita excepciones de ML V2.
AUTO_V2_SEGUIMIENTO_Y=false; reset(V2_ESPERANDO_CATCH_AUTOMATICO);
pieceY=-100; eventoBotonX=true; procesarModoAutomaticoV2(); assert(!zActive && !eventoBotonX);
pieceY=-55; procesarModoAutomaticoV2(); assert(zActive);
pieceY=6; procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
puts("PASS: seguimiento real, ciclo automatico hasta entrega, ruta fija, DIN04, timeout, encoder, control, limites y resultado reservado hasta Z seguro");
}

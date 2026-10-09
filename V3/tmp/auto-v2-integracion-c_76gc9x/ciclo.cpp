#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <initializer_list>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/ProtocoloRS485.h"
using namespace ProtocoloRS485;
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/CapturaFijaV2.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/AjusteTemporalCapturaV2.h"
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
    V2_SIGUIENDO_PIEZA = 16,
    V2_EVALUANDO_CATCH = 17
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
    int32_t ajusteProbadoMs;
    bool referenciaCatchRegistrada;
    bool referenciaCatchProyectada;
    unsigned long referenciaCatchMs;
    unsigned long disparoCatchMs;
    unsigned long din04CatchMs;
    unsigned long cierreCatchMs;
    float errorYDisparo;
    float velocidadDisparo;
    int32_t conteoDisparoCatch;
    float brazoYDisparo;
    float piezaYDisparo;
    uint8_t flagsReferencia;
    uint8_t anguloPreparado;
    unsigned long aceptadoMs;
    unsigned long inicioXEstable;
    unsigned long inicioAbajoMs;
    unsigned long ultimoLogPrediccion;
    int32_t conteoInicioCierre;
    unsigned long ordenCierreMs;
    unsigned long inicioRetiradaMs;
    int32_t desfaseCatchMs;
    bool confirmacionDin04Pendiente;
    unsigned long inicioConfirmacionDin04Ms;
};
const float RANGO_FISICO_Y_MM = 336.0f;
const float MARGEN_SEGURIDAD_MM = 2.0f;
const long PASOS_SEPARACION = 300;
constexpr float ENCODER_DIAMETRO_RUEDA_MM = 49.0f;
constexpr float ENCODER_RELACION_ENCODER_RUEDA = 1.0f;
constexpr int ENCODER_CUENTAS_X2_POR_VUELTA = 2048;
constexpr float ENCODER_PI = 3.14159265358979323846f;
constexpr float ENCODER_MM_POR_CUENTA =
    ENCODER_PI * ENCODER_DIAMETRO_RUEDA_MM /
    (static_cast<float>(ENCODER_CUENTAS_X2_POR_VUELTA) *
     ENCODER_RELACION_ENCODER_RUEDA);
const uint16_t DIV_CAL_LENTA = 8;
const uint16_t DIV_POSICION = 1;
constexpr float V2_POSICION_CATCH_Y_MM = 0.0f;
constexpr long V2_Z_SEGURO_PASOS = 0;
constexpr float V2_ERROR_ESTABLE_MM = 5.0f;
constexpr unsigned long V2_TIMEOUT_FASE_MS = 30000UL;
constexpr float V2_RESERVA_MANUAL_CATCH_S = 0.50f;
constexpr unsigned long V2_TIEMPO_COMPLETADO_MS = 500UL;
constexpr unsigned long V2_TIEMPO_CIERRE_PINZA_MS = 450UL;
constexpr unsigned long V2_LATENCIA_ORDEN_PINZA_MS = 158UL;
constexpr long V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS = 2L * PASOS_SEPARACION;
constexpr uint16_t V2_DIV_BUSQUEDA_FINAL_Z = DIV_CAL_LENTA;
constexpr unsigned long ML_TIEMPO_SERVO_MS = 450UL;
constexpr float ML_MARGEN_FINAL_DERECHO_MM = 10.0f;
constexpr float ML_SEGUIMIENTO_MARGEN_Y_MM = 4.0f;
constexpr float ML_SEGUIMIENTO_RESERVA_S = 0.20f;
constexpr unsigned long ML_SEGUIMIENTO_LOG_MS = 200UL;
constexpr unsigned long V2_SEGUIMIENTO_ESTABLE_MS = 300UL;
constexpr int32_t V2_AJUSTE_DISPARO_CATCH_MS = 0;
constexpr int32_t V2_DESFASE_CATCH_MS = 0;
constexpr unsigned long V2_PERIODO_LOG_TELEMETRIA_MS = 50UL;
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/AjusteCatchV2.h"
bool V2_CAPTURA_FIJA_VALIDADA = false;
bool V2_HABILITAR_PRUEBAS_CATCH = true;
float V2_VENTANA_CAPTURA_Y_MM = 5.0f;
float V2_ERROR_GEOMETRIA_CAPTURA_MM = 2.0f;
float V2_MARGEN_TIEMPO_Z_S = 0.12f;
float V2_TIEMPO_CONTACTO_MIN_S = 0.0f;
float V2_TIEMPO_CONTACTO_MAX_S = 0.45f;
float V2_LIMITE_ACELERACION_MM_S2 = 200.0f;
float V2_ERROR_MODELO_ACELERACION_MM_S2 = 20.0f;
float V2_VELOCIDAD_MIN_CAPTURA_MM_S = 1.0f;
float V2_VELOCIDAD_MAX_CAPTURA_MM_S = 100.0f;
float V2_VELOCIDAD_MAX_PRUEBA_MM_S = 150.0f;
float V2_ERROR_RELATIVO_ESCALA = 0.01f;
unsigned long V2_ERROR_REFERENCIA_CAMARA_MS = 50UL;
long V2_ALTURA_LIBRE_BANDA_PASOS = 1000L;
float V2_TIEMPO_RETIRADA_BANDA_MAX_S = 0.25f;
float V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM = 10.0f;
unsigned long V2_VENTANA_VELOCIDAD_MS = 100UL;
unsigned long V2_EDAD_MAXIMA_VELOCIDAD_MS = 150UL;
unsigned long V2_ASENTAMIENTO_X_MS = 80UL;
unsigned long V2_ASENTAMIENTO_GIRO_MS = 450UL;
unsigned long V2_ESPERA_MAXIMA_ABAJO_MS = 80UL;
unsigned long V2_CONFIRMACION_DIN04_MS = 40UL;

CapturaFijaV2::Estimador estimadorCapturaFijaV2;
float velocidadMotores=0.0001f;
float escalaEncoderMmPorCuenta=0.001f;
int8_t signoEncoderAvance=1;
AjusteCatchV2::Sesion ajusteCatchV2;
uint8_t paginaCambiosCatch=0;
void imprimirCambiosCatchV2() {}
// Alternar la ruta solo en el simulador permite verificar ambas configuraciones.
bool AUTO_V2_SEGUIMIENTO_Y=true;
bool calMode=false;
bool ajusteCatchV2Seleccionado() { return calMode; }
ContextoAutomaticoV2 automaticoV2;
int32_t desfaseCatchConfiguradoMs=V2_DESFASE_CATCH_MS;
struct Terminal { template<class T> void print(const T&, int=0) {};
template<class T> void println(const T&, int=0) {}; } Serial;
unsigned long now=1000, tAnteriorEstadoESP;
unsigned long ultimoPaqueteValidoMs=1000;
unsigned long millis() { return now; }
bool btConectado, eventoBotonTriangulo, eventoBotonX, eventoBotonCuadrado, eventoBotonCirculo;
bool limiteZarriba, limiteZabajo, limiteXmas, limiteXmenos, limiteYmas, limiteYmenos;
bool encoderOK, bandaOK, camaraOK, predictionOK, xyOK, comunicacionRS485Habilitada;
bool movimientoPosicionadoActivo, objetivoYActivo, xActive, zActive;
int movX, movY, movZ, divisorY=DIV_POSICION, posServoRot=59, posServoPin=0;
long objetivoY, ySteps, zSteps, zTarget;
float pasosPorMmY=100, armX, pieceY, velocidadBandaMmS;
int32_t conteoEncoderBanda;
uint16_t secuenciaObjetivoRecibida, ackSecuenciaObjetivo, secuenciaObjetivoEnMovimiento;
uint8_t codigoAckObjetivo;
int errores, cancelaciones, cierres, resultados, aceptados, busquedas, yStarts, tx;
const char* motivoCancelacion=nullptr;
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
float anticipacionCierreCatchSegundos() {
  return (V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS)*.001f;
}
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
void iniciarCancelacionAutomaticoV2(const char* motivo, bool, bool=false) {
  motivoCancelacion=motivo;
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
int muestrasCal;
const char* resultadoCal;
void registrarAjusteCatchV2(const char* evento, const char* resultado="PENDIENTE") {
  if (!strcmp(evento,"CAL_SAMPLE")) { ++muestrasCal; resultadoCal=resultado; }
}
void registrarAckPortentaV2(const char*) {}
bool enviarPaquetePortenta() { ++tx; return true; }
void enviarOrdenCierreCatchAhora() { assert(limiteZabajo); ++cierres;
  codigoAckObjetivo=ACK_OBJ_CERRAR_PINZA; }
void cambiarFaseAutomaticoV2(FaseAutomaticoV2 f) {
  automaticoV2.fase=f; automaticoV2.inicioFase=now; automaticoV2.inicioEstable=0; }
void completarResultadoV2(const char* r) {
  assert(!strcmp(r,"CICLO_ENTREGADO_NO_VERIFICADO") || !strcmp(r,"CICLO_EVALUADO_POR_OPERADOR")); assert(zSteps==0);
  ++resultados; codigoAckObjetivo=ACK_OBJ_COMPLETADO;
  cambiarFaseAutomaticoV2(V2_COMPLETADO); }
void reiniciarAutomaticoV2() { automaticoV2={}; automaticoV2.fase=V2_ESPERANDO_PIEZA; }
void reset(FaseAutomaticoV2 phase=V2_SIGUIENDO_PIEZA) {
  automaticoV2={}; automaticoV2.fase=phase; automaticoV2.secuencia=42;
  automaticoV2.objetivoBrazoX=armX=10;
  now=1000; automaticoV2.inicioFase=now;
  btConectado=encoderOK=bandaOK=camaraOK=predictionOK=xyOK=comunicacionRS485Habilitada=true;
  eventoBotonTriangulo=eventoBotonX=eventoBotonCuadrado=eventoBotonCirculo=limiteZarriba=limiteZabajo=false;
  muestrasCal=0; resultadoCal=nullptr;
  limiteXmas=limiteXmenos=limiteYmas=limiteYmenos=false;
  movimientoPosicionadoActivo=xActive=zActive=objetivoYActivo=false;
  movX=movY=movZ=0; pieceY=0; ySteps=0; zSteps=-4300; velocidadBandaMmS=50;
  errores=cancelaciones=cierres=resultados=aceptados=busquedas=yStarts=tx=0;
  motivoCancelacion=nullptr;
  intentosV2=exitosV2=0; codigoAckObjetivo=ACK_OBJ_ACEPTADO;
  secuenciaObjetivoRecibida=ackSecuenciaObjetivo=42;
  ultimoPaqueteValidoMs=now;
  posServoRot=59; posServoPin=0;
  automaticoV2.flagsReferencia=MASCARA_FLAGS_OBJETIVO_V2;
  automaticoV2.anguloPreparado=59;
  automaticoV2.aceptadoMs=now-V2_ASENTAMIENTO_GIRO_MS;
  automaticoV2.inicioXEstable=now-V2_ASENTAMIENTO_X_MS;
}
void finishXYZ() { detenerTodos(); movimientoPosicionadoActivo=false; zSteps=zTarget; }
bool seguimientoYAutomaticoV2() {
    // AJUSTE CATCH conserva sus ensayos de seguimiento y etiquetas humanas.
    return ajusteCatchV2Seleccionado() || AUTO_V2_SEGUIMIENTO_Y;
}
bool capturaFijaV2EnPrueba() {
    return !V2_CAPTURA_FIJA_VALIDADA && V2_HABILITAR_PRUEBAS_CATCH;
}
float tiempoContactoNominalFijoV2() {
    const float baseS = (V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) * 0.001f;
    return AjusteTemporalCapturaV2::horizonte(baseS,
        capturaFijaV2EnPrueba() ? automaticoV2.desfaseCatchMs : 0);
}
bool perfilCapturaFijaV2Valido() {
    return (V2_CAPTURA_FIJA_VALIDADA || capturaFijaV2EnPrueba()) &&
        isfinite(V2_TIEMPO_CONTACTO_MIN_S) && isfinite(V2_TIEMPO_CONTACTO_MAX_S) &&
        V2_TIEMPO_CONTACTO_MAX_S >= V2_TIEMPO_CONTACTO_MIN_S &&
        V2_TIEMPO_CONTACTO_MIN_S >= 0.0f &&
        V2_TIEMPO_CONTACTO_MAX_S <= V2_TIEMPO_CIERRE_PINZA_MS * 0.001f &&
        isfinite(V2_MARGEN_TIEMPO_Z_S) && V2_MARGEN_TIEMPO_Z_S >= 0.0f &&
        isfinite(V2_VENTANA_CAPTURA_Y_MM) && isfinite(V2_ERROR_GEOMETRIA_CAPTURA_MM) &&
        V2_VENTANA_CAPTURA_Y_MM > V2_ERROR_GEOMETRIA_CAPTURA_MM &&
        V2_ERROR_GEOMETRIA_CAPTURA_MM >= 0.0f &&
        isfinite(V2_ERROR_RELATIVO_ESCALA) &&
        V2_ERROR_RELATIVO_ESCALA >= 0.0f && V2_ERROR_RELATIVO_ESCALA < 1.0f &&
        isfinite(V2_VELOCIDAD_MIN_CAPTURA_MM_S) && isfinite(V2_VELOCIDAD_MAX_CAPTURA_MM_S) &&
        V2_VELOCIDAD_MIN_CAPTURA_MM_S > 0.0f &&
        V2_VELOCIDAD_MAX_CAPTURA_MM_S > V2_VELOCIDAD_MIN_CAPTURA_MM_S &&
        (!capturaFijaV2EnPrueba() ||
         (isfinite(V2_VELOCIDAD_MAX_PRUEBA_MM_S) &&
          V2_VELOCIDAD_MAX_PRUEBA_MM_S > V2_VELOCIDAD_MIN_CAPTURA_MM_S &&
          automaticoV2.desfaseCatchMs >= AjusteTemporalCapturaV2::MIN_MS &&
          automaticoV2.desfaseCatchMs <= AjusteTemporalCapturaV2::MAX_MS &&
          isfinite(tiempoContactoNominalFijoV2()) && tiempoContactoNominalFijoV2() >= 0.05f)) &&
        isfinite(V2_LIMITE_ACELERACION_MM_S2) &&
        isfinite(V2_ERROR_MODELO_ACELERACION_MM_S2) &&
        V2_LIMITE_ACELERACION_MM_S2 > V2_ERROR_MODELO_ACELERACION_MM_S2 &&
        V2_ERROR_MODELO_ACELERACION_MM_S2 >= 0.0f &&
        isfinite(V2_TIEMPO_RETIRADA_BANDA_MAX_S) && V2_TIEMPO_RETIRADA_BANDA_MAX_S > 0.0f &&
        V2_ALTURA_LIBRE_BANDA_PASOS > 0 &&
        V2_ALTURA_LIBRE_BANDA_PASOS <= posicionPrecapturaZV2() - posicionCapturaZV2() &&
        V2_TIEMPO_RETIRADA_BANDA_MAX_S >=
            V2_ALTURA_LIBRE_BANDA_PASOS * 2.0f * velocidadMotores &&
        isfinite(V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM) && V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM > 0.0f;
}
CapturaFijaV2::Movimiento movimientoCapturaFijaV2() {
    CapturaFijaV2::Movimiento m = estimadorCapturaFijaV2.movimiento();
    const float edadS = static_cast<float>(millis() - m.muestraMs) * 0.001f;
    m.valido = m.valido && millis() - m.muestraMs <= V2_EDAD_MAXIMA_VELOCIDAD_MS;
    m.errorAceleracion += V2_ERROR_MODELO_ACELERACION_MM_S2;
    // La ventana mide velocidad en su centro. Proyectarla al instante actual
    // conserva en la envolvente la antiguedad de la muestra y su cuantizacion.
    const float desfaseS = edadS + 0.5f * V2_VENTANA_VELOCIDAD_MS * 0.001f;
    m.velocidad += m.aceleracion * desfaseS;
    m.errorVelocidad += m.errorAceleracion * desfaseS;
    // En ensayo se exige el rango nominal; el perfil validado exige que
    // tambien quepa toda la envolvente de incertidumbre en ese rango.
    const float margen = capturaFijaV2EnPrueba() ? 0.0f : m.errorVelocidad;
    const float velocidadMaxima = capturaFijaV2EnPrueba()
        ? V2_VELOCIDAD_MAX_PRUEBA_MM_S : V2_VELOCIDAD_MAX_CAPTURA_MM_S;
    m.valido = m.valido && isfinite(m.velocidad) && isfinite(m.aceleracion) &&
        m.velocidad - margen >= V2_VELOCIDAD_MIN_CAPTURA_MM_S &&
        m.velocidad + margen <= velocidadMaxima;
    return m;
}
float errorPosicionCapturaFijaV2(const CapturaFijaV2::Movimiento &m) {
    const float recorrido = fabsf(static_cast<float>(CapturaFijaV2::diferencia(
        conteoEncoderBanda, automaticoV2.conteoReferencia)) * escalaEncoderMmPorCuenta);
    const float incertidumbreTiempo = V2_ERROR_REFERENCIA_CAMARA_MS * 0.001f;
    return V2_ERROR_GEOMETRIA_CAPTURA_MM +
        recorrido * V2_ERROR_RELATIVO_ESCALA +
        fmaxf(V2_VELOCIDAD_MAX_CAPTURA_MM_S, fabsf(m.velocidad) + m.errorVelocidad) *
            incertidumbreTiempo +
        0.5f * fmaxf(V2_LIMITE_ACELERACION_MM_S2,
            fabsf(m.aceleracion) + m.errorAceleracion) *
            incertidumbreTiempo * incertidumbreTiempo;
}
bool retencionBandaAdmisibleV2(bool cierreEnCurso = false, float hastaCierreS = 0.0f) {
    const CapturaFijaV2::Movimiento m = movimientoCapturaFijaV2();
    // El presupuesto provisional de arrastre no es una medida fisica.
    // En ensayo se mantiene el encoder fresco y avance dentro del rango;
    // se mide el arrastre durante el ciclo antes de validar ese presupuesto.
    if (capturaFijaV2EnPrueba()) return m.valido;
    const float totalS = (V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) * 0.001f +
        V2_TIEMPO_RETIRADA_BANDA_MAX_S;
    const float transcurrido = cierreEnCurso
        ? (millis() - automaticoV2.ordenCierreMs) * 0.001f : 0.0f;
    const float recorrido = cierreEnCurso
        ? fmaxf(0.0f, signoEncoderAvance * escalaEncoderMmPorCuenta *
            static_cast<float>(CapturaFijaV2::diferencia(conteoEncoderBanda,
                automaticoV2.conteoInicioCierre))) : 0.0f;
    // Incluye todo el avance desde la orden: una cota conservadora del arrastre
    // posterior al primer contacto, que aun no dispone de un sensor fisico.
    const float inicioS = cierreEnCurso ? 0.0f : hastaCierreS;
    const CapturaFijaV2::Intervalo antes = CapturaFijaV2::predecir(0.0f, m,
        inicioS, inicioS, 0.0f, V2_LIMITE_ACELERACION_MM_S2);
    const CapturaFijaV2::Intervalo despues = CapturaFijaV2::predecir(recorrido, m,
        inicioS, inicioS + fmaxf(0.0f, totalS - transcurrido),
        0.0f, V2_LIMITE_ACELERACION_MM_S2);
    return antes.valido && despues.valido &&
        despues.maximo - antes.minimo <= V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM;
}
float horizonteContactoMinV2() { return V2_TIEMPO_CONTACTO_MIN_S; }
float horizonteContactoMaxV2() {
    return V2_TIEMPO_CONTACTO_MAX_S +
        static_cast<float>(V2_LATENCIA_ORDEN_PINZA_MS) * 0.001f;
}
CapturaFijaV2::Intervalo predecirContactoFijoV2(float restanteZMinS,
                                             float restanteZMaxS) {
    const CapturaFijaV2::Movimiento m = movimientoCapturaFijaV2();
    return CapturaFijaV2::predecir(automaticoV2.objetivoBrazoY, m,
        restanteZMinS + horizonteContactoMinV2(),
        restanteZMaxS + horizonteContactoMaxV2(),
        errorPosicionCapturaFijaV2(m), V2_LIMITE_ACELERACION_MM_S2);
}
CapturaFijaV2::Intervalo contactoNominalFijoV2(float tiempoS) {
    CapturaFijaV2::Movimiento nominal = movimientoCapturaFijaV2();
    const CapturaFijaV2::Movimiento muestra = estimadorCapturaFijaV2.movimiento();
    // No extrapolar aceleracion que la propia cuantizacion de la ventana
    // no permite distinguir. Mantener m/v/a originales en el registro.
    if (fabsf(nominal.aceleracion) <= muestra.errorAceleracion) {
        nominal.velocidad = muestra.velocidad;
        nominal.aceleracion = 0.0f;
    }
    nominal.valido = nominal.valido &&
        fabsf(muestra.aceleracion) <= V2_LIMITE_ACELERACION_MM_S2;
    // Solo para decidir el ensayo. La incertidumbre original se conserva
    // y registra por separado; cero aqui no representa un error medido.
    nominal.errorVelocidad = 0.0f;
    nominal.errorAceleracion = 0.0f;
    return CapturaFijaV2::predecir(automaticoV2.objetivoBrazoY, nominal,
        tiempoS, tiempoS, 0.0f, V2_LIMITE_ACELERACION_MM_S2);
}
CapturaFijaV2::Decision decisionContactoFijoV2(const CapturaFijaV2::Intervalo &perfil,
                                             float tiempoNominalS) {
    const CapturaFijaV2::Intervalo p = capturaFijaV2EnPrueba()
        ? contactoNominalFijoV2(tiempoNominalS) : perfil;
    return CapturaFijaV2::evaluar(p, posicionCatchYV2(), V2_VENTANA_CAPTURA_Y_MM);
}
void registrarPerfilCapturaFijaV2() {
    Serial.print(F("V2LOG|P|event=FIXED_PROFILE|physical_validated="));
    Serial.print(V2_CAPTURA_FIJA_VALIDADA ? 1 : 0);
    Serial.print(F("|test_mode=")); Serial.print(capturaFijaV2EnPrueba() ? 1 : 0);
    Serial.print(F("|configured_catch_offset_ms=")); Serial.print(desfaseCatchConfiguradoMs);
    Serial.print(F("|catch_offset_ms=")); Serial.print(capturaFijaV2EnPrueba() ? automaticoV2.desfaseCatchMs : 0);
    Serial.print(F("|nominal_contact_s=")); Serial.print(tiempoContactoNominalFijoV2(), 3);
    Serial.print(F("|timing_preliminary=")); Serial.print(V2_CAPTURA_FIJA_VALIDADA ? 0 : 1);
    Serial.print(F("|capture_window_mm=")); Serial.print(V2_VENTANA_CAPTURA_Y_MM, 3);
    Serial.print(F("|geometry_error_mm=")); Serial.print(V2_ERROR_GEOMETRIA_CAPTURA_MM, 3);
    Serial.print(F("|scale_error_relative=")); Serial.print(V2_ERROR_RELATIVO_ESCALA, 6);
    Serial.print(F("|reference_error_ms=")); Serial.print(V2_ERROR_REFERENCIA_CAMARA_MS);
    Serial.print(F("|model_acceleration_error_mm_s2=")); Serial.print(V2_ERROR_MODELO_ACELERACION_MM_S2, 3);
    Serial.print(F("|velocity_min_mm_s=")); Serial.print(V2_VELOCIDAD_MIN_CAPTURA_MM_S, 3);
    Serial.print(F("|velocity_max_mm_s=")); Serial.print(V2_VELOCIDAD_MAX_CAPTURA_MM_S, 3);
    Serial.print(F("|trial_velocity_max_mm_s=")); Serial.print(V2_VELOCIDAD_MAX_PRUEBA_MM_S, 3);
    Serial.print(F("|admission_velocity_max_mm_s=")); Serial.print(capturaFijaV2EnPrueba()
        ? V2_VELOCIDAD_MAX_PRUEBA_MM_S : V2_VELOCIDAD_MAX_CAPTURA_MM_S, 3);
    Serial.print(F("|lift_clearance_steps=")); Serial.print(V2_ALTURA_LIBRE_BANDA_PASOS);
    Serial.print(F("|lift_clearance_max_s=")); Serial.print(V2_TIEMPO_RETIRADA_BANDA_MAX_S, 3);
    Serial.print(F("|drag_max_mm=")); Serial.print(V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM, 3);
    Serial.print(F("|z_margin_s=")); Serial.print(V2_MARGEN_TIEMPO_Z_S, 3);
    Serial.print(F("|contact_min_s=")); Serial.print(V2_TIEMPO_CONTACTO_MIN_S, 3);
    Serial.print(F("|contact_max_s=")); Serial.print(V2_TIEMPO_CONTACTO_MAX_S, 3);
    Serial.print(F("|transport_max_ms=")); Serial.println(V2_LATENCIA_ORDEN_PINZA_MS);
}
bool xYGiroListosCapturaFijaV2() {
    const bool poseLista = !objetivoXEnCurso() && !objetivoYEnCurso() &&
        movX == 0 && movY == 0 &&
        fabsf(automaticoV2.objetivoBrazoX - posicionXmm()) <= V2_ERROR_ESTABLE_MM &&
        fabsf(posicionYmm() - posicionCatchYV2()) <= V2_ERROR_ESTABLE_MM &&
        posServoRot == automaticoV2.anguloPreparado && posServoPin == 0 &&
        (automaticoV2.flagsReferencia & MASCARA_FLAGS_OBJETIVO_V2) ==
            MASCARA_FLAGS_OBJETIVO_V2;
    if (!poseLista) { automaticoV2.inicioXEstable = 0; return false; }
    if (automaticoV2.inicioXEstable == 0) automaticoV2.inicioXEstable = millis();
    return millis() - automaticoV2.inicioXEstable >= V2_ASENTAMIENTO_X_MS &&
        millis() - automaticoV2.aceptadoMs >= V2_ASENTAMIENTO_GIRO_MS;
}
void registrarPrediccionCapturaFijaV2(const char *etapa,
                                    const CapturaFijaV2::Intervalo &p,
                                    float tMinS, float tMaxS, float tiempoNominalS) {
    if (automaticoV2.ultimoLogPrediccion != 0 &&
        millis() - automaticoV2.ultimoLogPrediccion < V2_PERIODO_LOG_TELEMETRIA_MS) return;
    automaticoV2.ultimoLogPrediccion = millis();
    const CapturaFijaV2::Movimiento m = movimientoCapturaFijaV2();
    Serial.print(F("V2LOG|P|event=FIXED_PREDICTION|ms=")); Serial.print(millis());
    Serial.print(F("|obj=")); Serial.print(automaticoV2.secuencia);
    Serial.print(F("|stage=")); Serial.print(etapa);
    Serial.print(F("|physical_validated=")); Serial.print(V2_CAPTURA_FIJA_VALIDADA ? 1 : 0);
    Serial.print(F("|test_mode=")); Serial.print(capturaFijaV2EnPrueba() ? 1 : 0);
    Serial.print(F("|catch_offset_ms=")); Serial.print(capturaFijaV2EnPrueba() ? automaticoV2.desfaseCatchMs : 0);
    Serial.print(F("|enc=")); Serial.print(conteoEncoderBanda);
    Serial.print(F("|remaining_mm=")); Serial.print(posicionCatchYV2() - automaticoV2.objetivoBrazoY, 3);
    Serial.print(F("|velocity_mm_s=")); Serial.print(m.velocidad, 3);
    Serial.print(F("|acceleration_mm_s2=")); Serial.print(m.aceleracion, 3);
    Serial.print(F("|velocity_error_mm_s=")); Serial.print(m.errorVelocidad, 3);
    Serial.print(F("|acceleration_error_mm_s2=")); Serial.print(m.errorAceleracion, 3);
    Serial.print(F("|position_error_mm=")); Serial.print(errorPosicionCapturaFijaV2(m), 3);
    Serial.print(F("|horizon_min_s=")); Serial.print(tMinS, 3);
    Serial.print(F("|horizon_max_s=")); Serial.print(tMaxS, 3);
    Serial.print(F("|pred_y_min_mm=")); Serial.print(p.valido ? p.minimo : NAN, 3);
    Serial.print(F("|pred_y_max_mm=")); Serial.print(p.valido ? p.maximo : NAN, 3);
    Serial.print(F("|window_decision=")); Serial.print(static_cast<uint8_t>(
        CapturaFijaV2::evaluar(p, posicionCatchYV2(), V2_VENTANA_CAPTURA_Y_MM)));
    const CapturaFijaV2::Intervalo nominal = contactoNominalFijoV2(tiempoNominalS);
    Serial.print(F("|nominal_horizon_s=")); Serial.print(tiempoNominalS, 3);
    Serial.print(F("|nominal_y_mm=")); Serial.print(nominal.valido ? nominal.minimo : NAN, 3);
    const CapturaFijaV2::Movimiento muestra = estimadorCapturaFijaV2.movimiento();
    Serial.print(F("|nominal_acceleration_mm_s2=")); Serial.print(
        fabsf(m.aceleracion) <= muestra.errorAceleracion ? 0.0f : m.aceleracion, 3);
    Serial.print(F("|nominal_window_decision=")); Serial.println(static_cast<uint8_t>(
        CapturaFijaV2::evaluar(nominal, posicionCatchYV2(), V2_VENTANA_CAPTURA_Y_MM)));
}
bool procesarCapturaFijaV2() {
    if (seguimientoYAutomaticoV2() ||
        (automaticoV2.fase != V2_ESPERANDO_CATCH_AUTOMATICO &&
         automaticoV2.fase != V2_BAJANDO_CATCH &&
         automaticoV2.fase != V2_CERRANDO_PINZA &&
         automaticoV2.fase != V2_SUBIENDO_Z)) return false;
    if (automaticoV2.fase == V2_SUBIENDO_Z &&
        leerPasosZ() - posicionCapturaZV2() >= V2_ALTURA_LIBRE_BANDA_PASOS) return false;
    detenerX(); detenerY(); eventoBotonX = false;
    if (!perfilCapturaFijaV2Valido()) {
        iniciarCancelacionAutomaticoV2("perfil fisico de captura fija no validado", false);
        return true;
    }
    if (!actualizarObjetivoMovilV2()) {
        iniciarCancelacionAutomaticoV2("referencia fija no valida", false);
        return true;
    }
    if (fabsf(automaticoV2.objetivoBrazoX - posicionXmm()) > V2_ERROR_ESTABLE_MM ||
        fabsf(posicionYmm() - posicionCatchYV2()) > V2_ERROR_ESTABLE_MM ||
        posServoRot != automaticoV2.anguloPreparado) {
        iniciarCancelacionAutomaticoV2("X/Y/giro perdieron alineacion fija", false);
        return true;
    }
    if (automaticoV2.fase == V2_SUBIENDO_Z) {
        if ((!capturaFijaV2EnPrueba() &&
             millis() - automaticoV2.inicioRetiradaMs > V2_TIEMPO_RETIRADA_BANDA_MAX_S * 1000.0f) ||
            !retencionBandaAdmisibleV2(true))
            iniciarCancelacionAutomaticoV2("retirada fija fuera del perfil de arrastre", false);
        return true;
    }
    if (automaticoV2.fase == V2_CERRANDO_PINZA) {
        detenerZ();
        if (!limiteZabajo) {
            iniciarCancelacionAutomaticoV2("DIN04 perdido durante cierre fijo", false);
            return true;
        }
        if (!retencionBandaAdmisibleV2(true)) {
            iniciarCancelacionAutomaticoV2("arrastre de banda excede perfil durante cierre", false);
            return true;
        }
        const float transcurrido = static_cast<float>(millis() - automaticoV2.inicioFase) * 0.001f;
        const float horizonteControl = capturaFijaV2EnPrueba()
            ? tiempoContactoNominalFijoV2() : horizonteContactoMaxV2();
        if (transcurrido < horizonteControl) {
            const float tMin = fmaxf(0.0f, horizonteContactoMinV2() - transcurrido);
            const float tMax = fmaxf(0.0f, horizonteContactoMaxV2() - transcurrido);
            const CapturaFijaV2::Intervalo p = CapturaFijaV2::predecir(
                automaticoV2.objetivoBrazoY, movimientoCapturaFijaV2(), tMin, tMax,
                errorPosicionCapturaFijaV2(movimientoCapturaFijaV2()), V2_LIMITE_ACELERACION_MM_S2);
            const float tNominal = fmaxf(0.0f, tiempoContactoNominalFijoV2() - transcurrido);
            registrarPrediccionCapturaFijaV2("CLOSE", p, tMin, tMax, tNominal);
            if (decisionContactoFijoV2(p, tNominal) !=
                CapturaFijaV2::DENTRO)
                iniciarCancelacionAutomaticoV2("contacto fijo salio de ventana durante cierre", false);
            return true;
        }
        // El tiempo de contacto y el tiempo hasta cierre completo son distintos.
        if (millis() - automaticoV2.inicioFase <
            V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) return true;
        const uint32_t despuesDeOrden = static_cast<uint32_t>(
            ultimoPaqueteValidoMs - automaticoV2.ordenCierreMs);
        if (posServoPin != 130 || despuesDeOrden == 0 || despuesDeOrden >= 0x80000000UL ||
            secuenciaObjetivoRecibida != automaticoV2.secuencia) {
            registrarEventoPortentaV2("GRIP_NOT_APPLIED", "sin respuesta ESP posterior confirmando orden");
            iniciarCancelacionAutomaticoV2("cierre fijo no confirmado por ESP", false);
            return true;
        }
        registrarEventoPortentaV2("CAPTURE", "CIERRE_ORDENADO_AGARRE_NO_VERIFICADO");
        // Retirada vertical completa antes del traslado lateral de entrega.
        automaticoV2.inicioRetiradaMs = millis();
        moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
        cambiarFaseAutomaticoV2(V2_SUBIENDO_Z);
        return true;
    }
    if (automaticoV2.fase == V2_ESPERANDO_CATCH_AUTOMATICO) {
        detenerZ();
        if (limiteZabajo || leerPasosZ() != posicionPrecapturaZV2()) {
            iniciarCancelacionAutomaticoV2("Z no esta en precaptura fija", false);
            return true;
        }
        if (!xYGiroListosCapturaFijaV2()) return true;
        if (!retencionBandaAdmisibleV2(false,
                tiempoDescensoFinalZV2Segundos() + V2_MARGEN_TIEMPO_Z_S)) {
            iniciarCancelacionAutomaticoV2("sin margen fisico para cierre/retirada fija", false);
            return true;
        }
        const float tZ = tiempoDescensoFinalZV2Segundos();
        const float tMin = fmaxf(0.0f, tZ - V2_MARGEN_TIEMPO_Z_S);
        const float tMax = tZ + V2_MARGEN_TIEMPO_Z_S;
        const CapturaFijaV2::Intervalo p = predecirContactoFijoV2(tMin, tMax);
        const float tNominal = tZ + tiempoContactoNominalFijoV2();
        registrarPrediccionCapturaFijaV2("WAIT", p,
            tMin + horizonteContactoMinV2(), tMax + horizonteContactoMaxV2(), tNominal);
        const CapturaFijaV2::Decision d = decisionContactoFijoV2(p, tNominal);
        if (d == CapturaFijaV2::ANTES) return true;
        // No disparar el ensayo en el borde temprano (-ventana). Apuntar al
        // centro deja margen a la cuantizacion del encoder al recalcular v/a
        // durante descenso y cierre; sus comprobaciones conservan +/-ventana.
        if (capturaFijaV2EnPrueba() && d == CapturaFijaV2::DENTRO &&
            contactoNominalFijoV2(tNominal).minimo < posicionCatchYV2()) return true;
        if (d != CapturaFijaV2::DENTRO) {
            iniciarCancelacionAutomaticoV2("sin ventana segura para descenso fijo", false);
            return true;
        }
        automaticoV2.disparoCatchMs = millis();
        ++intentosV2;
        automaticoV2.inicioAbajoMs = 0;
        automaticoV2.confirmacionDin04Pendiente = false;
        automaticoV2.velocidadDisparo = velocidadBandaMmS;
        automaticoV2.conteoDisparoCatch = conteoEncoderBanda;
        moverZHasta(posicionCapturaZV2(), DIV_POSICION);
        registrarEventoPortentaV2("TRIGGER", capturaFijaV2EnPrueba()
            ? "ENSAYO_NOMINAL; descenso final por encoder"
            : "ventana fija valida; descenso final por encoder");
        cambiarFaseAutomaticoV2(V2_BAJANDO_CATCH);
        return true;
    }
    if (posServoPin != 0) {
        iniciarCancelacionAutomaticoV2("pinza no abierta antes del contacto fijo", false);
        return true;
    }
    if (limiteZabajo) {
        detenerZ(); fijarPasosZ(limiteMinimoZPasos());
        if (automaticoV2.inicioAbajoMs == 0) automaticoV2.inicioAbajoMs = millis();
    }
    const float pasosRestantes = fabsf(static_cast<float>(leerPasosZ() - posicionCapturaZV2()));
    const float tRestante = limiteZabajo ? 0.0f : pasosRestantes * 2.0f * velocidadMotores;
    const float tMin = limiteZabajo ? 0.0f : fmaxf(0.0f, tRestante - V2_MARGEN_TIEMPO_Z_S);
    const float tMax = limiteZabajo ? 0.0f : tRestante + V2_MARGEN_TIEMPO_Z_S;
    const CapturaFijaV2::Intervalo p = predecirContactoFijoV2(tMin, tMax);
    if (!retencionBandaAdmisibleV2(false, tMax)) {
        iniciarCancelacionAutomaticoV2("sin margen de arrastre tras descenso fijo", false);
        return true;
    }
    registrarPrediccionCapturaFijaV2("DESCEND", p,
        tMin + horizonteContactoMinV2(), tMax + horizonteContactoMaxV2(),
        tRestante + tiempoContactoNominalFijoV2());
    const CapturaFijaV2::Decision d = decisionContactoFijoV2(p,
        tRestante + tiempoContactoNominalFijoV2());
    if (d == CapturaFijaV2::INVALIDA || d == CapturaFijaV2::INCIERTA ||
        d == CapturaFijaV2::DESPUES) {
        // La ultima prediccion puede caer entre dos logs periodicos. Emitir
        // esa muestra al cancelar para distinguir ruido, velocidad y desfase.
        automaticoV2.ultimoLogPrediccion = 0;
        registrarPrediccionCapturaFijaV2("DESCEND_ABORT", p,
            tMin + horizonteContactoMinV2(), tMax + horizonteContactoMaxV2(),
            tRestante + tiempoContactoNominalFijoV2());
        iniciarCancelacionAutomaticoV2("velocidad/incertidumbre impide contacto fijo", false);
        return true;
    }
    if (objetivoZEnCurso() || movZ != 0) return true;
    if (!limiteZabajo) {
        // DIN04 se lee antes de esta maquina; el ticker pudo terminar el
        // ultimo paso despues de esa lectura. Confirmar quieto sin buscar
        // por debajo del limite ni cerrar con el sensor ausente.
        if (!automaticoV2.confirmacionDin04Pendiente) {
            automaticoV2.confirmacionDin04Pendiente = true;
            automaticoV2.inicioConfirmacionDin04Ms = millis();
            registrarEventoPortentaV2("DIN04_WAIT", "Z detenido; esperando lectura posterior al ultimo paso");
        } else if (millis() - automaticoV2.inicioConfirmacionDin04Ms >= V2_CONFIRMACION_DIN04_MS) {
            registrarEventoPortentaV2("DIN04_TIMEOUT", "sensor ausente despues de confirmar Z quieto");
            iniciarCancelacionAutomaticoV2("DIN04 ausente en descenso fijo", false);
        }
        return true;
    }
    if (d == CapturaFijaV2::ANTES) {
        if (millis() - automaticoV2.inicioAbajoMs > V2_ESPERA_MAXIMA_ABAJO_MS)
            iniciarCancelacionAutomaticoV2("espera abajo excedida sin ventana de contacto", false);
        return true;
    }
    automaticoV2.catchAutomaticoDisparado = true;
    automaticoV2.anguloCatch = posServoRot;
    automaticoV2.conteoInicioCierre = conteoEncoderBanda;
    automaticoV2.ordenCierreMs = millis();
    automaticoV2.fase = V2_CERRANDO_PINZA;
    automaticoV2.inicioFase = millis();
    enviarOrdenCierreCatchAhora();
    registrarEventoPortentaV2("GRIP_COMMAND", capturaFijaV2EnPrueba()
        ? "ENSAYO_NOMINAL; DIN04 y prediccion nominal validos"
        : "DIN04 y ventana de contacto fija validos");
    return true;
}
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
void llenarResumenCatchV2(PaquetePortentaAESP &p) {
    // Semantica exclusiva de SISTEMA_CAMBIOS_CATCH; nunca son datos de encoder.
    p.faseCalibracionBrazo = paginaCambiosCatch;
    p.conteoEncoder = ajusteCatchV2.offsetMs;
    p.velocidadEncoderUmS = ajusteCatchV2.ultimoProbadoMs;
    p.secuenciaEncoder = ajusteCatchV2.ensayos;
    p.nmPorCuentaEncoder = ajusteCatchV2.pasoMs;
    p.estadoEncoder = (ajusteCatchV2.confirmado() ? 1U : 0U) |
        (ajusteCatchV2.limiteAlcanzado ? 2U : 0U) | (ajusteCatchV2.aciertosConsecutivos << 2);
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
        automaticoV2.fase != V2_EVALUANDO_CATCH &&
        millis() - automaticoV2.inicioFase > V2_TIMEOUT_FASE_MS) {
        iniciarCancelacionAutomaticoV2("timeout de fase", false);
        return;
    }

    if (procesarCapturaFijaV2()) return;

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
            // La igualdad solo bloquea la repeticion del mismo mensaje RS485.
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
                cambiarFaseAutomaticoV2(seguimientoYAutomaticoV2()
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
            const int32_t offset = ajusteCatchV2Seleccionado()
                ? ajusteCatchV2.offsetMs : AjusteCatchV2::limitar(V2_AJUSTE_DISPARO_CATCH_MS);
            const unsigned long espera = static_cast<unsigned long>(
                static_cast<int32_t>(V2_SEGUIMIENTO_ESTABLE_MS) + offset);
            if (millis() - automaticoV2.inicioEstable < espera) break;
            if (ajusteCatchV2Seleccionado()) {
                automaticoV2.ajusteProbadoMs = offset;
                automaticoV2.referenciaCatchRegistrada = true;
                automaticoV2.referenciaCatchProyectada =
                    millis() - automaticoV2.inicioEstable < V2_SEGUIMIENTO_ESTABLE_MS;
                automaticoV2.referenciaCatchMs = automaticoV2.inicioEstable + V2_SEGUIMIENTO_ESTABLE_MS;
                registrarAjusteCatchV2("CAL_REFERENCE");
                automaticoV2.disparoCatchMs = millis();
                automaticoV2.errorYDisparo = automaticoV2.ultimoErrorY;
                automaticoV2.velocidadDisparo = velocidadBandaMmS;
                automaticoV2.conteoDisparoCatch = conteoEncoderBanda;
                automaticoV2.brazoYDisparo = posicionYmm();
                automaticoV2.piezaYDisparo = automaticoV2.objetivoBrazoY;
            }
            automaticoV2.busquedaFinalZActiva = false;
            moverZHasta(posicionCapturaZV2(), DIV_POSICION);
            registrarEventoPortentaV2("TRIGGER", "seguimiento estable; descenso automatico");
            cambiarFaseAutomaticoV2(V2_BAJANDO_CATCH);
            if (ajusteCatchV2Seleccionado()) registrarAjusteCatchV2("CAL_TRIGGER");
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
            if (!seguimientoYAutomaticoV2()) detenerY();
            eventoBotonX = false;
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion invalida durante descenso final", false);
                return;
            }
            if (seguimientoYAutomaticoV2() && !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("fin recorrido Y bajando catch", false);
                return;
            }
            if (!seguimientoYAutomaticoV2() && automaticoV2.objetivoBrazoY >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2(
                    "pieza rebaso el catch antes de DIN04", false);
                return;
            }
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                automaticoV2.busquedaFinalZActiva = false;
                if (ajusteCatchV2Seleccionado()) automaticoV2.din04CatchMs = millis();
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
            if (seguimientoYAutomaticoV2() &&
                fabsf(automaticoV2.ultimoErrorY) > V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2("Y no alineada al confirmar DIN04", false);
                return;
            }
            automaticoV2.catchAutomaticoDisparado = true;
            automaticoV2.anguloCatch = posServoRot;
            ++intentosV2;
            automaticoV2.ultimoErrorY =
                automaticoV2.objetivoBrazoY -
                (seguimientoYAutomaticoV2() ? posicionYmm() : posicionCatchYV2());
            automaticoV2.fase = V2_CERRANDO_PINZA;
            automaticoV2.inicioFase = millis();
            enviarOrdenCierreCatchAhora();
            if (ajusteCatchV2Seleccionado()) {
                automaticoV2.cierreCatchMs = automaticoV2.inicioFase;
                registrarAjusteCatchV2("CAL_GRIP");
            }
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
            if (!seguimientoYAutomaticoV2()) detenerY();
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
            if (seguimientoYAutomaticoV2() && !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("fin recorrido Y cerrando pinza", false);
                return;
            }
            automaticoV2.ultimoErrorY = automaticoV2.objetivoBrazoY -
                (seguimientoYAutomaticoV2() ? posicionYmm() : posicionCatchYV2());
            if (millis() - automaticoV2.inicioFase <
                V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) break;

            Serial.print(F("[AUTO V2] PINZA CERRADA piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 3);
            Serial.print(F(" errorY="));
            Serial.println(automaticoV2.ultimoErrorY, 3);
            registrarEventoPortentaV2(
                "CAPTURE", "PINZA CERRADA POR PREDICCION DE ENCODER");
            if (ajusteCatchV2Seleccionado()) registrarAjusteCatchV2("CAL_CLOSE");
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
            if (comunicacionRS485Habilitada && enviarPaquetePortenta())
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
                if (ajusteCatchV2Seleccionado()) {
                    eventoBotonX = eventoBotonCuadrado = eventoBotonCirculo = false;
                    cambiarFaseAutomaticoV2(V2_EVALUANDO_CATCH);
                    registrarAjusteCatchV2("CAL_AWAIT_FEEDBACK");
                    Serial.println(F("[AJUSTE CATCH] X=LA AGARRO; cuadrado=ANTES; circulo=DESPUES; triangulo=descartar/salir"));
                } else {
                    ++exitosV2;
                    completarResultadoV2("CICLO_ENTREGADO_NO_VERIFICADO");
                }
            }
            break;

        case V2_EVALUANDO_CATCH:
            detenerTodos();
            if (!eventoBotonX && !eventoBotonCuadrado && !eventoBotonCirculo) break;
            {
                const char *resultado = eventoBotonX ? "CORRECTO" :
                    (eventoBotonCuadrado ? "TEMPRANO" : "TARDE");
                eventoBotonX = eventoBotonCuadrado = eventoBotonCirculo = false;
                ajusteCatchV2.evaluar(strcmp(resultado, "CORRECTO") == 0 ? AjusteCatchV2::AGARRO :
                    (strcmp(resultado, "TEMPRANO") == 0 ? AjusteCatchV2::ANTES : AjusteCatchV2::DESPUES),
                    automaticoV2.ajusteProbadoMs);
                registrarAjusteCatchV2("CAL_SAMPLE", resultado);
                imprimirCambiosCatchV2();
                if (strcmp(resultado, "CORRECTO") == 0) ++exitosV2;
                completarResultadoV2("CICLO_EVALUADO_POR_OPERADOR");
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
void medirMovimientoSimulado() {
  estimadorCapturaFijaV2.reiniciar(0,now-400,escalaEncoderMmPorCuenta,1);
  for(int n=1;n<=4;++n)
    estimadorCapturaFijaV2.actualizar(n*1000,now-400+n*100,escalaEncoderMmPorCuenta,1);
  conteoEncoderBanda=4000;
}
void medirMovimientoPrueba() {
  estimadorCapturaFijaV2.reiniciar(0,now-400,escalaEncoderMmPorCuenta,1);
  for(int n=1;n<=4;++n)
    estimadorCapturaFijaV2.actualizar(n*67,now-400+n*100,escalaEncoderMmPorCuenta,1);
  conteoEncoderBanda=268;
  velocidadBandaMmS=estimadorCapturaFijaV2.movimiento().velocidad;
}
void medirVelocidadPrueba(float velocidad) {
  const int32_t porVentana=static_cast<int32_t>(lround(velocidad*.1f/escalaEncoderMmPorCuenta));
  estimadorCapturaFijaV2.reiniciar(0,now-400,escalaEncoderMmPorCuenta,1);
  for(int n=1;n<=4;++n)
    estimadorCapturaFijaV2.actualizar(n*porVentana,now-400+n*100,escalaEncoderMmPorCuenta,1);
  conteoEncoderBanda=4*porVentana;
  velocidadBandaMmS=estimadorCapturaFijaV2.movimiento().velocidad;
}
int32_t pulsosCuantizados(unsigned long tiempoMs, float velocidad, unsigned long faseMs) {
  return static_cast<int32_t>(floor((tiempoMs+faseMs)*.001*velocidad/escalaEncoderMmPorCuenta));
}
unsigned long comprobarCatchCuantizado(float velocidad, unsigned long faseMs=0, int32_t desfaseMs=0) {
  reset(V2_ESPERANDO_CATCH_AUTOMATICO);
  automaticoV2.desfaseCatchMs=desfaseMs;
  const unsigned long inicio=now;
  estimadorCapturaFijaV2.reiniciar(pulsosCuantizados(inicio-400,velocidad,faseMs),
    inicio-400,escalaEncoderMmPorCuenta,1);
  for(unsigned long t=inicio-390;t<=inicio;t+=10)
    estimadorCapturaFijaV2.actualizar(pulsosCuantizados(t,velocidad,faseMs),t,
      escalaEncoderMmPorCuenta,1,V2_VENTANA_VELOCIDAD_MS);
  const int32_t referencia=pulsosCuantizados(inicio,velocidad,faseMs);
  automaticoV2.conteoReferencia=referencia;
  const float inicialY=-fmaxf(100.0f,velocidad*(tiempoDescensoFinalZV2Segundos()+
    anticipacionCierreCatchSegundos()+.5f));
  unsigned long inicioDescenso=0,inicioCierre=0;
  unsigned muestrasAparenteAceleracion=0;
  for(unsigned long dt=0;dt<=9000&&automaticoV2.fase!=V2_SUBIENDO_Z&&!cancelaciones;dt+=10) {
    now=inicio+dt;
    conteoEncoderBanda=pulsosCuantizados(now,velocidad,faseMs);
    estimadorCapturaFijaV2.actualizar(conteoEncoderBanda,now,
      escalaEncoderMmPorCuenta,1,V2_VENTANA_VELOCIDAD_MS);
    if(fabsf(estimadorCapturaFijaV2.movimiento().aceleracion)>.001f)
      ++muestrasAparenteAceleracion;
    velocidadBandaMmS=estimadorCapturaFijaV2.movimiento().velocidad;
    pieceY=inicialY+escalaEncoderMmPorCuenta*CapturaFijaV2::diferencia(conteoEncoderBanda,referencia);
    if(inicioDescenso&&automaticoV2.fase==V2_BAJANDO_CATCH) {
      zSteps=-4300-static_cast<long>(now-inicioDescenso)*5;
      if(zSteps<=-7300) { zSteps=-7300; limiteZabajo=true; }
    }
    if(inicioCierre&&now-inicioCierre>=V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS) {
      posServoPin=130;ultimoPaqueteValidoMs=now;
    }
    procesarModoAutomaticoV2();
    if(!inicioDescenso&&automaticoV2.fase==V2_BAJANDO_CATCH) inicioDescenso=now;
    if(!inicioCierre&&automaticoV2.fase==V2_CERRANDO_PINZA) inicioCierre=now;
  }
  if(cancelaciones) printf("Catch cuantizado v=%.3f fase=%lu desfase=%ld: %s\n",velocidad,faseMs,
    static_cast<long>(desfaseMs),motivoCancelacion);
  assert(cierres==1&&automaticoV2.fase==V2_SUBIENDO_Z&&zTarget==0);
  assert(!cancelaciones&&!busquedas&&!movY&&!yStarts&&!V2_CAPTURA_FIJA_VALIDADA);
  assert(muestrasAparenteAceleracion>0);
  assert(now-inicioCierre>=V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS+
    static_cast<unsigned long>(desfaseMs>0?desfaseMs:0));
  return inicioDescenso-inicio;
}
int main() {
static_assert(sizeof(PaquetePortentaAESP)==32 && sizeof(PaqueteESPAPortenta)==32,"Paquetes");
int32_t desfaseAnalizado=0;
for(int32_t valor : {-500,-100,0,100,500}) {
  char texto[20]; snprintf(texto,sizeof(texto),"%+ld",static_cast<long>(valor));
  assert(AjusteTemporalCapturaV2::analizar(texto,desfaseAnalizado)&&desfaseAnalizado==valor);
}
for(const char* texto : {"", " ", "+", "501", "-501", "100x", "100.0", "1e2", "100 200", "999999999999999999999999"}) {
  desfaseAnalizado=123;
  assert(!AjusteTemporalCapturaV2::analizar(texto,desfaseAnalizado)&&desfaseAnalizado==123);
}
assert(!AjusteTemporalCapturaV2::analizar(nullptr,desfaseAnalizado));
assert(fabsf(AjusteTemporalCapturaV2::horizonte(.608f,500)-1.108f)<.0001f);
assert(fabsf(AjusteTemporalCapturaV2::horizonte(.608f,-500)-.108f)<.0001f);
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
now+=V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS-1;
procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_CERRANDO_PINZA && !resultados);
now+=1; procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_MOVIENDO_ENTREGA && codigoAckObjetivo==ACK_OBJ_CERRAR_PINZA);
assert(armX==213 && objetivoY==0 && zTarget==0);
// La banda puede detenerse durante entrega sin cancelar la pieza ya capturada.
bandaOK=encoderOK=false; limiteZabajo=false; finishXYZ();
procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_BAJANDO_ENTREGA && zActive);
procesarModoAutomaticoV2(); assert(codigoAckObjetivo==ACK_OBJ_CERRAR_PINZA);
limiteZabajo=true; procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_ABRIENDO_PINZA && codigoAckObjetivo==ACK_OBJ_ABRIR_PINZA && !resultados);
now+=ML_TIEMPO_SERVO_MS+V2_LATENCIA_ORDEN_PINZA_MS-1;
procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_ABRIENDO_PINZA && !resultados);
now+=1; procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_SUBIENDO_FINAL);
limiteZabajo=false; finishXYZ(); procesarModoAutomaticoV2();
assert(resultados==1 && exitosV2==1 && !errores && !cancelaciones);
now+=V2_TIEMPO_COMPLETADO_MS; procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_ESPERANDO_PIEZA);
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
reset(V2_CERRANDO_PINZA); limiteZabajo=true;
now+=V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS; xyOK=false;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !resultados);
// Modo independiente: disparo autonomo y adelanto/retardo acotado.
calMode=true; reset(); procesarModoAutomaticoV2(); now+=299;
procesarModoAutomaticoV2(); assert(!zActive);
now+=1; procesarModoAutomaticoV2();
assert(zActive && automaticoV2.disparoCatchMs==1300 && automaticoV2.ajusteProbadoMs==0);
ajusteCatchV2.evaluar(AjusteCatchV2::ANTES,0);
assert(ajusteCatchV2.offsetMs==50 && ajusteCatchV2.ensayos==1);
reset(); procesarModoAutomaticoV2(); now+=300; procesarModoAutomaticoV2(); assert(!zActive);
now+=50; procesarModoAutomaticoV2(); assert(zActive && automaticoV2.ajusteProbadoMs==50);
ajusteCatchV2.evaluar(AjusteCatchV2::DESPUES,50);
assert(ajusteCatchV2.offsetMs==25 && ajusteCatchV2.pasoMs==25);
ajusteCatchV2=AjusteCatchV2::Sesion(-100);
reset(); eventoBotonX=true; procesarModoAutomaticoV2(); assert(!zActive);
now+=199; procesarModoAutomaticoV2(); assert(!zActive);
now+=1; procesarModoAutomaticoV2();
assert(zActive && automaticoV2.referenciaCatchProyectada && automaticoV2.referenciaCatchMs==1300);
assert(static_cast<int32_t>(automaticoV2.disparoCatchMs-automaticoV2.referenciaCatchMs)==-100);
// Perder alineacion reinicia la espera y ninguna tecla omite las guardas.
reset(); procesarModoAutomaticoV2(); now+=199; ySteps=1000;
procesarModoAutomaticoV2(); assert(!zActive && !automaticoV2.inicioEstable);
ySteps=0; procesarModoAutomaticoV2(); now+=199; procesarModoAutomaticoV2(); assert(!zActive);
now+=1; procesarModoAutomaticoV2(); assert(zActive);
ajusteCatchV2=AjusteCatchV2::Sesion();
// Entrega conserva la reserva hasta etiquetar; tres resultados, una muestra.
for (int button=0; button<3; ++button) {
  ajusteCatchV2=AjusteCatchV2::Sesion();
  reset(V2_SUBIENDO_FINAL); zSteps=0; eventoBotonX=true;
  procesarModoAutomaticoV2(); assert(automaticoV2.fase==V2_EVALUANDO_CATCH && !eventoBotonX && !resultados);
  now+=60000; procesarModoAutomaticoV2(); assert(!cancelaciones && !muestrasCal);
  eventoBotonX=button==0; eventoBotonCuadrado=button==1; eventoBotonCirculo=button==2;
  procesarModoAutomaticoV2(); procesarModoAutomaticoV2();
  assert(muestrasCal==1 && resultados==1 && ajusteCatchV2.ensayos==1);
  assert(ajusteCatchV2.offsetMs==(button==0 ? 0 : button==1 ? 50 : -50));
  assert(exitosV2==(button==0 ? 1 : 0));
  assert(!strcmp(resultadoCal,button==0 ? "CORRECTO" : button==1 ? "TEMPRANO" : "TARDE"));
}
reset(V2_EVALUANDO_CATCH); btConectado=false; procesarModoAutomaticoV2(); assert(cancelaciones==1 && !muestrasCal);
reset(); encoderOK=false; eventoBotonX=true; procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
// Tres agarres al mismo valor confirman; un fallo retira esa confirmacion.
ajusteCatchV2=AjusteCatchV2::Sesion(-100);
for(int n=0;n<3;++n) ajusteCatchV2.evaluar(AjusteCatchV2::AGARRO,-100);
assert(ajusteCatchV2.confirmado() && ajusteCatchV2.offsetMs==-100);
PaquetePortentaAESP resumen={}; llenarResumenCatchV2(resumen);
resumen.estadoSistema=SISTEMA_CAMBIOS_CATCH; prepararPaquete(resumen);
assert(validarPaquete(resumen) && resumen.conteoEncoder==-100 && resumen.velocidadEncoderUmS==-100);
assert(resumen.secuenciaEncoder==3 && resumen.estadoEncoder==13);
ajusteCatchV2.evaluar(AjusteCatchV2::DESPUES,-100); assert(!ajusteCatchV2.confirmado());
ajusteCatchV2=AjusteCatchV2::Sesion(-200);
ajusteCatchV2.evaluar(AjusteCatchV2::DESPUES,-200);
assert(ajusteCatchV2.offsetMs==-200 && ajusteCatchV2.limiteAlcanzado);
ajusteCatchV2=AjusteCatchV2::Sesion(1000);
ajusteCatchV2.evaluar(AjusteCatchV2::ANTES,1000);
assert(ajusteCatchV2.offsetMs==1000 && ajusteCatchV2.limiteAlcanzado);
ajusteCatchV2=AjusteCatchV2::Sesion();
for(int n=0;n<10;++n) {
  ajusteCatchV2.evaluar(n%2 ? AjusteCatchV2::DESPUES : AjusteCatchV2::ANTES, ajusteCatchV2.offsetMs);
  assert(ajusteCatchV2.pasoMs>=10);
}
assert(ajusteCatchV2.pasoMs==10);
ajusteCatchV2=AjusteCatchV2::Sesion(1000);
// Sesion persiste entre ciclos; modo normal conserva el valor configurado en codigo.
reset(); assert(ajusteCatchV2.offsetMs==1000);
calMode=false; reset(); procesarModoAutomaticoV2(); now+=300;
procesarModoAutomaticoV2(); assert(zActive);
// Los parametros entregados permiten ensayar sin declarar validacion fisica.
AUTO_V2_SEGUIMIENTO_Y=false;
assert(!V2_CAPTURA_FIJA_VALIDADA && V2_HABILITAR_PRUEBAS_CATCH);
assert(V2_DESFASE_CATCH_MS==0&&desfaseCatchConfiguradoMs==0);
// Cambiar la configuracion de la siguiente pieza no altera el desfase
// reservado por la pieza activa ni el horizonte utilizado durante su ciclo.
reset(V2_ESPERANDO_CATCH_AUTOMATICO); automaticoV2.desfaseCatchMs=100;
desfaseCatchConfiguradoMs=-100;
assert(fabsf(tiempoContactoNominalFijoV2()-.708f)<.0001f);
assert(automaticoV2.desfaseCatchMs==100);
desfaseCatchConfiguradoMs=0;
escalaEncoderMmPorCuenta=ENCODER_MM_POR_CUENTA;
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba();
assert(capturaFijaV2EnPrueba() && perfilCapturaFijaV2Valido());
const float velocidadPrueba=velocidadBandaMmS;
const float horizontePrueba=tiempoDescensoFinalZV2Segundos()+anticipacionCierreCatchSegundos();
pieceY=-100; eventoBotonX=true; procesarModoAutomaticoV2();
assert(!zActive && !cierres && !cancelaciones && !eventoBotonX);
// Estar en el borde temprano de la ventana no basta: esperar al centro evita
// consumir todo el margen antes de actualizar otra ventana del encoder.
pieceY=-velocidadPrueba*horizontePrueba-4.0f;
procesarModoAutomaticoV2();
assert(!zActive&&!cancelaciones&&automaticoV2.fase==V2_ESPERANDO_CATCH_AUTOMATICO);
pieceY=-velocidadPrueba*horizontePrueba;
actualizarObjetivoMovilV2();
const auto perfilPendiente=predecirContactoFijoV2(.48f,.72f);
assert(CapturaFijaV2::evaluar(perfilPendiente,0,V2_VENTANA_CAPTURA_Y_MM)!=CapturaFijaV2::DENTRO);
assert(decisionContactoFijoV2(perfilPendiente,horizontePrueba)==CapturaFijaV2::DENTRO);
procesarModoAutomaticoV2();
assert(zActive && automaticoV2.fase==V2_BAJANDO_CATCH && intentosV2==1);
assert(!movY && !yStarts && !V2_CAPTURA_FIJA_VALIDADA);
const unsigned long descensoInicio=now;
for(int ms=100;ms<=500;ms+=100) {
  now=descensoInicio+ms; zSteps=-4300-ms*5;
  pieceY=-velocidadPrueba*(horizontePrueba-ms*.001f);
  medirMovimientoPrueba(); procesarModoAutomaticoV2();
  assert(automaticoV2.fase==V2_BAJANDO_CATCH && !cierres && !cancelaciones);
}
now=descensoInicio+600; zSteps=-7300; limiteZabajo=true;
pieceY=-velocidadPrueba*anticipacionCierreCatchSegundos();
medirMovimientoPrueba(); procesarModoAutomaticoV2();
assert(cierres==1 && automaticoV2.fase==V2_CERRANDO_PINZA && !movY);
const unsigned long cierreInicio=now;
for(int ms : {200,400,607}) {
  now=cierreInicio+ms;
  pieceY=-velocidadPrueba*fmaxf(0.0f,anticipacionCierreCatchSegundos()-ms*.001f);
  medirMovimientoPrueba(); procesarModoAutomaticoV2();
  assert(automaticoV2.fase==V2_CERRANDO_PINZA && cierres==1 && !cancelaciones);
}
now=cierreInicio+V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS;
pieceY=0; posServoPin=130; ultimoPaqueteValidoMs=now;
medirMovimientoPrueba(); procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_SUBIENDO_Z && zTarget==0 && !movY && !yStarts);
// La retirada puede exceder el tiempo provisional; XY espera hasta Z seguro.
now+=300; limiteZabajo=false; zSteps=-6500;
medirMovimientoPrueba(); procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_SUBIENDO_Z && !cancelaciones && !yStarts && armX==10);
finishXYZ(); procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_MOVIENDO_ENTREGA && armX==213 && objetivoY==0);
finishXYZ(); procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_BAJANDO_ENTREGA);
limiteZabajo=true; procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_ABRIENDO_PINZA && codigoAckObjetivo==ACK_OBJ_ABRIR_PINZA);
now+=ML_TIEMPO_SERVO_MS+V2_LATENCIA_ORDEN_PINZA_MS;
procesarModoAutomaticoV2(); limiteZabajo=false; finishXYZ(); procesarModoAutomaticoV2();
assert(resultados==1 && !cancelaciones && !V2_CAPTURA_FIJA_VALIDADA);
// Banda realmente constante con cuentas enteras y loop de 10 ms: el
// estimador conserva su historial durante espera, descenso y orden de cierre.
for(float velocidad : {20.0f,40.0f,128.0f}) {
  const unsigned long sinDesfase=comprobarCatchCuantizado(velocidad);
  const unsigned long adelanto=comprobarCatchCuantizado(velocidad,0,100);
  const unsigned long retardo=comprobarCatchCuantizado(velocidad,0,-100);
  assert(adelanto<sinDesfase&&sinDesfase<retardo);
}
// Con +100 ms, el intervalo extra de contacto sigue vigilado aunque ESP
// ya confirme cierre completo; una pieza que sale de ventana cancela.
reset(V2_CERRANDO_PINZA); automaticoV2.desfaseCatchMs=100;
medirVelocidadPrueba(128.0f); limiteZabajo=true;
automaticoV2.ordenCierreMs=now; automaticoV2.conteoInicioCierre=conteoEncoderBanda;
now+=620; medirVelocidadPrueba(128.0f); posServoPin=130; ultimoPaqueteValidoMs=now;
pieceY=-velocidadBandaMmS*(tiempoContactoNominalFijoV2()-.620f)+6.0f;
procesarModoAutomaticoV2();
assert(cancelaciones==1&&automaticoV2.fase==V2_CANCELANDO&&!resultados);
// La velocidad observada en la maqueta entra en el rango de ensayo separado.
// No extender con ella el perfil fisico estricto, que conserva maximo 100.
assert(V2_VELOCIDAD_MAX_CAPTURA_MM_S==100.0f&&V2_VELOCIDAD_MAX_PRUEBA_MM_S==150.0f);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirVelocidadPrueba(128.0f);
assert(velocidadBandaMmS>127.0f&&velocidadBandaMmS<129.0f&&movimientoCapturaFijaV2().valido);
V2_CAPTURA_FIJA_VALIDADA=true; assert(!movimientoCapturaFijaV2().valido);
V2_CAPTURA_FIJA_VALIDADA=false;
pieceY=-velocidadBandaMmS*horizontePrueba; procesarModoAutomaticoV2();
assert(zActive&&automaticoV2.fase==V2_BAJANDO_CATCH&&!cancelaciones&&!cierres);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirVelocidadPrueba(151.0f);
assert(!movimientoCapturaFijaV2().valido);
procesarModoAutomaticoV2(); assert(cancelaciones==1&&!zActive&&!cierres);
// Ensayar conserva DIN04, encoder/control, frescura y limites de movimiento.
reset(V2_BAJANDO_CATCH); medirMovimientoPrueba(); zSteps=-7300;
pieceY=-velocidadBandaMmS*anticipacionCierreCatchSegundos(); detenerZ();
procesarModoAutomaticoV2(); assert(!cancelaciones&&!cierres&&!busquedas&&!movZ);
now+=39; pieceY+=velocidadBandaMmS*.039f; medirMovimientoPrueba();
procesarModoAutomaticoV2(); assert(!cancelaciones&&!cierres&&!busquedas&&!movZ);
now+=2; pieceY+=velocidadBandaMmS*.002f; medirMovimientoPrueba();
procesarModoAutomaticoV2(); assert(cancelaciones==1&&!cierres&&!busquedas);
// DIN04 recibido en el siguiente poll confirma la posicion, sin mover Z
// fuera del objetivo ni emitir pasos de busqueda para forzar el sensor.
reset(V2_BAJANDO_CATCH); medirVelocidadPrueba(20.0f); zSteps=-7300;
pieceY=-velocidadBandaMmS*anticipacionCierreCatchSegundos(); detenerZ();
procesarModoAutomaticoV2(); assert(!cancelaciones&&!cierres&&!movZ);
now+=40; pieceY+=velocidadBandaMmS*.04f; medirVelocidadPrueba(20.0f); limiteZabajo=true;
procesarModoAutomaticoV2();
assert(cierres==1&&automaticoV2.fase==V2_CERRANDO_PINZA&&!busquedas&&!cancelaciones);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba(); encoderOK=false;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba(); btConectado=false;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba(); eventoBotonTriangulo=true;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba(); bandaOK=false;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba();
now+=V2_EDAD_MAXIMA_VELOCIDAD_MS+1;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba(); movX=1; limiteXmas=true;
procesarModoAutomaticoV2(); assert(errores==1 && !cierres);
// Acelerar despues del disparo invalida la intercepcion si la pieza ya no
// llegaria a Y=0 al contactar; no espera a DIN04 para descubrir ese rebase.
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoPrueba();
pieceY=-velocidadBandaMmS*horizontePrueba; procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_BAJANDO_CATCH && !cancelaciones);
now+=100; zSteps=-4800; pieceY+=velocidadBandaMmS*.1f;
estimadorCapturaFijaV2.reiniciar(0,now-300,escalaEncoderMmPorCuenta,1);
estimadorCapturaFijaV2.actualizar(67,now-200,escalaEncoderMmPorCuenta,1);
estimadorCapturaFijaV2.actualizar(147,now-100,escalaEncoderMmPorCuenta,1);
estimadorCapturaFijaV2.actualizar(240,now,escalaEncoderMmPorCuenta,1);
assert(movimientoCapturaFijaV2().valido);
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
// Tambien en ensayo, un temporizador sin confirmacion ESP no retira la pieza.
reset(V2_CERRANDO_PINZA); medirMovimientoPrueba(); limiteZabajo=true;
automaticoV2.ordenCierreMs=now; automaticoV2.conteoInicioCierre=conteoEncoderBanda;
now+=V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS;
medirMovimientoPrueba(); procesarModoAutomaticoV2();
assert(cancelaciones==1 && automaticoV2.fase==V2_CANCELANDO && !resultados);
// Desactivar ensayos recupera el bloqueo de perfil pendiente. Un perfil
// validado conserva la decision estricta aunque la opcion de ensayo este activa.
V2_HABILITAR_PRUEBAS_CATCH=false; escalaEncoderMmPorCuenta=.001f;
reset(V2_ESPERANDO_CATCH_AUTOMATICO);
pieceY=-100; eventoBotonX=true; procesarModoAutomaticoV2();
assert(cancelaciones==1 && !zActive && !cierres && !eventoBotonX);
V2_HABILITAR_PRUEBAS_CATCH=true;
V2_CAPTURA_FIJA_VALIDADA=true; V2_MARGEN_TIEMPO_Z_S=.001f;
V2_TIEMPO_CONTACTO_MIN_S=V2_TIEMPO_CONTACTO_MAX_S=.2f;
V2_ERROR_GEOMETRIA_CAPTURA_MM=.5f;
V2_ERROR_MODELO_ACELERACION_MM_S2=.1f;
V2_VELOCIDAD_MIN_CAPTURA_MM_S=.1f; V2_VELOCIDAD_MAX_CAPTURA_MM_S=200;
V2_ERROR_RELATIVO_ESCALA=0; V2_ERROR_REFERENCIA_CAMARA_MS=0;
V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM=100;
assert(!capturaFijaV2EnPrueba());
automaticoV2.desfaseCatchMs=100;
assert(fabsf(tiempoContactoNominalFijoV2()-anticipacionCierreCatchSegundos())<.0001f);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoSimulado();
pieceY=-100; eventoBotonX=true; procesarModoAutomaticoV2();
assert(!zActive && !eventoBotonX && !cancelaciones && !yStarts);
pieceY=-8.25f; procesarModoAutomaticoV2(); assert(zActive && !movY && !yStarts);
assert(automaticoV2.fase==V2_BAJANDO_CATCH && intentosV2==1);
// DIN04 solo no fuerza cierre: actualiza la prediccion tras la bajada.
pieceY=-2.25f; limiteZabajo=true; procesarModoAutomaticoV2();
assert(cierres==1 && automaticoV2.fase==V2_CERRANDO_PINZA && !movY);
procesarModoAutomaticoV2(); assert(cierres==1);
now+=V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS;
posServoPin=130; ultimoPaqueteValidoMs=now;
medirMovimientoSimulado(); procesarModoAutomaticoV2();
assert(automaticoV2.fase==V2_SUBIENDO_Z && zTarget==0 && !movY);
// Cumplir un temporizador sin feedback de la orden aplicada no habilita retirada/entrega.
reset(V2_CERRANDO_PINZA); medirMovimientoSimulado(); limiteZabajo=true;
automaticoV2.ordenCierreMs=now; automaticoV2.conteoInicioCierre=conteoEncoderBanda;
now+=V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS;
medirMovimientoSimulado(); procesarModoAutomaticoV2();
assert(cancelaciones==1 && automaticoV2.fase==V2_CANCELANDO);
// Cambiar la velocidad/posicion fuera de ventana cancela sin cerrar.
reset(V2_BAJANDO_CATCH); medirMovimientoSimulado(); pieceY=6;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_BAJANDO_CATCH); medirMovimientoSimulado(); pieceY=-2.25f;
detenerZ(); procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres && !busquedas);
// La garra no permanece indefinidamente abajo esperando una llegada tardia.
reset(V2_BAJANDO_CATCH); medirMovimientoSimulado(); pieceY=-100; limiteZabajo=true;
procesarModoAutomaticoV2(); assert(!cierres && !cancelaciones);
now+=V2_ESPERA_MAXIMA_ABAJO_MS+1; medirMovimientoSimulado();
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoSimulado();
pieceY=-8.25f; posServoRot=155; procesarModoAutomaticoV2();
assert(cancelaciones==1 && !cierres && !zActive);
reset(V2_ESPERANDO_CATCH_AUTOMATICO); medirMovimientoSimulado();
pieceY=-8.25f; now+=V2_EDAD_MAXIMA_VELOCIDAD_MS+1;
procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
puts("PASS: seguimiento real, ciclo automatico hasta entrega, ruta fija, DIN04, timeout, encoder, control, limites y resultado reservado hasta Z seguro");
}

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/ProtocoloRS485.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/CapturaFijaV2.h"
using namespace ProtocoloRS485;
constexpr bool CAMERA_SWAP_XY = false;
constexpr int8_t CAMERA_SIGN_X = -1;
constexpr int8_t CAMERA_SIGN_Y = -1;
constexpr float DESFASE_CAMARA_X_MM = -5.0f;
constexpr float DESFASE_CAMARA_Y_MM = 0.0f;
constexpr float CAMERA_OFFSET_X_MM = DESFASE_CAMARA_X_MM;
constexpr float CAMERA_OFFSET_Y_MM = DESFASE_CAMARA_Y_MM;
constexpr float CAMARA_A_HOME_Y_BASE_MM = 510.0f;
constexpr float V2_AJUSTE_DISTANCIA_CATCH_MM = 335.0f;
constexpr float CAMARA_A_HOME_Y_MM =
    CAMARA_A_HOME_Y_BASE_MM + V2_AJUSTE_DISTANCIA_CATCH_MM;
constexpr float ENCODER_DIAMETRO_RUEDA_MM = 49.0f;
constexpr float ENCODER_RELACION_ENCODER_RUEDA = 1.0f;
constexpr int ENCODER_CUENTAS_X2_POR_VUELTA = 2048;
constexpr float ENCODER_PI = 3.14159265358979323846f;
constexpr float ENCODER_MM_POR_CUENTA =
    ENCODER_PI * ENCODER_DIAMETRO_RUEDA_MM /
    (static_cast<float>(ENCODER_CUENTAS_X2_POR_VUELTA) *
     ENCODER_RELACION_ENCODER_RUEDA);
const uint32_t PERIODO_ENCODER_MS = 10UL;
const uint32_t TIMEOUT_MOVIMIENTO_ENCODER_MS = 500UL;
constexpr uint32_t V2_VENTANA_VELOCIDAD_MS = 100UL;
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
    uint32_t inicioFase;
    uint32_t inicioEstable;
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
    uint32_t instanteObjetivoAnterior;
    float velocidadObjetivoY;
    float velocidadInicialY;
    bool busquedaFinalZActiva;
    float umbralDisparoY;
    uint32_t instanteListoCatch;
    int32_t conteoListoCatch;
    bool catchAutomaticoDisparado;
    uint8_t anguloCatch;
    float umbralCierrePinzaY;
    float ultimoErrorY;
    float ultimoErrorX;
    uint32_t ultimoLogSeguimiento;
    int32_t ajusteProbadoMs;
    bool referenciaCatchRegistrada;
    bool referenciaCatchProyectada;
    uint32_t referenciaCatchMs;
    uint32_t disparoCatchMs;
    uint32_t din04CatchMs;
    uint32_t cierreCatchMs;
    float errorYDisparo;
    float velocidadDisparo;
    int32_t conteoDisparoCatch;
    float brazoYDisparo;
    float piezaYDisparo;
    uint8_t flagsReferencia;
    uint8_t anguloPreparado;
    uint32_t aceptadoMs;
    uint32_t inicioXEstable;
    uint32_t inicioAbajoMs;
    uint32_t ultimoLogPrediccion;
    int32_t conteoInicioCierre;
    uint32_t ordenCierreMs;
    uint32_t inicioRetiradaMs;
};

uint32_t ahora=1000;
uint32_t millis() { return ahora; }
struct QEI {
  int32_t conteo=0;int revoluciones=0;uint8_t estado=0;
  int32_t getPulses() { return conteo; }
  int getRevolutions() { return revoluciones; }
  uint8_t getCurrentState() { return estado; }
} encoders[1];
struct Terminal {
  template<class T> void print(const T&,int=0) {}
  template<class T> void println(const T&,int=0) {}
  void println() {}
} Serial;
ContextoAutomaticoV2 automaticoV2={};
int32_t conteoEncoderBanda=0,ultimoConteoEncoderVelocidad=0,conteoReferenciaIndiceDiagnostico=0;
int ultimoIndiceReportado=0,indiceReferenciaDiagnostico=0,revolucionesIndiceEncoder=0;
uint8_t estadoEncoderBanda=0,ultimoEstadoABEncoder=0;
uint16_t secuenciaEncoder=0;
uint32_t ultimaMuestraEncoder=0,ultimoPulsoEncoder=0,intervalosIndiceCompletos=0;
uint64_t cuentasIndiceAcumuladas=0;
bool referenciaIndiceDiagnosticoValida=false;
float velocidadBandaMmS=0,aceleracionBandaMmS2=0,frecuenciaEncoderCuentasS=0;
float ultimaMedicionCuentasPorVuelta=0,escalaEncoderMmPorCuenta=0.1f;
int8_t signoEncoderAvance=1;
CapturaFijaV2::Estimador estimadorCapturaFijaV2;
void cerca(float a,float b,float tolerancia=.002f) { assert(isfinite(a)&&fabsf(a-b)<=tolerancia); }
int32_t sumar(int32_t a,int32_t b) { return static_cast<int32_t>(static_cast<uint32_t>(a)+static_cast<uint32_t>(b)); }
bool actualizarObjetivoMovilV2() {
    // La posicion usa el contador vivo; la velocidad conserva su ventana de 10 ms.
    conteoEncoderBanda = static_cast<int32_t>(encoders[0].getPulses());
    const int32_t delta = diferenciaConteosConWrap(
        conteoEncoderBanda,
        automaticoV2.conteoReferencia
    );
    const float baseX = CAMERA_SWAP_XY
        ? automaticoV2.camYReferencia : automaticoV2.camXReferencia;
    const float baseY = CAMERA_SWAP_XY
        ? automaticoV2.camXReferencia : automaticoV2.camYReferencia;
    const float brazoX = static_cast<float>(CAMERA_SIGN_X) * baseX +
                         CAMERA_OFFSET_X_MM;
    const float yLocalCamara = static_cast<float>(CAMERA_SIGN_Y) * baseY +
                               CAMERA_OFFSET_Y_MM;
    const float brazoY = -CAMARA_A_HOME_Y_MM + yLocalCamara +
        static_cast<float>(signoEncoderAvance) *
        static_cast<float>(delta) * escalaEncoderMmPorCuenta;
    if (!isfinite(brazoX) || !isfinite(brazoY)) {
        return false;
    }

    const uint32_t ahora = millis();
    if (conteoEncoderBanda != automaticoV2.ultimoConteoProcesado &&
        automaticoV2.instanteObjetivoAnterior != 0 &&
        ahora > automaticoV2.instanteObjetivoAnterior) {
        const float dt = static_cast<float>(
            ahora - automaticoV2.instanteObjetivoAnterior
        ) / 1000.0f;
        const float instantanea =
            (brazoY - automaticoV2.objetivoBrazoYAnterior) / dt;
        automaticoV2.velocidadObjetivoY =
            0.8f * automaticoV2.velocidadObjetivoY + 0.2f * instantanea;
        automaticoV2.objetivoBrazoYAnterior = brazoY;
        automaticoV2.ultimoConteoProcesado = conteoEncoderBanda;
        automaticoV2.instanteObjetivoAnterior = ahora;
    }
    automaticoV2.objetivoBrazoX = brazoX;
    automaticoV2.objetivoBrazoY = brazoY;
    automaticoV2.velocidadObjetivoY = velocidadBandaMmS;
    return true;
}
void actualizarEncoderBanda() {
    const uint32_t ahora = millis();
    if (ahora - ultimaMuestraEncoder < PERIODO_ENCODER_MS) return;
    const uint32_t dt = ultimaMuestraEncoder == 0
        ? PERIODO_ENCODER_MS : ahora - ultimaMuestraEncoder;
    ultimaMuestraEncoder = ahora;

    const int32_t conteo = static_cast<int32_t>(encoders[0].getPulses());
    const int revoluciones = encoders[0].getRevolutions();
    ultimoEstadoABEncoder = encoders[0].getCurrentState();
    const int32_t delta = diferenciaConteosConWrap(
        conteo, ultimoConteoEncoderVelocidad
    );

    estadoEncoderBanda = ENC_FLAG_HW_LISTO;
    if (escalaEncoderMmPorCuenta > 0.0f && isfinite(escalaEncoderMmPorCuenta)) {
        estadoEncoderBanda |= ENC_FLAG_ESCALA_VALIDA;
    }
    if (ultimoPulsoEncoder != 0) estadoEncoderBanda |= ENC_FLAG_PULSOS_VISTOS;

    if (estimadorCapturaFijaV2.actualizar(conteo, ahora,
            escalaEncoderMmPorCuenta, signoEncoderAvance, V2_VENTANA_VELOCIDAD_MS)) {
        const CapturaFijaV2::Movimiento m = estimadorCapturaFijaV2.movimiento();
        velocidadBandaMmS = m.velocidad;
        aceleracionBandaMmS2 = m.aceleracion;
        frecuenciaEncoderCuentasS = escalaEncoderMmPorCuenta > 0.0f
            ? m.velocidad / (escalaEncoderMmPorCuenta * signoEncoderAvance) : 0.0f;
    }
    if (delta != 0) {
        const float instantanea = static_cast<float>(delta) *
            escalaEncoderMmPorCuenta * static_cast<float>(signoEncoderAvance) *
            1000.0f / static_cast<float>(dt);
        ultimoPulsoEncoder = ahora;
        estadoEncoderBanda |= ENC_FLAG_PULSOS_VISTOS | ENC_FLAG_EN_MOVIMIENTO;
        if (instantanea > 0.0f) estadoEncoderBanda |= ENC_FLAG_DIRECCION_POSITIVA;
    } else if (ultimoPulsoEncoder != 0 &&
               ahora - ultimoPulsoEncoder <= TIMEOUT_MOVIMIENTO_ENCODER_MS) {
        estadoEncoderBanda |= ENC_FLAG_PULSOS_VISTOS | ENC_FLAG_EN_MOVIMIENTO;
        if (velocidadBandaMmS > 0.0f)
            estadoEncoderBanda |= ENC_FLAG_DIRECCION_POSITIVA;
    } else {
        velocidadBandaMmS = 0.0f;
        frecuenciaEncoderCuentasS = 0.0f;
    }

    conteoEncoderBanda = conteo;
    revolucionesIndiceEncoder = revoluciones;
    ultimoConteoEncoderVelocidad = conteo;
    ++secuenciaEncoder;

    if (revoluciones != ultimoIndiceReportado) {
        if (referenciaIndiceDiagnosticoValida) {
            const int deltaIndices = revoluciones - indiceReferenciaDiagnostico;
            if (deltaIndices != 0) {
                const int32_t deltaCuentas = diferenciaConteosConWrap(
                    conteo, conteoReferenciaIndiceDiagnostico
                );
                const uint32_t vueltasCompletas = static_cast<uint32_t>(
                    deltaIndices < 0 ? -deltaIndices : deltaIndices
                );
                const uint64_t cuentasCompletas = static_cast<uint64_t>(
                    deltaCuentas < 0
                        ? -static_cast<int64_t>(deltaCuentas)
                        : static_cast<int64_t>(deltaCuentas)
                );
                ultimaMedicionCuentasPorVuelta =
                    static_cast<float>(cuentasCompletas) /
                    static_cast<float>(vueltasCompletas);
                intervalosIndiceCompletos += vueltasCompletas;
                cuentasIndiceAcumuladas += cuentasCompletas;
            }
        }
        referenciaIndiceDiagnosticoValida = true;
        indiceReferenciaDiagnostico = revoluciones;
        conteoReferenciaIndiceDiagnostico = conteo;
        Serial.print(F("[ENC][INDEX] vueltas="));
        Serial.print(revoluciones);
        Serial.print(F(" conteo="));
        Serial.print(conteo);
        if (ultimaMedicionCuentasPorVuelta > 0.0f) {
            Serial.print(F(" ultima="));
            Serial.print(ultimaMedicionCuentasPorVuelta, 2);
            Serial.print(F(" cuentas/vuelta"));
        }
        Serial.println();
        ultimoIndiceReportado = revoluciones;
    }
}

void reset() {
  ahora=1000;automaticoV2={};encoders[0]={};
  signoEncoderAvance=1;escalaEncoderMmPorCuenta=.1f;velocidadBandaMmS=0;
  aceleracionBandaMmS2=frecuenciaEncoderCuentasS=0;
  conteoEncoderBanda=ultimoConteoEncoderVelocidad=0;
  ultimaMuestraEncoder=ultimoPulsoEncoder=0;secuenciaEncoder=0;
  ultimoIndiceReportado=indiceReferenciaDiagnostico=revolucionesIndiceEncoder=0;
  conteoReferenciaIndiceDiagnostico=0;referenciaIndiceDiagnosticoValida=false;
  cuentasIndiceAcumuladas=intervalosIndiceCompletos=0;ultimaMedicionCuentasPorVuelta=0;
  estimadorCapturaFijaV2=CapturaFijaV2::Estimador();
}
void observar(float x,float y,int32_t referencia,int32_t actual,float velocidad) {
  automaticoV2.camXReferencia=x;automaticoV2.camYReferencia=y;
  automaticoV2.conteoReferencia=referencia;encoders[0].conteo=actual;
  velocidadBandaMmS=velocidad;
  assert(actualizarObjetivoMovilV2());
}
int main() {
  static_assert(!CAMERA_SWAP_XY&&CAMERA_SIGN_X==-1&&CAMERA_SIGN_Y==-1,"Transformacion vigente");
  cerca(CAMERA_OFFSET_X_MM,-5);cerca(CAMERA_OFFSET_Y_MM,0);cerca(CAMARA_A_HOME_Y_MM,845);
  cerca(ENCODER_MM_POR_CUENTA,49.0f*ENCODER_PI/2048.0f,.0000001f);
  // Una pieza observada en cinco lugares tiene la misma posicion actual y
  // el mismo conteo absoluto de cruce por Y=0, aunque cambie la velocidad.
  for(int yCam=100;yCam>=-100;yCam-=50) {
    reset();const int32_t nRef=1000+(100-yCam)*10;
    observar(25,yCam,nRef,4000,static_cast<float>(yCam));
    cerca(automaticoV2.objetivoBrazoX,-30);cerca(automaticoV2.objetivoBrazoY,-645);
    const int32_t cruce=sumar(nRef,lroundf((CAMARA_A_HOME_Y_MM+yCam)/escalaEncoderMmPorCuenta));
    assert(cruce==10450);encoders[0].conteo=cruce;ahora+=10;
    assert(actualizarObjetivoMovilV2());cerca(automaticoV2.objetivoBrazoY,0);
  }
  // X no depende de Y, cuentas, velocidad o aceleracion. El contador vivo
  // gobierna Y incluso si la velocidad global es errada o no se muestreo aun.
  reset();observar(-20,35,500,700,9999);cerca(automaticoV2.objetivoBrazoX,15);
  cerca(automaticoV2.objetivoBrazoY,-860);automaticoV2.instanteObjetivoAnterior=ahora;
  const float velocidades[]={-1000,0,10,5000};
  for(float v: velocidades) {
    encoders[0].conteo=sumar(encoders[0].conteo,10);velocidadBandaMmS=v;ahora+=10;
    assert(actualizarObjetivoMovilV2());cerca(automaticoV2.objetivoBrazoX,15);
    const float esperado=-845-35+.1f*diferenciaConteosConWrap(encoders[0].conteo,500);
    cerca(automaticoV2.objetivoBrazoY,esperado);
  }
  // Signo negativo y wrap del contador en ambas direcciones.
  reset();const int32_t limite=INT32_MAX-40;
  observar(10,-20,limite,sumar(limite,100),0);cerca(automaticoV2.objetivoBrazoY,-815);
  signoEncoderAvance=-1;const int32_t minimo=INT32_MIN+40;
  observar(10,-20,minimo,sumar(minimo,-100),0);cerca(automaticoV2.objetivoBrazoY,-815);
  // La posicion continua correcta al envolver millis; no se integra v*tiempo.
  reset();ahora=UINT32_MAX-20;observar(1,2,100,120,100);
  automaticoV2.instanteObjetivoAnterior=ahora;ahora=15;encoders[0].conteo=140;
  assert(actualizarObjetivoMovilV2());cerca(automaticoV2.objetivoBrazoY,-843);
  
  automaticoV2.camXReferencia=NAN;assert(!actualizarObjetivoMovilV2());
  automaticoV2.camXReferencia=0;automaticoV2.camYReferencia=NAN;
  assert(!actualizarObjetivoMovilV2());
  // Funcion de muestreo real: intervalos sin pulsos forman parte de la
  // ventana; debe reconocer velocidad cero antes del timeout de 500 ms.
  reset();actualizarEncoderBanda();
  for(int i=0;i<30;++i) { ahora+=10;encoders[0].conteo+=20;actualizarEncoderBanda(); }
  cerca(velocidadBandaMmS,200);assert(estadoEncoderBanda&ENC_FLAG_EN_MOVIMIENTO);
  const uint32_t pulso=ultimoPulsoEncoder;
  for(int i=0;i<10;++i) { ahora+=10;actualizarEncoderBanda(); }
  assert(ahora-pulso<TIMEOUT_MOVIMIENTO_ENCODER_MS);cerca(velocidadBandaMmS,0);
  assert(aceleracionBandaMmS2<0&&conteoEncoderBanda==600);
  // El estimador y funcion real usan resta de reloj de 32 bits, incluso wrap.
  reset();ahora=UINT32_MAX-150;actualizarEncoderBanda();
  for(int i=0;i<30;++i) { ahora+=10;encoders[0].conteo+=10;actualizarEncoderBanda(); }
  cerca(velocidadBandaMmS,100);assert(estimadorCapturaFijaV2.movimiento().valido);
  // Lectura por indice: no modifica escala ni posicion y conserva diagnostico.
  encoders[0].revoluciones=1;ahora+=10;actualizarEncoderBanda();
  encoders[0].conteo+=2048;encoders[0].revoluciones=2;ahora+=10;actualizarEncoderBanda();
  cerca(ultimaMedicionCuentasPorVuelta,2048);assert(intervalosIndiceCompletos==1);
  puts("PASS: posicion/encoder Portenta reales; puntos de deteccion, X fijo, velocidad erronea, wraps, parada e indice.");

  return 0;
}

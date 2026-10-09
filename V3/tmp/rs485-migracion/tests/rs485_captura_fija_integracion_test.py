"""Ejecuta el ciclo V2 real y el handshake ESP con motores/encoder simulados."""
import argparse
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path('C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3')
sys.path.insert(0, str(ROOT / 'tests'))
from recuperacion_enlace_test import run_recovery


def definition(source, marker):
    found = re.search(re.escape(marker) + ('' if marker.endswith('{') else r'[^;{]*\{'), source)
    if not found:
        raise ValueError('Definicion ausente: ' + marker)
    start, opening = found.start(), found.end() - 1
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    folder = ROOT / 'automatico v2 rs485'
    source = (folder / 'PORTENTA/PORTENTA.ino').read_text(encoding='utf-8-sig')
    assert (folder / 'ESP/ProtocoloRS485.h').read_bytes() == (folder / 'PORTENTA/ProtocoloRS485.h').read_bytes()
    cpp = '#include <assert.h>\n#include <math.h>\n#include <stdint.h>\n#include <stdlib.h>\n#include <string.h>\n#include <stdio.h>\n#include <initializer_list>\n#define F(x) x\n'
    cpp += '#include "' + (folder / 'PORTENTA/ProtocoloRS485.h').as_posix() + '"\nusing namespace ProtocoloRS485;\n'
    cpp += '#include "' + (folder / 'PORTENTA/CapturaFijaV2.h').as_posix() + '"\n'
    cpp += '#include "' + (folder / 'PORTENTA/AjusteTemporalCapturaV2.h').as_posix() + '"\n'
    cpp += definition(source, 'enum FaseAutomaticoV2 :') + ';\n'
    cpp += definition(source, 'struct ContextoAutomaticoV2 {') + ';\n'
    for name in ['RANGO_FISICO_Y_MM', 'MARGEN_SEGURIDAD_MM', 'PASOS_SEPARACION',
                 'ENCODER_DIAMETRO_RUEDA_MM', 'ENCODER_RELACION_ENCODER_RUEDA',
                 'ENCODER_CUENTAS_X2_POR_VUELTA', 'ENCODER_PI', 'ENCODER_MM_POR_CUENTA',
                 'DIV_CAL_LENTA', 'DIV_POSICION', 'V2_POSICION_CATCH_Y_MM',
                 'V2_Z_SEGURO_PASOS', 'V2_ERROR_ESTABLE_MM', 'V2_TIMEOUT_FASE_MS',
                 'V2_RESERVA_MANUAL_CATCH_S',
                 'V2_TIEMPO_COMPLETADO_MS', 'V2_TIEMPO_CIERRE_PINZA_MS',
                 'V2_LATENCIA_ORDEN_PINZA_MS', 'V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS',
                 'V2_DIV_BUSQUEDA_FINAL_Z', 'ML_TIEMPO_SERVO_MS',
                 'ML_MARGEN_FINAL_DERECHO_MM', 'ML_SEGUIMIENTO_MARGEN_Y_MM',
                 'ML_SEGUIMIENTO_RESERVA_S', 'ML_SEGUIMIENTO_LOG_MS',
                 'V2_SEGUIMIENTO_ESTABLE_MS', 'V2_AJUSTE_DISPARO_CATCH_MS',
                 'V2_DESFASE_CATCH_MS',
                 'V2_PERIODO_LOG_TELEMETRIA_MS']:
        cpp += re.search(r'(?:const|constexpr) [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source).group() + '\n'
    cpp += '#include \"' + (folder / 'PORTENTA/AjusteCatchV2.h').as_posix() + '\"\n'
    # Verificar primero los valores entregados para ensayo; solo despues el
    # simulador configura un perfil medido sintetico para la ruta estricta.
    for name in ['V2_CAPTURA_FIJA_VALIDADA', 'V2_HABILITAR_PRUEBAS_CATCH',
                 'V2_VENTANA_CAPTURA_Y_MM',
                 'V2_ERROR_GEOMETRIA_CAPTURA_MM', 'V2_MARGEN_TIEMPO_Z_S',
                 'V2_TIEMPO_CONTACTO_MIN_S', 'V2_TIEMPO_CONTACTO_MAX_S',
                 'V2_LIMITE_ACELERACION_MM_S2', 'V2_ERROR_MODELO_ACELERACION_MM_S2',
                 'V2_VELOCIDAD_MIN_CAPTURA_MM_S', 'V2_VELOCIDAD_MAX_CAPTURA_MM_S',
                 'V2_VELOCIDAD_MAX_PRUEBA_MM_S',
                 'V2_ERROR_RELATIVO_ESCALA', 'V2_ERROR_REFERENCIA_CAMARA_MS',
                 'V2_ALTURA_LIBRE_BANDA_PASOS', 'V2_TIEMPO_RETIRADA_BANDA_MAX_S',
                 'V2_DESPLAZAMIENTO_SOSTENIDO_MAX_MM',
                 'V2_VENTANA_VELOCIDAD_MS', 'V2_EDAD_MAXIMA_VELOCIDAD_MS',
                 'V2_ASENTAMIENTO_X_MS', 'V2_ASENTAMIENTO_GIRO_MS',
                 'V2_ESPERA_MAXIMA_ABAJO_MS', 'V2_CONFIRMACION_DIN04_MS']:
        declaration = re.search(r'constexpr [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source)
        if declaration:
            cpp += declaration.group().replace('constexpr ', '') + '\n'
    cpp += '''
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
'''
    for marker in ['bool seguimientoYAutomaticoV2()', 'bool capturaFijaV2EnPrueba()',
                   'float tiempoContactoNominalFijoV2()',
                   'bool perfilCapturaFijaV2Valido()',
                   'CapturaFijaV2::Movimiento movimientoCapturaFijaV2()',
                   'float errorPosicionCapturaFijaV2(', 'bool retencionBandaAdmisibleV2(',
                   'float horizonteContactoMinV2()', 'float horizonteContactoMaxV2()',
                   'CapturaFijaV2::Intervalo predecirContactoFijoV2(',
                   'CapturaFijaV2::Intervalo contactoNominalFijoV2(',
                   'CapturaFijaV2::Decision decisionContactoFijoV2(',
                   'void registrarPerfilCapturaFijaV2()', 'bool xYGiroListosCapturaFijaV2()',
                   'void registrarPrediccionCapturaFijaV2(', 'bool procesarCapturaFijaV2()',
                   'bool seguirPiezaY(float', 'bool seguirPiezaYAutomaticoV2()',
                   'bool iniciarTrasladoEntrega()', 'void llenarResumenCatchV2(', 'void procesarModoAutomaticoV2()']:
        cpp += definition(source, marker) + '\n'
    cpp += '''void medirMovimientoSimulado() {
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
  if(cancelaciones) printf("Catch cuantizado v=%.3f fase=%lu desfase=%ld: %s\\n",velocidad,faseMs,
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
'''
    esp = (folder / 'ESP/ESP.ino').read_text(encoding='utf-8-sig')
    handshake = '#include <assert.h>\n#include <stdint.h>\n#define F(x) x\n'
    handshake += '#include "' + (folder / 'ESP/ProtocoloRS485.h').as_posix() + '"\nusing namespace ProtocoloRS485;\n'
    handshake += '''struct Terminal { template<class T> void print(const T&) {};
template<class T> void println(const T&) {}; void println() {} } Serial;
unsigned long millis() { return 100; }
struct ContextoCamara { bool pruebaEncoderAnterior=false, esperandoDesaparicion=false,
rearmada=true, portentaEstabaActiva=true, autoEstabaActivo=true, objetivoValido=true,
objetivoV2=true; uint16_t secuenciaObjetivo=42, ultimaSecuenciaComando=0, ackComando=0;
unsigned long proximaLectura=0; };
struct ControlCamaraCompartido { bool pruebaEncoderActiva=false, portentaActiva=true,
automaticoActivo=true, registrarAngulo=false; uint8_t codigoAckObjetivo=0;
uint16_t ackObjetivo=42; };
void limpiarObjetivo(ContextoCamara& ctx, bool) { ctx.objetivoValido=false; }
PaquetePortentaAESP estadoPortenta={};
bool estadoPortentaValido=true, automaticoV2PinzaAnterior=false;
unsigned long ultimoEstadoPortenta=100;
constexpr unsigned long TIMEOUT_PORTENTA_MS=150;
uint16_t ultimaSecuenciaPinzaV2=0, sesionArranque=1;
uint8_t ultimoCodigoPinzaV2=0;
int anguloServoRotacion=59, anguloServoPinza=0, aperturas=0, cierres=0;
constexpr int PULSO_PINZA_CERRADA_US=1816;
struct EstadoCamaraPublicado { int sugerenciaAngulo=-1; };
EstadoCamaraPublicado copiarEstadoCamara() { return {}; }
void escribirPinzaCalibrada(bool cerrar) {
  if (cerrar) ++cierres; else ++aperturas;
}
'''
    handshake += '#define portENTER_CRITICAL(x) ((void)0)\n#define portEXIT_CRITICAL(x) ((void)0)\n'
    handshake += definition(esp, 'struct EstadoEncoderCompartido {') + ';\nEstadoEncoderCompartido estadoEncoder;\n'
    handshake += definition(esp, 'void actualizarEncoderDesdePortenta(')
    handshake += definition(esp, 'bool paquetePortentaSemanticamenteValido(')
    handshake += definition(esp, 'void procesarHandshakeObjetivo(')
    handshake += definition(esp, 'void procesarPinzaAutomaticaV2()')
    handshake += '''int main() {
ContextoCamara ctx; ControlCamaraCompartido control;
for (uint8_t ack : {ACK_OBJ_ACEPTADO, ACK_OBJ_CERRAR_PINZA, ACK_OBJ_ABRIR_PINZA}) {
  control.codigoAckObjetivo=ack; procesarHandshakeObjetivo(ctx,control);
  assert(ctx.objetivoValido);
}
control.codigoAckObjetivo=ACK_OBJ_COMPLETADO; procesarHandshakeObjetivo(ctx,control);
assert(!ctx.objetivoValido);
ctx.objetivoValido=true; control.codigoAckObjetivo=ACK_OBJ_CANCELADO;
procesarHandshakeObjetivo(ctx,control); assert(!ctx.objetivoValido);
// Ordenes reales de ESP: entrada abierta, cierre/apertura unicos aunque RS485 repita.
estadoPortenta.estadoSistema=SISTEMA_MODO_AUTOMATICO_V2;
estadoPortenta.opcionMenu=MENU_MODO_AUTOMATICO_V2;
estadoPortenta.ackSecuenciaObjetivo=42;
estadoPortenta.codigoAckObjetivo=ACK_OBJ_ACEPTADO;
procesarPinzaAutomaticaV2(); assert(aperturas==1 && !cierres);
estadoPortenta.codigoAckObjetivo=ACK_OBJ_CERRAR_PINZA;
procesarPinzaAutomaticaV2(); procesarPinzaAutomaticaV2(); assert(cierres==1);
estadoPortenta.codigoAckObjetivo=ACK_OBJ_ABRIR_PINZA;
procesarPinzaAutomaticaV2(); procesarPinzaAutomaticaV2(); assert(aperturas==2);
estadoPortenta.opcionMenu=MENU_AJUSTE_CATCH_V2;
estadoPortenta.ackSecuenciaObjetivo=43;
estadoPortenta.codigoAckObjetivo=ACK_OBJ_CERRAR_PINZA;
procesarPinzaAutomaticaV2(); procesarPinzaAutomaticaV2(); assert(cierres==2);
// Resumen: semantica de OLED aceptada, sin convertir el ajuste en encoder.
PaquetePortentaAESP resumen={}; resumen.estadoSistema=SISTEMA_CAMBIOS_CATCH;
resumen.opcionMenu=MENU_CAMBIOS_CATCH; resumen.signoEncoder=1;
resumen.nmPorCuentaEncoder=50; resumen.conteoEncoder=-100;
resumen.velocidadEncoderUmS=-100;
for(int streak=0;streak<4;++streak) {
  resumen.estadoEncoder=(streak<<2)|(streak==3?1:0);
  assert(paquetePortentaSemanticamenteValido(resumen));
  actualizarEncoderDesdePortenta(resumen);
  assert(!estadoEncoder.flags && !estadoEncoder.nmPorCuenta && !estadoEncoder.conteo && estadoEncoder.velocidadMmS==0);
}
resumen.estadoEncoder=1; assert(!paquetePortentaSemanticamenteValido(resumen));
resumen.estadoEncoder=0; resumen.opcionMenu=MENU_DIAGNOSTICO;
assert(!paquetePortentaSemanticamenteValido(resumen));
// Al salir del resumen se vuelven a publicar datos reales.
resumen.estadoSistema=SISTEMA_MENU_PRINCIPAL; resumen.estadoEncoder=ENC_FLAG_HW_LISTO|ENC_FLAG_ESCALA_VALIDA;
resumen.nmPorCuentaEncoder=75000; resumen.conteoEncoder=123; resumen.velocidadEncoderUmS=50000000;
assert(paquetePortentaSemanticamenteValido(resumen)); actualizarEncoderDesdePortenta(resumen);
assert(estadoEncoder.conteo==123 && estadoEncoder.velocidadMmS==50);
PaquetePortentaAESP wire={}; wire.codigoAckObjetivo=ACK_OBJ_ABRIR_PINZA;
prepararPaquete(wire); assert(validarPaquete(wire));
wire.version=VERSION_PROTOCOLO-1;
wire.checksum=calcularCRC8ATM(reinterpret_cast<const uint8_t*>(&wire),31);
assert(!validarPaquete(wire)); // Rechazar version vieja aun con CRC correcto.
}'''
    handshake = '#include <initializer_list>\n' + handshake
    protocol = (ROOT / 'tests/protocol_layout_test.cpp').read_text(encoding='utf-8-sig').replace('RS485', 'RS485')
    protocol = protocol.replace('I2C', 'RS485')
    protocol = protocol.replace('../ESP/ProtocoloRS485.h',
                                (folder / 'ESP/ProtocoloRS485.h').as_posix())
    # La maqueta publica encoder desde Portenta; adaptar los campos de la
    # prueba historica conservando sus vectores CRC, cabeceras y wrap.
    protocol = re.sub(r'    paquete.estadoEncoder = ENC_FLAG_HW_LISTO \|.*?;',
                      '', protocol, flags=re.S)
    protocol = protocol.replace('paquete.conteoEncoder = 123456',
                                'paquete.conteoReferenciaObjetivo = 123456')
    protocol = protocol.replace('valorPantalla1', 'conteoEncoder').replace(
        'valorPantalla2', 'velocidadEncoderUmS')
    build = Path(tempfile.mkdtemp(prefix='auto-v2-integracion-', dir=ROOT / 'tmp'))
    run_recovery(args.compiler, folder, 'RS485', build)
    for name, code in [('ciclo', cpp), ('handshake', handshake), ('protocolo', protocol)]:
        path = build / (name + '.cpp')
        path.write_text(code, encoding='utf-8')
        exe = build / (name + '.exe')
        subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', '-static',
                        str(path), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print('PASS: ajuste autonomo, aprendizaje acotado y etiquetas; handshake/pinza, reserva, idempotencia, CRC y rechazo de version previa')


if __name__ == '__main__':
    main()

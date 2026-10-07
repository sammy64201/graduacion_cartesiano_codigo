"""Ejecuta el ciclo V2 real y el handshake ESP con motores/encoder simulados."""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def definition(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    folder = ROOT / 'pruebas de automatico v2'
    source = (folder / 'PORTENTA/PORTENTA.ino').read_text(encoding='utf-8-sig')
    assert (folder / 'ESP/ProtocoloI2C.h').read_bytes() == (folder / 'PORTENTA/ProtocoloI2C.h').read_bytes()
    cpp = '#include <assert.h>\n#include <math.h>\n#include <stdint.h>\n#include <stdlib.h>\n#include <string.h>\n#include <stdio.h>\n#define F(x) x\n'
    cpp += '#include "' + (folder / 'PORTENTA/ProtocoloI2C.h').as_posix() + '"\nusing namespace ProtocoloI2C;\n'
    cpp += definition(source, 'enum FaseAutomaticoV2 :') + ';\n'
    cpp += definition(source, 'struct ContextoAutomaticoV2 {') + ';\n'
    for name in ['RANGO_FISICO_Y_MM', 'MARGEN_SEGURIDAD_MM', 'PASOS_SEPARACION',
                 'DIV_CAL_LENTA', 'DIV_POSICION', 'V2_POSICION_CATCH_Y_MM',
                 'V2_Z_SEGURO_PASOS', 'V2_ERROR_ESTABLE_MM', 'V2_TIMEOUT_FASE_MS',
                 'V2_RESERVA_MANUAL_CATCH_S',
                 'V2_TIEMPO_COMPLETADO_MS', 'V2_TIEMPO_CIERRE_PINZA_MS',
                 'V2_LATENCIA_ORDEN_PINZA_MS', 'V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS',
                 'V2_DIV_BUSQUEDA_FINAL_Z', 'ML_TIEMPO_SERVO_MS',
                 'ML_MARGEN_FINAL_DERECHO_MM', 'ML_SEGUIMIENTO_MARGEN_Y_MM',
                 'ML_SEGUIMIENTO_RESERVA_S', 'ML_SEGUIMIENTO_LOG_MS',
                 'V2_SEGUIMIENTO_ESTABLE_MS', 'V2_AJUSTE_DISPARO_CATCH_MS']:
        cpp += re.search(r'(?:const|constexpr) [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source).group() + '\n'
    cpp += '#include \"' + (folder / 'PORTENTA/AjusteCatchV2.h').as_posix() + '\"\n'
    cpp += '''
AjusteCatchV2::Sesion ajusteCatchV2;
uint8_t paginaCambiosCatch=0;
void imprimirCambiosCatchV2() {}
// Alternar la ruta solo en el simulador permite verificar ambas configuraciones.
bool AUTO_V2_SEGUIMIENTO_Y=true;
bool calMode=false;
bool ajusteCatchV2Seleccionado() { return calMode; }
ContextoAutomaticoV2 automaticoV2;
struct Terminal { template<class T> void print(const T&, int=0) {};
template<class T> void println(const T&, int=0) {}; } Serial;
unsigned long now=1000, tAnteriorEstadoESP;
unsigned long millis() { return now; }
bool btConectado, eventoBotonTriangulo, eventoBotonX, eventoBotonCuadrado, eventoBotonCirculo;
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
  btConectado=encoderOK=bandaOK=camaraOK=predictionOK=xyOK=comunicacionI2CHabilitada=true;
  eventoBotonTriangulo=eventoBotonX=eventoBotonCuadrado=eventoBotonCirculo=limiteZarriba=limiteZabajo=false;
  muestrasCal=0; resultadoCal=nullptr;
  limiteXmas=limiteXmenos=limiteYmas=limiteYmenos=false;
  movimientoPosicionadoActivo=xActive=zActive=objetivoYActivo=false;
  movX=movY=movZ=0; pieceY=0; ySteps=0; zSteps=-4300; velocidadBandaMmS=50;
  errores=cancelaciones=cierres=resultados=aceptados=busquedas=yStarts=tx=0;
  intentosV2=exitosV2=0; codigoAckObjetivo=ACK_OBJ_ACEPTADO;
  secuenciaObjetivoRecibida=ackSecuenciaObjetivo=42;
}
void finishXYZ() { detenerTodos(); movimientoPosicionadoActivo=false; zSteps=zTarget; }
'''
    for marker in ['bool seguirPiezaY(float', 'bool seguirPiezaYAutomaticoV2()',
                   'bool iniciarTrasladoEntrega()', 'void llenarResumenCatchV2(', 'void procesarModoAutomaticoV2()']:
        cpp += definition(source, marker) + '\n'
    cpp += '''int main() {
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
// Ruta fija sigue disponible; X no dispara ni habilita excepciones de ML V2.
AUTO_V2_SEGUIMIENTO_Y=false; reset(V2_ESPERANDO_CATCH_AUTOMATICO);
pieceY=-100; eventoBotonX=true; procesarModoAutomaticoV2(); assert(!zActive && !eventoBotonX);
pieceY=-55; procesarModoAutomaticoV2(); assert(zActive);
pieceY=6; procesarModoAutomaticoV2(); assert(cancelaciones==1 && !cierres);
puts("PASS: seguimiento real, ciclo automatico hasta entrega, ruta fija, DIN04, timeout, encoder, control, limites y resultado reservado hasta Z seguro");
}
'''
    esp = (folder / 'ESP/ESP.ino').read_text(encoding='utf-8-sig')
    handshake = '#include <assert.h>\n#include <stdint.h>\n#define F(x) x\n'
    handshake += '#include "' + (folder / 'ESP/ProtocoloI2C.h').as_posix() + '"\nusing namespace ProtocoloI2C;\n'
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
// Ordenes reales de ESP: entrada abierta, cierre/apertura unicos aunque I2C repita.
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
wire.version=15;
wire.checksum=calcularCRC8ATM(reinterpret_cast<const uint8_t*>(&wire),31);
assert(!validarPaquete(wire)); // Rechazar version vieja aun con CRC correcto.
}'''
    handshake = '#include <initializer_list>\n' + handshake
    protocol = (ROOT / 'tests/protocol_layout_test.cpp').read_text(encoding='utf-8-sig')
    protocol = protocol.replace('../ESP/ProtocoloI2C.h',
                                (folder / 'ESP/ProtocoloI2C.h').as_posix())
    # La maqueta publica encoder desde Portenta; adaptar los campos de la
    # prueba historica conservando sus vectores CRC, cabeceras y wrap.
    protocol = re.sub(r'    paquete.estadoEncoder = ENC_FLAG_HW_LISTO \|.*?;',
                      '', protocol, flags=re.S)
    protocol = protocol.replace('paquete.conteoEncoder = 123456',
                                'paquete.conteoReferenciaObjetivo = 123456')
    protocol = protocol.replace('valorPantalla1', 'conteoEncoder').replace(
        'valorPantalla2', 'velocidadEncoderUmS')
    build = Path(tempfile.mkdtemp(prefix='auto-v2-integracion-', dir=ROOT / 'tmp'))
    for name, code in [('ciclo', cpp), ('handshake', handshake), ('protocolo', protocol)]:
        path = build / (name + '.cpp')
        path.write_text(code, encoding='utf-8')
        exe = build / (name + '.exe')
        subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', '-static',
                        str(path), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print('PASS: ajuste autonomo, aprendizaje acotado y etiquetas; handshake/pinza, reserva, idempotencia, CRC version 16')


if __name__ == '__main__':
    main()

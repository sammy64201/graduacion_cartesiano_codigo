"""Ejecuta posicion y muestreo encoder reales de ambas Portenta, sin placas.

La extraccion omite prototipos. Solo normaliza unsigned long a uint32_t para
representar el reloj Arduino de 32 bits tambien en hosts donde long es de 64.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def definition(source, marker):
    match = re.search(r'\b' + re.escape(marker) + r'[^;{]*\{', source)
    if not match:
        raise AssertionError('No existe definicion: ' + marker)
    opening = match.end() - 1
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def board_types(code):
    return code.replace('unsigned long', 'uint32_t')


def program(folder, protocol, namespace):
    source = (folder / 'PORTENTA/PORTENTA.ino').read_text(encoding='utf-8-sig')
    position = definition(source, 'bool actualizarObjetivoMovilV2(')
    # Demuestra que un prototipo generado por Arduino no confunde el extractor.
    assert position == definition('bool actualizarObjetivoMovilV2();\n' + source,
                                  'bool actualizarObjetivoMovilV2(')
    cpp = '#include <assert.h>\n#include <math.h>\n#include <stdint.h>\n#include <stdio.h>\n#define F(x) x\n'
    cpp += '#include "' + (folder / 'PORTENTA' / protocol).as_posix() + '"\n'
    cpp += '#include "' + (folder / 'PORTENTA/CapturaFijaV2.h').as_posix() + '"\n'
    cpp += 'using namespace ' + namespace + ';\n'
    for name in ['CAMERA_SWAP_XY', 'CAMERA_SIGN_X', 'CAMERA_SIGN_Y',
                 'DESFASE_CAMARA_X_MM', 'DESFASE_CAMARA_Y_MM',
                 'CAMERA_OFFSET_X_MM', 'CAMERA_OFFSET_Y_MM',
                 'CAMARA_A_HOME_Y_BASE_MM', 'V2_AJUSTE_DISTANCIA_CATCH_MM',
                 'CAMARA_A_HOME_Y_MM', 'ENCODER_DIAMETRO_RUEDA_MM',
                 'ENCODER_RELACION_ENCODER_RUEDA', 'ENCODER_CUENTAS_X2_POR_VUELTA',
                 'ENCODER_PI', 'ENCODER_MM_POR_CUENTA', 'PERIODO_ENCODER_MS',
                 'TIMEOUT_MOVIMIENTO_ENCODER_MS', 'V2_VENTANA_VELOCIDAD_MS']:
        declaration = re.search(r'(?:const|constexpr) [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source)
        if not declaration:
            raise AssertionError('No existe parametro: ' + name)
        cpp += board_types(declaration.group()) + '\n'
    for marker in ['enum FaseAutomaticoV2 :', 'struct ContextoAutomaticoV2']:
        cpp += board_types(definition(source, marker)) + ';\n'
    cpp += r'''
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
'''
    cpp += board_types(position) + '\n'
    cpp += board_types(definition(source, 'void actualizarEncoderBanda(')) + '\n'
    cpp += r'''
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
'''
    return cpp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    env = os.environ.copy()
    compiler_dir = Path(args.compiler).resolve().parent
    env['PATH'] = str(compiler_dir) + os.pathsep + str(compiler_dir / 'cpp') + os.pathsep + env.get('PATH', '')
    (ROOT / 'tmp').mkdir(exist_ok=True)
    build = Path(tempfile.mkdtemp(prefix='captura-fija-posicion-', dir=ROOT / 'tmp'))
    variants = [('i2c', 'pruebas de automatico v2', 'ProtocoloI2C.h', 'ProtocoloI2C'),
                ('rs485', 'automatico v2 rs485', 'ProtocoloRS485.h', 'ProtocoloRS485')]
    for name, folder, protocol, namespace in variants:
        code = build / (name + '.cpp')
        code.write_text(program(ROOT / folder, protocol, namespace), encoding='utf-8')
        exe = build / (name + ('.exe' if os.name == 'nt' else ''))
        subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', str(code), '-o', str(exe)], check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)


if __name__ == '__main__':
    main()

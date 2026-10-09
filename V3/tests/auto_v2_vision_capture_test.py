"""Ejecuta deteccion/reserva ESP real para captura fija, sin placas."""
import argparse
import os
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


def program(folder, protocol, namespace):
    source = (folder / 'ESP/ESP.ino').read_text(encoding='utf-8-sig')
    cpp = '#include <cassert>\n#include <math.h>\n#include <cstdint>\n#include <cstdlib>\n#include <cstdio>\n#include <string>\n#include <vector>\n#include <sstream>\n#define F(x) x\n'
    for header in [protocol, 'VisionModelo129.h', 'CalibracionAnguloMLV2.h']:
        cpp += '#include "' + (folder / 'ESP' / header).as_posix() + '"\n'
    cpp += 'using namespace ' + namespace + ';\n'
    cpp += 'constexpr bool ensayoRS485=' + ('true' if namespace == 'ProtocoloRS485' else 'false') + ';\n'
    for name in ['CAMERA_SIGNO_Y_LOCAL', 'TIMEOUT_MUESTRA_ENCODER_MS',
                 'CAMERA_ANCHO_IMAGEN_PX', 'CAMERA_ALTO_IMAGEN_PX', 'CAMERA_MARGEN_CAJA_PX',
                 'DETECCIONES_ESTABLES_V2', 'DESPLAZAMIENTO_MINIMO_V2_MM',
                 'TOLERANCIA_TRAYECTORIA_V2_MM', 'DESPLAZAMIENTO_CAMARA_REPETIDA_MM',
                 'TOLERANCIA_CAMARA_REPETIDA_MM', 'SOLAPE_MINIMO_DUPLICADO',
                 'MAX_CANDIDATOS_V2', 'RELACION_MINIMA_ORIENTACION',
                 'BELT_WIDTH_MM', 'TOTAL_WIDTH_MM', 'ALUMINUM_WIDTH_MM',
                 'TAG_X_FROM_CENTER_MM', 'TAG_ROWS_DISTANCE_MM',
                 'ANGULO_GARRA_EJE_X', 'ANGULO_GARRA_EJE_Y', 'ANGULO_SERVO_INICIAL',
                 'AUTO_V2_APLICAR_GIRO_POR_CAJA']:
        cpp += re.search(r'constexpr [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source).group() + '\n'
    for marker in ['struct Point2D {', 'struct CandidatoPiezaV2 {',
                   'struct FiltroDeteccion {', 'struct FiltroDeteccionV2 {',
                   'struct ContextoCamara {', 'struct ControlCamaraCompartido {',
                   'struct EstadoEncoderCompartido {', 'enum CausaDiagnosticoV2 :',
                   'struct DiagnosticoDeteccionV2 {']:
        cpp += definition(source, marker) + ';\n'
    cpp += 'constexpr uint8_t causaCajaFueraCalibracion=' + (
        'V2_DIAG_FUERA_CALIBRACION' if namespace == 'ProtocoloRS485'
        else 'V2_DIAG_CAJA_RECORTADA') + ';\n'
    cpp += r'''
unsigned long now=1000;
uint32_t millis() { return now; }
struct Terminal {
  std::string log;
  template<class T> void print(const T& v, int=0) { std::ostringstream s; s<<v; log+=s.str(); }
  void print(uint8_t v, int=0) { print(static_cast<int>(v)); }
  template<class T> void println(const T& v, int=0) { print(v); log+='\n'; }
  void println() { log+='\n'; }
  void setCursor(int,int) {}
} Serial;
struct Servo { int writes=0,last=-1; void write(int a) { ++writes;last=a; } } servoRotacion;
int anguloServoRotacion=88;
uint16_t sesionArranque=1;
uint32_t consultasLogV2=0,ultimoReporteCandidatosV2=0;
constexpr uint8_t COMMAND_RETURN_BLOCK=1;
constexpr int PIECE_MODEL=129;
double H[3][3]={{1,0,-320},{0,1,-240},{0,0,1}};
EstadoEncoderCompartido muestra;
EstadoEncoderCompartido copiarEstadoEncoder() { return muestra; }
struct Result { std::string name="pieza6"; int type=1,confidence=-128,xCenter=320,yCenter=240,width=20,height=60; };
struct Husky {
  std::vector<Result> results;
  size_t at=0;
  bool restoreFlags=false;
  int8_t getResult(int) { at=0; now+=5; muestra.recibidoMs=now;
    if(restoreFlags) muestra.flags=ENC_FLAG_HW_LISTO|ENC_FLAG_ESCALA_VALIDA|ENC_FLAG_PULSOS_VISTOS;
    return results.size(); }
  bool available(int) { return at<results.size(); }
  Result* popCachedResult(int) { return &results[at++]; }
} huskylens;
DiagnosticoDeteccionV2 ultimoDiag;
DiagnosticoDeteccionV2 copiarDiagnosticoV2() { return ultimoDiag; }
struct EstadoCamaraMock { bool objetivoValido=false; };
EstadoCamaraMock copiarEstadoCamara() { return {}; }
Terminal pantalla;
std::string titulo;
void dibujarTitulo(const char* t) { titulo=t;pantalla.log.clear(); }
void publicarCausaDiagnosticoV2(DiagnosticoDeteccionV2& d,uint8_t causa) { d.causa=causa;ultimoDiag=d; }
void publicarYRegistrarDiagnosticoV2(DiagnosticoDeteccionV2& d,uint8_t causa,
  const EstadoEncoderCompartido&,uint32_t,uint8_t=UINT8_MAX) { publicarCausaDiagnosticoV2(d,causa); }
void registrarConsultaLogV2(const DiagnosticoDeteccionV2& d,const EstadoEncoderCompartido&,uint32_t,uint8_t) { ultimoDiag=d; }
void registrarCandidatoLogV2(uint32_t,const EstadoEncoderCompartido&,const CandidatoPiezaV2&,uint8_t,uint8_t) {}
int16_t sugerirAnguloPorVotos(uint8_t x,uint8_t y) { return x>=2&&y==0?90:y>=2&&x==0?0:-1; }
'''
    markers = ['bool paquetePortentaSemanticamenteValido(',
                   'const char *nombreCortoDiagnosticoV2(',
                   'float escalaEncoderMm(', 'bool encoderRemotoVigente(',
                   'uint8_t diagnosticarEncoderV2(', 'bool pixelToMillimeters(',
                   'uint8_t estimarEjeCajaV2(', 'bool isInsideCalibrationArea(',
                   'bool isOverWhiteBelt(', 'bool cajaIntegraAutonomaV2(',
                   'bool capturaAutonomaV2(', 'void orientarGarraAutomatica(',
                   'void reiniciarFiltroV2(', 'double desplazamientoEncoderMm(',
                   'double compensarYConEncoder(', 'int32_t conteoMedioConWrap(',
                   'double areaCajaV2(', 'bool cajasDuplicadasV2(',
                   'bool candidatoV2MejorQue(', 'DiagnosticoDeteccionV2 crearDiagnosticoBaseV2(',
                   'void completarMetricasFiltroV2(', 'int8_t elegirRelacionYEncoderV2(',
                   'bool tamanoCompatibleV2(', 'bool mismaTrayectoriaV2(',
                   'void incorporarDeteccionV2(', 'bool publicarObjetivoV2(',
                   'bool leerPiezasV2UnaVez(', 'void mostrarModoAutomaticoV2(']
    if namespace == 'ProtocoloRS485':
        markers.insert(markers.index('bool cajaIntegraAutonomaV2('), 'bool cajaVisibleEnImagenV2(')
    for marker in markers:
        cpp += definition(source, marker) + '\n'
    cpp += r'''
ControlCamaraCompartido control;
ContextoCamara ctx;
void reset(bool teaching=false,bool ajuste=false) {
  ctx={};ctx.homografiaValida=true;ctx.rearmada=true;
  control={};control.portentaActiva=true;control.automaticoActivo=true;
  control.automaticoV2Activo=true;control.orientarGarraV2=!teaching;
  control.entrenamientoML=control.entrenamientoMLV2=teaching;control.ajusteCatchV2=ajuste;
  muestra={};muestra.nmPorCuenta=100000;muestra.signo=1;
  muestra.flags=ENC_FLAG_HW_LISTO|ENC_FLAG_ESCALA_VALIDA|ENC_FLAG_PULSOS_VISTOS;
  muestra.velocidadMmS=33.33f;
  servoRotacion={};anguloServoRotacion=88;Serial.log.clear();ultimoDiag={};
  huskylens.restoreFlags=false;now=1000;
  H[0][0]=H[1][1]=H[2][2]=1;H[0][2]=-320;H[1][2]=-240;
}
void frame(int n,int px=320,int py=240,int w=20,int h=60,bool duplicate=false,bool second=false) {
  now+=60;muestra.conteo=n*20;muestra.recibidoMs=now;muestra.secuencia++;
  Result r;r.xCenter=px;r.yCenter=py-2*n;r.width=w;r.height=h;
  huskylens.results={r};
  if(duplicate) { r.xCenter++;huskylens.results.push_back(r); }
  if(second) { r.xCenter+=90;huskylens.results.push_back(r); }
  assert(leerPiezasV2UnaVez(ctx,control,now));
}
int main() {
  // Toda la region interior calibrada: ninguna linea fija de deteccion.
  for(int px: {200,320,440}) for(int py: {100,240,380}) {
    reset();frame(0,px,py);frame(1,px,py);assert(!ctx.objetivoValido);
    frame(2,px,py);assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==7);
    assert(ctx.conteoReferenciaObjetivo==40&&ctx.objetivoX10==(px-320)*10);
    assert(ctx.objetivoY10==(py-244)*10);
    assert(servoRotacion.writes==1&&anguloServoRotacion==155);
    assert(Serial.log.find("capture_timestamp=NA")!=std::string::npos);
    assert(Serial.log.find("closure_axis_minor=X")!=std::string::npos);
    assert(Serial.log.find("mode=V2|event=CAMERA_SPEED")!=std::string::npos);
    const auto ref=ctx.conteoReferenciaObjetivo;const auto x=ctx.objetivoX10;
    frame(3,px+10,py);assert(ctx.conteoReferenciaObjetivo==ref&&ctx.objetivoX10==x);
    assert(ultimoDiag.causa==V2_DIAG_OBJETIVO_ACTIVO&&ultimoDiag.edadConsultaReferenciaObjetivoMs>0);
  }
  // Las cajas duplicadas de la misma pieza no representan dos objetivos.
  reset();for(int i=0;i<3;++i) frame(i,320,240,60,20,true);
  assert(ctx.objetivoValido&&anguloServoRotacion==59);
  assert(Serial.log.find("closure_axis_minor=Y")!=std::string::npos);
  // Multiples piezas, cajas cortadas por la calibracion y borde de imagen.
  reset();frame(0,320,240,20,60,false,true);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_MULTIPLES_PIEZAS);
  reset();frame(0,320,70);assert(!ctx.objetivoValido&&ultimoDiag.causa==causaCajaFueraCalibracion);
  reset();H[0][0]=.4;H[0][2]=-128;frame(0,2,240);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_CAJA_RECORTADA);
  // RS485_CALIBRATION_REPLAY
  // Sin consenso, el giro previo se conserva y no se publica autonomo.
  reset();for(int i=0;i<3;++i) frame(i,320,240,30,30);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_ORIENTACION_AMBIGUA);
  assert(anguloServoRotacion==88&&servoRotacion.writes==0);
  reset();frame(0,320,240,60,20);frame(1);frame(2);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_ORIENTACION_AMBIGUA);
  // Un extremo anterior invalido no puede hacerse valido por recibir otro nuevo.
  reset();muestra.flags=0;huskylens.restoreFlags=true;frame(0);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_REFERENCIA_CAMARA_INCIERTA);
  // Ensenanza y ajuste no heredan los nuevos bloqueos ni flags autonomos.
  reset(true);for(int i=0;i<3;++i) frame(i,320,70,30,30,false,true);
  assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==0&&anguloServoRotacion==88);
  reset(false,true);for(int i=0;i<3;++i) frame(i,320,70,30,30,false,true);
  assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==0&&anguloServoRotacion==88);
  // Modo encoder observa candidatos sin reserva/consenso.
  reset();control.pruebaEncoderActiva=true;frame(0,320,70,30,30);
  assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==0&&!capturaAutonomaV2(control));
  // El aviso viaja como telemetria valida. RS485 permite continuar el ensayo
  // y conserva la fase visible; el antecedente I2C conserva su aviso historico.
  PaquetePortentaAESP p={};p.signoEncoder=1;
  p.estadoSistema=SISTEMA_MODO_AUTOMATICO_V2;p.opcionMenu=MENU_MODO_AUTOMATICO_V2;
  p.errorSistema=SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA;
  assert(paquetePortentaSemanticamenteValido(p));mostrarModoAutomaticoV2(p);
  if(ensayoRS485) {
    assert(titulo=="AUTOMATICO V2");
    assert(pantalla.log.find("VALORES NOMINALES")!=std::string::npos);
    assert(pantalla.log.find("CALIBRAR CAPTURA")==std::string::npos);
    for(int fase : {6,10,11,12}) {
      p.faseCalibracionBrazo=fase;mostrarModoAutomaticoV2(p);
      assert(pantalla.log.find("FASE: "+std::to_string(fase))!=std::string::npos);
      assert(pantalla.log.find("VALORES NOMINALES")!=std::string::npos);
    }
  } else {
    assert(titulo=="AUTO V2 Y FIJO");
    assert(pantalla.log.find("CALIBRAR CAPTURA")!=std::string::npos);
    assert(pantalla.log.find("PERFIL FISICO")!=std::string::npos);
  }
  p.opcionMenu=MENU_AJUSTE_CATCH_V2;
  assert(!paquetePortentaSemanticamenteValido(p));mostrarModoAutomaticoV2(p);
  assert(titulo=="AJUSTE CATCH V2"&&pantalla.log.find("CALIBRAR CAPTURA")==std::string::npos);
  assert(pantalla.log.find("PRUEBA CATCH")==std::string::npos);
  p.errorSistema=SISTEMA_ERROR_NINGUNO;p.faseCalibracionBrazo=16;
  mostrarModoAutomaticoV2(p);assert(pantalla.log.find("Y SIGUE CATCH AUTO")!=std::string::npos);
  p.opcionMenu=MENU_MODO_AUTOMATICO_V2;mostrarModoAutomaticoV2(p);
  assert(pantalla.log.find("ESPERA Y FIJO")!=std::string::npos);
  assert(titulo==(ensayoRS485?"AUTOMATICO V2":"AUTO V2 Y FIJO"));
  p.errorSistema=SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA+1;
  assert(!paquetePortentaSemanticamenteValido(p));
  puts("PASS: ESP vision captura fija; region, cuentas, ejes, reserva, cajas, ambiguedad y compatibilidad de ensayos.");
}
'''
    replay = r'''
  static_assert(V2_DIAG_FUERA_BANDA==16&&V2_DIAG_REFERENCIA_CAMARA_INCIERTA==27,
    "Conservar los identificadores historicos del diagnostico");
  static_assert(V2_DIAG_FUERA_CALIBRACION==28,"Agregar diagnostico sin renumerar");
  // Consultas reales 169/170/171: X esta dentro de banda, Y o una esquina
  // requieren extrapolar la region calibrada. Nunca diagnosticar FUERA_BANDA.
  for(int py : {31,45,59,87}) {
    reset();
    const double realH[3][3]={{.715927700,.017177490,-264.915639449},
      {-.000604618,1.024852716,-247.932280373},{.000039225,.000073543,1}};
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)H[r][c]=realH[r][c];
    Point2D centro;assert(pixelToMillimeters(328,py,true,centro));
    assert(fabs(centro.x)<BELT_WIDTH_MM/2);
    if(py==31)assert(fabs(centro.x+29.117843)<.001&&fabs(centro.y+213.132140)<.001);
    frame(0,328,py,52,61);
    if(py<87) {
      assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_FUERA_CALIBRACION);
      assert(ultimoDiag.rechazadosFueraCalibracion==1&&ultimoDiag.rechazadosFueraBanda==0);
    } else {
      assert(ultimoDiag.candidatosValidos==1&&ultimoDiag.rechazadosFueraCalibracion==0);
      assert(ultimoDiag.rechazadosFueraBanda==0&&ultimoDiag.causa==V2_DIAG_DETECCIONES);
    }
  }
  // Salirse lateralmente de la banda mantiene su diagnostico independiente.
  reset();frame(0,500,240,20,60);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_FUERA_BANDA);
  assert(ultimoDiag.rechazadosFueraBanda==1&&ultimoDiag.rechazadosFueraCalibracion==0);
'''
    cpp = cpp.replace('// RS485_CALIBRATION_REPLAY', replay if namespace == 'ProtocoloRS485' else '')
    return source, cpp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    env = os.environ.copy()
    compiler_dir = str(Path(args.compiler).parent)
    env['PATH'] = compiler_dir + os.pathsep + str(Path(compiler_dir) / 'cpp') + os.pathsep + env.get('PATH', '')
    build = Path(tempfile.mkdtemp(prefix='auto-v2-vision-capture-', dir=ROOT / 'tmp'))
    variants = [('i2c', 'pruebas de automatico v2', 'ProtocoloI2C.h', 'ProtocoloI2C'),
                ('rs485', 'automatico v2 rs485', 'ProtocoloRS485.h', 'ProtocoloRS485')]
    sources = []
    for name, folder, protocol, namespace in variants:
        source, cpp = program(ROOT / folder, protocol, namespace)
        sources.append(source)
        code = build / (name + '.cpp')
        code.write_text(cpp, encoding='utf-8')
        exe = build / (name + '.exe')
        subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', '-static', str(code), '-o', str(exe)], check=True, env=env)
        subprocess.run([str(exe)], check=True, env=env)
    # La clasificacion de rechazos RS485 evoluciono; el antecedente I2C conserva
    # su implementacion. Los contratos de reserva y autonomia siguen comunes.
    for marker in ['bool capturaAutonomaV2(', 'bool publicarObjetivoV2(']:
        assert definition(sources[0], marker) == definition(sources[1], marker).replace('RS485', 'I2C'), marker


if __name__ == '__main__':
    main()

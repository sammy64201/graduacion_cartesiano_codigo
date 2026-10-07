"""Ejecuta el estado real ML_BAJANDO_CATCH y sus guardas con hardware simulado."""
import argparse
import csv
from pathlib import Path
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
    prelude = source[source.index('void procesarEntrenamientoML() {'):
                     source.index('    switch (entrenamientoML.fase) {', source.index('void procesarEntrenamientoML() {'))]
    case = definition(source, '        case ML_BAJANDO_CATCH: {')
    cpp = '#include <assert.h>\n#include <math.h>\n#include <stdint.h>\n#include <stdio.h>\n#include <string.h>\n#define F(x) x\n'
    cpp += definition(source, 'enum FaseEntrenamientoML :') + ';\n'
    cpp += definition(source, 'struct ContextoEntrenamientoML {') + ';\n'
    cpp += '''ContextoEntrenamientoML entrenamientoML;
enum { MENU_ENTRENAMIENTO_ML=0, MENU_ENTRENAMIENTO_ML_V2=1, MENU_PRUEBA_SEGUIMIENTO=2 };
int opcionMenu;
'''
    for name in ['ensenanzaMLV2Seleccionada', 'pruebaSeguimientoSeleccionada', 'entrenamientoConResultadoSeleccionado']:
        cpp += definition(source, 'bool ' + name + '() {') + '\n'
    cpp += '''struct Terminal { template<class T> void print(const T&, int=0) {};
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
'''
    cpp += prelude + 'switch (entrenamientoML.fase) {\n' + case + '\ndefault: break;\n}\n}\n'
    cpp += '''int main() {
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
  printf("%d\\n", seq);
} }
'''
    build = Path(tempfile.mkdtemp(prefix='mlv2-catch-', dir=ROOT / 'tmp'))
    (build / 'test.cpp').write_text(cpp, encoding='utf-8')
    exe = build / 'test.exe'
    subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', '-static',
                    str(build / 'test.cpp'), '-o', str(exe)], check=True)
    with (folder / 'registros_v2/ml_v2_2026-10-05_16-57-10.csv').open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    buttons = {r['objective_seq']: r for r in rows if r['event']=='ML_BUTTON_CATCH'}
    grips = {r['objective_seq'] for r in rows if r['event']=='ML_GRIP_APPLIED'}
    assert set(buttons)=={'5', '6', '7'} and grips=={'5'}
    log=(folder / 'registros_v2/i2c_2026-10-05_16-56-13.log').read_text(encoding='utf-8-sig', errors='replace')
    assert log.count('[ML] Cancelando: pieza rebaso el catch antes de DIN04 ML')==2
    result=subprocess.run([str(exe)], input='6\n7\n', text=True, capture_output=True, check=True)
    assert result.stdout.splitlines()==['6', '7']
    print('PASS: estado real de descenso + guardas; catch X ML V2 continua tras rebase, '
          'una sola orden con DIN04, aviso unico; sin DIN04/control/encoder o con timeout cancela. '
          'Registro: objetivos 6/7 sin cierre, 5 con pulso aplicado.')


if __name__ == '__main__':
    main()

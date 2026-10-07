"""Ejecuta la funcion real de orientacion con servo simulado, sin placas."""
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
    source = (ROOT / 'pruebas de automatico v2/ESP/ESP.ino').read_text(encoding='utf-8-sig')
    declarations = []
    for name in ['ANGULO_SERVO_INICIAL', 'ANGULO_GARRA_EJE_X', 'ANGULO_GARRA_EJE_Y',
                 'AUTO_V2_APLICAR_GIRO_POR_CAJA']:
        declarations.append(re.search(r'constexpr (?:int|bool) ' + name + r'\s*=\s*[^;]+;', source).group())
    cpp = '#include <assert.h>\n#include <stdint.h>\n#include <stdio.h>\n#define F(x) x\n'
    cpp += '#include "' + (ROOT / 'pruebas de automatico v2/ESP/CalibracionAnguloMLV2.h').as_posix() + '"\n'
    cpp += '\n'.join(declarations) + '\n'
    cpp += '''static_assert(AUTO_V2_APLICAR_GIRO_POR_CAJA, "V2 integra la calibracion ML V2");
struct MockSerial {
  template<class T> void print(T) {}
  template<class T> void println(T) {}
} Serial;
struct MockServo { int writes=0; int last=-1; void write(int value) { ++writes; last=value; } } servoRotacion;
int anguloServoRotacion;
uint16_t sesionArranque=1;
unsigned long millis() { return 100; }
'''
    cpp += definition(source, 'void orientarGarraAutomatica(') + '\n'
    cpp += '''int main() {
  const uint8_t votes[][2] = {{3,0},{0,3},{0,0},{3,3},{1,0},{0,1},{255,0},{0,255}};
  // V2 comparte la calibracion por clase; ambiguedad conserva el ajuste manual.
  for (int manual=0; manual<=180; ++manual) {
    for (int clase=0; clase<=8; ++clase) for (const auto &pair : votes) {
      anguloServoRotacion=manual; servoRotacion.writes=0; servoRotacion.last=-1;
      const int sug=CalibracionAnguloMLV2::sugerir(clase,pair[0],pair[1]);
      orientarGarraAutomatica(pair[0],pair[1],true,clase,42);
      assert(anguloServoRotacion==(sug>=0 ? sug : manual));
      assert(servoRotacion.writes==(sug>=0 ? 1 : 0));
    }
  }
  // El Automatico original conserva su comportamiento previo.
  servoRotacion.writes=0;
  orientarGarraAutomatica(3,0,false);
  assert(anguloServoRotacion==ANGULO_GARRA_EJE_X && servoRotacion.writes==1);
  orientarGarraAutomatica(0,3,false);
  assert(anguloServoRotacion==ANGULO_GARRA_EJE_Y && servoRotacion.writes==2);
  orientarGarraAutomatica(3,3,false);
  assert(anguloServoRotacion==ANGULO_SERVO_INICIAL && servoRotacion.writes==3);
  puts("PASS: V2 usa la calibracion ML V2; 181 ajustes, 9 clases, 8 combinaciones; ambiguedad conserva manual; Automatico original conserva su comportamiento.");
}
'''
    build = Path(tempfile.mkdtemp(prefix='auto-v2-giro-', dir=ROOT / 'tmp'))
    (build / 'test.cpp').write_text(cpp, encoding='utf-8')
    exe = build / 'test.exe'
    subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', '-static',
                    str(build / 'test.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    # La opcion de giro fijo permanece disponible para comparar lotes.
    fixed = cpp.replace('AUTO_V2_APLICAR_GIRO_POR_CAJA = true',
                        'AUTO_V2_APLICAR_GIRO_POR_CAJA = false')
    fixed = fixed[:fixed.index('int main()')] + '''int main() {
      for (int angle=0; angle<=180; ++angle) {
        anguloServoRotacion=angle; servoRotacion.writes=0;
        orientarGarraAutomatica(3,0,true,6,42);
        assert(anguloServoRotacion==angle && servoRotacion.writes==0);
      }
    }'''
    fixed = fixed.replace('static_assert(AUTO_V2_APLICAR_GIRO_POR_CAJA,',
                          'static_assert(!AUTO_V2_APLICAR_GIRO_POR_CAJA,')
    (build / 'fixed.cpp').write_text(fixed, encoding='utf-8')
    subprocess.run([args.compiler, '-std=c++11', '-static', str(build / 'fixed.cpp'),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()

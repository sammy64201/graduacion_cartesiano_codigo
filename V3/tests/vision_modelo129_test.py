"""Prueba nativa de los filtros y geometria reales, mas replay de la captura UART.

Ejecutar con Python y --compiler <g++.exe>. No requiere camara ni placas.
"""
import argparse
import csv
from collections import Counter
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    start = source.index('bool ' + name + '(') if name == 'pixelToMillimeters' else source.index('uint8_t ' + name + '(')
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    paths = ['ESP/VisionModelo129.h', 'pruebas de automatico v2/ESP/VisionModelo129.h',
             'tests/HUSKYLENS2_SEGMENTACION/VisionModelo129.h']
    assert len({(ROOT / p).read_bytes() for p in paths}) == 1, 'Headers distintos'
    variants = [
        ('principal', 'ESP/ESP.ino', 'estimarEjeCaja', 'estimarEjeCaja(&r, true)'),
        ('maqueta', 'pruebas de automatico v2/ESP/ESP.ino', 'estimarEjeCajaV2', 'estimarEjeCajaV2(c)'),
        ('aislada', 'tests/HUSKYLENS2_SEGMENTACION/HUSKYLENS2_SEGMENTACION.ino', 'estimateBoxAxis', 'estimateBoxAxis(&r)'),
    ]
    cpp = '#include <assert.h>\n#include <math.h>\n#include <stdio.h>\n#include <string.h>\n'
    cpp += '#include "' + (ROOT / paths[0]).as_posix() + '"\nusing namespace VisionModelo129;\n'
    cpp += '''static_assert(INDICE_MODELO == 1, "Modelo incorrecto");
static_assert(clasePermitida("pieza6") == 6, "Clase 6");
static_assert(clasePermitida("pieza7") == 7, "Clase 7");
static_assert(clasePermitida("pieza60") == 0, "No aceptar prefijos");
static_assert(clasePermitida("") == 0 && clasePermitida(nullptr) == 0, "Sin nombre");
static_assert(ejePorDimensiones(20, 10) == 1, "X");
static_assert(ejePorDimensiones(10, 20) == 2, "Y");
static_assert(ejePorDimensiones(10, 10) == 0, "Cuadrada");
static_assert(ejePorDimensiones(0, 10) == 0 && ejePorDimensiones(-1, 10) == 0, "Tamano invalido");
'''
    for namespace, path, estimator, call in variants:
        source = (ROOT / path).read_text(encoding='utf-8-sig')
        cpp += 'namespace ' + namespace + ' {\n'
        cpp += '''struct Point2D { double x, y; };
struct Result { int xCenter=100, yCenter=100, width=80, height=40; };
struct CandidatoPiezaV2 { int centroXpx=100, centroYpx=100, anchoPx=80, altoPx=40; };
double H[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
bool homographyValid=true;
constexpr double RELACION_MINIMA_ORIENTACION=1.35;
'''
        cpp += function(source, 'pixelToMillimeters') + '\n' + function(source, estimator) + '\n'
        cpp += 'void probar() { Result r; CandidatoPiezaV2 c;\n'
        cpp += 'assert(' + call + ' == 1);\n'
        # Correccion anisotropica: una caja ancha en pixeles es alta en mm.
        cpp += 'H[1][1]=4; assert(' + call + ' == 2); H[1][1]=2; assert(' + call + ' == 0);\n'
        cpp += 'H[2][2]=0; assert(' + call + ' == 0); H[2][2]=1; H[1][1]=1;\n'
        # Caja vacia y homografia que produce valores no finitos.
        cpp += 'r.width=c.anchoPx=0; assert(' + call + ' == 0); r.width=c.anchoPx=80;\n'
        cpp += 'H[0][0]=NAN; assert(' + call + ' == 0); H[0][0]=1;\n}\n}\n'
    cpp += 'int main() { principal::probar(); maqueta::probar(); aislada::probar();\n'
    cpp += '''char name[128]; while (fgets(name, sizeof(name), stdin)) {
  name[strcspn(name, "\\r\\n")] = '\\0';
  printf("%u\\n", clasePermitida(name));
} }
'''
    (ROOT / 'tmp').mkdir(exist_ok=True)
    build = Path(tempfile.mkdtemp(prefix='vision-modelo129-', dir=ROOT / 'tmp'))
    (build / 'test.cpp').write_text(cpp, encoding='utf-8')
    exe = build / 'test.exe'
    subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', '-static',
                    str(build / 'test.cpp'), '-o', str(exe)], check=True)
    capture = ROOT / 'tests/HUSKYLENS2_SEGMENTACION/registrador_un_com/registros/captura_COM14_20261005_113358_eeb17b.csv'
    with capture.open(encoding='utf-8-sig', newline='') as stream:
        rows = [r for r in csv.DictReader(stream) if r['tipo'] == 'segmentacion' and r['nombre']]
    names = [r['nombre'] for r in rows] + ['pieza60', 'pieza70', 'pieza6 extra', 'Pieza6', '', '6', 'pieza8']
    result = subprocess.run([str(exe)], input='\n'.join(names) + '\n', text=True,
                            capture_output=True, check=True)
    classes = list(map(int, result.stdout.splitlines()))
    assert classes == [{'pieza6': 6, 'pieza7': 7}.get(n, 0) for n in names]
    assert all(r['id'] == '0' and r['algoritmo'] == '129' for r in rows)
    counts = Counter(classes[:len(rows)])
    assert counts[6] == 17 and counts[7] == 16, counts
    print(f'PASS: geometria de los tres sketches, nombres exactos, headers identicos; '
          f'replay {len(rows)} detecciones: pieza6={counts[6]}, pieza7={counts[7]}, ignoradas={counts[0]}.')


if __name__ == '__main__':
    main()

"""Verifica la ISR y las paradas reales de RS485, sin placas ni hardware.

El ultimo paso debe conservar un pulso HIGH completo antes de bajar STEP.
Una parada entre interrupciones tampoco puede truncar ese pulso ni sumar pasos.
Los fuentes C++, la salida del compilador y el resultado quedan en tmp/.
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / 'automatico v2 rs485/PORTENTA/PORTENTA.ino'


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


def program():
    source = SKETCH.read_text(encoding='utf-8-sig')
    cpp = '#include <cassert>\n#include <cstdint>\n#include <cstdio>\n#include <vector>\n'
    cpp += '#define HIGH true\n#define LOW false\n'
    for name in ['pP_X', 'pP_Y', 'pP_Z', 'DIV_MANUAL']:
        declaration = re.search(r'const [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source)
        if not declaration:
            raise AssertionError('No existe parametro: ' + name)
        cpp += declaration.group() + '\n'
    for prefix in ['mov', 'pulso', 'cuenta', 'divisor', 'pasos', 'objetivo']:
        for axis in 'XYZ':
            for suffix in (['', 'Activo'] if prefix == 'objetivo' else ['']):
                name = prefix + axis + suffix
                declaration = re.search(r'volatile [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source)
                if not declaration:
                    raise AssertionError('No existe estado ticker: ' + name)
                cpp += declaration.group() + '\n'
    cpp += r'''
uint32_t tiempoUs = 0;
int bloqueoInterrupciones = 0;
void noInterrupts() { ++bloqueoInterrupciones; }
void interrupts() { assert(bloqueoInterrupciones > 0); --bloqueoInterrupciones; }
struct Cambio { int pin; bool high; uint32_t tiempoUs; };
struct Salidas {
    bool nivel[8] = {};
    std::vector<Cambio> cambios;
    void set(int pin, bool high) {
        assert(pin >= 0 && pin < 8);
        if (nivel[pin] != high) cambios.push_back({pin, high, tiempoUs});
        nivel[pin] = high;
    }
} digital_outputs;
'''
    for marker in ['void generarPulsoMotor(', 'void detenerX(',
                   'void detenerY(', 'void detenerZ(']:
        function = definition(source, marker)
        assert function == definition(marker + ');\n' + source, marker)
        cpp += function + '\n'
    cpp += r'''
struct Eje {
    char nombre;
    int pin;
    volatile int8_t &mov;
    volatile bool &pulso;
    volatile uint16_t &cuenta;
    volatile uint16_t &divisor;
    volatile long &pasos;
    volatile bool &activo;
    volatile long &objetivo;
    void (*detener)();
};
Eje ejes[] = {
    {'X', pP_X, movX, pulsoX, cuentaX, divisorX, pasosX, objetivoXActivo, objetivoX, detenerX},
    {'Y', pP_Y, movY, pulsoY, cuentaY, divisorY, pasosY, objetivoYActivo, objetivoY, detenerY},
    {'Z', pP_Z, movZ, pulsoZ, cuentaZ, divisorZ, pasosZ, objetivoZActivo, objetivoZ, detenerZ},
};
void reset() {
    assert(bloqueoInterrupciones == 0);
    tiempoUs = 0;
    digital_outputs = {};
    for (auto &e : ejes) {
        e.mov = 0; e.pulso = false; e.cuenta = 0; e.divisor = 1;
        e.pasos = 17; e.activo = false; e.objetivo = 17;
    }
}
void tick() {
    assert(bloqueoInterrupciones == 0);
    tiempoUs += 100;
    generarPulsoMotor();
}
void iniciar(Eje &e, int direccion, uint16_t divisor, int cantidad) {
    e.divisor = divisor;
    e.mov = static_cast<int8_t>(direccion);
    e.activo = true;
    e.objetivo = e.pasos + direccion * cantidad;
}
void comprobarOtrosEjes(const Eje &actual) {
    for (const auto &e : ejes) {
        if (e.nombre == actual.nombre) continue;
        assert(e.pasos == 17 && e.mov == 0 && !e.activo);
        assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    }
}
void comprobarPulsos(Eje &e, unsigned esperados) {
    unsigned subidas = 0, bajadas = 0;
    bool high = false;
    uint32_t inicio = 0;
    for (const auto &c : digital_outputs.cambios) {
        assert(c.pin == e.pin);
        if (c.high) {
            assert(!high);
            high = true; inicio = c.tiempoUs; ++subidas;
        } else {
            assert(high);
            // El ticker mas rapido dispone de un semiperiodo de 100 us.
            // Al completar una orden tampoco debe haber un pulso de 0 us.
            assert(c.tiempoUs - inicio >= 100);
            high = false; ++bajadas;
        }
    }
    assert(!high && subidas == esperados && bajadas == esperados);
}
void completarObjetivo(Eje &e, int direccion, uint16_t divisor, int cantidad) {
    reset(); iniciar(e, direccion, divisor, cantidad);
    const long destino = e.objetivo;
    unsigned ticks = 0;
    while (e.mov != 0) {
        tick(); assert(++ticks <= 2U * divisor * static_cast<unsigned>(cantidad));
    }
    assert(e.pasos == destino && !e.activo);
    // El contador y el movimiento ya terminaron, pero el ultimo HIGH sigue
    // presente hasta la siguiente ISR. No hay otro flanco de subida.
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    const uint32_t ultimoHigh = tiempoUs;
    assert(digital_outputs.cambios.back().high);
    assert(digital_outputs.cambios.back().tiempoUs == ultimoHigh);
    tiempoUs += 37;
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    tiempoUs = ultimoHigh;
    tick();
    assert(tiempoUs - ultimoHigh == 100);
    assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    assert(e.pasos == destino && e.mov == 0 && !e.activo);
    for (int i = 0; i < 20; ++i) tick();
    assert(e.pasos == destino);
    comprobarPulsos(e, static_cast<unsigned>(cantidad));
    comprobarOtrosEjes(e);
}
void detenerEntreISR(Eje &e, int direccion, uint16_t divisor, bool despuesDelObjetivo) {
    reset(); iniciar(e, direccion, divisor, despuesDelObjetivo ? 1 : 4);
    for (uint16_t i = 0; i < divisor; ++i) tick();
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    assert(e.pasos == 17 + direccion);
    const uint32_t flancoUs = tiempoUs;
    tiempoUs += 37; // Parada de loop, entre el flanco HIGH y el proximo ticker.
    e.detener();
    assert(bloqueoInterrupciones == 0);
    assert(e.mov == 0 && !e.activo);
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    // Paradas repetidas no convierten el HIGH pendiente en un pulso corto.
    e.detener();
    assert(e.pulso && digital_outputs.nivel[e.pin]);
    tiempoUs = flancoUs;
    tick();
    assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    assert(e.pasos == 17 + direccion);
    for (int i = 0; i < 20; ++i) tick();
    assert(e.pasos == 17 + direccion);
    comprobarPulsos(e, 1);
    comprobarOtrosEjes(e);
}
void detenerEnLOW(Eje &e, int direccion, uint16_t divisor) {
    reset(); iniciar(e, direccion, divisor, 4);
    for (uint16_t i = 0; i < 2 * divisor; ++i) tick();
    assert(!e.pulso && !digital_outputs.nivel[e.pin]);
    e.detener();
    assert(e.mov == 0 && !e.activo && !e.pulso);
    for (int i = 0; i < 20; ++i) tick();
    assert(e.pasos == 17 + direccion);
    comprobarPulsos(e, 1);
    comprobarOtrosEjes(e);
}
int main() {
    unsigned casos = 0;
    for (auto &e : ejes) {
        for (int direccion : {-1, 1}) {
            for (uint16_t divisor : {uint16_t(1), uint16_t(8)}) {
                for (int pasos : {1, 4}) {
                    completarObjetivo(e, direccion, divisor, pasos); ++casos;
                }
                detenerEntreISR(e, direccion, divisor, false); ++casos;
                detenerEntreISR(e, direccion, divisor, true); ++casos;
                detenerEnLOW(e, direccion, divisor); ++casos;
            }
        }
    }
    std::printf("PASS: %u casos ISR/paradas reales; X/Y/Z, +/- y divisor 1/8; ultimo HIGH >=100 us, sin pasos extra\n", casos);
}
'''
    return cpp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=shutil.which('g++') or
                        r'C:/Program Files/Webots/msys64/mingw64/bin/g++.exe')
    parser.add_argument('--build-dir', type=Path,
                        default=ROOT / 'tmp/captura-fija-v2/pulsos-motor')
    args = parser.parse_args()
    build = args.build_dir.resolve()
    build.mkdir(parents=True, exist_ok=True)
    source = build / 'pulsos_motor.cpp'
    executable = build / ('pulsos_motor.exe' if os.name == 'nt' else 'pulsos_motor')
    source.write_text(program(), encoding='utf-8')
    environment = os.environ.copy()
    compiler = str(Path(args.compiler).resolve()) if Path(args.compiler).exists() else args.compiler
    compiler_folder = Path(compiler).parent
    environment['PATH'] = str(compiler_folder) + os.pathsep + str(compiler_folder / 'cpp') + os.pathsep + environment.get('PATH', '')
    result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-pedantic',
                             str(source), '-o', str(executable)],
                            capture_output=True, text=True, env=environment)
    (build / 'compilacion.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    if result.returncode:
        print(result.stdout + result.stderr, end='')
        raise SystemExit(result.returncode)
    result = subprocess.run([str(executable)], capture_output=True, text=True, env=environment)
    (build / 'resultado.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    print(result.stdout + result.stderr, end='')
    print('Logs: ' + str(build))
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()

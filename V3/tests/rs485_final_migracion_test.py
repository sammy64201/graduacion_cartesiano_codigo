"""Verifica identidad con produccion y ejecuta ambos sketches sin hardware."""
import argparse
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    compiler = Path(args.compiler).resolve()
    os.environ['PATH'] = str(compiler.parent) + os.pathsep + str(compiler.parent / 'cpp') + os.pathsep + os.environ['PATH']
    banco = ROOT / 'tests/RS485_FINAL_MIGRACION'
    for name in ['FinalRS485.h', 'ProtocoloRS485.h', 'EnlaceRS485.h', 'ConfiguracionRS485.h']:
        assert (banco / 'ESP32' / name).read_bytes() == (banco / 'PORTENTA' / name).read_bytes(), name
        if name != 'FinalRS485.h':
            assert (banco / 'ESP32' / name).read_bytes() == (ROOT / 'automatico v2 rs485/ESP' / name).read_bytes(), name
        if name in ['ProtocoloRS485.h', 'EnlaceRS485.h']:
            assert (banco / 'ESP32' / name).read_bytes() == (ROOT / 'automatico v2 rs485/PORTENTA' / name).read_bytes(), name
    build = ROOT / 'tmp/rs485-final-migracion'
    build.mkdir(parents=True, exist_ok=True)
    exe = build / 'banco.exe'
    subprocess.run([str(compiler), '-std=c++11', '-Wall', '-Wextra', '-static',
                    '-I' + str(ROOT / 'tests/rs485_final_mocks'),
                    str(ROOT / 'tests/rs485_final_migracion_test.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    subprocess.run([str(exe), '--desconectado'], check=True)

if __name__ == '__main__':
    main()

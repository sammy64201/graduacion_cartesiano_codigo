"""Verifica calibracion de servo y su procedencia en los ensayos ML V2 reales."""
import argparse
import csv
from pathlib import Path
import re
import statistics
import subprocess
import tempfile

ROOT = Path('C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    folder = ROOT / 'automatico v2 rs485'
    capture = (ROOT / 'pruebas de automatico v2') / 'registros_v2/ml_v2_2026-10-05_16-34-24.csv'
    with capture.open(encoding='utf-8-sig', newline='') as stream:
        records = list(csv.DictReader(stream))
    samples = [r for r in records if r['event'] == 'ML_SAMPLE']
    suggestions = [r for r in records if r['event'] == 'ML_ANGLE_SUGGESTION']
    assert len(samples) == len(suggestions) == 10
    assert len({r['session_esp'] for r in samples + suggestions}) == 1
    by_id = {r['objective_seq']: r for r in suggestions}
    assert len(by_id) == 10 and len({r['objective_seq'] for r in samples}) == 10
    log = ((ROOT / 'pruebas de automatico v2') / 'registros_v2/i2c_2026-10-05_16-31-59.log').read_text(encoding='utf-8-sig', errors='replace')
    header = folder / 'ESP/CalibracionAnguloMLV2.h'
    successful = {6: [], 7: []}
    for sample in samples:
        suggestion = by_id[sample['objective_seq']]
        assert sample['class'] == suggestion['class']
        assert sample['mode'] == 'ML_V2'
        assert suggestion['suggested_rot_deg'] == '0'
        assert float(suggestion['height_px']) >= 1.35 * float(suggestion['width_px'])
        assert sample['raw'] in log and suggestion['raw'] in log
        if sample['physical_result'] == 'EXITO':
            successful[int(sample['class'])].append(int(sample['label_rot_deg']))
    assert len(successful[6]) == 7 and len(successful[7]) == 2
    medians = {piece: int(statistics.median(angles)) for piece, angles in successful.items()}
    header_text = header.read_text(encoding='utf-8')
    for piece, median in medians.items():
        match = re.search(rf'SERVO_PIEZA{piece}_RECTA_Y\s*=\s*(\d+)', header_text)
        assert match and int(match.group(1)) == median
    horizontal_capture = (ROOT / 'pruebas de automatico v2') / 'registros_v2/ml_v2_2026-10-05_17-13-14.csv'
    with horizontal_capture.open(encoding='utf-8-sig', newline='') as stream:
        horizontal_records = list(csv.DictReader(stream))
    horizontal_samples = [r for r in horizontal_records if r['event'] == 'ML_SAMPLE']
    horizontal_suggestions = {r['objective_seq']: r for r in horizontal_records
                              if r['event'] == 'ML_ANGLE_SUGGESTION'}
    horizontal_log = ((ROOT / 'pruebas de automatico v2') / 'registros_v2/i2c_2026-10-05_17-10-47.log').read_text(encoding='utf-8-sig', errors='replace')
    assert len(horizontal_samples) == 6
    horizontal_successful = {6: [], 7: []}
    for sample in horizontal_samples:
        suggestion = horizontal_suggestions[sample['objective_seq']]
        assert sample['physical_result'] == 'EXITO' and sample['mode'] == 'ML_V2'
        assert sample['class'] == suggestion['class']
        assert sample['raw'] in horizontal_log and suggestion['raw'] in horizontal_log
        horizontal_successful[int(sample['class'])].append(int(sample['label_rot_deg']))
    assert len(horizontal_successful[6]) == 1 and len(horizontal_successful[7]) == 5
    horizontal_medians = {piece: int(statistics.median(angles))
                          for piece, angles in horizontal_successful.items()}
    for piece, median in horizontal_medians.items():
        match = re.search(rf'SERVO_PIEZA{piece}_RECTA_X\s*=\s*(\d+)', header_text)
        assert match and int(match.group(1)) == median == 59
    cpp = '#include <assert.h>\n#include <stdio.h>\n'
    cpp += '#include "' + header.as_posix() + '"\nusing namespace CalibracionAnguloMLV2;\n'
    cpp += '''static_assert(sugerir(6, 0, 3) == 155, "Calibracion pieza6");
static_assert(sugerir(7, 0, 3) == 166, "Calibracion pieza7");
static_assert(sugerir(6, 3, 0) == 59, "Calibracion horizontal pieza6");
static_assert(sugerir(7, 3, 0) == 59, "Calibracion horizontal pieza7");
static_assert(sugerir(6, 0, 0) == -1, "Caja ambigua");
static_assert(sugerir(7, 0, 1) == -1, "Falta consenso");
static_assert(sugerir(6, 1, 3) == -1, "Votos contradictorios");
static_assert(sugerir(0, 0, 3) == -1 && sugerir(8, 0, 3) == -1, "Clase no permitida");
int main() { int piece, vx, vy; while (scanf("%d %d %d", &piece, &vx, &vy) == 3) {
  printf("%d\\n", sugerir(piece, vx, vy));
} }
'''
    build = Path(tempfile.mkdtemp(prefix='mlv2-rectas-', dir=ROOT / 'tmp'))
    (build / 'test.cpp').write_text(cpp, encoding='utf-8')
    exe = build / 'test.exe'
    subprocess.run([args.compiler, '-std=c++11', '-Wall', '-Wextra', '-static',
                    str(build / 'test.cpp'), '-o', str(exe)], check=True)
    replay = [f"{r['class']} 0 3" for r in samples]
    expected = [medians[int(r['class'])] for r in samples]
    for sample in horizontal_samples:
        suggestion = horizontal_suggestions[sample['objective_seq']]
        vx, vy = int(suggestion['orientation_votes_x']), int(suggestion['orientation_votes_y'])
        replay.append(f"{sample['class']} {vx} {vy}")
        expected.append(horizontal_medians[int(sample['class'])] if vx >= 2 and vy == 0 else -1)
    result = subprocess.run([str(exe)], input='\n'.join(replay) + '\n',
                            text=True, capture_output=True, check=True)
    assert list(map(int, result.stdout.splitlines())) == expected
    print('PASS: 16 intentos cruzados CSV/log; calibracion Y 155/166, X 59/59; '
          'replay C++ conserva NA en horizontal ambigua y votos contradictorios; '
          'sin consenso u otras clases no sugiere.')


if __name__ == '__main__':
    main()

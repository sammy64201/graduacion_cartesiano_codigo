"""Resume ensayos de AJUSTE CATCH V2; no modifica firmware ni calibraciones."""
import argparse
import csv
from collections import Counter, defaultdict
from pathlib import Path
import math
import statistics


def number(row, key):
    try:
        value = float(row.get(key, ''))
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def interval32(end, start):
    if not end or not start:
        return None
    return (int(end) - int(start)) % (2**32)


def summary(values, unit, digits=3):
    if not values:
        return 'sin datos validos'
    return (f'n={len(values)}, mediana={statistics.median(values):.{digits}f} {unit}, '
            f'rango={min(values):.{digits}f}..{max(values):.{digits}f} {unit}')


def analyze(paths):
    groups = defaultdict(list)
    for path in paths:
        with Path(path).open(encoding='utf-8-sig', newline='') as stream:
            reader = csv.DictReader(stream)
            required = {'event', 'mode', 'session_esp', 'objective_seq', 'timing_result'}
            if not required.issubset(reader.fieldnames or []):
                raise ValueError(f'{path}: no es un CSV actualizado de ajuste de catch')
            for row in reader:
                if row['mode'] == 'CATCH_CAL' and row.get('objective_seq'):
                    # Separar archivos evita colisiones tras sesiones distintas;
                    # dentro de un archivo no contar dos veces una fila repetida.
                    groups[(str(Path(path).resolve()), row['session_esp'], row['objective_seq'])].append(row)
    counts = Counter()
    correct_delays, descents, ratios, scales = [], [], [], []
    by_class = defaultdict(list)
    adaptive = defaultdict(Counter)
    latest = {}
    details = []
    conflicts = 0
    for key, rows in groups.items():
        samples = [r for r in rows if r['event'] == 'CAL_SAMPLE']
        if not samples:
            continue
        signatures = {(r.get('timing_result'), r.get('catch_start_ms'),
                       r.get('delta_reference_ms'), r.get('tested_offset_ms'),
                       r.get('next_offset_ms')) for r in samples}
        if len(signatures) != 1:
            conflicts += 1
            continue
        sample = samples[0]
        label = sample['timing_result']
        if label not in {'CORRECTO', 'TEMPRANO', 'TARDE'}:
            continue
        counts[label] += 1
        delta = number(sample, 'delta_reference_ms')
        speed = number(sample, 'trigger_speed_mm_s')
        tested = number(sample, 'tested_offset_ms')
        following = number(sample, 'next_offset_ms')
        if tested is not None:
            adaptive[(sample.get('class', '?'), int(tested))][label] += 1
            session_key = key[:2]
            trial = number(sample, 'adjust_trials') or 0
            if session_key not in latest or trial >= latest[session_key][0]:
                latest[session_key] = (trial, sample)
        descent = interval32(number(sample, 'catch_command_ms'), number(sample, 'catch_start_ms'))
        if descent is not None:
            descents.append(descent)
        if tested is None and label == 'CORRECTO' and sample.get('reference_status') == 'OBSERVADA' and delta is not None:
            correct_delays.append(delta)
            by_class[sample.get('class', '?')].append(delta)
        cameras = [r for r in rows if r['event'] == 'CAMERA_SPEED' and
                   r.get('camera_speed_valid') == '1']
        if cameras:
            ratio = number(cameras[-1], 'scale_ratio_camera_encoder')
            scale = number(cameras[-1], 'scale_camera_suggested_mm_count')
            if ratio is not None and ratio > 0:
                ratios.append(ratio)
            if scale is not None and scale > 0:
                scales.append(scale)
        details.append(f"| {key[1]} / {key[2]} | {sample.get('class','')} | {label} | "
                       f"{delta if delta is not None else 'NA'} | {sample.get('reference_status','')} | "
                       f"{speed if speed is not None else 'NA'} | {descent if descent is not None else 'NA'} | "
                       f"{int(tested) if tested is not None else 'NA'} | {int(following) if following is not None else 'NA'} |")
    lines = ['# Resultado de pruebas de catch V2', '',
             f"Ensayos etiquetados: {sum(counts.values())}; CORRECTO={counts['CORRECTO']}, "
             f"TEMPRANO={counts['TEMPRANO']}, TARDE={counts['TARDE']}. "
             f'Grupos con etiquetas contradictorias excluidos: {conflicts}.', '',
             'Ensayos manuales historicos CORRECTO, referencia observada: ' + summary(correct_delays, 'ms') + '.',
             'Tiempo desde inicio Z hasta orden de cierre: ' + summary(descents, 'ms') + '.',
             'Relacion recorrido camara / encoder: ' + summary(ratios, '', 6) + '.',
             'Escala candidata segun camara: ' + summary(scales, 'mm/cuenta', 7) + '.', '',
             'Ensayos actuales: tested_offset_ms es el ajuste ejecutado; next_offset_ms es el siguiente. '
             'Positivo retrasa el descenso, negativo reduce la espera estable respecto de 300 ms. '
             'La etiqueta CORRECTO confirma el agarre observado. Delta describe el disparo respecto de la '
             'referencia nominal; PROYECTADA no es un instante observado. '
             'CSV historicos sin tested_offset_ms corresponden al disparo manual con X.', '',
             'La mediana es una referencia de ensayos, no una correccion aplicada. '
             'La reaccion humana, la homografia, el centro de la caja y el seguimiento afectan estas medidas. '
             'Una relacion cercana a 1 indica acuerdo entre camara y encoder en esa ventana; '
             'no demuestra la distancia fisica ni garantiza el agarre. '
             'No aplicar una escala candidata a partir de una sola pieza.', '']
    if not adaptive and len(correct_delays) < 5:
        lines.append('Todavia faltan ensayos CORRECTO con referencia OBSERVADA: reunir al menos cinco y repetir a distintas velocidades.')
    for piece, values in sorted(by_class.items()):
        lines.append(f'Clase {piece}: ' + summary(values, 'ms') + '.')
    for (piece, offset), outcomes in sorted(adaptive.items()):
        lines.append(f"Clase {piece}, ajuste probado {offset} ms: agarres={outcomes['CORRECTO']}, "
                     f"antes={outcomes['TEMPRANO']}, despues={outcomes['TARDE']}.")
    for (path, session), (_, sample) in latest.items():
        tested = int(number(sample, 'tested_offset_ms'))
        following = number(sample, 'next_offset_ms')
        confirmed = sample.get('adjust_confirmed') == '1' and sample.get('timing_result') == 'CORRECTO'
        state = '3 agarres consecutivos registrados; comprobar otras velocidades' if confirmed else 'EN PRUEBA'
        lines += ['', f'{Path(path).name}, sesion {session}: {state}. '
                  f'Ultimo probado={tested} ms; proximo={following} ms.',
                  f'Linea del ultimo valor probado: `constexpr int32_t V2_AJUSTE_DISPARO_CATCH_MS = {tested};`']
        if sample.get('adjust_limit') == '1':
            lines.append('Limite alcanzado: revisar escala del encoder y medidas fisicas antes de ampliar el rango.')
    lines += ['', '| Sesion / objetivo | Clase | Resultado | Delta ms | Referencia | Velocidad mm/s | Z hasta orden ms | Probado ms | Proximo ms |',
              '|---|---|---|---:|---|---:|---:|---:|---:|'] + details
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', type=Path)
    parser.add_argument('--salida', type=Path)
    args = parser.parse_args()
    report = analyze(args.csv)
    if args.salida:
        args.salida.write_text(report, encoding='utf-8')
    print(report)


if __name__ == '__main__':
    main()

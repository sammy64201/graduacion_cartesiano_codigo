"""Verifica deduplicacion, referencias proyectadas y ventanas de camara invalidas."""
import csv
import importlib.util
from pathlib import Path
import tempfile

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('catch_analysis', root / 'pruebas de automatico v2/analizar_catch_v2.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
columns = ['event','mode','session_esp','objective_seq','class','timing_result',
           'catch_start_ms','catch_command_ms','delta_reference_ms','reference_status',
           'trigger_speed_mm_s','camera_speed_valid','scale_ratio_camera_encoder',
           'scale_camera_suggested_mm_count','tested_offset_ms','next_offset_ms',
           'adjust_trials','adjust_confirmed','adjust_limit']
data = []
for seq, label, status, delta in [('1','CORRECTO','OBSERVADA','100'),
                                  ('2','CORRECTO','PROYECTADA','-200'),
                                  ('3','TARDE','OBSERVADA','300')]:
    row = dict(event='CAL_SAMPLE',mode='CATCH_CAL',session_esp='9',objective_seq=seq,
               **{'class':'6'},timing_result=label,catch_start_ms='1200',catch_command_ms='1800',
               delta_reference_ms=delta,reference_status=status,trigger_speed_mm_s='75')
    data += [row, row.copy()]
data += [dict(event='CAMERA_SPEED',mode='CATCH_CAL',session_esp='9',objective_seq='1',
              camera_speed_valid='1',scale_ratio_camera_encoder='1.1',scale_camera_suggested_mm_count='0.0825'),
         dict(event='CAMERA_SPEED',mode='CATCH_CAL',session_esp='9',objective_seq='2',
              camera_speed_valid='0',scale_ratio_camera_encoder='99',scale_camera_suggested_mm_count='99')]
with tempfile.TemporaryDirectory(prefix='catch-analysis-', dir=root/'tmp') as folder:
    path = Path(folder)/'sample.csv'
    with path.open('w',encoding='utf-8-sig',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=columns)
        writer.writeheader(); writer.writerows(data)
    report = module.analyze([path])
    assert 'Ensayos etiquetados: 3' in report
    assert 'n=1, mediana=100.000 ms' in report
    assert 'mediana=1.100' in report and 'mediana=99' not in report
    assert 'mediana=600.000 ms' in report
    assert module.interval32(100,2**32-100)==200
    data.append({**data[0],'timing_result':'TEMPRANO'})
    with path.open('w',encoding='utf-8',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=columns)
        writer.writeheader(); writer.writerows(data)
    report = module.analyze([path])
    assert 'Ensayos etiquetados: 2' in report and 'contradictorias excluidos: 1' in report
    # La referencia proyectada de un disparo autonomo sigue siendo un ensayo
    # real del offset; no mezclarlo con la mediana historica de pulsaciones X.
    automatic = []
    for seq in range(1,4):
        automatic.append(dict(event='CAL_SAMPLE',mode='CATCH_CAL',session_esp='10',
            objective_seq=str(seq),timing_result='CORRECTO',**{'class':'7'},
            tested_offset_ms='-100',next_offset_ms='-100',adjust_trials=str(seq),
            adjust_confirmed='1' if seq==3 else '0',reference_status='PROYECTADA',
            delta_reference_ms='-100'))
    with path.open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns)
        writer.writeheader(); writer.writerows(automatic)
    report=module.analyze([path])
    assert 'Clase 7, ajuste probado -100 ms: agarres=3' in report
    assert '3 agarres consecutivos registrados' in report
    assert 'V2_AJUSTE_DISPARO_CATCH_MS = -100;' in report
    assert 'mediana=-100' not in report
    automatic.append({**automatic[-1], 'objective_seq':'4','timing_result':'TARDE',
        'tested_offset_ms':'-200','next_offset_ms':'-200','adjust_trials':'4',
        'adjust_confirmed':'0','adjust_limit':'1'})
    with path.open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns)
        writer.writeheader(); writer.writerows(automatic)
    report=module.analyze([path])
    assert 'sesion 10: EN PRUEBA' in report and 'Limite alcanzado' in report
    assert '3 agarres consecutivos registrados' not in report
print('PASS: analisis de catch deduplica, excluye contradicciones/proyecciones, valida camara y wrap de tiempos')

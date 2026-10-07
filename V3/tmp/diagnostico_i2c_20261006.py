import json
import re
from datetime import datetime, timezone, timedelta
from pathlib import Path

root = Path(__file__).resolve().parents[1]
path = root / 'pruebas de automatico v2/registros_v2/i2c_2026-10-06_17-16-41.log'
rows = []
for number, line in enumerate(path.read_text(encoding='utf-8-sig').splitlines()[1:], 2):
    timestamp, elapsed, board, raw = line.split('|', 3)
    t = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    values = {k: int(v) for k, v in re.findall(r'(\w+)=(-?\d+)', raw)}
    rows.append(dict(line=number, time=t, board=board, raw=raw, values=values))


def measure(board, start, end):
    selected = [r for r in rows if r['board'] == board and
                r['raw'].startswith('[I2C] rx') and start <= r['time'].strftime('%H:%M:%S.%f') <= end]
    first, last = selected[0], selected[-1]
    delta = {k: last['values'][k]-v for k, v in first['values'].items()
             if k in last['values'] and k not in ('estado', 'ultimoTx', 'encVel', 'encCps')
             and not (board == 'E' and k == 'len')}
    result = dict(board=board, first_line=first['line'], last_line=last['line'],
                  start=first['time'].isoformat(), end=last['time'].isoformat(),
                  duration_s=(last['time']-first['time']).total_seconds(), delta=delta)
    if board == 'P':
        result['read_error_percent'] = 100 * delta['rxError'] / (delta['rxOK']+delta['rxError'])
        result['write_error_percent'] = 100 * delta['txError'] / (delta['txOK']+delta['txError'])
    else:
        result['rejected_receive_percent'] = 100 * (delta['rx']-delta['ok']) / delta['rx']
    return result


stats = {}
for name, board, start, end in [
    ('whole_portenta', 'P', '23:16:42', '23:17:12'),
    ('whole_esp', 'E', '23:16:42', '23:17:12'),
    ('before_arm_portenta', 'P', '23:16:42', '23:16:46.400000'),
    ('before_arm_esp', 'E', '23:16:42', '23:16:46.600000'),
    ('moving_y_portenta', 'P', '23:17:04', '23:17:07.600000'),
    ('moving_y_esp', 'E', '23:17:04', '23:17:07.600000'),
    ('after_recovery_portenta', 'P', '23:17:08', '23:17:12'),
    ('after_recovery_esp', 'E', '23:17:08', '23:17:12'),
]:
    stats[name] = measure(board, start, end)

for board in ['E', 'P']:
    selected = [r for r in rows if r['board'] == board and r['raw'].startswith('[I2C] rx')]
    keys = ['rx', 'ok', 'requests', 'reinicios'] if board == 'E' else ['rxOK', 'rxError', 'txOK', 'txError', 'reinicios']
    assert all(all(b['values'][k] >= a['values'][k] for k in keys)
               for a, b in zip(selected, selected[1:])), 'Contador retrocedio'

events = [r for r in rows if any(marker in r['raw'] for marker in
          ['Enlace perdido', 'Maestro reiniciado', 'Primer paquete completo',
           'Movimiento interrumpido', 'BUSCANDO Y+', 'Inicio autonomo'])]
stats['events'] = [dict(line=r['line'], local_time=r['time'].astimezone(
    timezone(timedelta(hours=-6))).isoformat(), board=r['board'], raw=r['raw']) for r in events]
output = root / 'outputs/diagnostico_i2c_20261006_171641.json'
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(stats, ensure_ascii=False, indent=2))

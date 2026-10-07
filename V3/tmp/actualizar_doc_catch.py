from pathlib import Path

path = Path('pruebas de automatico v2/README_PRUEBAS.md')
text = path.read_text(encoding='utf-8-sig').replace('version 14', 'version 15').replace('protocolo 14', 'protocolo 15')
tick = chr(96)
lines = []
for line in text.splitlines():
    if 'ENSENANZA ML V2' in line and 'SEGUIMIENTO Y' in line and 'AJUSTE CATCH V2' not in line:
        line = line.rstrip(',') + ', ' + tick + 'AJUSTE CATCH V2' + tick + ','
    lines.append(line)
    if line.startswith('|') and 'V2' in line and 'Autom' in line:
        lines.append('| Ajuste catch V2 | Brazo, camara y encoder |')
path.write_text('\n'.join(lines) + '\n', encoding='utf-8')

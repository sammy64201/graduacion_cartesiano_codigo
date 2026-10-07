from pathlib import Path
import re

current = Path('pruebas de automatico v2/PORTENTA/PORTENTA.ino').read_text(encoding='utf-8-sig')
previous = Path('tmp/build-auto-v2-integracion-portenta/sketch/PORTENTA.ino.cpp').read_text(encoding='utf-8-sig')

def body(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    depth, end = 1, opening+1
    while depth:
        depth += (source[end]=='{') - (source[end]=='}')
        end += 1
    return source[start:end]

for marker in ['bool seguirPiezaY(float piezaY) {', 'bool seguirPiezaYAutomaticoV2() {',
               'bool actualizarObjetivoMovilV2() {', 'bool iniciarTrasladoEntrega() {']:
    assert body(current, marker) == body(previous, marker), marker
for name in ['AUTO_V2_SEGUIMIENTO_Y', 'V2_SEGUIMIENTO_ESTABLE_MS', 'ENCODER_DIAMETRO_RUEDA_MM',
             'ENCODER_CUENTAS_X2_POR_VUELTA', 'V2_AJUSTE_DISTANCIA_CATCH_MM', 'CATCH_ADELANTO_EXTRA_MS']:
    expr = r'constexpr [^;\n]+\b' + name + r'\s*=\s*[^;]+;'
    assert re.search(expr,current).group() == re.search(expr,previous).group(), name
print('PASS: seguimiento, prediccion encoder, entrega y parametros conservados respecto al firmware V2 anterior')

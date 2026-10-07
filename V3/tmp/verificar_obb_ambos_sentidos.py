"""Verificacion offline del helper existente; no controla hardware."""
import importlib.util
import math
from pathlib import Path

source = Path('C:/Users/samue/OneDrive/Documents/Universidad/CodexModelos/ENTRENAMIENTOS/OBB/CANMV_TEST/obb_geometry.py')
spec = importlib.util.spec_from_file_location('geometria_obb_existente', source)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)

def rectangle(degrees):
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return [value for x, y in [(-40, -10), (40, -10), (40, 10), (-40, 10)]
            for value in (120 + x*c - y*s, 90 + x*s + y*c)]

def box_size(vertices):
    xs, ys = vertices[::2], vertices[1::2]
    return max(xs)-min(xs), max(ys)-min(ys)

for degrees in [30, 45, 60]:
    positive, negative = rectangle(degrees), rectangle(-degrees)
    for p, n in zip(box_size(positive), box_size(negative)):
        assert math.isclose(p, n, abs_tol=1e-9)
    for angle, vertices in [(degrees, positive), (-degrees, negative)]:
        for offset in range(4):
            points = [vertices[i:i+2] for i in range(0, 8, 2)]
            for order in [points, points[::-1]]:
                rotated = order[offset:] + order[:offset]
                result = geometry.detection_record([v for point in rotated for v in point], 0, .9, ['pieza6'])
                assert math.isclose(result['angle'], angle, abs_tol=1e-9)
    print(f'PASS: OBB distingue +{degrees}/-{degrees}; ancho/alto de AABB identicos.')

def duplicar_lista(numeros):
    """Devuelve una lista con cada numero duplicado."""
    return [numero * 2 for numero in numeros]


def filtrar_mayores_que(numeros, limite):
    """Devuelve los numeros mayores que el limite indicado."""
    return [numero for numero in numeros if numero > limite]

from ejercicios.cadenas import saludar, contar_letras
from ejercicios.listas import duplicar_lista, filtrar_mayores_que
from ejercicios.numeros import sumar, es_par


def mostrar_titulo(texto):
    print()
    print("=" * len(texto))
    print(texto)
    print("=" * len(texto))


def main():
    mostrar_titulo("Practica con funciones simples")

    nombre = "Samuel"
    print("Saludo:", saludar(nombre))
    print("Letras en el nombre:", contar_letras(nombre))

    a = 4
    b = 7
    print("Suma:", sumar(a, b))
    print("El primer numero es par:", es_par(a))

    numeros = [1, 2, 3, 4, 5]
    print("Lista original:", numeros)
    print("Lista duplicada:", duplicar_lista(numeros))
    print("Mayores que 3:", filtrar_mayores_que(numeros, 3))


if __name__ == "__main__":
    main()

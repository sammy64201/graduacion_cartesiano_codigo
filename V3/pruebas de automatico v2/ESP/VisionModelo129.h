#ifndef VISION_MODELO_129_H
#define VISION_MODELO_129_H

#include <stdint.h>

// Copia identica en ESP/, pruebas de automatico v2/ESP/ y la prueba de
// segmentacion. Arduino compila cada carpeta como un sketch independiente.
namespace VisionModelo129 {
constexpr uint8_t INDICE_MODELO = 1;  // ALGORITHM_CUSTOM_BEGIN + 1 = 129
constexpr double RELACION_MINIMA_EJE = 1.35;

constexpr bool nombresIguales(const char *a, const char *b) {
  return *a == *b && (*a == '\0' || nombresIguales(a + 1, b + 1));
}

// Captura 2026-10-05: ID raw=0 para todas las clases. La identidad es name.
// No aceptar un ID numerico sin nombre ni coincidencias parciales (pieza60).
constexpr uint8_t clasePermitida(const char *nombre) {
  return nombre == nullptr ? 0 :
    (nombresIguales(nombre, "pieza6") ? 6 :
      (nombresIguales(nombre, "pieza7") ? 7 : 0));
}

// Aproximacion del eje de la caja en mm: 0 ambiguo, 1 X, 2 Y.
// No representa un giro continuo de la silueta ni resuelve diagonales.
constexpr uint8_t ejePorDimensiones(double ancho, double alto,
                                   double relacion = RELACION_MINIMA_EJE) {
  return !(ancho > 0.0 && alto > 0.0 && relacion > 1.0) ? 0 :
    (ancho >= relacion * alto ? 1 : (alto >= relacion * ancho ? 2 : 0));
}
}  // namespace VisionModelo129
#endif

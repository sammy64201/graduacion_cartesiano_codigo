#ifndef CALIBRACION_ANGULO_MLV2_H
#define CALIBRACION_ANGULO_MLV2_H

#include <stdint.h>

namespace CalibracionAnguloMLV2 {
// Fuente: ml_v2_2026-10-05_16-34-24.csv, ML_SAMPLE confirmados EXITO.
// El operador confirma que todas las piezas estaban rectas. Las diez cajas
// altas activaron eje Y. Son angulos de servo del montaje, no de la silueta.
// pieza6: 7 exitos, mediana 155, rango 151..161. Se excluye el fallo a 165.
// pieza7: 2 exitos, mediana 166, rango 161..171. Calibracion preliminar.
constexpr int16_t SERVO_PIEZA6_RECTA_Y = 155;
constexpr int16_t SERVO_PIEZA7_RECTA_Y = 166;
static_assert(SERVO_PIEZA6_RECTA_Y >= 0 && SERVO_PIEZA6_RECTA_Y <= 180,
              "Calibracion pieza6 fuera del servo");
static_assert(SERVO_PIEZA7_RECTA_Y >= 0 && SERVO_PIEZA7_RECTA_Y <= 180,
              "Calibracion pieza7 fuera del servo");

constexpr int16_t sugerir(uint8_t clase, uint8_t votosX, uint8_t votosY) {
  return votosY < 2 || votosX != 0 ? -1 :
    (clase == 6 ? SERVO_PIEZA6_RECTA_Y :
      (clase == 7 ? SERVO_PIEZA7_RECTA_Y : -1));
}
}  // namespace CalibracionAnguloMLV2
#endif

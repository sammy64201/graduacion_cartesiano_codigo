#ifndef CALIBRACION_ANGULO_MLV2_H
#define CALIBRACION_ANGULO_MLV2_H

#include <stdint.h>

namespace CalibracionAnguloMLV2 {
// Fuentes: ml_v2_2026-10-05_16-34-24.csv (rectas Y) y
// ml_v2_2026-10-05_17-13-14.csv (horizontales X), solo ML_SAMPLE EXITO.
// Son angulos de servo del montaje, no medidas del giro de la silueta.
// pieza6: 7 exitos, mediana 155, rango 151..161. Se excluye el fallo a 165.
// pieza7: 2 exitos, mediana 166, rango 161..171. Calibracion preliminar.
// Horizontal: pieza6 1 exito y pieza7 5 exitos, todos con servo 59.
// Un angulo que funciona repetido no determina su precision fisica.
constexpr int16_t SERVO_PIEZA6_RECTA_X = 59;
constexpr int16_t SERVO_PIEZA7_RECTA_X = 59;
constexpr int16_t SERVO_PIEZA6_RECTA_Y = 155;
constexpr int16_t SERVO_PIEZA7_RECTA_Y = 166;
static_assert(SERVO_PIEZA6_RECTA_X >= 0 && SERVO_PIEZA6_RECTA_X <= 180 &&
              SERVO_PIEZA7_RECTA_X >= 0 && SERVO_PIEZA7_RECTA_X <= 180,
              "Calibracion horizontal fuera del servo");
static_assert(SERVO_PIEZA6_RECTA_Y >= 0 && SERVO_PIEZA6_RECTA_Y <= 180,
              "Calibracion pieza6 fuera del servo");
static_assert(SERVO_PIEZA7_RECTA_Y >= 0 && SERVO_PIEZA7_RECTA_Y <= 180,
              "Calibracion pieza7 fuera del servo");

constexpr int16_t sugerir(uint8_t clase, uint8_t votosX, uint8_t votosY) {
  // Diagonales/cajas ambiguas siguen sin sugerencia. La caja no contiene
  // el sentido del giro: no interpolar entre X/Y como si fuera una medicion.
  return votosX >= 2 && votosY == 0 ?
    (clase == 6 ? SERVO_PIEZA6_RECTA_X :
      (clase == 7 ? SERVO_PIEZA7_RECTA_X : -1)) :
    (votosY >= 2 && votosX == 0 ?
      (clase == 6 ? SERVO_PIEZA6_RECTA_Y :
        (clase == 7 ? SERVO_PIEZA7_RECTA_Y : -1)) : -1);
}
}  // namespace CalibracionAnguloMLV2
#endif

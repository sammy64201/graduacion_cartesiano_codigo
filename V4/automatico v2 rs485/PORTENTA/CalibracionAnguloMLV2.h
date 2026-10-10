#ifndef CALIBRACION_ANGULO_MLV2_H
#define CALIBRACION_ANGULO_MLV2_H

#include <math.h>
#include <stdint.h>

// Copia identica en ESP/ y PORTENTA/: la ESP decide el giro y la Portenta
// deduce del giro aplicado el retraso del catch.
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
static_assert(SERVO_PIEZA6_RECTA_Y > SERVO_PIEZA6_RECTA_X &&
              SERVO_PIEZA7_RECTA_Y > SERVO_PIEZA7_RECTA_X,
              "La interpolacion supone recta Y por encima de recta X");

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

// ---------------------------------------------------------------------------
// Giro aproximado por dimensiones de caja (2026-10-09).
//
// HUSKYLENS 2 entrega para el modelo 129 solo centro, ancho y alto de una caja
// paralela a la imagen: ni angulo ni signo. Para un rectangulo de largo L y
// ancho W girado theta respecto de X, la caja mide L|cos|+W|sin| en X y
// L|sin|+W|cos| en Y. Normalizando cada eje con las cajas rectas medidas por
// clase se despeja |theta| (0 = eje largo en X, 90 = eje largo en Y) sin
// depender de la escala ni del margen propio de cada eje. El signo
// (+theta/-theta) no existe en la caja: lo elige quien llama.
//
// Cajas rectas: medianas de ml_v2_2026-10-05_*.csv sin cajas recortadas,
// convertidas a la homografia vigente (TAG_ROWS_DISTANCE_MM 382 -> 255).
// pieza6: eje X n=5, eje Y n=42. pieza7: eje X n=102, eje Y n=16.
// ---------------------------------------------------------------------------
struct CajaRecta {
  float anchoEjeX;  // extension X (mm) con el eje largo en X
  float altoEjeX;   // extension Y (mm) con el eje largo en X
  float anchoEjeY;  // extension X (mm) con el eje largo en Y
  float altoEjeY;   // extension Y (mm) con el eje largo en Y
};
constexpr CajaRecta CAJA_PIEZA6 = {49.2f, 24.1f, 21.5f, 42.5f};
constexpr CajaRecta CAJA_PIEZA7 = {52.0f, 24.2f, 21.6f, 50.9f};
// Hasta este |theta| (o desde 90 menos este valor) se usa el servo recto
// calibrado. Replay de 224 cajas rectas reales: todas quedan rectas y el
// peor promedio de 3+ se desvia 8 grados (pieza7 horizontal). Un error de
// ~10 grados ya fallo en ML V2 (pieza6 a 165 frente a 155).
constexpr float TOLERANCIA_RECTA_DEG = 12.0f;
// Una caja que no corresponde a una pieza entera de la clase (recortada,
// dos piezas, falsa deteccion) se aleja de norma 1. Cajas rectas reales:
// pieza7 0.87..1.16; pieza6 0.38..1.45 (p5 0.67). Solo filtra errores
// groseros; la decision usa el promedio de las detecciones compatibles.
constexpr float NORMA_MINIMA = 0.50f;
constexpr float NORMA_MAXIMA = 1.80f;

inline bool cajaRecta(uint8_t clase, CajaRecta &caja) {
  if (clase == 6) { caja = CAJA_PIEZA6; return true; }
  if (clase == 7) { caja = CAJA_PIEZA7; return true; }
  return false;
}

inline bool servosRectos(uint8_t clase, int16_t &servoX, int16_t &servoY) {
  if (clase == 6) {
    servoX = SERVO_PIEZA6_RECTA_X; servoY = SERVO_PIEZA6_RECTA_Y; return true;
  }
  if (clase == 7) {
    servoX = SERVO_PIEZA7_RECTA_X; servoY = SERVO_PIEZA7_RECTA_Y; return true;
  }
  return false;
}

struct EstimacionGiro {
  bool valida;
  float thetaDeg;  // |theta| 0..90 del eje largo respecto de X
  float norma;     // ~1 cuando la caja es de una pieza entera de la clase
};

inline EstimacionGiro estimarGiro(uint8_t clase, float anchoMm, float altoMm) {
  EstimacionGiro e = {false, 0.0f, 0.0f};
  CajaRecta r;
  if (!cajaRecta(clase, r) || !(anchoMm > 0.0f) || !(altoMm > 0.0f)) return e;
  // Ancho/largo de la pieza por el eje X, el de menor margen en las cajas.
  const float k = r.anchoEjeY / r.anchoEjeX;
  const float u = (anchoMm - r.anchoEjeY) / (r.anchoEjeX - r.anchoEjeY);
  const float v = (altoMm - r.altoEjeX) / (r.altoEjeY - r.altoEjeX);
  // u = (c + k s - k)/(1 - k) y v = (s + k c - k)/(1 - k); se despejan c, s.
  const float p = k + u * (1.0f - k);
  const float q = k + v * (1.0f - k);
  const float c = fmaxf(0.0f, p - k * q);
  const float s = fmaxf(0.0f, q - k * p);
  e.norma = sqrtf(c * c + s * s) / (1.0f - k * k);
  e.thetaDeg = atan2f(s, c) * 57.2957795f;
  e.valida = isfinite(e.thetaDeg) && isfinite(e.norma) &&
             e.norma >= NORMA_MINIMA && e.norma <= NORMA_MAXIMA;
  return e;
}

// Servo para el eje largo a thetaDeg de X (-90..90). Interpola entre las
// rectas calibradas de la clase. Por debajo de 0 usa la simetria de 180
// grados de la pinza; si ningun equivalente cabe en 0..180 elige el extremo
// mas cercano. -1 si la clase no tiene calibracion.
inline int16_t servoParaGiro(uint8_t clase, float thetaDeg) {
  int16_t servoX = 0, servoY = 0;
  if (!servosRectos(clase, servoX, servoY) || !isfinite(thetaDeg)) return -1;
  const float t = fmaxf(-90.0f, fminf(90.0f, thetaDeg));
  const float g = static_cast<float>(servoY - servoX) / 90.0f;
  if (t >= 0.0f) return static_cast<int16_t>(lroundf(servoX + g * t));
  const float bajoX = servoX + g * t;
  const float sobreY = servoY + g * (t + 90.0f);
  if (bajoX >= 0.0f) return static_cast<int16_t>(lroundf(bajoX));
  if (sobreY <= 180.0f) return static_cast<int16_t>(lroundf(sobreY));
  return -bajoX <= sobreY - 180.0f ? 0 : 180;
}

// Decision completa: rectas con su servo calibrado; diagonales con el signo
// elegido (+1: servo entre recta X y recta Y; -1: el otro sentido).
inline int16_t servoPorGiro(uint8_t clase, float thetaDeg, int8_t signo) {
  int16_t servoX = 0, servoY = 0;
  if (!servosRectos(clase, servoX, servoY) || !isfinite(thetaDeg) ||
      thetaDeg < 0.0f || thetaDeg > 90.0f) return -1;
  if (thetaDeg <= TOLERANCIA_RECTA_DEG) return servoX;
  if (thetaDeg >= 90.0f - TOLERANCIA_RECTA_DEG) return servoY;
  return servoParaGiro(clase, signo < 0 ? -thetaDeg : thetaDeg);
}

// |cos theta| del eje largo que corresponde a un servo aplicado (inversa de
// servoParaGiro; el signo no cambia el resultado). 1 = eje largo en X,
// 0 = eje largo en Y. -1 si la clase no tiene calibracion.
inline float fraccionEjeXPorServo(uint8_t clase, int16_t servo) {
  int16_t servoX = 0, servoY = 0;
  if (!servosRectos(clase, servoX, servoY)) return -1.0f;
  const float g = static_cast<float>(servoY - servoX) / 90.0f;
  const float t = servo <= servoY
    ? static_cast<float>(servo - servoX) / g
    : static_cast<float>(servo - servoY) / g - 90.0f;
  return fminf(1.0f, fabsf(cosf(t * 0.0174532925f)));
}
}  // namespace CalibracionAnguloMLV2
#endif

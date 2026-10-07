#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/ESP/VisionModelo129.h"
using namespace VisionModelo129;
static_assert(INDICE_MODELO == 1, "Modelo incorrecto");
static_assert(clasePermitida("pieza6") == 6, "Clase 6");
static_assert(clasePermitida("pieza7") == 7, "Clase 7");
static_assert(clasePermitida("pieza60") == 0, "No aceptar prefijos");
static_assert(clasePermitida("") == 0 && clasePermitida(nullptr) == 0, "Sin nombre");
static_assert(ejePorDimensiones(20, 10) == 1, "X");
static_assert(ejePorDimensiones(10, 20) == 2, "Y");
static_assert(ejePorDimensiones(10, 10) == 0, "Cuadrada");
static_assert(ejePorDimensiones(0, 10) == 0 && ejePorDimensiones(-1, 10) == 0, "Tamano invalido");
namespace principal {
struct Point2D { double x, y; };
struct Result { int xCenter=100, yCenter=100, width=80, height=40; };
struct CandidatoPiezaV2 { int centroXpx=100, centroYpx=100, anchoPx=80, altoPx=40; };
double H[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
bool homographyValid=true;
constexpr double RELACION_MINIMA_ORIENTACION=1.35;
bool pixelToMillimeters(
  double u,
  double v,
  bool homographyValid,
  Point2D &physicalPoint
) {
  if (!homographyValid) {
    return false;
  }

  const double denominator = H[2][0] * u + H[2][1] * v + H[2][2];
  if (fabs(denominator) < 1e-12 || !isfinite(denominator)) {
    return false;
  }

  physicalPoint.x =
    (H[0][0] * u + H[0][1] * v + H[0][2]) / denominator;
  physicalPoint.y =
    (H[1][0] * u + H[1][1] * v + H[1][2]) / denominator;
  return isfinite(physicalPoint.x) && isfinite(physicalPoint.y);
}
uint8_t estimarEjeCaja(const Result *result, bool homografiaValida) {
  if (result->width <= 0 || result->height <= 0) return 0;
  const double izquierda = result->xCenter - result->width * 0.5;
  const double derecha = result->xCenter + result->width * 0.5;
  const double arriba = result->yCenter - result->height * 0.5;
  const double abajo = result->yCenter + result->height * 0.5;
  Point2D esquinas[4];
  if (!pixelToMillimeters(izquierda, arriba, homografiaValida, esquinas[0]) ||
      !pixelToMillimeters(derecha, arriba, homografiaValida, esquinas[1]) ||
      !pixelToMillimeters(izquierda, abajo, homografiaValida, esquinas[2]) ||
      !pixelToMillimeters(derecha, abajo, homografiaValida, esquinas[3])) return 0;
  double minX = esquinas[0].x, maxX = minX;
  double minY = esquinas[0].y, maxY = minY;
  for (uint8_t i = 1; i < 4; ++i) {
    minX = fmin(minX, esquinas[i].x); maxX = fmax(maxX, esquinas[i].x);
    minY = fmin(minY, esquinas[i].y); maxY = fmax(maxY, esquinas[i].y);
  }
  return VisionModelo129::ejePorDimensiones(maxX - minX, maxY - minY);
}
void probar() { Result r; CandidatoPiezaV2 c;
assert(estimarEjeCaja(&r, true) == 1);
H[1][1]=4; assert(estimarEjeCaja(&r, true) == 2); H[1][1]=2; assert(estimarEjeCaja(&r, true) == 0);
H[2][2]=0; assert(estimarEjeCaja(&r, true) == 0); H[2][2]=1; H[1][1]=1;
r.width=c.anchoPx=0; assert(estimarEjeCaja(&r, true) == 0); r.width=c.anchoPx=80;
H[0][0]=NAN; assert(estimarEjeCaja(&r, true) == 0); H[0][0]=1;
}
}
namespace maqueta {
struct Point2D { double x, y; };
struct Result { int xCenter=100, yCenter=100, width=80, height=40; };
struct CandidatoPiezaV2 { int centroXpx=100, centroYpx=100, anchoPx=80, altoPx=40; };
double H[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
bool homographyValid=true;
constexpr double RELACION_MINIMA_ORIENTACION=1.35;
bool pixelToMillimeters(
  double u,
  double v,
  bool homographyValid,
  Point2D &physicalPoint
) {
  if (!homographyValid) {
    return false;
  }

  const double denominator = H[2][0] * u + H[2][1] * v + H[2][2];
  if (fabs(denominator) < 1e-12 || !isfinite(denominator)) {
    return false;
  }

  physicalPoint.x =
    (H[0][0] * u + H[0][1] * v + H[0][2]) / denominator;
  physicalPoint.y =
    (H[1][0] * u + H[1][1] * v + H[1][2]) / denominator;
  return isfinite(physicalPoint.x) && isfinite(physicalPoint.y);
}
uint8_t estimarEjeCajaV2(const CandidatoPiezaV2 &candidato) {
  if (candidato.anchoPx <= 0 || candidato.altoPx <= 0) return 0;
  const double izquierda = candidato.centroXpx - candidato.anchoPx * 0.5;
  const double derecha = candidato.centroXpx + candidato.anchoPx * 0.5;
  const double arriba = candidato.centroYpx - candidato.altoPx * 0.5;
  const double abajo = candidato.centroYpx + candidato.altoPx * 0.5;
  Point2D esquinas[4];
  if (!pixelToMillimeters(izquierda, arriba, true, esquinas[0]) ||
      !pixelToMillimeters(derecha, arriba, true, esquinas[1]) ||
      !pixelToMillimeters(izquierda, abajo, true, esquinas[2]) ||
      !pixelToMillimeters(derecha, abajo, true, esquinas[3])) return 0;
  double minX = esquinas[0].x, maxX = minX;
  double minY = esquinas[0].y, maxY = minY;
  for (uint8_t i = 1; i < 4; ++i) {
    minX = fmin(minX, esquinas[i].x);
    maxX = fmax(maxX, esquinas[i].x);
    minY = fmin(minY, esquinas[i].y);
    maxY = fmax(maxY, esquinas[i].y);
  }
  const double anchoMm = maxX - minX;
  const double altoMm = maxY - minY;
  return VisionModelo129::ejePorDimensiones(
    anchoMm, altoMm, RELACION_MINIMA_ORIENTACION);
}
void probar() { Result r; CandidatoPiezaV2 c;
assert(estimarEjeCajaV2(c) == 1);
H[1][1]=4; assert(estimarEjeCajaV2(c) == 2); H[1][1]=2; assert(estimarEjeCajaV2(c) == 0);
H[2][2]=0; assert(estimarEjeCajaV2(c) == 0); H[2][2]=1; H[1][1]=1;
r.width=c.anchoPx=0; assert(estimarEjeCajaV2(c) == 0); r.width=c.anchoPx=80;
H[0][0]=NAN; assert(estimarEjeCajaV2(c) == 0); H[0][0]=1;
}
}
namespace aislada {
struct Point2D { double x, y; };
struct Result { int xCenter=100, yCenter=100, width=80, height=40; };
struct CandidatoPiezaV2 { int centroXpx=100, centroYpx=100, anchoPx=80, altoPx=40; };
double H[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
bool homographyValid=true;
constexpr double RELACION_MINIMA_ORIENTACION=1.35;
bool pixelToMillimeters(
  double u,
  double v,
  Point2D &physicalPoint
) {
  if (!homographyValid) {
    return false;
  }

  const double denominator =
    H[2][0] * u +
    H[2][1] * v +
    H[2][2];

  if (fabs(denominator) < 1e-12) {
    return false;
  }

  physicalPoint.x =
    (
      H[0][0] * u +
      H[0][1] * v +
      H[0][2]
    ) / denominator;

  physicalPoint.y =
    (
      H[1][0] * u +
      H[1][1] * v +
      H[1][2]
    ) / denominator;

  return true;
}
uint8_t estimateBoxAxis(const Result *result) {
  if (!homographyValid || result->width <= 0 || result->height <= 0) return 0;
  const double left = result->xCenter - result->width * 0.5;
  const double right = result->xCenter + result->width * 0.5;
  const double top = result->yCenter - result->height * 0.5;
  const double bottom = result->yCenter + result->height * 0.5;
  Point2D corners[4];
  if (!pixelToMillimeters(left, top, corners[0]) ||
      !pixelToMillimeters(right, top, corners[1]) ||
      !pixelToMillimeters(left, bottom, corners[2]) ||
      !pixelToMillimeters(right, bottom, corners[3])) return 0;
  double minX = corners[0].x, maxX = minX;
  double minY = corners[0].y, maxY = minY;
  for (uint8_t i = 1; i < 4; ++i) {
    minX = fmin(minX, corners[i].x); maxX = fmax(maxX, corners[i].x);
    minY = fmin(minY, corners[i].y); maxY = fmax(maxY, corners[i].y);
  }
  return VisionModelo129::ejePorDimensiones(maxX - minX, maxY - minY);
}
void probar() { Result r; CandidatoPiezaV2 c;
assert(estimateBoxAxis(&r) == 1);
H[1][1]=4; assert(estimateBoxAxis(&r) == 2); H[1][1]=2; assert(estimateBoxAxis(&r) == 0);
H[2][2]=0; assert(estimateBoxAxis(&r) == 0); H[2][2]=1; H[1][1]=1;
r.width=c.anchoPx=0; assert(estimateBoxAxis(&r) == 0); r.width=c.anchoPx=80;
H[0][0]=NAN; assert(estimateBoxAxis(&r) == 0); H[0][0]=1;
}
}
int main() { principal::probar(); maqueta::probar(); aislada::probar();
std::string name; while (std::getline(std::cin, name)) {
  std::cout << static_cast<int>(clasePermitida(name.c_str())) << "\n";
} }

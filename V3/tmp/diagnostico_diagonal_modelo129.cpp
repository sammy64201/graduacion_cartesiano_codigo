#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../pruebas de automatico v2/ESP/VisionModelo129.h"
#include "../pruebas de automatico v2/ESP/CalibracionAnguloMLV2.h"

int main() {
  int previous = -1;
  for (int i = 0; i < 2; ++i) {
    const int degrees = i == 0 ? 30 : -30;
    const double theta = degrees * 3.14159265358979323846 / 180.0;
    const double width = 80.0 * fabs(cos(theta)) + 20.0 * fabs(sin(theta));
    const double height = 80.0 * fabs(sin(theta)) + 20.0 * fabs(cos(theta));
    const unsigned char axis = VisionModelo129::ejePorDimensiones(width, height);
    const int servo = CalibracionAnguloMLV2::sugerir(6, axis == 1 ? 3 : 0, axis == 2 ? 3 : 0);
    assert(axis == 1 && servo == 59);
    if (i) assert(servo == previous);
    previous = servo;
    printf("Pieza %+d grados, caja %.3f x %.3f: eje X, servo %d\n", degrees, width, height, servo);
  }
  puts("CONFIRMADO: una diagonal puede recibir la sugerencia horizontal; la caja pierde el signo.");
}

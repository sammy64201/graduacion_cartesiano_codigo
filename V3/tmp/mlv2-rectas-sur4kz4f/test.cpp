#include <assert.h>
#include <stdio.h>
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/ESP/CalibracionAnguloMLV2.h"
using namespace CalibracionAnguloMLV2;
static_assert(sugerir(6, 0, 3) == 155, "Calibracion pieza6");
static_assert(sugerir(7, 0, 3) == 166, "Calibracion pieza7");
static_assert(sugerir(6, 3, 0) == -1, "Eje X sin calibrar");
static_assert(sugerir(7, 0, 1) == -1, "Falta consenso");
static_assert(sugerir(6, 1, 3) == -1, "Votos contradictorios");
static_assert(sugerir(0, 0, 3) == -1 && sugerir(8, 0, 3) == -1, "Clase no permitida");
int main() { int piece; while (scanf("%d", &piece) == 1) {
  printf("%d\n", sugerir(piece, 0, 3));
} }

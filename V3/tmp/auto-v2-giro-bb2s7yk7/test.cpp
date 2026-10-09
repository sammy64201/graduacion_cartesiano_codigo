#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/ESP/CalibracionAnguloMLV2.h"
constexpr int ANGULO_SERVO_INICIAL = 90;
constexpr int ANGULO_GARRA_EJE_X = 90;
constexpr int ANGULO_GARRA_EJE_Y = 0;
constexpr bool AUTO_V2_APLICAR_GIRO_POR_CAJA = true;
static_assert(AUTO_V2_APLICAR_GIRO_POR_CAJA, "V2 integra la calibracion ML V2");
struct MockSerial {
  template<class T> void print(T) {}
  template<class T> void println(T) {}
} Serial;
struct MockServo { int writes=0; int last=-1; void write(int value) { ++writes; last=value; } } servoRotacion;
int anguloServoRotacion;
uint16_t sesionArranque=1;
unsigned long millis() { return 100; }
void orientarGarraAutomatica(uint8_t votosX, uint8_t votosY, bool modoV2,
                            uint8_t clase = 0, uint16_t secuencia = 0) {
  if (modoV2 && !AUTO_V2_APLICAR_GIRO_POR_CAJA) {
    Serial.print(F("[AUTO V2] Giro por caja desactivado; conserva servo_deg="));
    Serial.print(anguloServoRotacion);
    Serial.print(F(" votos_x="));
    Serial.print(votosX);
    Serial.print(F(" votos_y="));
    Serial.println(votosY);
    return;
  }
  if (modoV2) {
    const int16_t sugerencia = CalibracionAnguloMLV2::sugerir(clase, votosX, votosY);
    if (sugerencia >= 0) {
      anguloServoRotacion = sugerencia;
      servoRotacion.write(anguloServoRotacion);
    }
    Serial.print(F("V2LOG|E|event=AUTO_ANGLE_SUGGESTION|session="));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(millis());
    Serial.print(F("|obj=")); Serial.print(secuencia);
    Serial.print(F("|class=")); Serial.print(clase);
    Serial.print(F("|suggested_rot="));
    if (sugerencia >= 0) Serial.print(sugerencia);
    else Serial.print(F("NA"));
    Serial.print(F("|suggestion_source=MLV2_EJES_20261005|votes_x="));
    Serial.print(votosX);
    Serial.print(F("|votes_y=")); Serial.print(votosY);
    Serial.print(F("|servo_rot_deg=")); Serial.println(anguloServoRotacion);
    return;
  }
  const bool ejeX = votosX >= 2 && votosY == 0;
  const bool ejeY = votosY >= 2 && votosX == 0;
  anguloServoRotacion = ejeX ? ANGULO_GARRA_EJE_X :
    (ejeY ? ANGULO_GARRA_EJE_Y : ANGULO_SERVO_INICIAL);
  servoRotacion.write(anguloServoRotacion);
  Serial.print(modoV2 ? F("[AUTO V2]") : F("[AUTO]"));
  Serial.print(F(" Orientacion aprox eje="));
  Serial.print(ejeX ? F("X") : (ejeY ? F("Y") : F("INDETERMINADO")));
  Serial.print(F(" votos_x="));
  Serial.print(votosX);
  Serial.print(F(" votos_y="));
  Serial.print(votosY);
  Serial.print(F(" servo_deg="));
  Serial.println(anguloServoRotacion);
}
int main() {
  const uint8_t votes[][2] = {{3,0},{0,3},{0,0},{3,3},{1,0},{0,1},{255,0},{0,255}};
  // V2 comparte la calibracion por clase; ambiguedad conserva el ajuste manual.
  for (int manual=0; manual<=180; ++manual) {
    for (int clase=0; clase<=8; ++clase) for (const auto &pair : votes) {
      anguloServoRotacion=manual; servoRotacion.writes=0; servoRotacion.last=-1;
      const int sug=CalibracionAnguloMLV2::sugerir(clase,pair[0],pair[1]);
      orientarGarraAutomatica(pair[0],pair[1],true,clase,42);
      assert(anguloServoRotacion==(sug>=0 ? sug : manual));
      assert(servoRotacion.writes==(sug>=0 ? 1 : 0));
    }
  }
  // El Automatico original conserva su comportamiento previo.
  servoRotacion.writes=0;
  orientarGarraAutomatica(3,0,false);
  assert(anguloServoRotacion==ANGULO_GARRA_EJE_X && servoRotacion.writes==1);
  orientarGarraAutomatica(0,3,false);
  assert(anguloServoRotacion==ANGULO_GARRA_EJE_Y && servoRotacion.writes==2);
  orientarGarraAutomatica(3,3,false);
  assert(anguloServoRotacion==ANGULO_SERVO_INICIAL && servoRotacion.writes==3);
  puts("PASS: V2 usa la calibracion ML V2; 181 ajustes, 9 clases, 8 combinaciones; ambiguedad conserva manual; Automatico original conserva su comportamiento.");
}

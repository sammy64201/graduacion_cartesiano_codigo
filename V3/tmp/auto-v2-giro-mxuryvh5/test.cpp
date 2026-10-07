#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define F(x) x
constexpr int ANGULO_SERVO_INICIAL = 90;
constexpr int ANGULO_GARRA_EJE_X = 90;
constexpr int ANGULO_GARRA_EJE_Y = 0;
constexpr bool AUTO_V2_APLICAR_GIRO_POR_CAJA = false;
static_assert(!AUTO_V2_APLICAR_GIRO_POR_CAJA, "V2 no debe usar la caja como giro medido");
struct MockSerial {
  template<class T> void print(T) {}
  template<class T> void println(T) {}
} Serial;
struct MockServo { int writes=0; int last=-1; void write(int value) { ++writes; last=value; } } servoRotacion;
int anguloServoRotacion;
void orientarGarraAutomatica(uint8_t votosX, uint8_t votosY, bool modoV2) {
  if (modoV2 && !AUTO_V2_APLICAR_GIRO_POR_CAJA) {
    Serial.print(F("[AUTO V2] Giro por caja desactivado; conserva servo_deg="));
    Serial.print(anguloServoRotacion);
    Serial.print(F(" votos_x="));
    Serial.print(votosX);
    Serial.print(F(" votos_y="));
    Serial.println(votosY);
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
  // No movimiento ni sustitucion por el inicial, incluso con votos X/Y fuertes.
  for (int manual=0; manual<=180; ++manual) {
    for (const auto &pair : votes) {
      anguloServoRotacion=manual; servoRotacion.writes=0; servoRotacion.last=-1;
      orientarGarraAutomatica(pair[0],pair[1],true);
      assert(anguloServoRotacion==manual && servoRotacion.writes==0 && servoRotacion.last==-1);
    }
  }
  // El Automatico original conserva su comportamiento previo.
  orientarGarraAutomatica(3,0,false);
  assert(anguloServoRotacion==ANGULO_GARRA_EJE_X && servoRotacion.writes==1);
  orientarGarraAutomatica(0,3,false);
  assert(anguloServoRotacion==ANGULO_GARRA_EJE_Y && servoRotacion.writes==2);
  orientarGarraAutomatica(3,3,false);
  assert(anguloServoRotacion==ANGULO_SERVO_INICIAL && servoRotacion.writes==3);
  puts("PASS: Automatico V2 conserva 181 ajustes para 8 combinaciones de votos, sin escribir servo; Automatico original conserva su comportamiento.");
}

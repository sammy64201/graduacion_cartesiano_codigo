#include <cassert>
#include <cstdio>
#include <stdint.h>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/EnlaceRS485.h"

struct Terminal {
  bool iniciado=false;
  void begin(unsigned baud) { assert(baud==115200); iniciado=true; }
  template<class T> void print(T, int=0) {}
  template<class T> void println(T) {}
} Serial;
struct BusInterno {
  bool listo=false; uint32_t hz=0;
  void begin() { listo=true; }
  void setClock(uint32_t valor) { assert(listo); hz=valor; }
} Wire;
unsigned lecturasDIN=0;
struct Entradas {
  void init() { assert(Wire.listo && Wire.hz==100000); ++lecturasDIN; }
} digital_inputs;
struct Encoder { void reset() {} } encoders[1];
struct Salidas { void set(int, int) {} } digital_outputs;
struct Ticker { void attach(void (*)(), float) {} } motorTicker;
void generarPulsoMotor() {}
constexpr int LOW=0, CAM_CMD_NINGUNO=0;
constexpr int pP_X=4, pP_Y=2, pP_Z=0, pD_X=5, pD_Y=3, pD_Z=1;
constexpr float velocidadMotores=0.0001f;
constexpr float CAMARA_A_HOME_Y_MM=845, V2_AJUSTE_DISTANCIA_CATCH_MM=335;
float escalaEncoderMmPorCuenta=0.075f;
uint32_t tiempoEncendidoSistema=0, inicioEstadoGeneral=0, tAnteriorRS485=0, tAnteriorEstadoESP=0;
bool comunicacionRS485Habilitada=true;
uint8_t comandoCamaraActual=99, secuenciaComandoCamara=99;
uint32_t millis() { return 1234; }
void iniciarRS485Maestro() { assert(Serial.iniciado); }
void detenerTodos() {}
void leerFinalesCarrera() { assert(Wire.listo); ++lecturasDIN; }
void mostrarAyudaTerminal() {}
void setup() {
    Serial.begin(115200);

    // I2C interno de Machine Control: el expansor de entradas/finales usa Wire.
    // Migrar el enlace externo a RS485 no elimina este bus de la Portenta.
    // Sin begin(), el core mbed accede a un master nulo al leer digital_inputs.
    Wire.begin();
    Wire.setClock(100000);

    iniciarRS485Maestro();

    digital_inputs.init();
    encoders[0].reset();
    digital_outputs.set(pP_X, LOW);
    digital_outputs.set(pP_Y, LOW);
    digital_outputs.set(pP_Z, LOW);
    digital_outputs.set(pD_X, LOW);
    digital_outputs.set(pD_Y, LOW);
    digital_outputs.set(pD_Z, LOW);
    detenerTodos();
    leerFinalesCarrera();

    motorTicker.attach(&generarPulsoMotor, velocidadMotores);

    tiempoEncendidoSistema = millis();
    inicioEstadoGeneral = tiempoEncendidoSistema;
    tAnteriorRS485 = tiempoEncendidoSistema;
    tAnteriorEstadoESP = tiempoEncendidoSistema;
    comunicacionRS485Habilitada = false;
    comandoCamaraActual = CAM_CMD_NINGUNO;
    secuenciaComandoCamara = 0;

    Serial.println(F("[BOOT] Portenta coordinadora iniciada en estado seguro"));
    Serial.print(F("[BOOT] RS485 integrado, "));
    Serial.print(EnlaceRS485::BAUD);
    Serial.println(F(" 8N1; retencion inicial 3000 ms"));
    Serial.println(F("[BOOT] RS485: intercambio nominal 10 ms, una solicitud pendiente; OLED conserva I2C"));
    Serial.println(F("[BOOT] Finales Z: DIN04=abajo, DIN05=arriba"));
    Serial.println(F("[BOOT] Encoder 0: OUTA=A0 OUTB=B0 OUTC=Z0, decodificacion X2"));
    Serial.print(F("[BOOT] Escala encoder compilada="));
    Serial.print(escalaEncoderMmPorCuenta, 9);
    Serial.println(F(" mm/cuenta"));
    Serial.print(F("[BOOT] Distancia efectiva camara-catch V2="));
    Serial.print(CAMARA_A_HOME_Y_MM, 1);
    Serial.print(F(" mm (ajuste="));
    Serial.print(V2_AJUSTE_DISTANCIA_CATCH_MM, 1);
    Serial.println(F(" mm)"));
    Serial.println(F("[BOOT] Monitor serial: 115200 baudios"));
    mostrarAyudaTerminal();
}

int main() {
  setup();
  assert(Serial.iniciado && Wire.listo && lecturasDIN==2);
  assert(!comunicacionRS485Habilitada && comandoCamaraActual==CAM_CMD_NINGUNO);
  assert(tiempoEncendidoSistema==1234 && tAnteriorRS485==1234);
  std::puts("PASS: setup real: USB serial y Wire interno antes de DIN/finales, arranque seguro RS485");
}

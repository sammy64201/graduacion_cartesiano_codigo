#include <Wire.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

#define I2C_SDA 27
#define I2C_SCL 14
#define DIRECCION_ESCLAVO 0x40

Servo servo25; // Servo 1 (Rotación)
Servo servo26; // Servo 2 (Pinza)

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int8_t ejeX = 0, ejeY = 0, ejeZ = 0;
uint8_t btConectado = 0, botonX = 0, botonTri = 0, dpadRaw = 0;

int anguloS1 = 90; 
int anguloS2 = 90; 
unsigned long ultimoUpdate = 0;

void requestEvent() {
  uint8_t paquete[8];
  paquete[0] = (ejeX == 0) ? 0 : (ejeX > 0 ? 1 : 2);
  paquete[1] = (ejeY == 0) ? 0 : (ejeY > 0 ? 1 : 2);
  paquete[2] = (ejeZ == 0) ? 0 : (ejeZ > 0 ? 1 : 2);
  paquete[3] = btConectado;
  paquete[4] = botonX;
  paquete[5] = botonTri;
  paquete[6] = (uint8_t)anguloS1;
  paquete[7] = (uint8_t)anguloS2;
  // Nota: Para la navegación, el Portenta usará los ejes X/Y o el D-Pad procesado
  Wire.write(paquete, 8);
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      // 1. Joystick Izquierdo (X e Y)
      ejeX = (abs(ctl->axisX()) > 100) ? (ctl->axisX() > 0 ? 1 : -1) : 0;
      ejeY = (abs(ctl->axisY()) > 100) ? (ctl->axisY() > 0 ? 1 : -1) : 0;

      // 2. Joystick Derecho (Eje Z en vertical | Servo 1 en horizontal)
      int rY = ctl->axisRY();
      int rX = ctl->axisRX();
      ejeZ = (abs(rY) > 100) ? (rY > 0 ? 1 : -1) : 0;

      // 3. Control de Servos (Cada 15ms para suavidad)
      if (millis() - ultimoUpdate > 5) {
        ultimoUpdate = millis();
        
        // Servo 1 con Joystick Derecho Horizontal (Incremental)
        if (rX > 150) anguloS1 = min(180, anguloS1 + 1);
        else if (rX < -150) anguloS1 = max(0, anguloS1 - 1);

        // Servo 2 con D-Pad (Flechas Izquierda/Derecha o Arriba/Abajo según prefieras)
        dpadRaw = ctl->dpad();
        if (dpadRaw & 0x08) anguloS2 = min(180, anguloS2 + 1); // Derecha
        if (dpadRaw & 0x04) anguloS2 = max(0, anguloS2 - 1);   // Izquierda
        
        servo25.write(anguloS1);
        servo26.write(anguloS2);
      }

      botonX = ctl->a() ? 1 : 0; 
      botonTri = ctl->y() ? 1 : 0;
      btConectado = 1;
    }
  }
}

void setup() {
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin(DIRECCION_ESCLAVO);
  Wire.onRequest(requestEvent);
  ESP32PWM::allocateTimer(0);
  servo25.attach(25, 500, 2400);
  servo26.attach(26, 500, 2400);
  BP32.setup(&onConnectedController, nullptr);
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) { myControllers[i] = ctl; break; }
  }
}

void loop() {
  BP32.update();
  processControllers();
  delay(1);
}
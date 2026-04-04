/* * PROYECTO: BRAZO CARTESIANO - ESP32 WROVER (ESCLAVO BT)
 * ARQUITECTURA: Polleo Rápido (15ms) a 400kHz
 * PAYLOAD: [X, Y, Z, BT_Status, Boton_X, Boton_Triangulo] (6 Bytes)
 */

#include <Wire.h>
#include <Bluepad32.h>

#define I2C_SDA 27
#define I2C_SCL 14
#define DIRECCION_ESCLAVO 0x40

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int8_t ejeX = 0, ejeY = 0, ejeZ = 0;
uint8_t btConectado = 0;
uint8_t botonX = 0;        // En PS4 es la 'X' (En Bluepad32 es ctl->a())
uint8_t botonTriangulo = 0; // En PS4 es 'Triangulo' (En Bluepad32 es ctl->y())

void requestEvent() {
  uint8_t paquete[6];
  
  paquete[0] = (ejeX == 0) ? 0 : (ejeX > 0 ? 1 : 2);
  paquete[1] = (ejeY == 0) ? 0 : (ejeY > 0 ? 1 : 2);
  paquete[2] = (ejeZ == 0) ? 0 : (ejeZ > 0 ? 1 : 2);
  paquete[3] = btConectado;
  paquete[4] = botonX;
  paquete[5] = botonTriangulo;
  
  Wire.write(paquete, 6);
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      break;
    }
  }
}

void processControllers() {
  bool hayControlActivo = false;

  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected()) {
      hayControlActivo = true;
      if (ctl->hasData()) {
        int x = ctl->axisX();
        int y = ctl->axisY();
        int z = ctl->axisRY();

        ejeX = (abs(x) > 100) ? (x > 0 ? 1 : -1) : 0;
        ejeY = (abs(y) > 100) ? (y > 0 ? 1 : -1) : 0;
        ejeZ = (abs(z) > 100) ? (z > 0 ? 1 : -1) : 0;

        // Leer botones
        botonX = ctl->a() ? 1 : 0; 
        botonTriangulo = ctl->y() ? 1 : 0;
      }
    }
  }

  btConectado = hayControlActivo ? 1 : 0;

  if (!hayControlActivo) {
      ejeX = 0; ejeY = 0; ejeZ = 0;
      botonX = 0; botonTriangulo = 0;
  }
}

void setup() {
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin(DIRECCION_ESCLAVO);
  Wire.setClock(400000); 
  Wire.onRequest(requestEvent);
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); 
}

void loop() {
  BP32.update();
  processControllers();
  delay(1); 
}
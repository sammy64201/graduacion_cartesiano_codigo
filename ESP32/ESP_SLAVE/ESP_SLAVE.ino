/* * PROYECTO: BRAZO CARTESIANO - ESP32 WROVER (ESCLAVO BT)
 * ARQUITECTURA: Polleo Rápido (15ms) a 400kHz
 * FUNCIÓN: Leer mando Bluetooth y empaquetar 4 bytes por I2C
 */

#include <Wire.h>
#include <Bluepad32.h>

// --- CONFIGURACIÓN DE PINES I2C ---
#define I2C_SDA 27
#define I2C_SCL 14
#define DIRECCION_ESCLAVO 0x40

// --- VARIABLES DE CONTROL ---
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int8_t ejeX = 0, ejeY = 0, ejeZ = 0;
uint8_t btConectado = 0; // 0: Desconectado, 1: Conectado

// --- RUTINA I2C: EL PORTENTA PIDE DATOS ---
void requestEvent() {
  uint8_t paquete[4];
  
  // Mapeo lógico: 0 = STOP, 1 = ADELANTE, 2 = ATRAS
  paquete[0] = (ejeX == 0) ? 0 : (ejeX > 0 ? 1 : 2);
  paquete[1] = (ejeY == 0) ? 0 : (ejeY > 0 ? 1 : 2);
  paquete[2] = (ejeZ == 0) ? 0 : (ejeZ > 0 ? 1 : 2);
  paquete[3] = btConectado;
  
  // Enviamos los 4 bytes al Portenta
  Wire.write(paquete, 4);
}

// --- CALLBACKS BLUEPAD32 ---
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.println("Control Bluetooth Conectado!");
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.println("Control Bluetooth Desconectado.");
      break;
    }
  }
}

// --- PROCESAMIENTO DE JOYSTICKS ---
void processControllers() {
  bool hayControlActivo = false;

  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected()) {
      hayControlActivo = true;
      
      if (ctl->hasData()) {
        int x = ctl->axisX();
        int y = ctl->axisY();
        int z = ctl->axisRY(); // Eje vertical derecho para Z

        // Zona Muerta de 100 (-511 a 511)
        ejeX = (abs(x) > 100) ? (x > 0 ? 1 : -1) : 0;
        ejeY = (abs(y) > 100) ? (y > 0 ? 1 : -1) : 0;
        ejeZ = (abs(z) > 100) ? (z > 0 ? 1 : -1) : 0;
      }
    }
  }

  btConectado = hayControlActivo ? 1 : 0;

  // FAIL-SAFE: Si se desconecta el mando, forzamos STOP
  if (!hayControlActivo) {
      ejeX = 0; 
      ejeY = 0; 
      ejeZ = 0;
  }
}

void setup() {
  Serial.begin(115200);

  // Configurar I2C
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin(DIRECCION_ESCLAVO);
  
  // Fast Mode a 400kHz para coincidir con tu código de Portenta
  Wire.setClock(400000); 
  
  Wire.onRequest(requestEvent);
  
  // Iniciar Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); 
  
  Serial.println("ESP32 Listo. Polleo a 400kHz activado.");
}

void loop() {
  BP32.update();
  processControllers();
  
  // Retardo de 1 milisegundo para mantener el procesador estable y responder rapidísimo
  delay(1); 
}
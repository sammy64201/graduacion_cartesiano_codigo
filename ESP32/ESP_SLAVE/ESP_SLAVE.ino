/* * PROYECTO: BRAZO CARTESIANO - CONTROL TELEOPERADO (NIVEL 4)
 * HARDWARE: ESP32 WROVER (Esclavo) + Bluepad32
 * FUNCIÓN: Leer mando Bluetooth y avisar al Maestro vía Interrupción en GPIO 4
 */

#include <Wire.h>
#include <Bluepad32.h>

// --- CONFIGURACIÓN DE PINES ---
#define I2C_SDA 27
#define I2C_SCL 14
#define PIN_DRDY 4   // Pin de interrupción hacia el transistor 2N2222
#define DIRECCION_ESCLAVO 0x40

// --- VARIABLES DE CONTROL ---
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int8_t ejeX = 0, ejeY = 0, ejeZ = 0;
uint8_t btConectado = 0; // 0: Desconectado, 1: Conectado

// --- RUTINA DE SOLICITUD I2C (MAESTRO PIDE DATOS) ---
void requestEvent() {
  uint8_t paquete[4];
  
  // Mapeo: 0 = STOP, 1 = ADELANTE, 2 = ATRAS
  paquete[0] = (ejeX == 0) ? 0 : (ejeX > 0 ? 1 : 2);
  paquete[1] = (ejeY == 0) ? 0 : (ejeY > 0 ? 1 : 2);
  paquete[2] = (ejeZ == 0) ? 0 : (ejeZ > 0 ? 1 : 2);
  paquete[3] = btConectado;
  
  // Enviamos los 4 bytes al Portenta
  Wire.write(paquete, 4);
  
  // Importante: Bajamos el aviso después de que el maestro leyó los datos
  digitalWrite(PIN_DRDY, LOW);
}

// --- CALLBACKS DE BLUEPAD32 ---
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      foundEmptySlot = true;
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

// --- PROCESAMIENTO DE DATOS DEL MANDO ---
void processControllers() {
  bool hayControlActivo = false;
  
  // Guardamos estados anteriores para detectar cambios
  int8_t prevX = ejeX, prevY = ejeY, prevZ = ejeZ;

  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected()) {
      hayControlActivo = true;
      
      if (ctl->hasData()) {
        // Joystick Izquierdo (X, Y)
        int x = ctl->axisX();
        int y = ctl->axisY();
        // Joystick Derecho (Z) - Usamos el eje vertical derecho (axisRY)
        int z = ctl->axisRY();

        // Aplicamos Zona Muerta de 100 (Rango es -511 a 511)
        ejeX = (abs(x) > 100) ? (x > 0 ? 1 : -1) : 0;
        ejeY = (abs(y) > 100) ? (y > 0 ? 1 : -1) : 0;
        ejeZ = (abs(z) > 100) ? (z > 0 ? 1 : -1) : 0;

        // NIVEL 4: Si hubo un cambio en los joysticks, disparamos el pin de aviso (DRDY)
        if (ejeX != prevX || ejeY != prevY || ejeZ != prevZ) {
            digitalWrite(PIN_DRDY, HIGH);
        }
      }
    }
  }

  btConectado = hayControlActivo ? 1 : 0;

  // Si el mando se desconecta, forzamos STOP y avisamos al maestro
  if (!hayControlActivo && (prevX != 0 || prevY != 0 || prevZ != 0)) {
      ejeX = 0; ejeY = 0; ejeZ = 0;
      digitalWrite(PIN_DRDY, HIGH);
  }
}

void setup() {
  Serial.begin(115200);

  // Configurar Pin de Interrupción
  pinMode(PIN_DRDY, OUTPUT);
  digitalWrite(PIN_DRDY, LOW);

  // Configurar I2C
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin(DIRECCION_ESCLAVO);
  Wire.setClock(400000); // 400kHz para sincronizar con Portenta
  Wire.onRequest(requestEvent);
  
  // Iniciar Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  
  // Olvidar dispositivos previos para asegurar un emparejamiento limpio en pruebas
  BP32.forgetBluetoothKeys(); 
  
  Serial.println("ESP32 WROVER Listo. Esperando control Bluetooth...");
}

void loop() {
  // Actualiza el stack de Bluetooth
  BP32.update();
  
  // Procesa los datos de los mandos
  processControllers();
  
  // Delay mínimo para estabilidad del sistema operativo (Mbed/FreeRTOS)
  delay(1); 
}
#include <Wire.h>

#define I2C_SDA 27
#define I2C_SCL 14
#define DIRECCION_ESCLAVO 0x40 

// Variable volátil para comunicarnos entre la interrupción y el loop
volatile bool peticionRecibida = false;

void setup() {
  Serial.begin(115200);
  
  Wire.setPins(I2C_SDA, I2C_SCL); 
  Wire.begin(DIRECCION_ESCLAVO);
  Wire.onRequest(enviarRespuesta);
  
  Serial.println("ESP32 Listo (0x40)");
}

void loop() {
  // Si la interrupción nos avisa que hubo comunicación, imprimimos aquí.
  if (peticionRecibida) {
    Serial.println("Ping recibido -> Respondido OK");
    peticionRecibida = false; // Reiniciamos la bandera
  }
  delay(10); 
}

// --- Rutina de Interrupción I2C ---
// ESTA FUNCIÓN DEBE SER EXTREMADAMENTE RÁPIDA (Nada de Serial.print aquí)
void enviarRespuesta() {
  Wire.write((const uint8_t*)"OK", 2); 
  peticionRecibida = true; // Levantamos la bandera para el loop
}
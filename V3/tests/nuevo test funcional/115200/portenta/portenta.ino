
#include <Arduino_PortentaMachineControl.h>

#define BAUD_RS485 9600

unsigned long bytesRecibidos = 0;
unsigned long ultimoReporte = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Inicializando RS485...");

  MachineControl_RS485Comm.begin(
    BAUD_RS485,
    SERIAL_8N1,
    0,
    500
  );

  // RS485, no RS232
  MachineControl_RS485Comm.setModeRS232(false);

  // Half-duplex: dos hilos
  MachineControl_RS485Comm.setFullDuplex(false);

  // Habilitar receptor
  MachineControl_RS485Comm.receive();

  Serial.println("RS485 configurado a 115200");
}

void loop() {

  while (MachineControl_RS485Comm.available() > 0) {

    int dato = MachineControl_RS485Comm.read();

    if (dato >= 0) {
      bytesRecibidos++;

      Serial.print("Byte recibido: 0x");

      if (dato < 16) Serial.print("0");

      Serial.print(dato, HEX);

      Serial.print(" | ASCII: ");
      Serial.write((uint8_t)dato);
      Serial.println();
    }
  }

  if (millis() - ultimoReporte >= 2000) {
    ultimoReporte = millis();

    Serial.print("Total bytes recibidos: ");
    Serial.println(bytesRecibidos);
  }
}

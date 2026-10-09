
#include <Arduino.h>

#define RS485_TX 14
#define RS485_RX 27
#define RS485_DIR 18

#define BAUD_RS485 9600

unsigned long contador = 0;

void setup() {
  Serial.begin(115200);

  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, LOW);

  Serial2.begin(
    BAUD_RS485,
    SERIAL_8N1,
    RS485_RX,
    RS485_TX
  );

  delay(2000);
  Serial.println("ESP32 RS485 iniciado");
}

void loop() {
  contador++;

  digitalWrite(RS485_DIR, HIGH);
  delayMicroseconds(50);

  Serial2.print("PING:");
  Serial2.println(contador);

  Serial2.flush();
  delayMicroseconds(150);

  digitalWrite(RS485_DIR, LOW);

  Serial.print("Enviado PING:");
  Serial.println(contador);

  delay(500);
}

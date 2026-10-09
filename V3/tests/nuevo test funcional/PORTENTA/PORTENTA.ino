
#include <Arduino_MachineControl.h>
#include <stdio.h>
#include <string.h>

using namespace machinecontrol;

char buffer[8];
uint8_t indice = 0;

uint32_t recibidos = 0;
uint32_t corruptos = 0;
uint32_t respuestas = 0;

// CRC-8, polinomio 0x07
uint8_t crc8(const char *datos) {

  uint8_t crc = 0;

  for (uint8_t j = 0; j < 5; j++) {
    crc ^= (uint8_t)datos[j];

    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }

  return crc;
}

char aHex(uint8_t n) {
  return "0123456789ABCDEF"[n & 0x0F];
}

int desdeHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

void crearTrama(char tipo, uint16_t numero, char *trama) {

  snprintf(trama, 9, "%c%04u",
           tipo, (unsigned int)numero);

  uint8_t crc = crc8(trama);

  trama[5] = aHex(crc >> 4);
  trama[6] = aHex(crc);
  trama[7] = '\n';
  trama[8] = '\0';
}

bool validarTrama(const char *trama,
                  char tipo,
                  uint16_t &numero) {

  if (strlen(trama) != 7 || trama[0] != tipo)
    return false;

  numero = 0;

  for (uint8_t i = 1; i <= 4; i++) {

    if (trama[i] < '0' || trama[i] > '9')
      return false;

    numero = numero * 10 + (trama[i] - '0');
  }

  if (numero < 1 || numero > 1000)
    return false;

  int h = desdeHex(trama[5]);
  int l = desdeHex(trama[6]);

  if (h < 0 || l < 0)
    return false;

  uint8_t recibido = (h << 4) | l;

  return recibido == crc8(trama);
}

void responder(uint16_t numero) {

  char respuesta[9];

  crearTrama('A', numero, respuesta);

  // Esperar que el Nano cambie a recepción
  delay(15);

  comm_protocols.rs485.noReceive();
  comm_protocols.rs485.beginTransmission();

  comm_protocols.rs485.print(respuesta);

  comm_protocols.rs485.endTransmission();
  comm_protocols.rs485.receive();

  respuestas++;
}

void procesarTrama() {

  uint16_t numero = 0;

  if (validarTrama(buffer, 'Q', numero)) {

    if (numero == 1) {
      recibidos = 0;
      corruptos = 0;
      respuestas = 0;

      Serial.println("NUEVA PRUEBA");
    }

    recibidos++;

    responder(numero);

    if (recibidos % 50 == 0) {

      Serial.print("Recibidos validos: ");
      Serial.println(recibidos);

      Serial.print("Respuestas enviadas: ");
      Serial.println(respuestas);

      Serial.print("Corruptos: ");
      Serial.println(corruptos);

      Serial.println("----------------------");
    }

  } else {

    corruptos++;

    Serial.print("TRAMA CORRUPTA #");
    Serial.println(corruptos);
  }
}

void setup() {

  Serial.begin(115200);
  delay(2000);

  comm_protocols.init();
  comm_protocols.rs485Enable(true);

  comm_protocols.rs485.begin(9600, 0, 2000);
  comm_protocols.rs485.receive();

  Serial.println("================================");
  Serial.println("PORTENTA - RS485 SIN SEPARADORES");
  Serial.println("Esperando mensajes...");
  Serial.println("================================");
}

void loop() {

  while (comm_protocols.rs485.available()) {

    char c = comm_protocols.rs485.read();

    if (c == '\r')
      continue;

    if (c == '\n') {

      buffer[indice] = '\0';

      if (indice > 0)
        procesarTrama();

      indice = 0;

    } else if (indice < 7) {

      buffer[indice++] = c;

    } else {

      corruptos++;
      indice = 0;
    }
  }
}

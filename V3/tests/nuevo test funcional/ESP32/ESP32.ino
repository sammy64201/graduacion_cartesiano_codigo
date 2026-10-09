
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

// MAX485 mediante convertidor lógico

#define RS485_RX  14  // RO mediante canal 1
#define RS485_TX  27  // DI mediante canal 2
#define RS485_DE  18  // DE y RE mediante canal 3


#define TOTAL 1000
#define TIMEOUT_MS 400

HardwareSerial RS485(2);

uint16_t correctos = 0;
uint16_t sinACK = 0;
uint32_t corruptos = 0;
uint32_t secuenciaIncorrecta = 0;

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

// Formato: Q0001FF\n
void crearTrama(char tipo, uint16_t numero,
                char *trama) {

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

  if (numero < 1 || numero > TOTAL)
    return false;

  int h = desdeHex(trama[5]);
  int l = desdeHex(trama[6]);

  if (h < 0 || l < 0)
    return false;

  uint8_t recibido = (h << 4) | l;

  return recibido == crc8(trama);
}

bool esperarACK(uint16_t esperado) {

  char buffer[8];
  uint8_t indice = 0;

  unsigned long inicio = millis();

  while (millis() - inicio < TIMEOUT_MS) {

    while (RS485.available()) {

      char c = RS485.read();

      if (c == '\r')
        continue;

      if (c == '\n') {

        buffer[indice] = '\0';

        if (indice > 0) {
          uint16_t numero = 0;

          if (validarTrama(buffer, 'A', numero)) {

            if (numero == esperado)
              return true;

            secuenciaIncorrecta++;

          } else {
            corruptos++;
          }
        }

        indice = 0;

      } else if (indice < 7) {
        buffer[indice++] = c;
      } else {
        corruptos++;
        indice = 0;
      }
    }
  }

  return false;
}

void resumen(uint16_t enviados) {

  Serial.println("---------------------------");
  Serial.print("Enviados: ");
  Serial.println(enviados);

  Serial.print("Correctos: ");
  Serial.println(correctos);

  Serial.print("Sin ACK valido: ");
  Serial.println(sinACK);

  Serial.print("Tramas corruptas: ");
  Serial.println(corruptos);

  Serial.print("Secuencia incorrecta: ");
  Serial.println(secuenciaIncorrecta);

  Serial.print("Exito: ");
  Serial.print(100.0f * correctos / enviados, 2);
  Serial.println("%");
}

void setup() {

  Serial.begin(115200);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  // UART2 del ESP32
  RS485.begin(9600, SERIAL_8N1,
              RS485_RX, RS485_TX);

  delay(3000);

  Serial.println("==========================");
  Serial.println("ESP32 - PRUEBA RS485");
  Serial.println("1000 paquetes con CRC8");
  Serial.println("==========================");
}

void loop() {

  for (uint16_t i = 1; i <= TOTAL; i++) {

    char trama[9];
    crearTrama('Q', i, trama);

    // Descartar datos anteriores
    while (RS485.available())
      RS485.read();

    // Activar transmisión
    digitalWrite(RS485_DE, HIGH);
    delayMicroseconds(200);

    RS485.print(trama);

    // Esperar fin de transmisión UART
    RS485.flush();
    delayMicroseconds(200);

    // Regresar a recepción
    digitalWrite(RS485_DE, LOW);

    if (esperarACK(i)) {
      correctos++;
    } else {
      sinACK++;
    }

    if (i % 50 == 0) {
      Serial.print("Progreso: ");
      Serial.print(i);
      Serial.print("/");
      Serial.println(TOTAL);

      resumen(i);
    }

    delay(100);
  }

  Serial.println();
  Serial.println("=== PRUEBA FINALIZADA ===");
  resumen(TOTAL);

  while (true) {
    delay(1000);
  }
}

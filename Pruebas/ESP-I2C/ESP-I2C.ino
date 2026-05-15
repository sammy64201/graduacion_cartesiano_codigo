#include <Wire.h>

// ====================== CONFIGURACION I2C ESP32 ======================
#define I2C_SDA 27
#define I2C_SCL 14
#define I2C_SLAVE_ADDR 0x40

// ====================== CONFIGURACION SERIAL ======================
const unsigned long SERIAL_BAUD = 115200;

// ====================== VARIABLES DEBUG ======================
volatile uint32_t contadorRequests = 0;
volatile uint32_t contadorReceives = 0;
volatile uint8_t ultimoComando = 0x00;

uint8_t secuencia = 0;

unsigned long tAnteriorDebug = 0;
const unsigned long INTERVALO_DEBUG_MS = 1000;

// ================================================================
// Callback: cuando el maestro escribe al ESP32
// ================================================================
void onReceiveEvent(int numBytes) {
  contadorReceives++;

  while (Wire.available()) {
    ultimoComando = Wire.read();
  }
}

// ================================================================
// Callback: cuando el maestro solicita datos al ESP32
// IMPORTANTE: no usar Serial.print aqui.
// ================================================================
void onRequestEvent() {
  contadorRequests++;

  uint8_t paquete[8];

  paquete[0] = 0xAB;                     // Byte de identificacion
  paquete[1] = secuencia++;              // Secuencia
  paquete[2] = ultimoComando;            // Ultimo comando recibido
  paquete[3] = (uint8_t)(millis() / 1000); // Uptime en segundos, byte bajo
  paquete[4] = (uint8_t)(contadorRequests & 0xFF);
  paquete[5] = (uint8_t)((contadorRequests >> 8) & 0xFF);
  paquete[6] = (uint8_t)(contadorReceives & 0xFF);

  // Checksum simple XOR de los primeros 7 bytes
  paquete[7] = paquete[0] ^ paquete[1] ^ paquete[2] ^ paquete[3] ^
               paquete[4] ^ paquete[5] ^ paquete[6];

  Wire.write(paquete, 8);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" ESP32 UWB DW3000 - I2C SLAVE DEBUG");
  Serial.println(" Direccion: 0x40");
  Serial.println(" SDA: GPIO27");
  Serial.println(" SCL: GPIO14");
  Serial.println("==========================================");

  // Forma recomendada para ESP32 como esclavo I2C
  bool ok = Wire.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SDA, I2C_SCL, 100000);

  if (ok) {
    Serial.println("[ESP32] Wire.begin OK.");
  } else {
    Serial.println("[ESP32] ERROR en Wire.begin.");
  }

  Wire.onReceive(onReceiveEvent);
  Wire.onRequest(onRequestEvent);

  Serial.println("[ESP32] Esperando solicitudes del maestro...");
}

void loop() {
  unsigned long ahora = millis();

  if (ahora - tAnteriorDebug >= INTERVALO_DEBUG_MS) {
    tAnteriorDebug = ahora;

    Serial.println();
    Serial.println("========== ESTADO ESP32 I2C ==========");
    Serial.print("[ESP32] Direccion esclavo: 0x");
    Serial.println(I2C_SLAVE_ADDR, HEX);

    Serial.print("[ESP32] Requests del maestro: ");
    Serial.println(contadorRequests);

    Serial.print("[ESP32] Writes/Receives del maestro: ");
    Serial.println(contadorReceives);

    Serial.print("[ESP32] Ultimo comando recibido: 0x");
    if (ultimoComando < 16) Serial.print("0");
    Serial.println(ultimoComando, HEX);

    Serial.print("[ESP32] Uptime ms: ");
    Serial.println(millis());

    Serial.println("======================================");
  }

  delay(10);
}
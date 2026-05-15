#include <Arduino_PortentaMachineControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ====================== CONFIGURACION OLED ======================
#define OLED_ADDR 0x3C
Adafruit_SH1106G pantalla(128, 64, &Wire, -1);

// ====================== DIRECCIONES I2C A REVISAR ======================
#define ADDR_OLED  0x3C
#define ADDR_ESP32 0x40

// ====================== CONFIGURACION GENERAL ======================
const unsigned long SERIAL_BAUD = 115200;
const unsigned long INTERVALO_TEST_MS = 1000;

unsigned long tAnterior = 0;

struct EstadoI2C {
  uint8_t addr;
  const char* nombre;
  bool conectado;
  uint8_t error;
  int bytesLeidos;
  uint8_t datos[8];
};

EstadoI2C dispositivos[] = {
  {ADDR_OLED,  "OLED",  false, 255, 0, {0}},
  {ADDR_ESP32, "ESP32", false, 255, 0, {0}}
};

const uint8_t NUM_DISPOSITIVOS = sizeof(dispositivos) / sizeof(dispositivos[0]);

// ================================================================
// Funcion: prueba si una direccion responde con ACK
// ================================================================
bool revisarACK(uint8_t addr, uint8_t &errorCode) {
  Wire.beginTransmission(addr);
  errorCode = Wire.endTransmission();

  // error 0 = ACK correcto
  return (errorCode == 0);
}

// ================================================================
// Funcion: enviar ping opcional al ESP32
// ================================================================
bool enviarPingESP32() {
  Wire.beginTransmission(ADDR_ESP32);
  Wire.write(0xA5);   // Comando simple de prueba
  uint8_t err = Wire.endTransmission();

  Serial.print("[MASTER] Ping a ESP32 0x40 -> error=");
  Serial.println(err);

  return (err == 0);
}

// ================================================================
// Funcion: leer paquete de 8 bytes del ESP32
// ================================================================
int leerESP32(uint8_t *buffer, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) buffer[i] = 0;

  int bytes = Wire.requestFrom((uint8_t)ADDR_ESP32, (uint8_t)n);

  Serial.print("[MASTER] requestFrom ESP32 -> bytes recibidos=");
  Serial.println(bytes);

  for (int i = 0; i < bytes && i < n; i++) {
    buffer[i] = Wire.read();
  }

  // Limpiar cualquier byte extra por seguridad
  while (Wire.available()) {
    Wire.read();
  }

  return bytes;
}

// ================================================================
// Funcion: escaneo completo del bus I2C
// ================================================================
void escaneoCompletoBus() {
  Serial.println();
  Serial.println("========== ESCANEO COMPLETO I2C ==========");

  uint8_t encontrados = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("[I2C] Dispositivo encontrado en 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      encontrados++;
    }
  }

  if (encontrados == 0) {
    Serial.println("[I2C] No se encontro ningun dispositivo.");
  } else {
    Serial.print("[I2C] Total encontrados: ");
    Serial.println(encontrados);
  }

  Serial.println("==========================================");
}

// ================================================================
// Funcion: actualizar OLED
// ================================================================
void actualizarPantalla() {
  pantalla.clearDisplay();
  pantalla.setTextSize(1);
  pantalla.setTextColor(SH110X_WHITE);
  pantalla.setCursor(0, 0);

  pantalla.println("DEBUG I2C MASTER");
  pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

  int y = 16;

  for (uint8_t i = 0; i < NUM_DISPOSITIVOS; i++) {
    pantalla.setCursor(0, y);

    pantalla.print(dispositivos[i].nombre);
    pantalla.print(" 0x");
    if (dispositivos[i].addr < 16) pantalla.print("0");
    pantalla.print(dispositivos[i].addr, HEX);
    pantalla.print(": ");

    if (dispositivos[i].conectado) {
      pantalla.println("OK");
    } else {
      pantalla.print("FAIL ");
      pantalla.println(dispositivos[i].error);
    }

    y += 12;
  }

  pantalla.setCursor(0, 48);
  pantalla.print("ESP bytes: ");
  pantalla.print(dispositivos[1].bytesLeidos);

  pantalla.setCursor(0, 56);
  pantalla.print("Serial: 115200");

  pantalla.display();
}

// ================================================================
// Funcion: imprimir estado por Serial
// ================================================================
void imprimirEstadoSerial() {
  Serial.println();
  Serial.println("========== ESTADO DISPOSITIVOS ==========");

  for (uint8_t i = 0; i < NUM_DISPOSITIVOS; i++) {
    Serial.print("[MASTER] ");
    Serial.print(dispositivos[i].nombre);
    Serial.print(" direccion 0x");
    if (dispositivos[i].addr < 16) Serial.print("0");
    Serial.print(dispositivos[i].addr, HEX);
    Serial.print(" -> ");

    if (dispositivos[i].conectado) {
      Serial.print("CONECTADO");
    } else {
      Serial.print("NO CONECTADO");
    }

    Serial.print(" | error=");
    Serial.print(dispositivos[i].error);

    if (dispositivos[i].addr == ADDR_ESP32) {
      Serial.print(" | bytes=");
      Serial.print(dispositivos[i].bytesLeidos);

      Serial.print(" | datos=");
      for (int j = 0; j < dispositivos[i].bytesLeidos; j++) {
        Serial.print("0x");
        if (dispositivos[i].datos[j] < 16) Serial.print("0");
        Serial.print(dispositivos[i].datos[j], HEX);
        Serial.print(" ");
      }
    }

    Serial.println();
  }

  Serial.println("=========================================");
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" PORTENTA MACHINE CONTROL - I2C MASTER");
  Serial.println(" OLED 0x3C + ESP32 SLAVE 0x40");
  Serial.println("==========================================");

  Serial.println("[MASTER] Iniciando Portenta Machine Control...");

  // Inicializacion general de la Machine Control
  //MachineControl.begin();

  Serial.println("[MASTER] Machine Control inicializada.");

  // Inicializacion I2C
  Wire.begin();
  Wire.setClock(100000);   // 100 kHz para debug estable

  delay(200);

  Serial.println("[MASTER] Inicializando OLED...");

  if (pantalla.begin(OLED_ADDR, true)) {
    Serial.println("[MASTER] OLED inicializada correctamente en 0x3C.");
    pantalla.clearDisplay();
    pantalla.display();
  } else {
    Serial.println("[MASTER] ERROR: No se pudo inicializar la OLED en 0x3C.");
  }

  pantalla.setTextSize(1);
  pantalla.setTextColor(SH110X_WHITE);
  pantalla.clearDisplay();
  pantalla.setCursor(0, 0);
  pantalla.println("Iniciando I2C...");
  pantalla.display();

  delay(500);

  escaneoCompletoBus();
}

void loop() {
  unsigned long ahora = millis();

  if (ahora - tAnterior >= INTERVALO_TEST_MS) {
    tAnterior = ahora;

    // Revisar ACK de cada dispositivo conocido
    for (uint8_t i = 0; i < NUM_DISPOSITIVOS; i++) {
      dispositivos[i].conectado = revisarACK(dispositivos[i].addr, dispositivos[i].error);
      dispositivos[i].bytesLeidos = 0;
    }

    // Si el ESP32 responde, hacer prueba adicional de escritura + lectura
    if (dispositivos[1].conectado) {
      enviarPingESP32();
      delay(5);
      dispositivos[1].bytesLeidos = leerESP32(dispositivos[1].datos, 8);
    }

    imprimirEstadoSerial();
    actualizarPantalla();
  }
}
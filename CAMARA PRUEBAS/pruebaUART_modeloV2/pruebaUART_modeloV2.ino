/***************************************************
 HUSKYLENS 2 - Modelo personalizado por UART
 ESP32 RX: GPIO32
 ESP32 TX: GPIO33
 ****************************************************/

#include <Arduino.h>
#include <DFRobot_HuskylensV2.h>

// --------------------------------------------------
// UART de la HUSKYLENS
// --------------------------------------------------
constexpr int HUSKY_RX_PIN = 32;
constexpr int HUSKY_TX_PIN = 33;
constexpr uint32_t HUSKY_BAUDRATE = 115200;

// Índice del modelo personalizado:
// 0 = primer modelo instalado  -> ID 128
// 1 = segundo modelo instalado -> ID 129
// 2 = tercer modelo instalado  -> ID 130
constexpr uint8_t CUSTOM_MODEL_INDEX = 0;

// ID real del modelo personalizado
const eAlgorithm_t MY_CUSTOM_MODEL =
  static_cast<eAlgorithm_t>(
    static_cast<uint8_t>(ALGORITHM_CUSTOM_BEGIN) +
    CUSTOM_MODEL_INDEX
  );

HardwareSerial HuskyUART(2);
HuskylensV2 huskylens;

uint32_t lastRead = 0;
constexpr uint32_t READ_PERIOD_MS = 200;

// --------------------------------------------------
// Inicialización
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("HUSKYLENS 2 - Modelo propio");
  Serial.println("==============================");

  HuskyUART.begin(
    HUSKY_BAUDRATE,
    SERIAL_8N1,
    HUSKY_RX_PIN,
    HUSKY_TX_PIN
  );

  delay(500);

  // Verificar comunicación
  while (!huskylens.begin(HuskyUART)) {
    Serial.println("No se pudo comunicar con HUSKYLENS 2.");
    Serial.println("Revise UART 115200, TX, RX y GND.");
    delay(1000);
  }

  Serial.println("HUSKYLENS 2 conectada.");

  Serial.print("Intentando abrir modelo personalizado ID ");
  Serial.println(
    static_cast<uint8_t>(MY_CUSTOM_MODEL)
  );

  // Abrir el modelo personalizado
  if (!huskylens.switchAlgorithm(MY_CUSTOM_MODEL)) {
    Serial.println("ERROR: no se pudo abrir el modelo.");
    Serial.println("Pruebe cambiando CUSTOM_MODEL_INDEX:");
    Serial.println("0 = modelo 128");
    Serial.println("1 = modelo 129");
    Serial.println("2 = modelo 130");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("Modelo personalizado abierto correctamente.");
  Serial.println("Esperando que termine de cargar...");

  // Los modelos personalizados pueden tardar
  // varios segundos en inicializarse.
  delay(8000);

  Serial.println("Comenzando detección.");
  Serial.println();
}

// --------------------------------------------------
// Programa principal
// --------------------------------------------------
void loop() {
  if (millis() - lastRead < READ_PERIOD_MS) {
    return;
  }

  lastRead = millis();

  /*
   * Consultar específicamente el modelo personalizado.
   *
   * Resultado:
   * -1 = error de comunicación
   *  0 = comunicación correcta, sin detecciones
   * >0 = número de objetos detectados
   */
  int8_t resultCount =
    huskylens.getResult(MY_CUSTOM_MODEL);

  if (resultCount < 0) {
    Serial.println("Error de comunicación con la cámara.");
    return;
  }

  if (resultCount == 0) {
    static uint32_t lastEmptyMessage = 0;

    if (millis() - lastEmptyMessage >= 1000) {
      Serial.println(
        "Modelo activo, pero sin objetos detectados."
      );

      lastEmptyMessage = millis();
    }

    return;
  }

  Serial.println();
  Serial.print("Objetos detectados: ");
  Serial.println(resultCount);

  while (huskylens.available(MY_CUSTOM_MODEL)) {
    Result *result =
      static_cast<Result *>(
        huskylens.popCachedResult(MY_CUSTOM_MODEL)
      );

    if (result == nullptr) {
      continue;
    }

    Serial.println("--------------------------------");

    Serial.print("ID: ");
    Serial.println(result->ID);

    Serial.print("Nombre: ");
    Serial.println(result->name);

    Serial.print("Centro X: ");
    Serial.println(result->xCenter);

    Serial.print("Centro Y: ");
    Serial.println(result->yCenter);

    Serial.print("Ancho: ");
    Serial.println(result->width);

    Serial.print("Alto: ");
    Serial.println(result->height);

    Serial.print("Contenido: ");
    Serial.println(result->content);

    Serial.println("--------------------------------");
  }
}
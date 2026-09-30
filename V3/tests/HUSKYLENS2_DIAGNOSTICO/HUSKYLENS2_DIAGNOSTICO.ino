/*
 * Diagnostico minimo de HUSKYLENS 2 para ESP32.
 *
 * Conexion usada por este proyecto:
 *   HUSKYLENS TX  -> ESP32 GPIO32 (RX)
 *   HUSKYLENS RX  <- ESP32 GPIO33 (TX)
 *   HUSKYLENS GND -- ESP32 GND
 *
 * Configure la HUSKYLENS 2 en protocolo UART a 115200 baudios.
 * Abra el monitor serie del ESP32 a 115200 baudios.
 */

#include <Arduino.h>
#include <DFRobot_HuskylensV2.h>

constexpr int HUSKY_RX_PIN = 32;
constexpr int HUSKY_TX_PIN = 33;
constexpr uint32_t HUSKY_BAUDRATE = 115200;
constexpr uint32_t MONITOR_BAUDRATE = 115200;
constexpr uint32_t MODEL_LOAD_TIME_MS = 8000;
constexpr uint32_t BUILTIN_LOAD_TIME_MS = 3000;

// UART1 coincide con el sketch minimo validado fisicamente.
HardwareSerial HuskyUART(1);
HuskylensV2 huskylens;

bool cameraConnected = false;
eAlgorithm_t currentAlgorithm = ALGORITHM_ANY;
const char *currentAlgorithmName = "ninguno";

void discardCameraBytes() {
  while (HuskyUART.available() > 0) {
    HuskyUART.read();
  }
}

void printHelp() {
  Serial.println();
  Serial.println(F("========== COMANDOS =========="));
  Serial.println(F("c : probar/reintentar conexion"));
  Serial.println(F("t : abrir Tag Recognition"));
  Serial.println(F("f : abrir Face Recognition"));
  Serial.println(F("0 : abrir modelo personalizado 128 (usado en el proyecto)"));
  Serial.println(F("1 : abrir modelo personalizado 129"));
  Serial.println(F("2 : abrir modelo personalizado 130"));
  Serial.println(F("a : probar en orden los modelos 128, 129 y 130"));
  Serial.println(F("p : pedir una lectura al algoritmo abierto"));
  Serial.println(F("l : loopback de GPIO32/GPIO33 (camara desconectada)"));
  Serial.println(F("h : mostrar esta ayuda"));
  Serial.println(F("=============================="));
  Serial.println();
}

bool connectCamera() {
  Serial.println();
  Serial.println(F("[CONEXION] Limpiando UART y enviando handshake..."));
  discardCameraBytes();

  // Un solo intento evita que una camara desconectada deje el sketch atrapado
  // indefinidamente. La libreria puede tardar hasta unos segundos en responder.
  huskylens.retry = 1;
  cameraConnected = huskylens.begin(HuskyUART);

  if (cameraConnected) {
    Serial.println(F("[OK] HUSKYLENS 2 respondio correctamente."));
    Serial.println(F("La alimentacion, UART del ESP32 y enlace TX/RX funcionan."));
  } else {
    currentAlgorithm = ALGORITHM_ANY;
    currentAlgorithmName = "ninguno";
    Serial.println(F("[FALLO] La HUSKYLENS 2 no respondio al handshake."));
    Serial.println(F("Revise: camara en UART 115200, TX/RX cruzados, GND comun y alimentacion."));
    Serial.println(F("Use 'c' para reintentar o 'l' para probar los pines del ESP32."));
  }

  return cameraConnected;
}

bool openAlgorithm(eAlgorithm_t algorithm, const char *name,
                   uint32_t loadTimeMs) {
  if (!cameraConnected) {
    Serial.println(F("[AVISO] Primero se intentara reconectar la camara."));
    if (!connectCamera()) {
      return false;
    }
  }

  Serial.println();
  Serial.print(F("[MODELO] Abriendo "));
  Serial.print(name);
  Serial.print(F(" (ID "));
  Serial.print(static_cast<uint8_t>(algorithm));
  Serial.println(F(")..."));

  if (!huskylens.switchAlgorithm(algorithm)) {
    cameraConnected = false;
    currentAlgorithm = ALGORITHM_ANY;
    currentAlgorithmName = "ninguno";
    Serial.println(F("[FALLO] La camara no confirmo el cambio de algoritmo."));
    Serial.println(F("Puede ser enlace UART perdido o modelo no instalado."));
    return false;
  }

  Serial.print(F("[MODELO] Confirmado. Esperando carga "));
  Serial.print(loadTimeMs / 1000U);
  Serial.println(F(" s..."));
  delay(loadTimeMs);

  // Una consulta con cero detecciones tambien es valida. Un valor negativo
  // significa que no hubo respuesta valida del algoritmo.
  const int8_t count = huskylens.getResult(algorithm);
  if (count < 0) {
    cameraConnected = false;
    currentAlgorithm = ALGORITHM_ANY;
    currentAlgorithmName = "ninguno";
    Serial.println(F("[FALLO] El modelo se selecciono, pero no respondio despues de cargar."));
    return false;
  }

  currentAlgorithm = algorithm;
  currentAlgorithmName = name;
  Serial.print(F("[OK] Modelo activo y comunicando. Detecciones actuales: "));
  Serial.println(count);
  return true;
}

void printResult(const Result &result, int index) {
  Serial.print(F("  #"));
  Serial.print(index);
  Serial.print(F(" ID="));
  Serial.print(result.ID);
  Serial.print(F(" centro=("));
  Serial.print(result.xCenter);
  Serial.print(',');
  Serial.print(result.yCenter);
  Serial.print(F(") tamano=("));
  Serial.print(result.width);
  Serial.print('x');
  Serial.print(result.height);
  Serial.print(F(") nombre="));
  Serial.println(result.name);
}

void requestOneReading() {
  if (!cameraConnected || currentAlgorithm == ALGORITHM_ANY) {
    Serial.println(F("[AVISO] No hay un modelo abierto. Use t, f, 0, 1 o 2."));
    return;
  }

  Serial.println();
  Serial.print(F("[LECTURA] Consultando "));
  Serial.println(currentAlgorithmName);

  const int8_t count = huskylens.getResult(currentAlgorithm);
  if (count < 0) {
    cameraConnected = false;
    Serial.println(F("[FALLO] Sin respuesta valida. Se marco la camara desconectada."));
    Serial.println(F("Use 'c' para intentar un nuevo handshake."));
    return;
  }

  Serial.print(F("[OK] Respuesta recibida. Resultados: "));
  Serial.println(count);

  int index = 0;
  while (huskylens.available(currentAlgorithm)) {
    Result *result = huskylens.popCachedResult(currentAlgorithm);
    if (result != nullptr) {
      printResult(*result, index++);
    }
  }

  if (count == 0) {
    Serial.println(F("No detectar objetos NO es fallo: la comunicacion si respondio."));
  }
}

void scanCustomModels() {
  Serial.println();
  Serial.println(F("[PRUEBA] Modelos personalizados instalables: 128, 129 y 130."));

  static const char *const names[] = {
    "Personalizado 128",
    "Personalizado 129",
    "Personalizado 130"
  };

  for (uint8_t index = 0; index < 3; ++index) {
    const uint8_t id = static_cast<uint8_t>(ALGORITHM_CUSTOM_BEGIN) + index;

    const bool ok = openAlgorithm(static_cast<eAlgorithm_t>(id), names[index],
                                  MODEL_LOAD_TIME_MS);
    Serial.print(F("[RESUMEN] Modelo "));
    Serial.print(id);
    Serial.println(ok ? F(": ABRIO Y RESPONDIO") : F(": NO RESPONDIO"));

    if (!ok) {
      // Si la camara sigue viva, un nuevo handshake permitira continuar con
      // el siguiente ID. Si no, tambien deja un diagnostico claro.
      connectCamera();
    }
  }

  Serial.println(F("[PRUEBA] Exploracion terminada."));
}

void runLoopbackTest() {
  Serial.println();
  Serial.println(F("[LOOPBACK] Desconecte completamente la HUSKYLENS."));
  Serial.println(F("[LOOPBACK] Una temporalmente GPIO33 (TX) con GPIO32 (RX)."));
  Serial.println(F("[LOOPBACK] La prueba comenzara en 5 segundos..."));
  delay(5000);

  discardCameraBytes();

  const uint8_t pattern[] = {0x55, 0xAA, 0x00, 0xFF, 0x3C, 0xC3};
  HuskyUART.write(pattern, sizeof(pattern));
  HuskyUART.flush();
  delay(100);

  bool ok = true;
  size_t received = 0;
  while (HuskyUART.available() > 0 && received < sizeof(pattern)) {
    const int value = HuskyUART.read();
    if (value < 0 || static_cast<uint8_t>(value) != pattern[received]) {
      ok = false;
    }
    ++received;
  }

  if (received != sizeof(pattern)) {
    ok = false;
  }

  if (ok) {
    Serial.println(F("[OK] Loopback correcto: UART1, GPIO32 y GPIO33 responden."));
    Serial.println(F("Quite el puente y vuelva a conectar la camara."));
  } else {
    Serial.print(F("[FALLO] Loopback incorrecto. Bytes recibidos: "));
    Serial.print(received);
    Serial.print('/');
    Serial.println(sizeof(pattern));
    Serial.println(F("Revise el puente; si esta bien, puede haber dano en pines o placa."));
  }

  cameraConnected = false;
  currentAlgorithm = ALGORITHM_ANY;
  currentAlgorithmName = "ninguno";
}

void handleCommand(char command) {
  switch (command) {
    case 'c':
    case 'C':
      connectCamera();
      break;

    case 't':
    case 'T':
      openAlgorithm(ALGORITHM_TAG_RECOGNITION, "Tag Recognition",
                    BUILTIN_LOAD_TIME_MS);
      break;

    case 'f':
    case 'F':
      openAlgorithm(ALGORITHM_FACE_RECOGNITION, "Face Recognition",
                    BUILTIN_LOAD_TIME_MS);
      break;

    case '0':
    case '1':
    case '2': {
      const uint8_t id = static_cast<uint8_t>(ALGORITHM_CUSTOM_BEGIN) +
                         static_cast<uint8_t>(command - '0');
      const char *name = command == '0' ? "Personalizado 128"
                         : command == '1' ? "Personalizado 129"
                                          : "Personalizado 130";
      openAlgorithm(static_cast<eAlgorithm_t>(id), name, MODEL_LOAD_TIME_MS);
      break;
    }

    case 'a':
    case 'A':
      scanCustomModels();
      break;

    case 'p':
    case 'P':
      requestOneReading();
      break;

    case 'l':
    case 'L':
      runLoopbackTest();
      break;

    case 'h':
    case 'H':
      printHelp();
      break;

    case '\r':
    case '\n':
    case ' ':
    case '\t':
      break;

    default:
      Serial.print(F("[AVISO] Comando desconocido: "));
      Serial.println(command);
      printHelp();
      break;
  }
}

void setup() {
  Serial.begin(MONITOR_BAUDRATE);
  delay(1500);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" DIAGNOSTICO HUSKYLENS 2 + ESP32 UART1"));
  Serial.println(F(" RX=GPIO32  TX=GPIO33  UART=115200"));
  Serial.println(F("========================================"));

  HuskyUART.begin(HUSKY_BAUDRATE, SERIAL_8N1, HUSKY_RX_PIN, HUSKY_TX_PIN);
  delay(500);

  connectCamera();
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    handleCommand(static_cast<char>(Serial.read()));
  }

  delay(10);
}

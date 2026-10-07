/* Prueba ESP32 + HUSKYLENS 2: tags -> homografia -> modelo personalizado.
 * UART1 RX32 <- TX camara; TX33 -> RX camara; GND comun; 115200 baudios.
 * Monitor serie: 115200. No controla motores, servos, OLED ni Portenta.
 */
#include <Arduino.h>
#include <DFRobot_HuskylensV2.h>
#include <math.h>
#include <stdlib.h>
#include "VisionModelo129.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

struct Point2D;
struct CalibrationTag;

// 0 = algoritmo 128; 1 = 129; 2 = 130. Cambiable tambien por terminal.
constexpr uint8_t SEGMENTATION_MODEL_INDEX = VisionModelo129::INDICE_MODELO;
static_assert(SEGMENTATION_MODEL_INDEX < 3, "Indice personalizado: 0, 1 o 2");
constexpr double BELT_WIDTH_MM = 292.0;
constexpr double TOTAL_WIDTH_MM = 412.0;
constexpr double ALUMINUM_WIDTH_MM = (TOTAL_WIDTH_MM - BELT_WIDTH_MM) / 2.0;
constexpr double TAG_X_FROM_CENTER_MM = BELT_WIDTH_MM / 2.0 + ALUMINUM_WIDTH_MM / 2.0;
constexpr double TAG_ROWS_DISTANCE_MM = 382.0;
constexpr uint8_t NUMBER_OF_TAGS = 4;
constexpr uint16_t SAMPLES_PER_TAG = 25;
constexpr uint32_t READ_PERIOD_MS = 200;
constexpr uint32_t TAG_LOAD_MS = 3000;
constexpr uint32_t MODEL_LOAD_MS = 8000;
constexpr uint32_t MODEL_RESPONSE_TIMEOUT_MS = 30000;
constexpr uint32_t CALIBRATION_TIMEOUT_MS = 120000;
constexpr uint32_t RECONNECT_MS = 2000;

HardwareSerial HuskyUART(1);
HuskylensV2 huskylens;
QueueHandle_t commandQueue = nullptr;

struct Point2D { double x; double y; };
struct CalibrationTag { int code; double sumU; double sumV; uint16_t samples; };
// 0 superior izquierdo, 1 superior derecho, 2 inferior derecho, 3 inferior izquierdo.
CalibrationTag tags[NUMBER_OF_TAGS] = {{0,0,0,0}, {1,0,0,0}, {2,0,0,0}, {3,0,0,0}};
double H[3][3] = {{0,0,0}, {0,0,0}, {0,0,1}};
bool homographyValid = false;
bool connected = false;
bool paused = false;
uint8_t modelIndex = SEGMENTATION_MODEL_INDEX;
uint8_t consecutiveErrors = 0;
uint32_t lastRead = 0;
uint32_t lastStatus = 0;
uint32_t frameNumber = 0;

enum class CameraState { CONNECTING, WAIT_TAGS, CALIBRATING, WAIT_MODEL, STREAMING, ERROR };
CameraState state = CameraState::CONNECTING;
uint32_t stateSince = 0;

eAlgorithm_t selectedModel() {
  return static_cast<eAlgorithm_t>(static_cast<uint8_t>(ALGORITHM_CUSTOM_BEGIN) + modelIndex);
}

void setState(CameraState next) {
  state = next;
  stateSince = millis();
  lastRead = stateSince;
  lastStatus = stateSince;
  consecutiveErrors = 0;
}

// Funciones copiadas del algoritmo funcional del proyecto, sin cambiar su matematica.
Point2D getPhysicalTagPosition(uint8_t index) {
  const double halfHeight =
    TAG_ROWS_DISTANCE_MM / 2.0;

  switch (index) {
    case 0:
      return {
        -TAG_X_FROM_CENTER_MM,
        -halfHeight
      };

    case 1:
      return {
        TAG_X_FROM_CENTER_MM,
        -halfHeight
      };

    case 2:
      return {
        TAG_X_FROM_CENTER_MM,
        halfHeight
      };

    case 3:
      return {
        -TAG_X_FROM_CENTER_MM,
        halfHeight
      };

    default:
      return {0.0, 0.0};
  }
}

// ==================================================
// Lectura del código del tag
// ==================================================

bool parseLastInteger(
  const String &text,
  int &value
) {
  const char *cursor = text.c_str();
  bool found = false;

  while (*cursor != '\0') {
    const bool positiveNumber =
      *cursor >= '0' &&
      *cursor <= '9';

    const bool negativeNumber =
      *cursor == '-' &&
      *(cursor + 1) >= '0' &&
      *(cursor + 1) <= '9';

    if (positiveNumber || negativeNumber) {
      char *endPointer = nullptr;

      const long parsed =
        strtol(cursor, &endPointer, 10);

      if (endPointer != cursor) {
        value = static_cast<int>(parsed);
        found = true;
        cursor = endPointer;
        continue;
      }
    }

    cursor++;
  }

  return found;
}

bool extractTagCode(
  const Result *result,
  int &tagCode
) {
  /*
   Primero se intenta leer el contenido real
   del AprilTag.
  */

  if (parseLastInteger(result->content, tagCode)) {
    return true;
  }

  /*
   Si el contenido no está disponible, se usa el ID
   como respaldo.
  */

  tagCode = result->ID;
  return true;
}

int findTagIndex(int tagCode) {
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; i++) {
    if (tags[i].code == tagCode) {
      return i;
    }
  }

  return -1;
}

bool solveLinearSystem8(
  double matrix[8][8],
  double vector[8],
  double solution[8]
) {
  double augmented[8][9];

  for (uint8_t row = 0; row < 8; row++) {
    for (uint8_t column = 0; column < 8; column++) {
      augmented[row][column] =
        matrix[row][column];
    }

    augmented[row][8] = vector[row];
  }

  for (uint8_t column = 0; column < 8; column++) {
    uint8_t pivotRow = column;

    double largestValue =
      fabs(augmented[column][column]);

    for (
      uint8_t row = column + 1;
      row < 8;
      row++
    ) {
      const double candidate =
        fabs(augmented[row][column]);

      if (candidate > largestValue) {
        largestValue = candidate;
        pivotRow = row;
      }
    }

    if (largestValue < 1e-12) {
      return false;
    }

    if (pivotRow != column) {
      for (
        uint8_t currentColumn = column;
        currentColumn < 9;
        currentColumn++
      ) {
        const double temporary =
          augmented[column][currentColumn];

        augmented[column][currentColumn] =
          augmented[pivotRow][currentColumn];

        augmented[pivotRow][currentColumn] =
          temporary;
      }
    }

    const double pivot =
      augmented[column][column];

    for (
      uint8_t currentColumn = column;
      currentColumn < 9;
      currentColumn++
    ) {
      augmented[column][currentColumn] /= pivot;
    }

    for (uint8_t row = 0; row < 8; row++) {
      if (row == column) {
        continue;
      }

      const double factor =
        augmented[row][column];

      for (
        uint8_t currentColumn = column;
        currentColumn < 9;
        currentColumn++
      ) {
        augmented[row][currentColumn] -=
          factor *
          augmented[column][currentColumn];
      }
    }
  }

  for (uint8_t i = 0; i < 8; i++) {
    solution[i] = augmented[i][8];
  }

  return true;
}

// ==================================================
// Cálculo de homografía
// ==================================================

bool calculateHomography() {
  if (TAG_ROWS_DISTANCE_MM <= 0.0) {
    Serial.println();
    Serial.println(
      "ERROR: TAG_ROWS_DISTANCE_MM no está configurado."
    );

    Serial.println(
      "Mida la distancia vertical centro a centro."
    );

    return false;
  }

  double A[8][8] = {};
  double b[8] = {};

  for (uint8_t i = 0; i < NUMBER_OF_TAGS; i++) {
    const double u =
      tags[i].sumU / tags[i].samples;

    const double v =
      tags[i].sumV / tags[i].samples;

    const Point2D physical =
      getPhysicalTagPosition(i);

    const double X = physical.x;
    const double Y = physical.y;

    const uint8_t rowX = 2 * i;
    const uint8_t rowY = rowX + 1;

    // Ecuación para X

    A[rowX][0] = u;
    A[rowX][1] = v;
    A[rowX][2] = 1.0;

    A[rowX][3] = 0.0;
    A[rowX][4] = 0.0;
    A[rowX][5] = 0.0;

    A[rowX][6] = -X * u;
    A[rowX][7] = -X * v;

    b[rowX] = X;

    // Ecuación para Y

    A[rowY][0] = 0.0;
    A[rowY][1] = 0.0;
    A[rowY][2] = 0.0;

    A[rowY][3] = u;
    A[rowY][4] = v;
    A[rowY][5] = 1.0;

    A[rowY][6] = -Y * u;
    A[rowY][7] = -Y * v;

    b[rowY] = Y;
  }

  double parameters[8];

  if (!solveLinearSystem8(A, b, parameters)) {
    return false;
  }

  H[0][0] = parameters[0];
  H[0][1] = parameters[1];
  H[0][2] = parameters[2];

  H[1][0] = parameters[3];
  H[1][1] = parameters[4];
  H[1][2] = parameters[5];

  H[2][0] = parameters[6];
  H[2][1] = parameters[7];
  H[2][2] = 1.0;

  return true;
}

// ==================================================
// Conversión píxeles -> milímetros
// ==================================================

bool pixelToMillimeters(
  double u,
  double v,
  Point2D &physicalPoint
) {
  if (!homographyValid) {
    return false;
  }

  const double denominator =
    H[2][0] * u +
    H[2][1] * v +
    H[2][2];

  if (fabs(denominator) < 1e-12) {
    return false;
  }

  physicalPoint.x =
    (
      H[0][0] * u +
      H[0][1] * v +
      H[0][2]
    ) / denominator;

  physicalPoint.y =
    (
      H[1][0] * u +
      H[1][1] * v +
      H[1][2]
    ) / denominator;

  return true;
}



bool allTagsReady() {
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    if (tags[i].samples < SAMPLES_PER_TAG) return false;
  }
  return true;
}

void printCalibrationProgress() {
  Serial.print(F("[CAL] Muestras "));
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    Serial.printf("tag%d=%u/%u ", tags[i].code, tags[i].samples, SAMPLES_PER_TAG);
  }
  Serial.println();
}

void printHelp() {
  Serial.println(F("[AYUDA] C=recalibrar; M=reabrir modelo conservando H; P=pausar/reanudar salida; H=ayuda"));
  Serial.println(F("[AYUDA] 0=modelo 128; 1=modelo 129; 2=modelo 130 (conservan H si ya esta calibrada)"));
  Serial.printf("[MODELO] Indice=%u algoritmo=%u\n", modelIndex, static_cast<uint8_t>(selectedModel()));
}

void printJsonString(const String &value) {
  Serial.write('"');
  for (unsigned int i = 0; i < value.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(value[i]);
    if (c == '"' || c == '\\') {
      Serial.write('\\');
      Serial.write(c);
    } else if (c < 0x20) {
      Serial.printf("\\u%04x", c);
    } else {
      Serial.write(c);
    }
  }
  Serial.write('"');
}

void resetCalibration() {
  homographyValid = false;
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    tags[i].sumU = tags[i].sumV = 0;
    tags[i].samples = 0;
  }
  for (uint8_t r = 0; r < 3; ++r) {
    for (uint8_t c = 0; c < 3; ++c) H[r][c] = 0;
  }
  H[2][2] = 1;
}

void startCalibration() {
  resetCalibration();
  paused = false;
  if (!connected) {
    setState(CameraState::CONNECTING);
    return;
  }
  Serial.println(F("[CAL] Abriendo Tag Recognition; espera de carga 3 s."));
  if (!huskylens.switchAlgorithm(ALGORITHM_TAG_RECOGNITION)) {
    Serial.println(F("[ERROR] No se confirmo Tag Recognition. C para reintentar."));
    connected = false;
    setState(CameraState::ERROR);
    return;
  }
  setState(CameraState::WAIT_TAGS);
}

void openModel() {
  if (!homographyValid || !connected) {
    Serial.println(F("[AVISO] Primero se necesita calibrar: envie C."));
    return;
  }
  Serial.printf("[MODELO] Abriendo personalizado %u; espera de carga 8 s.\n",
                static_cast<uint8_t>(selectedModel()));
  if (!huskylens.switchAlgorithm(selectedModel())) {
    Serial.println(F("[ERROR] No se confirmo el modelo. Revise su indice; use 0/1/2 o M."));
    setState(CameraState::ERROR);
    return;
  }
  paused = false;
  setState(CameraState::WAIT_MODEL);
}

void handleReadError() {
  Serial.printf("[UART] Lectura sin respuesta valida (%u/3).\n", ++consecutiveErrors);
  if (consecutiveErrors >= 3) {
    Serial.println(F("[UART] Reconectando; se repetira la calibracion antes de abrir el modelo."));
    connected = false;
    resetCalibration();
    setState(CameraState::CONNECTING);
  }
}

void readCalibration() {
  const int8_t count = huskylens.getResult(ALGORITHM_TAG_RECOGNITION);
  if (count < 0) { handleReadError(); return; }
  consecutiveErrors = 0;
  bool sampled[NUMBER_OF_TAGS] = {};
  while (huskylens.available(ALGORITHM_TAG_RECOGNITION)) {
    Result *result = huskylens.popCachedResult(ALGORITHM_TAG_RECOGNITION);
    if (!result) break;
    if (result->type != COMMAND_RETURN_BLOCK) continue;
    int code;
    extractTagCode(result, code);
    const int index = findTagIndex(code);
    if (index < 0 || sampled[index] || tags[index].samples >= SAMPLES_PER_TAG) continue;
    sampled[index] = true;
    tags[index].sumU += result->xCenter;
    tags[index].sumV += result->yCenter;
    ++tags[index].samples;
  }
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    printCalibrationProgress();
  }
  if (!allTagsReady()) return;
  if (!calculateHomography()) {
    Serial.println(F("[ERROR] Homografia singular. Revise ubicacion de los tags y envie C."));
    setState(CameraState::ERROR);
    return;
  }
  homographyValid = true;
  Serial.println(F("[CAL] COMPLETA. Matriz pixel -> mm:"));
  for (uint8_t r = 0; r < 3; ++r) {
    Serial.printf("[CAL] %.9f %.9f %.9f\n", H[r][0], H[r][1], H[r][2]);
  }
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    const double u = tags[i].sumU / tags[i].samples;
    const double v = tags[i].sumV / tags[i].samples;
    const Point2D expected = getPhysicalTagPosition(i);
    Serial.printf("[CAL] Tag%d pixel=(%.2f,%.2f) mm=(%.2f,%.2f)\n",
                  tags[i].code, u, v, expected.x, expected.y);
  }
  openModel();
}

// Misma estimacion que la maqueta: transformar las cuatro esquinas de la
// caja y comparar su extension X/Y en mm. Solo eje dominante, no giro real.
uint8_t estimateBoxAxis(const Result *result) {
  if (!homographyValid || result->width <= 0 || result->height <= 0) return 0;
  const double left = result->xCenter - result->width * 0.5;
  const double right = result->xCenter + result->width * 0.5;
  const double top = result->yCenter - result->height * 0.5;
  const double bottom = result->yCenter + result->height * 0.5;
  Point2D corners[4];
  if (!pixelToMillimeters(left, top, corners[0]) ||
      !pixelToMillimeters(right, top, corners[1]) ||
      !pixelToMillimeters(left, bottom, corners[2]) ||
      !pixelToMillimeters(right, bottom, corners[3])) return 0;
  double minX = corners[0].x, maxX = minX;
  double minY = corners[0].y, maxY = minY;
  for (uint8_t i = 1; i < 4; ++i) {
    minX = fmin(minX, corners[i].x); maxX = fmax(maxX, corners[i].x);
    minY = fmin(minY, corners[i].y); maxY = fmax(maxY, corners[i].y);
  }
  return VisionModelo129::ejePorDimensiones(maxX - minX, maxY - minY);
}

void printModelResults(int8_t count) {
  const uint32_t frame = ++frameNumber;
  const uint32_t timestamp = millis();
  const uint8_t algo = static_cast<uint8_t>(selectedModel());
  const Result *accepted[MAX_RESULT_NUM] = {};
  uint8_t indices[MAX_RESULT_NUM] = {};
  uint8_t acceptedCount = 0;
  uint8_t ignoredCount = 0;
  uint8_t index = 0;
  while (huskylens.available(selectedModel())) {
    const Result *result = huskylens.popCachedResult(selectedModel());
    if (!result) break;
    const uint8_t rawIndex = index++;
    if (VisionModelo129::clasePermitida(result->name.c_str()) == 0 ||
        result->type != COMMAND_RETURN_BLOCK ||
        result->width <= 0 || result->height <= 0) {
      ++ignoredCount;
      continue;
    }
    if (acceptedCount < MAX_RESULT_NUM) {
      indices[acceptedCount] = rawIndex;
      accepted[acceptedCount++] = result;
    }
  }
  Serial.printf("{\"tipo\":\"frame\",\"frame\":%lu,\"ms\":%lu,\"algoritmo\":%u,\"resultados\":%d,\"permitidos\":%u,\"ignorados\":%u}\n",
                static_cast<unsigned long>(frame), static_cast<unsigned long>(timestamp),
                algo, count, acceptedCount, ignoredCount);
  for (uint8_t i = 0; i < acceptedCount; ++i) {
    const Result *result = accepted[i];
    const uint8_t clase = VisionModelo129::clasePermitida(result->name.c_str());
    Point2D position = {};
    const bool coordinatesValid = result->type == COMMAND_RETURN_BLOCK &&
      result->width > 0 && result->height > 0 &&
      pixelToMillimeters(result->xCenter, result->yCenter, position) &&
      isfinite(position.x) && isfinite(position.y);
    const bool inside = coordinatesValid &&
      fabs(position.x) <= TAG_X_FROM_CENTER_MM && fabs(position.y) <= TAG_ROWS_DISTANCE_MM / 2.0;
    const bool belt = inside && fabs(position.x) <= BELT_WIDTH_MM / 2.0;
    Serial.printf("{\"tipo\":\"segmentacion\",\"frame\":%lu,\"ms\":%lu,\"algoritmo\":%u,\"indice\":%u,\"id\":%u,\"nombre\":",
                  static_cast<unsigned long>(frame), static_cast<unsigned long>(timestamp), algo, indices[i], result->ID);
    printJsonString(result->name);
    Serial.print(F(",\"contenido\":"));
    printJsonString(result->content);
    // El segundo byte es una union level/confidence/rfu1; no asumir porcentaje.
    Serial.printf(",\"tipo_resultado_raw\":%d,\"level_raw\":%d,\"u_px\":%d,\"v_px\":%d,\"ancho_px\":%d,\"alto_px\":%d,\"x_mm\":",
                  result->type, result->level, result->xCenter, result->yCenter, result->width, result->height);
    if (coordinatesValid) Serial.print(position.x, 2); else Serial.print(F("null"));
    Serial.print(F(",\"y_mm\":"));
    if (coordinatesValid) Serial.print(position.y, 2); else Serial.print(F("null"));
    Serial.printf(",\"coordenadas_validas\":%s,\"en_calibracion\":%s,\"en_banda\":%s",
                  coordinatesValid ? "true" : "false", inside ? "true" : "false", belt ? "true" : "false");
    const uint8_t axis = coordinatesValid ? estimateBoxAxis(result) : 0;
    Serial.printf(",\"clase_pieza\":%u,\"recogible\":%s,\"eje_aprox\":\"%s\",\"orientacion_valida\":%s,\"orientacion_aprox_deg\":",
                  clase, belt ? "true" : "false", axis == 1 ? "X" : (axis == 2 ? "Y" : "INDETERMINADO"),
                  axis != 0 ? "true" : "false");
    if (axis != 0) Serial.print(axis == 1 ? 0 : 90); else Serial.print(F("null"));
    Serial.print(F(",\"servo_sugerido_deg\":"));
    // Montaje actual de la maqueta: eje X -> servo 90, eje Y -> servo 0.
    if (axis != 0) Serial.print(axis == 1 ? 90 : 0); else Serial.print(F("null"));
    Serial.println(F(",\"metodo_angulo\":\"MODEL129_BOX_AXIS_MM\"}"));
  }
}

void handleCommand(char command) {
  if (command >= 'a' && command <= 'z') command -= ('a' - 'A');
  switch (command) {
    case 'C': startCalibration(); break;
    case 'M': openModel(); break;
    case 'P':
      paused = !paused;
      Serial.println(paused ? F("[SALIDA] Pausada.") : F("[SALIDA] Reanudada."));
      break;
    case 'H': printHelp(); break;
    case '0': case '1': case '2':
      modelIndex = static_cast<uint8_t>(command - '0');
      Serial.printf("[MODELO] Seleccionado algoritmo %u.\n", static_cast<uint8_t>(selectedModel()));
      if (homographyValid && connected) openModel();
      else if (state == CameraState::ERROR) startCalibration();
      break;
    default: Serial.println(F("[AVISO] Comando desconocido. H muestra ayuda.")); break;
  }
}

void cameraTick() {
  const uint32_t now = millis();
  switch (state) {
    case CameraState::CONNECTING:
      if (now - stateSince < RECONNECT_MS) return;
      Serial.println(F("[UART] Intentando conectar HUSKYLENS 2..."));
      // Solo esta tarea usa UART y biblioteca. Limpieza acotada de bytes viejos.
      for (int bytes = HuskyUART.available(); bytes > 0; --bytes) HuskyUART.read();
      connected = huskylens.begin(HuskyUART);
      stateSince = millis();
      if (connected) {
        Serial.println(F("[UART] Conectada."));
        startCalibration();
      } else {
        Serial.println(F("[UART] Sin respuesta; revise UART 115200, RX32/TX33 y GND."));
      }
      return;
    case CameraState::WAIT_TAGS:
      if (now - stateSince >= TAG_LOAD_MS) {
        Serial.println(F("[CAL] Buscando tags 0,1,2,3. Mantenga camara y tags fijos."));
        setState(CameraState::CALIBRATING);
      }
      return;
    case CameraState::CALIBRATING:
      if (now - stateSince >= CALIBRATION_TIMEOUT_MS) {
        printCalibrationProgress();
        Serial.println(F("[ERROR] Faltaron tags en 120 s. Revise IDs/posicion y envie C."));
        setState(CameraState::ERROR);
        return;
      }
      if (now - lastRead >= READ_PERIOD_MS) { lastRead = now; readCalibration(); }
      return;
    case CameraState::WAIT_MODEL:
      if (now - stateSince < MODEL_LOAD_MS || now - lastRead < 1000) return;
      lastRead = now;
      {
        const int8_t count = huskylens.getResult(selectedModel());
        if (count >= 0) {
          Serial.println(F("[MODELO] Respondio. Comenzando JSON por terminal cada 200 ms."));
          Serial.println(F("[DATOS] Solo pieza6/pieza7 por nombre; ID raw no distingue clases. Angulo aproximado X/Y, sin mascara."));
          setState(CameraState::STREAMING);
          printModelResults(count);
        } else if (millis() - stateSince >= MODEL_RESPONSE_TIMEOUT_MS) {
          Serial.println(F("[ERROR] Modelo sin respuesta en 30 s. Revise indice; use 0/1/2, M o C."));
          setState(CameraState::ERROR);
        } else {
          Serial.println(F("[MODELO] Todavia sin respuesta; esperando carga."));
        }
      }
      return;
    case CameraState::STREAMING:
      if (now - lastRead < READ_PERIOD_MS) return;
      lastRead = now;
      {
        const int8_t count = huskylens.getResult(selectedModel());
        if (count < 0) { handleReadError(); return; }
        consecutiveErrors = 0;
        if (!paused) printModelResults(count);
      }
      return;
    case CameraState::ERROR: return;
  }
}

void cameraTask(void *) {
  // Biblioteca 1.0.9: sus consultas pueden esperar 5 s sin ceder CPU.
  // Prioridad idle permite al sistema seguir atendiendo USB y watchdog.
  huskylens.retry = 1;
  HuskyUART.begin(115200, SERIAL_8N1, 32, 33);
  printHelp();
  stateSince = millis();
  for (;;) {
    char command;
    while (xQueueReceive(commandQueue, &command, 0) == pdTRUE) handleCommand(command);
    cameraTick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n[BOOT] PRUEBA HUSKYLENS 2 SEGMENTACION + TAGS; UART1 RX32/TX33."));
  commandQueue = xQueueCreate(16, sizeof(char));
  if (!commandQueue) { Serial.println(F("[ERROR] Sin memoria para comandos.")); return; }
  if (xTaskCreate(cameraTask, "segmentacion", 8192, nullptr, tskIDLE_PRIORITY, nullptr) != pdPASS) {
    Serial.println(F("[ERROR] No se pudo crear la tarea de camara."));
    vQueueDelete(commandQueue);
    commandQueue = nullptr;
  }
}

void loop() {
  while (Serial.available()) {
    const char command = static_cast<char>(Serial.read());
    if (command == '\r' || command == '\n' || command == ' ' || command == '\t') continue;
    if (commandQueue) xQueueSend(commandQueue, &command, 0);
  }
  delay(1);
}

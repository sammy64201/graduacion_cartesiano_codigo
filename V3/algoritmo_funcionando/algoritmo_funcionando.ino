/***************************************************
 HUSKYLENS 2 - Calibración con AprilTags
 y detección mediante modelo personalizado

 Comunicación:
 ESP32 RX GPIO32 <- TX HUSKYLENS
 ESP32 TX GPIO33 -> RX HUSKYLENS

 Flujo:
 1. Calibración con tags 0, 1, 2 y 3.
 2. Cálculo de homografía píxel -> milímetros.
 3. Apertura automática del modelo personalizado.
 4. Impresión de posición X,Y de las piezas.

 Sistema de coordenadas:
 X positivo hacia la derecha.
 Y positivo hacia abajo.
 Origen en el centro entre los cuatro tags.

 Comandos por monitor serial:
 C = recalibrar
 M = abrir modelo personalizado
 T = abrir Tag Recognition sin borrar calibración
 H = ayuda
 ****************************************************/

#include <Arduino.h>
#include <DFRobot_HuskylensV2.h>
#include <math.h>
#include <stdlib.h>

// ==================================================
// UART de HUSKYLENS
// ==================================================

constexpr int HUSKY_RX_PIN = 32;
constexpr int HUSKY_TX_PIN = 33;
constexpr uint32_t HUSKY_BAUDRATE = 115200;

HardwareSerial HuskyUART(2);
HuskylensV2 huskylens;

// ==================================================
// Modelo personalizado
// ==================================================

// 0 = primer modelo instalado -> algoritmo 128
// 1 = segundo modelo         -> algoritmo 129
// 2 = tercer modelo          -> algoritmo 130
constexpr uint8_t CUSTOM_MODEL_INDEX = 1;

const eAlgorithm_t PIECE_MODEL =
  static_cast<eAlgorithm_t>(
    static_cast<uint8_t>(ALGORITHM_CUSTOM_BEGIN) +
    CUSTOM_MODEL_INDEX
  );

// ==================================================
// Dimensiones físicas
// ==================================================

constexpr double BELT_WIDTH_MM = 292.0;
constexpr double TOTAL_WIDTH_MM = 412.0;

// Cada borde de aluminio:
// (412 - 292) / 2 = 60 mm
constexpr double ALUMINUM_WIDTH_MM =
  (TOTAL_WIDTH_MM - BELT_WIDTH_MM) / 2.0;

// Si los tags están centrados sobre el aluminio:
// 292/2 + 60/2 = 176 mm
constexpr double TAG_X_FROM_CENTER_MM =
  BELT_WIDTH_MM / 2.0 +
  ALUMINUM_WIDTH_MM / 2.0;

// --------------------------------------------------
// CAMBIAR ESTA MEDIDA
// --------------------------------------------------
// Distancia vertical entre el centro de los tags
// superiores y el centro de los tags inferiores.
constexpr double TAG_ROWS_DISTANCE_MM = 382.0;

// ==================================================
// Configuración de lectura
// ==================================================

constexpr uint16_t SAMPLES_PER_TAG = 25;
constexpr uint32_t READ_PERIOD_MS = 200;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

constexpr uint8_t NUMBER_OF_TAGS = 4;

// ==================================================
// Estructuras
// ==================================================

struct Point2D {
  double x;
  double y;
};

struct CalibrationTag {
  int code;

  double sumU;
  double sumV;

  uint16_t samples;
};

// ==================================================
// Tags esperados
// ==================================================
/*
 Ubicación:

 Tag 0 = superior izquierdo
 Tag 1 = superior derecho
 Tag 2 = inferior derecho
 Tag 3 = inferior izquierdo
*/

CalibrationTag tags[NUMBER_OF_TAGS] = {
  {0, 0.0, 0.0, 0},
  {1, 0.0, 0.0, 0},
  {2, 0.0, 0.0, 0},
  {3, 0.0, 0.0, 0}
};

// ==================================================
// Estado
// ==================================================

enum class OperatingMode {
  CALIBRATING,
  DETECTING_PIECES
};

OperatingMode currentMode =
  OperatingMode::CALIBRATING;

bool homographyValid = false;
bool calibrationCalculationAttempted = false;

uint32_t lastReadTime = 0;
uint32_t lastStatusTime = 0;

/*
 Homografía:

 | h00 h01 h02 |
 | h10 h11 h12 |
 | h20 h21  1  |
*/

double H[3][3] = {
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 1.0}
};

// ==================================================
// Coordenadas físicas de los tags
// ==================================================

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

// ==================================================
// Control de algoritmos
// ==================================================

bool openAlgorithm(
  eAlgorithm_t algorithm,
  const char *name,
  uint32_t loadingTimeMs
) {
  Serial.println();
  Serial.print("Abriendo ");
  Serial.println(name);

  if (!huskylens.switchAlgorithm(algorithm)) {
    Serial.print("ERROR: no se pudo abrir ");
    Serial.println(name);
    return false;
  }

  Serial.println("Esperando carga...");
  delay(loadingTimeMs);

  Serial.print(name);
  Serial.println(" activo.");

  return true;
}

// ==================================================
// Reinicio de calibración
// ==================================================

void resetCalibrationData() {
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; i++) {
    tags[i].sumU = 0.0;
    tags[i].sumV = 0.0;
    tags[i].samples = 0;
  }

  homographyValid = false;
  calibrationCalculationAttempted = false;

  H[0][0] = 0.0;
  H[0][1] = 0.0;
  H[0][2] = 0.0;

  H[1][0] = 0.0;
  H[1][1] = 0.0;
  H[1][2] = 0.0;

  H[2][0] = 0.0;
  H[2][1] = 0.0;
  H[2][2] = 1.0;
}

void startCalibration() {
  resetCalibrationData();

  if (
    openAlgorithm(
      ALGORITHM_TAG_RECOGNITION,
      "Tag Recognition",
      3000
    )
  ) {
    currentMode = OperatingMode::CALIBRATING;

    Serial.println();
    Serial.println("Buscando tags 0, 1, 2 y 3...");
  }
}

// ==================================================
// Estado de muestras
// ==================================================

bool allTagsReady() {
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; i++) {
    if (tags[i].samples < SAMPLES_PER_TAG) {
      return false;
    }
  }

  return true;
}

void printCalibrationProgress() {
  Serial.println();
  Serial.println("Progreso de calibración:");

  for (uint8_t i = 0; i < NUMBER_OF_TAGS; i++) {
    Serial.print("Tag ");
    Serial.print(tags[i].code);
    Serial.print(": ");

    Serial.print(tags[i].samples);
    Serial.print("/");
    Serial.print(SAMPLES_PER_TAG);

    if (tags[i].samples > 0) {
      Serial.print(" | Píxel promedio: (");

      Serial.print(
        tags[i].sumU / tags[i].samples,
        2
      );

      Serial.print(", ");

      Serial.print(
        tags[i].sumV / tags[i].samples,
        2
      );

      Serial.print(")");
    }

    Serial.println();
  }
}

// ==================================================
// Sistema lineal 8 x 8
// ==================================================

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

// ==================================================
// Información de calibración
// ==================================================

void printHomography() {
  Serial.println();
  Serial.println("Matriz de homografía:");

  for (uint8_t row = 0; row < 3; row++) {
    Serial.print("[ ");

    for (uint8_t column = 0; column < 3; column++) {
      Serial.print(H[row][column], 9);
      Serial.print(" ");
    }

    Serial.println("]");
  }
}

void printCalibrationPoints() {
  Serial.println();
  Serial.println("Correspondencias calculadas:");

  for (uint8_t i = 0; i < NUMBER_OF_TAGS; i++) {
    const double u =
      tags[i].sumU / tags[i].samples;

    const double v =
      tags[i].sumV / tags[i].samples;

    const Point2D expected =
      getPhysicalTagPosition(i);

    Point2D calculated;

    pixelToMillimeters(u, v, calculated);

    Serial.print("Tag ");
    Serial.print(tags[i].code);

    Serial.print(" | Pixel: (");
    Serial.print(u, 2);
    Serial.print(", ");
    Serial.print(v, 2);

    Serial.print(") | Real esperado: (");
    Serial.print(expected.x, 2);
    Serial.print(", ");
    Serial.print(expected.y, 2);

    Serial.print(") | Calculado: (");
    Serial.print(calculated.x, 2);
    Serial.print(", ");
    Serial.print(calculated.y, 2);
    Serial.println(") mm");
  }

  /*
   Verificar dónde cae el centro de la imagen.
   HUSKYLENS trabaja normalmente con 640 x 480.
  */

  Point2D imageCenter;

  if (pixelToMillimeters(320.0, 240.0, imageCenter)) {
    Serial.println();

    Serial.print(
      "Centro de imagen (320,240) corresponde a: X="
    );

    Serial.print(imageCenter.x, 2);

    Serial.print(" mm, Y=");
    Serial.print(imageCenter.y, 2);
    Serial.println(" mm");
  }

  Serial.println();

  Serial.print("Zona blanca de la banda en X: ");
  Serial.print(-BELT_WIDTH_MM / 2.0, 1);
  Serial.print(" a ");
  Serial.print(BELT_WIDTH_MM / 2.0, 1);
  Serial.println(" mm");
}

// ==================================================
// Apertura del modelo personalizado
// ==================================================

bool openPieceModel() {
  if (!homographyValid) {
    Serial.println();
    Serial.println(
      "No se puede abrir el modelo: falta calibración."
    );

    return false;
  }

  Serial.println();
  Serial.print("Abriendo modelo personalizado ID ");
  Serial.println(
    static_cast<uint8_t>(PIECE_MODEL)
  );

  if (
    !openAlgorithm(
      PIECE_MODEL,
      "Modelo de piezas",
      8000
    )
  ) {
    return false;
  }

  currentMode =
    OperatingMode::DETECTING_PIECES;

  Serial.println();
  Serial.println("Calibración conservada.");
  Serial.println("Comenzando detección de piezas.");

  return true;
}

// ==================================================
// Procesamiento de tags
// ==================================================

void processTagCalibration() {
  const int8_t resultCount =
    huskylens.getResult(
      ALGORITHM_TAG_RECOGNITION
    );

  if (resultCount < 0) {
    Serial.println(
      "Error de comunicación en Tag Recognition."
    );

    return;
  }

  while (
    huskylens.available(
      ALGORITHM_TAG_RECOGNITION
    )
  ) {
    Result *result =
      huskylens.popCachedResult(
        ALGORITHM_TAG_RECOGNITION
      );

    if (result == nullptr) {
      continue;
    }

    int tagCode;

    if (!extractTagCode(result, tagCode)) {
      continue;
    }

    const int index =
      findTagIndex(tagCode);

    if (index < 0) {
      continue;
    }

    CalibrationTag &tag =
      tags[index];

    if (tag.samples >= SAMPLES_PER_TAG) {
      continue;
    }

    tag.sumU += result->xCenter;
    tag.sumV += result->yCenter;
    tag.samples++;
  }

  if (
    millis() - lastStatusTime >=
    STATUS_PERIOD_MS
  ) {
    lastStatusTime = millis();
    printCalibrationProgress();
  }

  if (
    allTagsReady() &&
    !calibrationCalculationAttempted
  ) {
    calibrationCalculationAttempted = true;

    Serial.println();
    Serial.println("Calculando calibración...");

    if (!calculateHomography()) {
      Serial.println(
        "No fue posible calcular la homografía."
      );

      return;
    }

    homographyValid = true;

    Serial.println();
    Serial.println(
      "CALIBRACIÓN COMPLETADA CORRECTAMENTE."
    );

    printHomography();
    printCalibrationPoints();

    delay(2000);

    openPieceModel();
  }
}

// ==================================================
// Comprobación del área
// ==================================================

bool isInsideCalibrationArea(
  const Point2D &position
) {
  const double halfCalibrationHeight =
    TAG_ROWS_DISTANCE_MM / 2.0;

  return
    position.x >= -TAG_X_FROM_CENTER_MM &&
    position.x <=  TAG_X_FROM_CENTER_MM &&
    position.y >= -halfCalibrationHeight &&
    position.y <=  halfCalibrationHeight;
}

bool isOverWhiteBelt(
  const Point2D &position
) {
  return
    fabs(position.x) <= BELT_WIDTH_MM / 2.0 &&
    isInsideCalibrationArea(position);
}

// ==================================================
// Procesamiento de piezas
// ==================================================

void processPieceDetection() {
  const int8_t resultCount =
    huskylens.getResult(PIECE_MODEL);

  if (resultCount < 0) {
    Serial.println(
      "Error de comunicación con el modelo."
    );

    return;
  }

  if (resultCount == 0) {
    static uint32_t lastEmptyMessage = 0;

    if (
      millis() - lastEmptyMessage >=
      STATUS_PERIOD_MS
    ) {
      Serial.println("Sin piezas detectadas.");
      lastEmptyMessage = millis();
    }

    return;
  }

  while (huskylens.available(PIECE_MODEL)) {
    Result *result =
      huskylens.popCachedResult(PIECE_MODEL);

    if (result == nullptr) {
      continue;
    }

    Point2D physicalPosition;

    if (
      !pixelToMillimeters(
        result->xCenter,
        result->yCenter,
        physicalPosition
      )
    ) {
      Serial.println(
        "No fue posible convertir la posición."
      );

      continue;
    }

    const bool insideCalibration =
      isInsideCalibrationArea(
        physicalPosition
      );

    const bool overWhiteBelt =
      isOverWhiteBelt(
        physicalPosition
      );

    /*
     Salida fácil de leer y reutilizar posteriormente:

     PIEZA,ID=1,NOMBRE=pieza6,U=350,V=220,
     X=20.50,Y=-15.20,EN_BANDA=1
    */

    Serial.print("PIEZA");

    Serial.print(",ID=");
    Serial.print(result->ID);

    Serial.print(",NOMBRE=");
    Serial.print(result->name);

    Serial.print(",U=");
    Serial.print(result->xCenter);

    Serial.print(",V=");
    Serial.print(result->yCenter);

    Serial.print(",X=");
    Serial.print(physicalPosition.x, 2);

    Serial.print(",Y=");
    Serial.print(physicalPosition.y, 2);

    Serial.print(",EN_CALIBRACION=");
    Serial.print(insideCalibration ? 1 : 0);

    Serial.print(",EN_BANDA=");
    Serial.print(overWhiteBelt ? 1 : 0);

    Serial.print(",ANCHO_PX=");
    Serial.print(result->width);

    Serial.print(",ALTO_PX=");
    Serial.println(result->height);
  }
}

// ==================================================
// Comandos del monitor serial
// ==================================================

void printHelp() {
  Serial.println();
  Serial.println("Comandos:");

  Serial.println(
    "C = borrar calibración y recalibrar"
  );

  Serial.println(
    "M = abrir modelo personalizado"
  );

  Serial.println(
    "T = abrir Tag Recognition sin borrar H"
  );

  Serial.println(
    "H = mostrar ayuda"
  );
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (
      command >= 'a' &&
      command <= 'z'
    ) {
      command -= 32;
    }

    switch (command) {
      case 'C':
        startCalibration();
        break;

      case 'M':
        openPieceModel();
        break;

      case 'T':
        if (
          openAlgorithm(
            ALGORITHM_TAG_RECOGNITION,
            "Tag Recognition",
            3000
          )
        ) {
          currentMode =
            OperatingMode::CALIBRATING;
        }
        break;

      case 'H':
        printHelp();
        break;

      default:
        break;
    }
  }
}

// ==================================================
// Inicialización
// ==================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("HUSKYLENS 2 - Calibración y detección");
  Serial.println("======================================");

  Serial.print("Ancho banda blanca: ");
  Serial.print(BELT_WIDTH_MM);
  Serial.println(" mm");

  Serial.print("Ancho total: ");
  Serial.print(TOTAL_WIDTH_MM);
  Serial.println(" mm");

  Serial.print("Ancho aluminio por lado: ");
  Serial.print(ALUMINUM_WIDTH_MM);
  Serial.println(" mm");

  Serial.print(
    "X estimada de centros laterales: +/-"
  );

  Serial.print(TAG_X_FROM_CENTER_MM);
  Serial.println(" mm");

  if (TAG_ROWS_DISTANCE_MM <= 0.0) {
    Serial.println();
    Serial.println(
      "ADVERTENCIA: debe configurar"
    );

    Serial.println(
      "TAG_ROWS_DISTANCE_MM antes de calibrar."
    );
  }

  HuskyUART.begin(
    HUSKY_BAUDRATE,
    SERIAL_8N1,
    HUSKY_RX_PIN,
    HUSKY_TX_PIN
  );

  delay(500);

  while (!huskylens.begin(HuskyUART)) {
    Serial.println(
      "No se pudo comunicar con HUSKYLENS 2."
    );

    Serial.println(
      "Revise RX, TX, GND, alimentación y UART."
    );

    delay(1000);
  }

  Serial.println();
  Serial.println("HUSKYLENS 2 conectada.");

  printHelp();
  startCalibration();
}

// ==================================================
// Programa principal
// ==================================================

void loop() {
  handleSerialCommands();

  const uint32_t currentTime = millis();

  if (
    currentTime - lastReadTime <
    READ_PERIOD_MS
  ) {
    return;
  }

  lastReadTime = currentTime;

  switch (currentMode) {
    case OperatingMode::CALIBRATING:
      processTagCalibration();
      break;

    case OperatingMode::DETECTING_PIECES:
      processPieceDetection();
      break;
  }
}
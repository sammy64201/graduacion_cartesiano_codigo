/**
 * @file ESP.ino
 * @brief Firmware integrado de la ESP32 del brazo cartesiano.
 *
 * Responsabilidades:
 *  - esclavo I2C de la Portenta H7 en el bus de control;
 *  - maestro del bus I2C independiente de la pantalla SH1106;
 *  - adquisicion del control Bluepad32 y control de los dos servos;
 *  - calibracion y deteccion HUSKYLENS 2 por UART2;
 *  - publicacion atomica de telemetria y objetivos estables.
 *
 * Todas las llamadas de DFRobot_HuskylensV2 se ejecutan en una tarea FreeRTOS
 * exclusiva. La version 1.0.9 de esa biblioteca espera activamente hasta 5 s
 * cuando el dispositivo no responde. Por eso la tarea usa prioridad idle y un
 * solo reintento: el loop, Bluepad32, I2C y OLED siguen siendo atendidos.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <DFRobot_HuskylensV2.h>
#include <math.h>
#include <stdlib.h>
#include <esp_system.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ProtocoloI2C.h"

using namespace ProtocoloI2C;

// Declaraciones adelantadas para el generador de prototipos de Arduino. Sin
// ellas, el preprocesador de sketches puede insertar firmas antes de que estas
// estructuras aparezcan en el archivo.
struct Point2D;
struct FiltroDeteccion;
struct ContextoCamara;

// =============================================================================
// Hardware fijo. Estos pines son parte del contrato electrico del prototipo.
// =============================================================================

constexpr int I2C_PORTENTA_SDA = 27;
constexpr int I2C_PORTENTA_SCL = 14;
constexpr uint32_t I2C_PORTENTA_HZ = 100000;

constexpr int OLED_SDA = 21;
constexpr int OLED_SCL = 22;
constexpr uint8_t OLED_DIRECCION = 0x3C;
constexpr uint32_t OLED_I2C_HZ = 100000;

constexpr int PIN_SERVO_ROTACION = 25;
constexpr int PIN_SERVO_PINZA = 26;

// UART cruzada: GPIO32 (RX ESP32) <- TX HUSKYLENS,
//                 GPIO33 (TX ESP32) -> RX HUSKYLENS.
constexpr int HUSKY_RX_PIN = 32;
constexpr int HUSKY_TX_PIN = 33;
constexpr uint32_t HUSKY_BAUDRATE = 115200;

TwoWire I2C_Pantalla(1);
Adafruit_SH1106G pantalla(128, 64, &I2C_Pantalla, -1);
Servo servoRotacion;
Servo servoPinza;
HardwareSerial HuskyUART(2);
HuskylensV2 huskylens;

// =============================================================================
// Periodos y limites operativos.
// =============================================================================

constexpr uint32_t PERIODO_SERVO_MS = 5;
constexpr uint32_t PERIODO_PUBLICACION_I2C_MS = 10;
constexpr uint32_t PERIODO_PANTALLA_MS = 100;
constexpr uint32_t PERIODO_REPORTE_I2C_MS = 1000;
constexpr uint32_t TIMEOUT_PORTENTA_MS = 1000;
constexpr uint32_t PERIODO_REINTENTO_OLED_MS = 1000;
constexpr uint32_t RETARDO_INICIAL_OLED_MS = 1500;

constexpr uint32_t CAM_RECONNECT_MS = 2000;
constexpr uint32_t CAM_TAG_LOAD_MS = 3000;
constexpr uint32_t CAM_POST_CALC_MS = 2000;
constexpr uint32_t CAM_MODEL_LOAD_MS = 8000;
constexpr uint32_t CAM_READ_PERIOD_MS = 200;
constexpr uint32_t CAM_HEALTH_PERIOD_MS = 1000;
constexpr uint32_t CAM_STATUS_PERIOD_MS = 1000;
constexpr uint32_t CAM_CALIBRATION_TIMEOUT_MS = 120000;

constexpr uint8_t DETECCIONES_ESTABLES = 4;
constexpr double TOLERANCIA_ESTABLE_MM = 4.0;
constexpr double TOLERANCIA_REARME_MM = 8.0;
constexpr uint32_t TIEMPO_DESAPARICION_MS = 1000;

constexpr int ANGULO_SERVO_INICIAL = 90;
constexpr uint32_t CAMERA_TASK_STACK_BYTES = 12288;

#if defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
constexpr BaseType_t CAMERA_TASK_CORE = 0;
#else
constexpr BaseType_t CAMERA_TASK_CORE = 1;
#endif

// =============================================================================
// Estado compartido y sincronizacion.
// =============================================================================

ControllerPtr controles[BP32_MAX_GAMEPADS];

int8_t joystickX = 0;
int8_t joystickY = 0;
int8_t joystickZ = 0;
uint8_t botonesControl = 0;
bool bluetoothConectado = false;
uint8_t dpadRaw = 0;
int anguloServoRotacion = ANGULO_SERVO_INICIAL;
int anguloServoPinza = ANGULO_SERVO_INICIAL;

bool sistemaBaseListo = false;
bool busPantallaIniciado = false;
bool pantallaInicializada = false;
bool i2cEsclavoIniciado = false;
uint32_t proximoIntentoOLED = 0;
uint32_t intentosInicioOLED = 0;
uint32_t ultimoUpdateServo = 0;
uint32_t ultimaPublicacionI2C = 0;
uint32_t ultimaPantalla = 0;
uint32_t ultimoReporteI2C = 0;
uint32_t ultimoReporteStack = 0;
uint8_t estadoRemotoAnterior = 0xFF;
uint32_t inicioEstadoRemoto = 0;

portMUX_TYPE txI2CMux = portMUX_INITIALIZER_UNLOCKED;
PaqueteESPAPortenta paqueteTxSnapshot = {};

struct RxPendiente {
  PaquetePortentaAESP paquete;
  uint8_t longitud;
};

portMUX_TYPE rxI2CMux = portMUX_INITIALIZER_UNLOCKED;
RxPendiente rxPendiente = {};
volatile bool hayRxPendiente = false;

PaquetePortentaAESP estadoPortenta = {};
bool estadoPortentaValido = false;
uint32_t ultimoEstadoPortenta = 0;

volatile uint32_t rxI2CTotal = 0;
volatile uint32_t rxI2COk = 0;
volatile uint32_t rxI2CLongitudIncorrecta = 0;
volatile uint32_t rxI2CProtocoloIncorrecto = 0;
volatile int ultimoTamanoRecibido = -1;

uint8_t secuenciaPaqueteI2C = 0;
uint16_t sesionArranque = 0;

struct ControlCamaraCompartido {
  bool portentaActiva;
  bool automaticoActivo;
  bool brazoOcupado;
  uint8_t comando;
  uint8_t secuenciaComando;
  uint16_t ackObjetivo;
  uint8_t codigoAckObjetivo;
};

portMUX_TYPE controlCamaraMux = portMUX_INITIALIZER_UNLOCKED;
ControlCamaraCompartido controlCamara = {};

struct EstadoCamaraPublicado {
  uint8_t estado;
  uint8_t error;
  uint8_t ackComando;
  uint8_t muestras[4];
  bool conectada;
  bool homografiaValida;
  bool modeloListo;
  bool ocupada;
  bool objetivoValido;
  uint8_t claseObjetivo;
  int16_t objetivoX10;
  int16_t objetivoY10;
  uint16_t secuenciaObjetivo;
};

portMUX_TYPE estadoCamaraMux = portMUX_INITIALIZER_UNLOCKED;
EstadoCamaraPublicado estadoCamaraPublicado = {
  CAMARA_OFFLINE, CAM_ERROR_NINGUNO, 0, {0, 0, 0, 0},
  false, false, false, false, false, 0, 0, 0, 0
};

TaskHandle_t tareaCamaraHandle = nullptr;

// =============================================================================
// Utilidades generales.
// =============================================================================

bool plazoCumplido(uint32_t ahora, uint32_t plazo) {
  return static_cast<int32_t>(ahora - plazo) >= 0;
}

EstadoCamaraPublicado copiarEstadoCamara() {
  EstadoCamaraPublicado copia;
  portENTER_CRITICAL(&estadoCamaraMux);
  copia = estadoCamaraPublicado;
  portEXIT_CRITICAL(&estadoCamaraMux);
  return copia;
}

ControlCamaraCompartido copiarControlCamara() {
  ControlCamaraCompartido copia;
  portENTER_CRITICAL(&controlCamaraMux);
  copia = controlCamara;
  portEXIT_CRITICAL(&controlCamaraMux);
  return copia;
}

void ponerControlEnNeutro() {
  joystickX = 0;
  joystickY = 0;
  joystickZ = 0;
  botonesControl = 0;
  dpadRaw = 0;
  bluetoothConectado = false;
}

// =============================================================================
// Callbacks I2C: no validan, no calculan CRC, no imprimen y no usan la camara.
// =============================================================================

void requestEvent() {
  PaqueteESPAPortenta copia;
  portENTER_CRITICAL(&txI2CMux);
  copia = paqueteTxSnapshot;
  portEXIT_CRITICAL(&txI2CMux);
  Wire.write(reinterpret_cast<const uint8_t *>(&copia), sizeof(copia));
}

void receiveEvent(int cantidadBytes) {
  // Un sondeo de direccion puede disparar el callback sin carga util.
  if (cantidadBytes <= 0) {
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  RxPendiente temporal = {};
  temporal.longitud = static_cast<uint8_t>(
    cantidadBytes > UINT8_MAX ? UINT8_MAX : cantidadBytes
  );

  uint8_t *destino = reinterpret_cast<uint8_t *>(&temporal.paquete);
  uint8_t recibidos = 0;
  while (Wire.available() && recibidos < sizeof(PaquetePortentaAESP)) {
    destino[recibidos++] = static_cast<uint8_t>(Wire.read());
  }
  while (Wire.available()) {
    Wire.read();
  }
  // Se conserva la longitud informada por Wire para rechazar tambien paquetes
  // sobredimensionados; si faltaron bytes, el CRC del relleno cero no validara.

  portENTER_CRITICAL(&rxI2CMux);
  rxPendiente = temporal;
  hayRxPendiente = true;
  portEXIT_CRITICAL(&rxI2CMux);
}

// =============================================================================
// Bluepad32 y servos.
// =============================================================================

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controles[i] == nullptr) {
      controles[i] = ctl;
      Serial.print(F("[BOOT] Control conectado en indice "));
      Serial.println(i);
      return;
    }
  }
  Serial.println(F("[ERROR] No hay espacio para otro control"));
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controles[i] == ctl) {
      controles[i] = nullptr;
    }
  }

  // Comportamiento seguro conservador: una desconexion neutraliza todas las
  // ordenes, incluso si habia mas de un mando registrado.
  ponerControlEnNeutro();
  Serial.println(F("[BOOT] Control desconectado; ejes neutralizados"));
}

void processControllers() {
  bool hayControlConectado = false;

  for (ControllerPtr ctl : controles) {
    if (ctl == nullptr || !ctl->isConnected()) {
      continue;
    }

    hayControlConectado = true;
    if (!ctl->hasData()) {
      continue;
    }

    // Se conservan los sentidos del firmware funcional original.
    joystickX = (abs(ctl->axisX()) > 100)
      ? (ctl->axisX() > 0 ? -1 : 1)
      : 0;
    joystickY = (abs(ctl->axisY()) > 100)
      ? (ctl->axisY() > 0 ? -1 : 1)
      : 0;

    const int ejeDerechoY = ctl->axisRY();
    const int ejeDerechoX = ctl->axisRX();
    joystickZ = (abs(ejeDerechoY) > 100)
      ? (ejeDerechoY > 0 ? 1 : -1)
      : 0;

    botonesControl = 0;
    if (ctl->a()) botonesControl |= BOTON_X;
    if (ctl->y()) botonesControl |= BOTON_TRIANGULO;
    dpadRaw = ctl->dpad();

    const uint32_t ahora = millis();
    if (ahora - ultimoUpdateServo >= PERIODO_SERVO_MS) {
      ultimoUpdateServo = ahora;

      if (ejeDerechoX > 150) {
        anguloServoRotacion = min(180, anguloServoRotacion + 1);
      } else if (ejeDerechoX < -150) {
        anguloServoRotacion = max(0, anguloServoRotacion - 1);
      }

      if (dpadRaw & 0x08U) {
        anguloServoPinza = min(180, anguloServoPinza + 1);
      }
      if (dpadRaw & 0x04U) {
        anguloServoPinza = max(0, anguloServoPinza - 1);
      }

      servoRotacion.write(anguloServoRotacion);
      servoPinza.write(anguloServoPinza);
    }
  }

  bluetoothConectado = hayControlConectado;
  if (!hayControlConectado) {
    ponerControlEnNeutro();
  }
}

// =============================================================================
// OLED recuperable. Nunca condiciona el resto del sistema.
// =============================================================================

void intentarInicializarOLEDNoBloqueante() {
  if (pantallaInicializada) {
    return;
  }

  const uint32_t ahora = millis();
  if (!sistemaBaseListo || ahora < RETARDO_INICIAL_OLED_MS) {
    return;
  }
  if (!plazoCumplido(ahora, proximoIntentoOLED)) {
    return;
  }

  proximoIntentoOLED = ahora + PERIODO_REINTENTO_OLED_MS;
  ++intentosInicioOLED;

  // Si incluso el begin() inicial del bus fallo, volver a levantarlo aqui. Un
  // fallo de OLED nunca condiciona Bluetooth, camara, servos ni el otro I2C.
  if (!busPantallaIniciado) {
    busPantallaIniciado = I2C_Pantalla.begin(
      OLED_SDA,
      OLED_SCL,
      OLED_I2C_HZ
    );
    if (!busPantallaIniciado) {
      Serial.println(F("[OLED] No se pudo iniciar el bus; se reintentara"));
      return;
    }
    Serial.println(F("[OLED] Bus recuperado"));
  }

  I2C_Pantalla.beginTransmission(OLED_DIRECCION);
  const uint8_t error = I2C_Pantalla.endTransmission(true);
  if (error != 0) {
    Serial.print(F("[OLED] Sin ACK, intento="));
    Serial.print(intentosInicioOLED);
    Serial.print(F(" codigo="));
    Serial.println(error);
    return;
  }

  if (!pantalla.begin(OLED_DIRECCION, true)) {
    Serial.println(F("[OLED] pantalla.begin() fallo; se reintentara"));
    return;
  }

  pantalla.clearDisplay();
  pantalla.setTextSize(1);
  pantalla.setTextColor(SH110X_WHITE);
  pantalla.setCursor(0, 0);
  pantalla.println(F("ESP32 INICIADA"));
  pantalla.println(F("OLED CONECTADA"));
  pantalla.display();
  pantallaInicializada = true;
  ultimaPantalla = 0;
  Serial.println(F("[OLED] Inicializada correctamente"));
}

// =============================================================================
// Recepcion validada y publicacion precomputada del protocolo.
// =============================================================================

void publicarControlCamaraDesdePortenta(const PaquetePortentaAESP &paquete) {
  ControlCamaraCompartido nuevo = {};
  nuevo.portentaActiva = true;
  nuevo.automaticoActivo = (paquete.flagsSistema & SIS_FLAG_AUTO_ACTIVO) != 0;
  nuevo.brazoOcupado = (paquete.flagsSistema & SIS_FLAG_BRAZO_OCUPADO) != 0;
  nuevo.comando = paquete.comandoCamara;
  nuevo.secuenciaComando = paquete.secuenciaComandoCamara;
  nuevo.ackObjetivo = paquete.ackSecuenciaObjetivo;
  nuevo.codigoAckObjetivo = paquete.codigoAckObjetivo;

  portENTER_CRITICAL(&controlCamaraMux);
  controlCamara = nuevo;
  portEXIT_CRITICAL(&controlCamaraMux);
}

void invalidarControlCamaraPorTimeout() {
  portENTER_CRITICAL(&controlCamaraMux);
  controlCamara.portentaActiva = false;
  controlCamara.automaticoActivo = false;
  controlCamara.brazoOcupado = false;
  controlCamara.comando = CAM_CMD_NINGUNO;
  controlCamara.codigoAckObjetivo = ACK_OBJ_NINGUNO;
  portEXIT_CRITICAL(&controlCamaraMux);
}

void procesarRecepcionI2C() {
  RxPendiente copia = {};
  bool hayCopia = false;

  portENTER_CRITICAL(&rxI2CMux);
  if (hayRxPendiente) {
    copia = rxPendiente;
    hayRxPendiente = false;
    hayCopia = true;
  }
  portEXIT_CRITICAL(&rxI2CMux);

  if (hayCopia) {
    ++rxI2CTotal;
    ultimoTamanoRecibido = copia.longitud;

    if (copia.longitud != sizeof(PaquetePortentaAESP)) {
      ++rxI2CLongitudIncorrecta;
    } else if (!validarPaquete(copia.paquete)) {
      ++rxI2CProtocoloIncorrecto;
    } else {
      estadoPortenta = copia.paquete;
      estadoPortentaValido = true;
      ultimoEstadoPortenta = millis();
      ++rxI2COk;
      publicarControlCamaraDesdePortenta(copia.paquete);
    }
  }

  if (
    estadoPortentaValido &&
    millis() - ultimoEstadoPortenta > TIMEOUT_PORTENTA_MS
  ) {
    estadoPortentaValido = false;
    invalidarControlCamaraPorTimeout();
    Serial.println(F("[I2C] Timeout de la Portenta"));
  }
}

void prepararSnapshotI2C() {
  const uint32_t ahora = millis();
  if (ahora - ultimaPublicacionI2C < PERIODO_PUBLICACION_I2C_MS) {
    return;
  }
  ultimaPublicacionI2C = ahora;

  const EstadoCamaraPublicado camara = copiarEstadoCamara();
  PaqueteESPAPortenta paquete = {};
  paquete.secuenciaPaquete = ++secuenciaPaqueteI2C;
  paquete.sesionArranque = sesionArranque;

  if (sistemaBaseListo) paquete.flags |= ESP_FLAG_BASE_LISTA;
  if (bluetoothConectado) paquete.flags |= ESP_FLAG_BT_CONECTADO;
  if (pantallaInicializada) paquete.flags |= ESP_FLAG_OLED_LISTA;
  if (camara.conectada) paquete.flags |= ESP_FLAG_CAMARA_CONECTADA;
  if (camara.homografiaValida) paquete.flags |= ESP_FLAG_HOMOGRAFIA_VALIDA;
  if (camara.modeloListo) paquete.flags |= ESP_FLAG_MODELO_LISTO;
  if (camara.objetivoValido) paquete.flags |= ESP_FLAG_OBJETIVO_VALIDO;
  if (camara.ocupada) paquete.flags |= ESP_FLAG_CAMARA_OCUPADA;

  paquete.joystickX = joystickX;
  paquete.joystickY = joystickY;
  paquete.joystickZ = joystickZ;
  paquete.botones = botonesControl;
  paquete.servoRotacion = static_cast<uint8_t>(anguloServoRotacion);
  paquete.servoPinza = static_cast<uint8_t>(anguloServoPinza);
  paquete.estadoCamara = camara.estado;
  paquete.errorCamara = camara.error;
  paquete.ackSecuenciaComandoCamara = camara.ackComando;
  memcpy(paquete.muestrasTag, camara.muestras, sizeof(paquete.muestrasTag));
  paquete.claseObjetivo = camara.claseObjetivo;
  paquete.objetivoX10 = camara.objetivoX10;
  paquete.objetivoY10 = camara.objetivoY10;
  paquete.secuenciaObjetivo = camara.secuenciaObjetivo;
  prepararPaquete(paquete);

  portENTER_CRITICAL(&txI2CMux);
  paqueteTxSnapshot = paquete;
  portEXIT_CRITICAL(&txI2CMux);
}

// =============================================================================
// Geometria y homografia HUSKYLENS. Se conserva el algoritmo funcional.
// =============================================================================

constexpr double BELT_WIDTH_MM = 292.0;
constexpr double TOTAL_WIDTH_MM = 412.0;
constexpr double ALUMINUM_WIDTH_MM =
  (TOTAL_WIDTH_MM - BELT_WIDTH_MM) / 2.0;
constexpr double TAG_X_FROM_CENTER_MM =
  BELT_WIDTH_MM / 2.0 + ALUMINUM_WIDTH_MM / 2.0;
constexpr double TAG_ROWS_DISTANCE_MM = 382.0;
constexpr uint16_t SAMPLES_PER_TAG = 25;
constexpr uint8_t NUMBER_OF_TAGS = 4;
constexpr uint8_t CUSTOM_MODEL_INDEX = 1;

const eAlgorithm_t PIECE_MODEL = static_cast<eAlgorithm_t>(
  static_cast<uint8_t>(ALGORITHM_CUSTOM_BEGIN) + CUSTOM_MODEL_INDEX
);

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

CalibrationTag tags[NUMBER_OF_TAGS] = {
  {0, 0.0, 0.0, 0},
  {1, 0.0, 0.0, 0},
  {2, 0.0, 0.0, 0},
  {3, 0.0, 0.0, 0}
};

// | h00 h01 h02 |
// | h10 h11 h12 |
// | h20 h21  1  |
double H[3][3] = {
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0},
  {0.0, 0.0, 1.0}
};

Point2D getPhysicalTagPosition(uint8_t index) {
  const double halfHeight = TAG_ROWS_DISTANCE_MM / 2.0;
  switch (index) {
    case 0: return {-TAG_X_FROM_CENTER_MM, -halfHeight};
    case 1: return { TAG_X_FROM_CENTER_MM, -halfHeight};
    case 2: return { TAG_X_FROM_CENTER_MM,  halfHeight};
    case 3: return {-TAG_X_FROM_CENTER_MM,  halfHeight};
    default: return {0.0, 0.0};
  }
}

bool parseLastInteger(const String &text, int &value) {
  const char *cursor = text.c_str();
  bool found = false;

  while (*cursor != '\0') {
    const bool positiveNumber = *cursor >= '0' && *cursor <= '9';
    const bool negativeNumber =
      *cursor == '-' && *(cursor + 1) >= '0' && *(cursor + 1) <= '9';

    if (positiveNumber || negativeNumber) {
      char *endPointer = nullptr;
      const long parsed = strtol(cursor, &endPointer, 10);
      if (endPointer != cursor) {
        value = static_cast<int>(parsed);
        found = true;
        cursor = endPointer;
        continue;
      }
    }
    ++cursor;
  }
  return found;
}

bool extractTagCode(const Result *result, int &tagCode) {
  if (result == nullptr) {
    return false;
  }
  if (parseLastInteger(result->content, tagCode)) {
    return true;
  }
  tagCode = result->ID;
  return true;
}

int findTagIndex(int tagCode) {
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    if (tags[i].code == tagCode) {
      return i;
    }
  }
  return -1;
}

void resetCalibrationData() {
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    tags[i].sumU = 0.0;
    tags[i].sumV = 0.0;
    tags[i].samples = 0;
  }

  H[0][0] = 0.0; H[0][1] = 0.0; H[0][2] = 0.0;
  H[1][0] = 0.0; H[1][1] = 0.0; H[1][2] = 0.0;
  H[2][0] = 0.0; H[2][1] = 0.0; H[2][2] = 1.0;
}

bool allTagsReady() {
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    if (tags[i].samples < SAMPLES_PER_TAG) {
      return false;
    }
  }
  return true;
}

bool solveLinearSystem8(
  double matrix[8][8],
  double vector[8],
  double solution[8]
) {
  double augmented[8][9];

  for (uint8_t row = 0; row < 8; ++row) {
    for (uint8_t column = 0; column < 8; ++column) {
      augmented[row][column] = matrix[row][column];
    }
    augmented[row][8] = vector[row];
  }

  for (uint8_t column = 0; column < 8; ++column) {
    uint8_t pivotRow = column;
    double largestValue = fabs(augmented[column][column]);

    for (uint8_t row = column + 1; row < 8; ++row) {
      const double candidate = fabs(augmented[row][column]);
      if (candidate > largestValue) {
        largestValue = candidate;
        pivotRow = row;
      }
    }

    if (largestValue < 1e-12 || !isfinite(largestValue)) {
      return false;
    }

    if (pivotRow != column) {
      for (uint8_t currentColumn = column; currentColumn < 9; ++currentColumn) {
        const double temporary = augmented[column][currentColumn];
        augmented[column][currentColumn] = augmented[pivotRow][currentColumn];
        augmented[pivotRow][currentColumn] = temporary;
      }
    }

    const double pivot = augmented[column][column];
    for (uint8_t currentColumn = column; currentColumn < 9; ++currentColumn) {
      augmented[column][currentColumn] /= pivot;
    }

    for (uint8_t row = 0; row < 8; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = augmented[row][column];
      for (uint8_t currentColumn = column; currentColumn < 9; ++currentColumn) {
        augmented[row][currentColumn] -=
          factor * augmented[column][currentColumn];
      }
    }
  }

  for (uint8_t i = 0; i < 8; ++i) {
    solution[i] = augmented[i][8];
    if (!isfinite(solution[i])) {
      return false;
    }
  }
  return true;
}

bool calculateHomography() {
  if (TAG_ROWS_DISTANCE_MM <= 0.0 || !allTagsReady()) {
    return false;
  }

  double A[8][8] = {};
  double b[8] = {};

  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    const double u = tags[i].sumU / tags[i].samples;
    const double v = tags[i].sumV / tags[i].samples;
    const Point2D physical = getPhysicalTagPosition(i);
    const double X = physical.x;
    const double Y = physical.y;
    const uint8_t rowX = 2 * i;
    const uint8_t rowY = rowX + 1;

    // X = (h00*u + h01*v + h02) / (h20*u + h21*v + 1)
    A[rowX][0] = u;
    A[rowX][1] = v;
    A[rowX][2] = 1.0;
    A[rowX][3] = 0.0;
    A[rowX][4] = 0.0;
    A[rowX][5] = 0.0;
    A[rowX][6] = -X * u;
    A[rowX][7] = -X * v;
    b[rowX] = X;

    // Y = (h10*u + h11*v + h12) / (h20*u + h21*v + 1)
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

bool pixelToMillimeters(
  double u,
  double v,
  bool homographyValid,
  Point2D &physicalPoint
) {
  if (!homographyValid) {
    return false;
  }

  const double denominator = H[2][0] * u + H[2][1] * v + H[2][2];
  if (fabs(denominator) < 1e-12 || !isfinite(denominator)) {
    return false;
  }

  physicalPoint.x =
    (H[0][0] * u + H[0][1] * v + H[0][2]) / denominator;
  physicalPoint.y =
    (H[1][0] * u + H[1][1] * v + H[1][2]) / denominator;
  return isfinite(physicalPoint.x) && isfinite(physicalPoint.y);
}

bool isInsideCalibrationArea(const Point2D &position) {
  const double halfCalibrationHeight = TAG_ROWS_DISTANCE_MM / 2.0;
  return
    position.x >= -TAG_X_FROM_CENTER_MM &&
    position.x <=  TAG_X_FROM_CENTER_MM &&
    position.y >= -halfCalibrationHeight &&
    position.y <=  halfCalibrationHeight;
}

bool isOverWhiteBelt(const Point2D &position) {
  return
    fabs(position.x) <= BELT_WIDTH_MM / 2.0 &&
    isInsideCalibrationArea(position);
}

void printHomography() {
  Serial.println(F("[CAL] Matriz de homografia:"));
  for (uint8_t row = 0; row < 3; ++row) {
    Serial.print(F("[CAL] [ "));
    for (uint8_t column = 0; column < 3; ++column) {
      Serial.print(H[row][column], 9);
      Serial.print(' ');
    }
    Serial.println(']');
  }
}

// =============================================================================
// Contexto privado de la tarea de camara y filtro de detecciones.
// =============================================================================

struct FiltroDeteccion {
  uint8_t clase;
  uint8_t consecutivas;
  double sumaX;
  double sumaY;
  double promedioX;
  double promedioY;
  double minimoX;
  double maximoX;
  double minimoY;
  double maximoY;
};

struct ContextoCamara {
  uint8_t estado;
  uint8_t error;
  uint8_t ackComando;
  uint8_t ultimaSecuenciaComando;
  bool conectada;
  bool homografiaValida;
  bool modeloListo;
  bool operacionEstadoIniciada;
  bool recuperarAutomaticamente;
  bool solicitarCalibracion;
  bool solicitarModelo;
  bool portentaEstabaActiva;
  bool autoEstabaActivo;
  uint32_t entradaEstado;
  uint32_t plazoEstado;
  uint32_t proximaConexion;
  uint32_t proximaLectura;
  uint32_t inicioCalibracion;
  uint32_t ultimoEstadoSerial;

  FiltroDeteccion filtro;
  bool rearmada;
  bool esperandoDesaparicion;
  uint32_t inicioAusencia;
  uint8_t claseEsperandoDesaparicion;
  double xEsperandoDesaparicion;
  double yEsperandoDesaparicion;

  bool objetivoValido;
  uint8_t claseObjetivo;
  int16_t objetivoX10;
  int16_t objetivoY10;
  uint16_t secuenciaObjetivo;
};

ContextoCamara camaraCtx = {};

bool estadoCamaraOcupado(uint8_t estado) {
  return
    estado == CAMARA_CONECTANDO ||
    estado == CAMARA_ABRIENDO_TAGS ||
    estado == CAMARA_CALIBRANDO ||
    estado == CAMARA_CALCULANDO ||
    estado == CAMARA_ABRIENDO_MODELO;
}

void publicarEstadoCamara(const ContextoCamara &ctx) {
  EstadoCamaraPublicado publicado = {};
  publicado.estado = ctx.estado;
  publicado.error = ctx.error;
  publicado.ackComando = ctx.ackComando;
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    publicado.muestras[i] = static_cast<uint8_t>(
      tags[i].samples < SAMPLES_PER_TAG ? tags[i].samples : SAMPLES_PER_TAG
    );
  }
  publicado.conectada = ctx.conectada;
  publicado.homografiaValida = ctx.homografiaValida;
  publicado.modeloListo = ctx.modeloListo;
  publicado.ocupada = estadoCamaraOcupado(ctx.estado);
  publicado.objetivoValido = ctx.objetivoValido;
  publicado.claseObjetivo = ctx.claseObjetivo;
  publicado.objetivoX10 = ctx.objetivoX10;
  publicado.objetivoY10 = ctx.objetivoY10;
  publicado.secuenciaObjetivo = ctx.secuenciaObjetivo;

  portENTER_CRITICAL(&estadoCamaraMux);
  estadoCamaraPublicado = publicado;
  portEXIT_CRITICAL(&estadoCamaraMux);
}

void cambiarEstadoCamara(ContextoCamara &ctx, uint8_t nuevoEstado) {
  ctx.estado = nuevoEstado;
  ctx.entradaEstado = millis();
  ctx.plazoEstado = 0;
  ctx.operacionEstadoIniciada = false;
  publicarEstadoCamara(ctx);
}

void reiniciarFiltro(ContextoCamara &ctx) {
  ctx.filtro = {
    0, 0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0
  };
}

void iniciarEsperaDesaparicion(
  ContextoCamara &ctx,
  uint8_t clase,
  int16_t x10,
  int16_t y10
) {
  ctx.esperandoDesaparicion = true;
  ctx.inicioAusencia = 0;
  ctx.rearmada = false;
  ctx.claseEsperandoDesaparicion = clase;
  ctx.xEsperandoDesaparicion = static_cast<double>(x10) / 10.0;
  ctx.yEsperandoDesaparicion = static_cast<double>(y10) / 10.0;
  reiniciarFiltro(ctx);
}

void limpiarObjetivo(ContextoCamara &ctx, bool exigirDesaparicion) {
  const bool habiaObjetivo = ctx.objetivoValido;
  const uint8_t claseAnterior = ctx.claseObjetivo;
  const int16_t x10Anterior = ctx.objetivoX10;
  const int16_t y10Anterior = ctx.objetivoY10;
  ctx.objetivoValido = false;
  ctx.claseObjetivo = 0;
  ctx.objetivoX10 = 0;
  ctx.objetivoY10 = 0;
  reiniciarFiltro(ctx);
  if (exigirDesaparicion && habiaObjetivo) {
    iniciarEsperaDesaparicion(
      ctx,
      claseAnterior,
      x10Anterior,
      y10Anterior
    );
  } else if (!ctx.esperandoDesaparicion) {
    ctx.rearmada = true;
  }
}

void registrarErrorCamara(
  ContextoCamara &ctx,
  uint8_t error,
  bool desconectar,
  bool recuperarAutomaticamente
) {
  ctx.error = error;
  ctx.recuperarAutomaticamente = recuperarAutomaticamente;
  if (desconectar) {
    ctx.conectada = false;
    ctx.modeloListo = false;
  }
  limpiarObjetivo(ctx, true);
  cambiarEstadoCamara(ctx, CAMARA_ERROR);
  ctx.plazoEstado = millis() + CAM_RECONNECT_MS;

  Serial.print(F("[ERROR] Camara codigo="));
  Serial.println(error);
}

void imprimirProgresoCalibracion() {
  Serial.print(F("[CAL] Tags"));
  for (uint8_t i = 0; i < NUMBER_OF_TAGS; ++i) {
    Serial.print(' ');
    Serial.print(tags[i].code);
    Serial.print('=');
    Serial.print(tags[i].samples);
    Serial.print('/');
    Serial.print(SAMPLES_PER_TAG);
  }
  Serial.println();
}

bool leerTagsUnaVez() {
  const int8_t resultCount = huskylens.getResult(ALGORITHM_TAG_RECOGNITION);
  if (resultCount < 0) {
    return false;
  }

  while (huskylens.available(ALGORITHM_TAG_RECOGNITION)) {
    Result *result = huskylens.popCachedResult(ALGORITHM_TAG_RECOGNITION);
    if (result == nullptr) {
      continue;
    }

    int tagCode = -1;
    if (!extractTagCode(result, tagCode)) {
      continue;
    }

    const int index = findTagIndex(tagCode);
    if (index < 0) {
      continue;
    }

    CalibrationTag &tag = tags[index];
    if (tag.samples >= SAMPLES_PER_TAG) {
      continue;
    }
    tag.sumU += result->xCenter;
    tag.sumV += result->yCenter;
    ++tag.samples;
  }
  return true;
}

void actualizarEsperaDesaparicion(
  ContextoCamara &ctx,
  bool piezaProcesadaPresente,
  uint32_t ahora,
  bool brazoOcupado
) {
  if (!ctx.esperandoDesaparicion) {
    return;
  }

  // Un objetivo aceptado no puede rearmar el detector durante el recorrido.
  // La ausencia solo empieza a contar cuando la Portenta informa brazo libre.
  if (brazoOcupado) {
    ctx.inicioAusencia = 0;
    return;
  }

  if (piezaProcesadaPresente) {
    ctx.inicioAusencia = 0;
    return;
  }

  if (ctx.inicioAusencia == 0) {
    ctx.inicioAusencia = ahora;
    return;
  }

  if (ahora - ctx.inicioAusencia >= TIEMPO_DESAPARICION_MS) {
    ctx.esperandoDesaparicion = false;
    ctx.inicioAusencia = 0;
    ctx.rearmada = true;
    ctx.claseEsperandoDesaparicion = 0;
    ctx.xEsperandoDesaparicion = 0.0;
    ctx.yEsperandoDesaparicion = 0.0;
    reiniciarFiltro(ctx);
    Serial.println(F("[AUTO] Detector rearmado tras desaparicion"));
  }
}

bool mismaDeteccionEstable(
  const FiltroDeteccion &filtro,
  uint8_t clase,
  const Point2D &posicion
) {
  const double nuevoMinX = fmin(filtro.minimoX, posicion.x);
  const double nuevoMaxX = fmax(filtro.maximoX, posicion.x);
  const double nuevoMinY = fmin(filtro.minimoY, posicion.y);
  const double nuevoMaxY = fmax(filtro.maximoY, posicion.y);

  return
    filtro.consecutivas > 0 &&
    filtro.clase == clase &&
    nuevoMaxX - nuevoMinX <= TOLERANCIA_ESTABLE_MM &&
    nuevoMaxY - nuevoMinY <= TOLERANCIA_ESTABLE_MM;
}

void incorporarDeteccion(
  ContextoCamara &ctx,
  uint8_t clase,
  const Point2D &posicion
) {
  FiltroDeteccion &filtro = ctx.filtro;
  if (!mismaDeteccionEstable(filtro, clase, posicion)) {
    filtro.clase = clase;
    filtro.consecutivas = 1;
    filtro.sumaX = posicion.x;
    filtro.sumaY = posicion.y;
    filtro.promedioX = posicion.x;
    filtro.promedioY = posicion.y;
    filtro.minimoX = posicion.x;
    filtro.maximoX = posicion.x;
    filtro.minimoY = posicion.y;
    filtro.maximoY = posicion.y;
    return;
  }

  if (filtro.consecutivas < UINT8_MAX) {
    ++filtro.consecutivas;
  }
  filtro.sumaX += posicion.x;
  filtro.sumaY += posicion.y;
  filtro.promedioX = filtro.sumaX / filtro.consecutivas;
  filtro.promedioY = filtro.sumaY / filtro.consecutivas;
  filtro.minimoX = fmin(filtro.minimoX, posicion.x);
  filtro.maximoX = fmax(filtro.maximoX, posicion.x);
  filtro.minimoY = fmin(filtro.minimoY, posicion.y);
  filtro.maximoY = fmax(filtro.maximoY, posicion.y);
}

void publicarObjetivoEstable(ContextoCamara &ctx) {
  if (ctx.filtro.consecutivas < DETECCIONES_ESTABLES) {
    return;
  }

  const long x10 = lround(ctx.filtro.promedioX * 10.0);
  const long y10 = lround(ctx.filtro.promedioY * 10.0);
  if (x10 < INT16_MIN || x10 > INT16_MAX || y10 < INT16_MIN || y10 > INT16_MAX) {
    reiniciarFiltro(ctx);
    return;
  }

  ++ctx.secuenciaObjetivo;
  if (ctx.secuenciaObjetivo == 0) {
    ++ctx.secuenciaObjetivo;
  }

  ctx.objetivoValido = true;
  ctx.claseObjetivo = ctx.filtro.clase;
  ctx.objetivoX10 = static_cast<int16_t>(x10);
  ctx.objetivoY10 = static_cast<int16_t>(y10);
  ctx.rearmada = false;
  reiniciarFiltro(ctx);

  Serial.print(F("[AUTO] Objetivo seq="));
  Serial.print(ctx.secuenciaObjetivo);
  Serial.print(F(" clase="));
  Serial.print(ctx.claseObjetivo);
  Serial.print(F(" X="));
  Serial.print(ctx.objetivoX10 / 10.0f, 1);
  Serial.print(F(" Y="));
  Serial.println(ctx.objetivoY10 / 10.0f, 1);
}

bool leerPiezasUnaVez(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control,
  uint32_t ahora
) {
  const int8_t resultCount = huskylens.getResult(PIECE_MODEL);
  if (resultCount < 0) {
    return false;
  }

  bool hayPiezaValida = false;
  bool piezaProcesadaPresente = false;
  bool hayPrimera = false;
  uint8_t clasePrimera = 0;
  Point2D posicionPrimera = {};
  bool hayCoincidente = false;
  uint8_t claseCoincidente = 0;
  Point2D posicionCoincidente = {};
  double distanciaCoincidente = HUGE_VAL;

  while (huskylens.available(PIECE_MODEL)) {
    Result *result = huskylens.popCachedResult(PIECE_MODEL);
    if (result == nullptr) {
      continue;
    }

    Point2D posicion;
    if (!pixelToMillimeters(
          result->xCenter,
          result->yCenter,
          ctx.homografiaValida,
          posicion
        ) || !isOverWhiteBelt(posicion)) {
      continue;
    }

    hayPiezaValida = true;

    if (
      ctx.esperandoDesaparicion &&
      result->ID == ctx.claseEsperandoDesaparicion &&
      fabs(posicion.x - ctx.xEsperandoDesaparicion) <= TOLERANCIA_REARME_MM &&
      fabs(posicion.y - ctx.yEsperandoDesaparicion) <= TOLERANCIA_REARME_MM
    ) {
      piezaProcesadaPresente = true;
    }

    if (!hayPrimera) {
      hayPrimera = true;
      clasePrimera = result->ID;
      posicionPrimera = posicion;
    }

    if (mismaDeteccionEstable(ctx.filtro, result->ID, posicion)) {
      const double dx = posicion.x - ctx.filtro.promedioX;
      const double dy = posicion.y - ctx.filtro.promedioY;
      const double distancia2 = dx * dx + dy * dy;
      if (
        distancia2 < distanciaCoincidente
      ) {
        distanciaCoincidente = distancia2;
        hayCoincidente = true;
        claseCoincidente = result->ID;
        posicionCoincidente = posicion;
      }
    }
  }

  actualizarEsperaDesaparicion(
    ctx,
    piezaProcesadaPresente,
    ahora,
    control.brazoOcupado
  );

  if (
    !control.portentaActiva ||
    !control.automaticoActivo ||
    control.brazoOcupado ||
    ctx.objetivoValido ||
    !ctx.rearmada ||
    ctx.esperandoDesaparicion
  ) {
    reiniciarFiltro(ctx);
    return true;
  }

  if (!hayPiezaValida) {
    reiniciarFiltro(ctx);
    return true;
  }

  if (hayCoincidente) {
    incorporarDeteccion(ctx, claseCoincidente, posicionCoincidente);
  } else {
    incorporarDeteccion(ctx, clasePrimera, posicionPrimera);
  }
  publicarObjetivoEstable(ctx);
  return true;
}

// =============================================================================
// Comandos y maquina de estados de camara.
// =============================================================================

void prepararNuevaCalibracion(ContextoCamara &ctx) {
  resetCalibrationData();
  ctx.homografiaValida = false;
  ctx.modeloListo = false;
  ctx.solicitarCalibracion = true;
  ctx.solicitarModelo = false;
  ctx.error = CAM_ERROR_NINGUNO;
  ctx.recuperarAutomaticamente = false;
  limpiarObjetivo(ctx, true);
  ctx.rearmada = true;
  ctx.esperandoDesaparicion = false;
  reiniciarFiltro(ctx);

  if (ctx.conectada) {
    cambiarEstadoCamara(ctx, CAMARA_ABRIENDO_TAGS);
  } else {
    ctx.proximaConexion = millis();
    cambiarEstadoCamara(ctx, CAMARA_OFFLINE);
  }
}

void solicitarAperturaModelo(ContextoCamara &ctx) {
  if (!ctx.homografiaValida) {
    registrarErrorCamara(
      ctx,
      CAM_ERROR_COMANDO_INVALIDO,
      false,
      false
    );
    return;
  }

  ctx.solicitarCalibracion = false;
  ctx.solicitarModelo = true;
  ctx.modeloListo = false;
  limpiarObjetivo(ctx, true);
  if (ctx.conectada) {
    cambiarEstadoCamara(ctx, CAMARA_ABRIENDO_MODELO);
  } else {
    ctx.proximaConexion = millis();
    cambiarEstadoCamara(ctx, CAMARA_OFFLINE);
  }
}

void procesarComandoCamara(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control
) {
  if (
    !control.portentaActiva ||
    control.secuenciaComando == ctx.ultimaSecuenciaComando
  ) {
    return;
  }

  ctx.ultimaSecuenciaComando = control.secuenciaComando;
  ctx.ackComando = control.secuenciaComando;

  switch (control.comando) {
    case CAM_CMD_NINGUNO:
      break;

    case CAM_CMD_STANDBY:
      ctx.solicitarCalibracion = false;
      ctx.solicitarModelo = false;
      limpiarObjetivo(ctx, true);
      if (ctx.conectada) {
        cambiarEstadoCamara(ctx, CAMARA_STANDBY);
      } else {
        cambiarEstadoCamara(ctx, CAMARA_OFFLINE);
      }
      Serial.println(F("[CAM] Comando STANDBY aceptado"));
      break;

    case CAM_CMD_CALIBRAR:
      Serial.println(F("[CAM] Comando CALIBRAR aceptado"));
      prepararNuevaCalibracion(ctx);
      break;

    case CAM_CMD_ABRIR_MODELO:
      Serial.println(F("[CAM] Comando ABRIR_MODELO aceptado"));
      solicitarAperturaModelo(ctx);
      break;

    case CAM_CMD_REINICIAR_ERROR:
      Serial.println(F("[CAM] Comando REINICIAR_ERROR aceptado"));
      ctx.error = CAM_ERROR_NINGUNO;
      ctx.recuperarAutomaticamente = false;
      if (ctx.conectada) {
        cambiarEstadoCamara(ctx, CAMARA_STANDBY);
      } else {
        ctx.proximaConexion = millis();
        cambiarEstadoCamara(ctx, CAMARA_OFFLINE);
      }
      break;

    default:
      registrarErrorCamara(
        ctx,
        CAM_ERROR_COMANDO_INVALIDO,
        false,
        false
      );
      break;
  }
  publicarEstadoCamara(ctx);
}

void procesarHandshakeObjetivo(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control
) {
  if (
    ctx.objetivoValido &&
    control.portentaActiva &&
    control.codigoAckObjetivo != ACK_OBJ_NINGUNO &&
    control.ackObjetivo == ctx.secuenciaObjetivo
  ) {
    Serial.print(F("[AUTO] ACK objetivo seq="));
    Serial.print(ctx.secuenciaObjetivo);
    Serial.print(F(" codigo="));
    Serial.println(control.codigoAckObjetivo);
    limpiarObjetivo(ctx, true);
  }

  if (!control.portentaActiva && ctx.portentaEstabaActiva) {
    Serial.println(F("[AUTO] Objetivo invalidado por perdida I2C"));
    limpiarObjetivo(ctx, true);

    // La Portenta puede reiniciarse sin que la ESP32 lo haga. Su contador de
    // comandos volvera a cero; olvidar el dominio anterior evita confundir un
    // CALIBRAR nuevo con una retransmision ya atendida antes del reinicio.
    ctx.ultimaSecuenciaComando = 0;
    ctx.ackComando = 0;
  }

  if (!control.automaticoActivo && ctx.autoEstabaActivo) {
    Serial.println(F("[AUTO] Filtro limpiado al salir de modo automatico"));
    limpiarObjetivo(ctx, true);
  }

  ctx.portentaEstabaActiva = control.portentaActiva;
  ctx.autoEstabaActivo = control.automaticoActivo;
}

void vaciarUARTCamara() {
  while (HuskyUART.available() > 0) {
    HuskyUART.read();
  }
}

void procesarEstadoCamara(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control
) {
  uint32_t ahora = millis();

  switch (ctx.estado) {
    case CAMARA_OFFLINE:
      if (!plazoCumplido(ahora, ctx.proximaConexion)) {
        break;
      }

      cambiarEstadoCamara(ctx, CAMARA_CONECTANDO);
      Serial.println(F("[CAM] Intentando conexion UART2"));
      vaciarUARTCamara();

      // Llamada potencialmente bloqueante, confinada a esta tarea prioridad 0.
      if (!huskylens.begin(HuskyUART)) {
        ctx.conectada = false;
        ctx.modeloListo = false;
        ctx.error = CAM_ERROR_SIN_RESPUESTA;
        ctx.proximaConexion = millis() + CAM_RECONNECT_MS;
        cambiarEstadoCamara(ctx, CAMARA_OFFLINE);
        break;
      }

      ctx.conectada = true;
      ctx.error = CAM_ERROR_NINGUNO;
      ctx.recuperarAutomaticamente = false;
      Serial.println(F("[CAM] HUSKYLENS conectada"));

      if (ctx.solicitarCalibracion) {
        cambiarEstadoCamara(ctx, CAMARA_ABRIENDO_TAGS);
      } else if (ctx.solicitarModelo && ctx.homografiaValida) {
        cambiarEstadoCamara(ctx, CAMARA_ABRIENDO_MODELO);
      } else {
        cambiarEstadoCamara(ctx, CAMARA_STANDBY);
      }
      break;

    case CAMARA_CONECTANDO:
      // La operacion begin() se completa en la misma iteracion que entra al
      // estado. Este caso solo protege ante una transicion futura incompleta.
      break;

    case CAMARA_STANDBY:
      if (ctx.solicitarCalibracion) {
        cambiarEstadoCamara(ctx, CAMARA_ABRIENDO_TAGS);
      } else if (ctx.solicitarModelo && ctx.homografiaValida) {
        cambiarEstadoCamara(ctx, CAMARA_ABRIENDO_MODELO);
      }
      break;

    case CAMARA_ABRIENDO_TAGS:
      if (!ctx.operacionEstadoIniciada) {
        publicarEstadoCamara(ctx);
        Serial.println(F("[CAM] Abriendo Tag Recognition"));
        if (!huskylens.switchAlgorithm(ALGORITHM_TAG_RECOGNITION)) {
          registrarErrorCamara(
            ctx,
            CAM_ERROR_ABRIR_TAGS,
            true,
            true
          );
          break;
        }
        ctx.operacionEstadoIniciada = true;
        ctx.plazoEstado = millis() + CAM_TAG_LOAD_MS;
      } else if (plazoCumplido(ahora, ctx.plazoEstado)) {
        ctx.inicioCalibracion = ahora;
        ctx.proximaLectura = ahora;
        ctx.ultimoEstadoSerial = 0;
        cambiarEstadoCamara(ctx, CAMARA_CALIBRANDO);
        Serial.println(F("[CAL] Capturando tags 0, 1, 2 y 3"));
      }
      break;

    case CAMARA_CALIBRANDO:
      if (ahora - ctx.inicioCalibracion > CAM_CALIBRATION_TIMEOUT_MS) {
        registrarErrorCamara(
          ctx,
          CAM_ERROR_TIMEOUT_TAGS,
          false,
          false
        );
        break;
      }

      if (ahora - ctx.ultimoEstadoSerial >= CAM_STATUS_PERIOD_MS) {
        ctx.ultimoEstadoSerial = ahora;
        imprimirProgresoCalibracion();
      }

      if (allTagsReady()) {
        cambiarEstadoCamara(ctx, CAMARA_CALCULANDO);
        break;
      }

      if (!plazoCumplido(ahora, ctx.proximaLectura)) {
        break;
      }
      ctx.proximaLectura = ahora + CAM_READ_PERIOD_MS;
      if (!leerTagsUnaVez()) {
        // No se mezclan promedios tomados antes y despues de una perdida UART.
        resetCalibrationData();
        ctx.homografiaValida = false;
        registrarErrorCamara(
          ctx,
          CAM_ERROR_LECTURA,
          true,
          true
        );
      } else if (allTagsReady()) {
        cambiarEstadoCamara(ctx, CAMARA_CALCULANDO);
      }
      break;

    case CAMARA_CALCULANDO:
      if (!ctx.operacionEstadoIniciada) {
        Serial.println(F("[CAL] Calculando homografia 8x8"));
        if (!calculateHomography()) {
          ctx.homografiaValida = false;
          registrarErrorCamara(
            ctx,
            CAM_ERROR_HOMOGRAFIA,
            false,
            false
          );
          break;
        }
        ctx.homografiaValida = true;
        ctx.operacionEstadoIniciada = true;
        ctx.plazoEstado = millis() + CAM_POST_CALC_MS;
        printHomography();
        publicarEstadoCamara(ctx);
      } else if (plazoCumplido(ahora, ctx.plazoEstado)) {
        ctx.solicitarCalibracion = false;
        ctx.solicitarModelo = true;
        cambiarEstadoCamara(ctx, CAMARA_ABRIENDO_MODELO);
      }
      break;

    case CAMARA_ABRIENDO_MODELO:
      if (!ctx.homografiaValida) {
        registrarErrorCamara(
          ctx,
          CAM_ERROR_COMANDO_INVALIDO,
          false,
          false
        );
        break;
      }

      if (!ctx.operacionEstadoIniciada) {
        publicarEstadoCamara(ctx);
        Serial.print(F("[CAM] Abriendo modelo personalizado "));
        Serial.println(static_cast<uint8_t>(PIECE_MODEL));
        if (!huskylens.switchAlgorithm(PIECE_MODEL)) {
          registrarErrorCamara(
            ctx,
            CAM_ERROR_ABRIR_MODELO,
            true,
            true
          );
          break;
        }
        ctx.operacionEstadoIniciada = true;
        ctx.plazoEstado = millis() + CAM_MODEL_LOAD_MS;
      } else if (plazoCumplido(ahora, ctx.plazoEstado)) {
        ctx.modeloListo = true;
        ctx.solicitarModelo = false;
        ctx.error = CAM_ERROR_NINGUNO;
        ctx.proximaLectura = ahora;
        cambiarEstadoCamara(ctx, CAMARA_LISTA);
        Serial.println(F("[CAM] Modelo listo; deteccion habilitada"));
      }
      break;

    case CAMARA_LISTA: {
      const bool deteccionNecesaria =
        control.automaticoActivo ||
        ctx.esperandoDesaparicion ||
        ctx.objetivoValido;
      const uint32_t periodo =
        deteccionNecesaria ? CAM_READ_PERIOD_MS : CAM_HEALTH_PERIOD_MS;

      if (!plazoCumplido(ahora, ctx.proximaLectura)) {
        break;
      }
      ctx.proximaLectura = ahora + periodo;

      if (!leerPiezasUnaVez(ctx, control, ahora)) {
        ctx.solicitarModelo = ctx.homografiaValida;
        registrarErrorCamara(
          ctx,
          CAM_ERROR_LECTURA,
          true,
          true
        );
      }
      break;
    }

    case CAMARA_ERROR:
      if (
        ctx.recuperarAutomaticamente &&
        plazoCumplido(ahora, ctx.plazoEstado)
      ) {
        ctx.proximaConexion = ahora;
        cambiarEstadoCamara(ctx, CAMARA_OFFLINE);
      }
      break;

    default:
      registrarErrorCamara(
        ctx,
        CAM_ERROR_COMANDO_INVALIDO,
        false,
        false
      );
      break;
  }

  publicarEstadoCamara(ctx);
}

void tareaCamara(void *parametro) {
  (void)parametro;
  huskylens.retry = 1;
  camaraCtx.estado = CAMARA_OFFLINE;
  camaraCtx.error = CAM_ERROR_NINGUNO;
  camaraCtx.rearmada = true;
  camaraCtx.proximaConexion = millis();
  publicarEstadoCamara(camaraCtx);

  Serial.print(F("[CAM] Tarea en core "));
  Serial.print(xPortGetCoreID());
  Serial.println(F(", prioridad idle, retry=1"));

  for (;;) {
    const ControlCamaraCompartido control = copiarControlCamara();
    procesarComandoCamara(camaraCtx, control);
    procesarHandshakeObjetivo(camaraCtx, control);
    procesarEstadoCamara(camaraCtx, control);

    // Cesion cooperativa entre operaciones. Los bloqueos internos de la
    // biblioteca quedan confinados a esta tarea de prioridad idle.
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// =============================================================================
// Presentacion OLED. Toda escritura se hace fuera de callbacks.
// =============================================================================

const char *nombreEstadoCamara(uint8_t estado) {
  switch (estado) {
    case CAMARA_OFFLINE: return "OFFLINE";
    case CAMARA_CONECTANDO: return "CONECTANDO";
    case CAMARA_STANDBY: return "STANDBY";
    case CAMARA_ABRIENDO_TAGS: return "ABRIENDO TAGS";
    case CAMARA_CALIBRANDO: return "CALIBRANDO";
    case CAMARA_CALCULANDO: return "CALCULANDO";
    case CAMARA_ABRIENDO_MODELO: return "ABRIENDO MODELO";
    case CAMARA_LISTA: return "LISTA";
    case CAMARA_ERROR: return "ERROR";
    default: return "DESCONOCIDA";
  }
}

const char *nombreFaseBrazo(uint8_t fase) {
  switch (fase) {
    case BRAZO_CAL_ESPERA: return "PREPARANDO";
    case BRAZO_CAL_X_MIN_1: return "BUSCANDO X-";
    case BRAZO_CAL_X_MIN_LIBERAR: return "LIBERANDO X-";
    case BRAZO_CAL_X_MIN_SEPARAR: return "SEPARANDO X-";
    case BRAZO_CAL_X_MIN_2: return "VERIFICANDO X-";
    case BRAZO_CAL_X_MAX_1: return "BUSCANDO X+";
    case BRAZO_CAL_X_MAX_LIBERAR: return "LIBERANDO X+";
    case BRAZO_CAL_X_MAX_SEPARAR: return "SEPARANDO X+";
    case BRAZO_CAL_X_MAX_2: return "VERIFICANDO X+";
    case BRAZO_CAL_Y_MIN_1: return "BUSCANDO Y-";
    case BRAZO_CAL_Y_MIN_LIBERAR: return "LIBERANDO Y-";
    case BRAZO_CAL_Y_MIN_SEPARAR: return "SEPARANDO Y-";
    case BRAZO_CAL_Y_MIN_2: return "VERIFICANDO Y-";
    case BRAZO_CAL_Y_MAX_1: return "BUSCANDO Y+";
    case BRAZO_CAL_Y_MAX_LIBERAR: return "LIBERANDO Y+";
    case BRAZO_CAL_Y_MAX_SEPARAR: return "SEPARANDO Y+";
    case BRAZO_CAL_Y_MAX_2: return "VERIFICANDO Y+";
    case BRAZO_CAL_Z_MIN_1: return "BUSCANDO Z-";
    case BRAZO_CAL_Z_MIN_LIBERAR: return "LIBERANDO Z-";
    case BRAZO_CAL_Z_MIN_SEPARAR: return "SEPARANDO Z-";
    case BRAZO_CAL_Z_MIN_2: return "VERIFICANDO Z-";
    case BRAZO_CAL_Z_MAX_1: return "BUSCANDO Z+";
    case BRAZO_CAL_Z_MAX_LIBERAR: return "LIBERANDO Z+";
    case BRAZO_CAL_Z_MAX_SEPARAR: return "SEPARANDO Z+";
    case BRAZO_CAL_Z_MAX_2: return "VERIFICANDO Z+";
    case BRAZO_CAL_HOME: return "YENDO A HOME";
    case BRAZO_CAL_COMPLETA: return "CALIBRACION OK";
    case BRAZO_CAL_ERROR: return "ERROR CALIBRACION";
    default: return "FASE DESCONOCIDA";
  }
}

const char *textoErrorSistema(uint8_t error) {
  switch (error) {
    case SISTEMA_ERROR_NINGUNO: return "SIN ERROR";
    case SISTEMA_ERROR_I2C: return "COMUNICACION I2C";
    case SISTEMA_ERROR_CAMARA: return "CAMARA";
    case SISTEMA_ERROR_CALIBRACION_BRAZO: return "CALIBRACION BRAZO";
    case SISTEMA_ERROR_CHECKLIST: return "CHECKLIST";
    case SISTEMA_ERROR_OBJETIVO_FUERA_RANGO: return "OBJ. FUERA RANGO";
    case SISTEMA_ERROR_TIMEOUT_MOVIMIENTO: return "TIMEOUT MOV.";
    case SISTEMA_ERROR_FINALES_INCOHERENTES: return "FINALES INCOH.";
    case SISTEMA_ERROR_CANCELADO: return "CANCELADO";
    default: return "ERROR DESCONOCIDO";
  }
}

void dibujarTitulo(const __FlashStringHelper *titulo) {
  pantalla.setCursor(0, 0);
  pantalla.println(titulo);
  pantalla.drawLine(0, 10, 127, 10, SH110X_WHITE);
}

void mostrarSinPortenta() {
  const EstadoCamaraPublicado camara = copiarEstadoCamara();
  pantalla.clearDisplay();
  dibujarTitulo(F("ESP32 ACTIVA"));
  pantalla.setCursor(0, 16);
  pantalla.println(F("ESPERANDO PORTENTA"));
  pantalla.setCursor(0, 30);
  pantalla.print(F("CAM: "));
  pantalla.println(nombreEstadoCamara(camara.estado));
  pantalla.setCursor(0, 44);
  pantalla.print(F("BT: "));
  pantalla.println(bluetoothConectado ? F("CONECTADO") : F("DESCONECTADO"));
  pantalla.setCursor(0, 56);
  pantalla.print(F("I2C RX: "));
  pantalla.print(rxI2COk);
  pantalla.display();
}

void mostrarCalibracionCamara() {
  const EstadoCamaraPublicado camara = copiarEstadoCamara();
  dibujarTitulo(F("CALIBRACION CAMARA"));
  pantalla.setCursor(0, 14);
  pantalla.print(F("CAM: "));
  pantalla.println(nombreEstadoCamara(camara.estado));

  pantalla.setCursor(0, 28);
  pantalla.print(F("T0:")); pantalla.print(camara.muestras[0]);
  pantalla.print(F(" T1:")); pantalla.print(camara.muestras[1]);
  pantalla.setCursor(0, 40);
  pantalla.print(F("T2:")); pantalla.print(camara.muestras[2]);
  pantalla.print(F(" T3:")); pantalla.print(camara.muestras[3]);

  pantalla.setCursor(0, 54);
  if (camara.error != CAM_ERROR_NINGUNO) {
    pantalla.print(F("ERROR CAM: "));
    pantalla.print(camara.error);
  } else {
    pantalla.print(F("25 MUESTRAS/TAG"));
  }
}

void mostrarMenuPrincipal(const PaquetePortentaAESP &p) {
  dibujarTitulo(F("MENU PRINCIPAL"));
  const char *opciones[4] = {
    "MODO MANUAL",
    "MODO AUTOMATICO",
    "CALIBRACION DE BRAZO",
    "CALIBRACION DE CAMARA"
  };

  for (uint8_t i = 0; i < 4; ++i) {
    pantalla.setCursor(0, 13 + i * 11);
    pantalla.println(opciones[i]);
    if (p.opcionMenu == i) {
      pantalla.drawFastVLine(127, 13 + i * 11, 8, SH110X_WHITE);
    }
  }

  pantalla.setCursor(0, 57);
  pantalla.print(F("X:OK TRI:SALIR"));
}

void mostrarModoManual(const PaquetePortentaAESP &p) {
  dibujarTitulo(F("MODO MANUAL"));
  pantalla.setCursor(0, 14);
  pantalla.print(F("X:"));
  pantalla.print(decodificarMovimientoX(p.movimientos));
  pantalla.print(F(" Y:"));
  pantalla.print(decodificarMovimientoY(p.movimientos));
  pantalla.print(F(" Z:"));
  pantalla.println(decodificarMovimientoZ(p.movimientos));

  pantalla.setCursor(0, 27);
  pantalla.print(F("X+:"));
  pantalla.print((p.flagsLimites & LIM_FLAG_X_MAS) ? F("1 ") : F("0 "));
  pantalla.print(F("X-:"));
  pantalla.print((p.flagsLimites & LIM_FLAG_X_MENOS) ? F("1") : F("0"));
  pantalla.setCursor(0, 39);
  pantalla.print(F("Y+:"));
  pantalla.print((p.flagsLimites & LIM_FLAG_Y_MAS) ? F("1 ") : F("0 "));
  pantalla.print(F("Y-:"));
  pantalla.print((p.flagsLimites & LIM_FLAG_Y_MENOS) ? F("1") : F("0"));

  pantalla.setCursor(0, 51);
  pantalla.print(F("S1:"));
  pantalla.print(anguloServoRotacion);
  pantalla.print(F(" S2:"));
  pantalla.print(anguloServoPinza);
  pantalla.setCursor(98, 51);
  pantalla.print(F("TRI"));
}

void mostrarModoAutomatico(const PaquetePortentaAESP &p) {
  const EstadoCamaraPublicado camara = copiarEstadoCamara();
  dibujarTitulo(F("MODO AUTOMATICO"));
  pantalla.setCursor(0, 12);
  pantalla.print(F("CAM: "));
  pantalla.println(nombreEstadoCamara(camara.estado));

  pantalla.setCursor(0, 23);
  pantalla.print(F("P X:"));
  pantalla.print(p.valorPantalla1 / 10.0f, 1);
  pantalla.print(F(" Y:"));
  pantalla.print(p.valorPantalla2 / 10.0f, 1);

  pantalla.setCursor(0, 34);
  pantalla.print(F("T X:"));
  pantalla.print(p.valorPantalla3 / 10.0f, 1);
  pantalla.print(F(" Y:"));
  pantalla.print(p.valorPantalla4 / 10.0f, 1);

  pantalla.setCursor(0, 45);
  if (p.flagsSistema & SIS_FLAG_BRAZO_OCUPADO) {
    pantalla.print(F("BRAZO MOVIENDO"));
  } else if (camara.objetivoValido) {
    pantalla.print(F("OBJETIVO LISTO"));
  } else {
    pantalla.print(F("ESPERANDO PIEZA"));
  }
  pantalla.setCursor(0, 56);
  pantalla.print(F("TRI:CANCELAR ACK:"));
  pantalla.print(p.ackSecuenciaObjetivo);
}

void actualizarPantallaESP32(const PaquetePortentaAESP &p) {
  if (p.estadoSistema != estadoRemotoAnterior) {
    estadoRemotoAnterior = p.estadoSistema;
    inicioEstadoRemoto = millis();
  }
  pantalla.clearDisplay();

  switch (p.estadoSistema) {
    case SISTEMA_ARRANQUE_SEGURO:
      dibujarTitulo(F("ARRANQUE SEGURO"));
      pantalla.setCursor(0, 18);
      pantalla.println(F("MOTORES DETENIDOS"));
      pantalla.println(F("INICIANDO SISTEMA"));
      break;

    case SISTEMA_ESPERANDO_I2C:
      dibujarTitulo(F("COMUNICACION I2C"));
      pantalla.setCursor(0, 20);
      pantalla.println(F("ESP32 RESPONDIENDO"));
      pantalla.println(F("ESPERANDO PORTENTA"));
      break;

    case SISTEMA_ESPERA_5S:
      dibujarTitulo(F("ENLACE I2C OK"));
      pantalla.setCursor(0, 20);
      pantalla.println(F("ESTABILIZANDO 5 S"));
      pantalla.setCursor(0, 38);
      pantalla.print(F("RESTANTE: "));
      {
        const uint32_t transcurrido = millis() - inicioEstadoRemoto;
        const uint32_t restante = transcurrido < 5000 ? 5000 - transcurrido : 0;
        pantalla.print(restante);
      }
      pantalla.println(F(" ms"));
      break;

    case SISTEMA_CALIBRANDO_CAMARA:
      mostrarCalibracionCamara();
      break;

    case SISTEMA_CALIBRANDO_BRAZO:
      dibujarTitulo(F("CALIBRACION BRAZO"));
      pantalla.setCursor(0, 14);
      pantalla.println(nombreFaseBrazo(p.faseCalibracionBrazo));
      pantalla.setCursor(0, 28);
      pantalla.print(F("X:")); pantalla.print(p.valorPantalla1);
      pantalla.print(F(" Y:")); pantalla.print(p.valorPantalla2);
      pantalla.setCursor(0, 41);
      pantalla.print(F("RX:")); pantalla.print(p.valorPantalla3);
      pantalla.print(F(" RY:")); pantalla.print(p.valorPantalla4);
      pantalla.setCursor(0, 55);
      pantalla.print(F("TRI: CANCELAR"));
      break;

    case SISTEMA_ESPERANDO_CONTROL:
      dibujarTitulo(F("BRAZO EN HOME"));
      pantalla.setCursor(0, 19);
      pantalla.println(F("CALIBRACION OK"));
      pantalla.setCursor(0, 34);
      pantalla.println(F("CONECTE EL CONTROL"));
      break;

    case SISTEMA_CHECKLIST:
      dibujarTitulo(F("CHECKLIST FINAL"));
      pantalla.setCursor(0, 14);
      pantalla.print(F("CAM:"));
      pantalla.print((copiarEstadoCamara().estado == CAMARA_LISTA) ? F("OK ") : F("-- "));
      pantalla.print(F("XY:"));
      pantalla.println((p.flagsSistema & SIS_FLAG_XY_CALIBRADO) ? F("OK") : F("--"));
      pantalla.setCursor(0, 28);
      pantalla.print(F("Z:"));
      pantalla.print((p.flagsSistema & SIS_FLAG_Z_CALIBRADO) ? F("OK ") : F("-- "));
      pantalla.print(F("HOME:"));
      pantalla.println((p.flagsSistema & SIS_FLAG_EN_HOME) ? F("OK") : F("--"));
      pantalla.setCursor(0, 42);
      pantalla.print(F("BT:"));
      pantalla.print(bluetoothConectado ? F("OK ") : F("-- "));
      pantalla.print(F("LIM:"));
      pantalla.println((p.flagsLimites & LIM_FLAG_COHERENTES) ? F("OK") : F("--"));
      pantalla.setCursor(0, 56);
      pantalla.print((p.flagsSistema & SIS_FLAG_CHECKLIST_OK) ? F("TODO CORRECTO") : F("VERIFICANDO..."));
      break;

    case SISTEMA_MENU_PRINCIPAL:
      mostrarMenuPrincipal(p);
      break;

    case SISTEMA_MODO_MANUAL:
      mostrarModoManual(p);
      break;

    case SISTEMA_MODO_AUTOMATICO:
      mostrarModoAutomatico(p);
      break;

    case SISTEMA_ERROR:
      dibujarTitulo(F("ERROR DEL SISTEMA"));
      pantalla.setCursor(0, 17);
      pantalla.println(textoErrorSistema(p.errorSistema));
      pantalla.setCursor(0, 32);
      pantalla.print(F("CODIGO DETALLE: "));
      pantalla.println(p.valorPantalla1);
      pantalla.setCursor(0, 49);
      pantalla.println(F("X O SERIAL: REINT."));
      break;

    default:
      dibujarTitulo(F("ESTADO DESCONOCIDO"));
      pantalla.setCursor(0, 22);
      pantalla.print(F("CODIGO: "));
      pantalla.println(p.estadoSistema);
      break;
  }

  pantalla.display();
}

void procesarPantalla() {
  if (!pantallaInicializada) {
    return;
  }

  const uint32_t ahora = millis();
  if (ahora - ultimaPantalla < PERIODO_PANTALLA_MS) {
    return;
  }
  ultimaPantalla = ahora;

  if (!estadoPortentaValido) {
    mostrarSinPortenta();
    return;
  }
  actualizarPantallaESP32(estadoPortenta);
}

// =============================================================================
// Inicializacion y lazo cooperativo principal.
// =============================================================================

void inicializarSnapshotSeguro() {
  PaqueteESPAPortenta paquete = {};
  paquete.secuenciaPaquete = 0;
  paquete.sesionArranque = sesionArranque;
  paquete.estadoCamara = CAMARA_OFFLINE;
  paquete.errorCamara = CAM_ERROR_NINGUNO;
  paquete.servoRotacion = ANGULO_SERVO_INICIAL;
  paquete.servoPinza = ANGULO_SERVO_INICIAL;
  prepararPaquete(paquete);

  portENTER_CRITICAL(&txI2CMux);
  paqueteTxSnapshot = paquete;
  portEXIT_CRITICAL(&txI2CMux);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("[BOOT] ESP32 integrada iniciando"));

  sesionArranque = static_cast<uint16_t>(esp_random());
  if (sesionArranque == 0) {
    sesionArranque = 1;
  }
  inicializarSnapshotSeguro();

  // Bluetooth es independiente de OLED, Portenta y camara.
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println(F("[BOOT] Bluepad32 configurado"));

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoRotacion.setPeriodHertz(50);
  servoPinza.setPeriodHertz(50);
  servoRotacion.attach(PIN_SERVO_ROTACION, 500, 2400);
  servoPinza.attach(PIN_SERVO_PINZA, 500, 2400);
  servoRotacion.write(anguloServoRotacion);
  servoPinza.write(anguloServoPinza);
  Serial.println(F("[BOOT] Servos GPIO25/GPIO26 configurados"));

  // Se prepara el bus OLED, pero pantalla.begin() queda para loop().
  I2C_Pantalla.setBufferSize(128);
  I2C_Pantalla.setTimeOut(25);
  busPantallaIniciado = I2C_Pantalla.begin(
    OLED_SDA,
    OLED_SCL,
    OLED_I2C_HZ
  );
  Serial.print(F("[BOOT] Bus OLED GPIO21/GPIO22: "));
  Serial.println(busPantallaIniciado ? F("OK") : F("ERROR"));

  // UART2 no consulta la camara; por tanto no bloquea setup().
  HuskyUART.begin(
    HUSKY_BAUDRATE,
    SERIAL_8N1,
    HUSKY_RX_PIN,
    HUSKY_TX_PIN
  );
  Serial.println(F("[BOOT] HUSKYLENS UART2 RX32/TX33 a 115200"));

  const BaseType_t tareaCreada = xTaskCreatePinnedToCore(
    tareaCamara,
    "HUSKYLENS",
    CAMERA_TASK_STACK_BYTES,
    nullptr,
    tskIDLE_PRIORITY,
    &tareaCamaraHandle,
    CAMERA_TASK_CORE
  );

  if (tareaCreada != pdPASS) {
    tareaCamaraHandle = nullptr;
    EstadoCamaraPublicado errorTarea = estadoCamaraPublicado;
    errorTarea.estado = CAMARA_ERROR;
    errorTarea.error = CAM_ERROR_SIN_RESPUESTA;
    portENTER_CRITICAL(&estadoCamaraMux);
    estadoCamaraPublicado = errorTarea;
    portEXIT_CRITICAL(&estadoCamaraMux);
    Serial.println(F("[ERROR] No se pudo crear la tarea HUSKYLENS"));
  }

  // El bus de control se activa al final para que los callbacks nunca observen
  // perifericos a medio inicializar.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Wire.setBufferSize(64);
  i2cEsclavoIniciado = Wire.begin(
    static_cast<uint8_t>(DIRECCION_ESP32),
    I2C_PORTENTA_SDA,
    I2C_PORTENTA_SCL,
    I2C_PORTENTA_HZ
  );

  sistemaBaseListo = true;
  prepararSnapshotI2C();

  Serial.print(F("[BOOT] I2C esclavo 0x40 GPIO27/GPIO14: "));
  Serial.println(i2cEsclavoIniciado ? F("OK") : F("ERROR"));
  Serial.print(F("[BOOT] Sesion ESP: "));
  Serial.println(sesionArranque);
  Serial.println(F("[BOOT] Sistema base listo"));
}

void loop() {
  // Bluepad32 conserva prioridad funcional en cada iteracion.
  BP32.update();
  processControllers();

  procesarRecepcionI2C();
  intentarInicializarOLEDNoBloqueante();
  prepararSnapshotI2C();
  procesarPantalla();

  const uint32_t ahora = millis();
  if (ahora - ultimoReporteI2C >= PERIODO_REPORTE_I2C_MS) {
    ultimoReporteI2C = ahora;
    Serial.print(F("[I2C] rx="));
    Serial.print(rxI2CTotal);
    Serial.print(F(" ok="));
    Serial.print(rxI2COk);
    Serial.print(F(" len="));
    Serial.print(ultimoTamanoRecibido);
    Serial.print(F(" errLen="));
    Serial.print(rxI2CLongitudIncorrecta);
    Serial.print(F(" errCRC="));
    Serial.println(rxI2CProtocoloIncorrecto);
  }

  if (
    tareaCamaraHandle != nullptr &&
    ahora - ultimoReporteStack >= 10000UL
  ) {
    ultimoReporteStack = ahora;
    Serial.print(F("[CAM] Stack libre minimo (words): "));
    Serial.println(uxTaskGetStackHighWaterMark(tareaCamaraHandle));
  }

  // Esta cesion permite Bluetooth, la tarea de camara de prioridad idle y las
  // tareas internas del core ESP32 compartir CPU sin introducir esperas largas.
  delay(1);
}
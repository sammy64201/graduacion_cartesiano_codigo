/**
 * @file ESP.ino
 * @brief Firmware integrado de la ESP32 del brazo cartesiano.
 *
 * Responsabilidades:
 *  - esclavo I2C de la Portenta H7 en el bus de control;
 *  - maestro del bus I2C independiente de la pantalla SH1106;
 *  - adquisicion del control Bluepad32 y control de los dos servos;
 *  - calibracion y deteccion HUSKYLENS 2 por UART1;
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
#include "VisionModelo129.h"

using namespace ProtocoloI2C;

// Declaraciones adelantadas para el generador de prototipos de Arduino. Sin
// ellas, el preprocesador de sketches puede insertar firmas antes de que estas
// estructuras aparezcan en el archivo.
struct Point2D;
struct CandidatoPiezaV2;
struct FiltroDeteccion;
struct FiltroDeteccionV2;
struct ContextoCamara;
struct RegistroLogV2;

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
constexpr uint8_t HUSKY_UART_NUMBER = 1;

// Debe coincidir con la transformacion de la Portenta (swap=false, Y=-1).
// El signo del encoder se define en el eje Y local del brazo, no en el eje
// crudo que entrega la homografia de la camara.
constexpr int8_t CAMERA_SIGNO_Y_LOCAL = -1;
static_assert(CAMERA_SIGNO_Y_LOCAL == 1 || CAMERA_SIGNO_Y_LOCAL == -1,
              "CAMERA_SIGNO_Y_LOCAL debe ser +/-1");

// El encoder de banda esta conectado exclusivamente al canal encoder 0 de la
// Portenta Machine Control. La ESP recibe su muestra por el enlace I2C.

TwoWire I2C_Pantalla(1);
Adafruit_SH1106G pantalla(128, 64, &I2C_Pantalla, -1);
Servo servoRotacion;
Servo servoPinza;
// UART1 es la configuracion comprobada fisicamente con esta HUSKYLENS.
// Los GPIO se reasignan explicitamente en begin(), por lo que no se usan los
// pines UART predeterminados del ESP32.
HardwareSerial HuskyUART(HUSKY_UART_NUMBER);
HuskylensV2 huskylens;

// =============================================================================
// Periodos y limites operativos.
// =============================================================================

constexpr uint32_t PERIODO_SERVO_MS = 5;
constexpr uint32_t PERIODO_PUBLICACION_I2C_MS = 10;
constexpr uint32_t PERIODO_PANTALLA_MS = 100;
constexpr uint32_t PERIODO_REPORTE_I2C_MS = 1000;
constexpr uint32_t TIMEOUT_PORTENTA_MS = 1000;
constexpr uint32_t PERIODO_REINTENTO_I2C_MS = 1000;
constexpr uint32_t TIMEOUT_ACTIVIDAD_I2C_MS = 5000;
constexpr uint32_t PERIODO_REINTENTO_OLED_MS = 1000;
constexpr uint32_t RETARDO_INICIAL_OLED_MS = 1500;

constexpr uint32_t CAM_RECONNECT_MS = 2000;
constexpr uint32_t CAM_UART_STARTUP_MS = 500;
constexpr uint32_t CAM_TAG_LOAD_MS = 3000;
constexpr uint32_t CAM_POST_CALC_MS = 2000;
constexpr uint32_t CAM_MODEL_LOAD_MS = 8000;
constexpr uint32_t CAM_READ_PERIOD_MS = 200;
constexpr uint32_t CAM_READ_PERIOD_V2_MS = 50;
constexpr uint32_t CAM_HEALTH_PERIOD_MS = 1000;
constexpr uint32_t CAM_STATUS_PERIOD_MS = 1000;
constexpr uint32_t CAM_CALIBRATION_TIMEOUT_MS = 120000;

constexpr uint8_t DETECCIONES_ESTABLES = 4;
constexpr uint8_t DETECCIONES_ESTABLES_V2 = 3;
constexpr double TOLERANCIA_ESTABLE_MM = 4.0;
constexpr double TOLERANCIA_TRAYECTORIA_V2_MM = 6.0;
constexpr double DESPLAZAMIENTO_MINIMO_V2_MM = 2.0;
constexpr double DESPLAZAMIENTO_CAMARA_REPETIDA_MM = 8.0;
constexpr double TOLERANCIA_CAMARA_REPETIDA_MM = 2.0;
constexpr double SOLAPE_MINIMO_DUPLICADO = 0.50;
constexpr uint8_t MAX_CANDIDATOS_V2 = 10;
constexpr double TOLERANCIA_REARME_MM = 8.0;
constexpr uint32_t TIEMPO_DESAPARICION_MS = 1000;
constexpr uint32_t TIEMPO_REARME_V2_MS = 500;
constexpr uint32_t TIMEOUT_MUESTRA_ENCODER_MS = 50;
constexpr uint32_t BAUD_LOG_ESP = 460800;
constexpr uint16_t CAPACIDAD_COLA_LOG_V2 = 128;

constexpr int ANGULO_SERVO_INICIAL = 90;
// Ajustar estos dos valores tras comprobar en la maquina la correspondencia
// entre los ejes X/Y del brazo y el eje de cierre de la garra.
constexpr int ANGULO_GARRA_EJE_X = 90;
constexpr int ANGULO_GARRA_EJE_Y = 0;
constexpr double RELACION_MINIMA_ORIENTACION = VisionModelo129::RELACION_MINIMA_EJE;
constexpr int ANGULO_PINZA_ABIERTA = 0;
constexpr int ANGULO_PINZA_CERRADA = 130;
constexpr int PULSO_PINZA_ABIERTA_US = 937;
constexpr int PULSO_PINZA_CERRADA_US = 1816;
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
uint8_t dpadRaw = 0;
bool bluetoothConectado = false;
int anguloServoRotacion = ANGULO_SERVO_INICIAL;
bool ajusteAnguloMLActivo = false;
uint32_t inicioAjusteAnguloML = 0;
uint32_t ultimoCambioAnguloML = 0;
int anguloAntesAjusteML = 0;
uint16_t objetivoAjusteML = 0;
int anguloServoPinza = ANGULO_SERVO_INICIAL;

bool sistemaBaseListo = false;
bool busPantallaIniciado = false;
bool pantallaInicializada = false;
bool i2cEsclavoIniciado = false;
uint32_t ultimoIntentoI2C = 0;
uint32_t inicioInstanciaI2C = 0;
uint32_t reiniciosI2CEsclavo = 0;
uint32_t proximoIntentoOLED = 0;
uint32_t intentosInicioOLED = 0;
uint32_t ultimoUpdateServo = 0;
bool botonCirculoServoAnterior = false;
bool automaticoV2PinzaAnterior = false;
uint16_t ultimaSecuenciaPinzaV2 = 0;
uint8_t ultimoCodigoPinzaV2 = ACK_OBJ_NINGUNO;
uint32_t ultimaPublicacionI2C = 0;
uint32_t ultimaPantalla = 0;
uint32_t ultimoReporteI2C = 0;
uint32_t ultimoReporteStack = 0;
uint32_t ultimoReporteDiagnosticoV2 = 0;
uint32_t ultimoReporteCandidatosV2 = 0;
uint8_t ultimaCausaDiagnosticoV2Reportada = 0xFF;
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
volatile uint32_t solicitudesLecturaI2C = 0;
volatile uint32_t ultimaActividadI2C = 0;

uint8_t secuenciaPaqueteI2C = 0;
uint16_t sesionArranque = 0;

struct ControlCamaraCompartido {
  bool portentaActiva;
  bool automaticoActivo;
  bool automaticoV2Activo;
  bool registrarAngulo;
  bool orientarGarraV2;
  bool entrenamientoML;
  bool entrenamientoMLV2;
  bool pruebaSeguimiento;
  bool pruebaEncoderActiva;
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
  bool objetivoV2;
  int32_t conteoReferenciaObjetivo;
  int16_t sugerenciaAngulo;
};

portMUX_TYPE estadoCamaraMux = portMUX_INITIALIZER_UNLOCKED;
EstadoCamaraPublicado estadoCamaraPublicado = {
  CAMARA_OFFLINE, CAM_ERROR_NINGUNO, 0, {0, 0, 0, 0},
  false, false, false, false, false, 0, 0, 0, 0, false, 0, -1
};

TaskHandle_t tareaCamaraHandle = nullptr;

struct EstadoEncoderCompartido {
  int32_t conteo;
  float velocidadMmS;
  uint32_t nmPorCuenta;
  uint16_t secuencia;
  uint8_t flags;
  int8_t signo;
  uint32_t recibidoMs;
};

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
EstadoEncoderCompartido estadoEncoder = {};

enum CausaDiagnosticoV2 : uint8_t {
  V2_DIAG_SIN_BLOQUEO = 0,
  V2_DIAG_INACTIVO,
  V2_DIAG_BRAZO_OCUPADO,
  V2_DIAG_OBJETIVO_ACTIVO,
  V2_DIAG_NO_REARMADO,
  V2_DIAG_ESPERANDO_DESAPARICION,
  V2_DIAG_ENCODER_NO_RECIBIDO,
  V2_DIAG_ENCODER_ANTIGUO,
  V2_DIAG_ENCODER_HW,
  V2_DIAG_ENCODER_ESCALA,
  V2_DIAG_ENCODER_SIN_PULSOS,
  V2_DIAG_ENCODER_SATURADO,
  V2_DIAG_ENCODER_SIGNO,
  V2_DIAG_HUSKY_ERROR,
  V2_DIAG_SIN_RESULTADOS,
  V2_DIAG_HOMOGRAFIA,
  V2_DIAG_FUERA_BANDA,
  V2_DIAG_SIN_PIEZA,
  V2_DIAG_CAMARA_REPETIDA,
  V2_DIAG_TRAYECTORIA,
  V2_DIAG_DETECCIONES,
  V2_DIAG_DESPLAZAMIENTO,
  V2_DIAG_OBJETIVO_LISTO,
  V2_DIAG_PUBLICADO
};

struct DiagnosticoDeteccionV2 {
  uint8_t causa;
  uint32_t actualizadoMs;
  uint32_t edadEncoderMs;
  int32_t conteoEncoder;
  float velocidadEncoderMmS;
  uint8_t flagsEncoder;
  int8_t signoEncoder;
  int8_t resultadosHusky;
  uint8_t candidatosValidos;
  uint8_t rechazadosHomografia;
  uint8_t rechazadosFueraBanda;
  uint8_t clase;
  uint8_t deteccionesConsecutivas;
  double xMm;
  double yMm;
  double yCompensadaMm;
  double dispersionXmm;
  double dispersionYCompensadaMm;
  double desplazamientoMm;
  uint16_t secuenciaPublicada;
  uint8_t candidatosUnicos;
  uint8_t duplicadosDescartados;
  uint32_t duracionConsultaMs;
  int32_t conteoAntesConsulta;
  int32_t conteoDespuesConsulta;
  int32_t conteoAsociadoCamara;
  double recorridoDuranteConsultaMm;
  int8_t relacionYEncoder;
};

void publicarCausaDiagnosticoV2(
  DiagnosticoDeteccionV2 &diag,
  uint8_t causa
);

portMUX_TYPE diagnosticoV2Mux = portMUX_INITIALIZER_UNLOCKED;
DiagnosticoDeteccionV2 diagnosticoV2 = {
  V2_DIAG_INACTIVO, 0, UINT32_MAX, 0, 0.0f, 0, 0, 0,
  0, 0, 0, 0, 0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0,
  0, 0, 0, 0, 0, 0, 0.0, 0
};

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

EstadoEncoderCompartido copiarEstadoEncoder() {
  EstadoEncoderCompartido copia;
  portENTER_CRITICAL(&encoderMux);
  copia = estadoEncoder;
  portEXIT_CRITICAL(&encoderMux);
  return copia;
}

DiagnosticoDeteccionV2 copiarDiagnosticoV2() {
  DiagnosticoDeteccionV2 copia;
  portENTER_CRITICAL(&diagnosticoV2Mux);
  copia = diagnosticoV2;
  portEXIT_CRITICAL(&diagnosticoV2Mux);
  return copia;
}

void publicarDiagnosticoV2(const DiagnosticoDeteccionV2 &nuevo) {
  portENTER_CRITICAL(&diagnosticoV2Mux);
  diagnosticoV2 = nuevo;
  portEXIT_CRITICAL(&diagnosticoV2Mux);
}

const char *nombreCausaDiagnosticoV2(uint8_t causa) {
  switch (causa) {
    case V2_DIAG_SIN_BLOQUEO: return "SIN_BLOQUEO";
    case V2_DIAG_INACTIVO: return "V2_INACTIVO";
    case V2_DIAG_BRAZO_OCUPADO: return "BRAZO_OCUPADO";
    case V2_DIAG_OBJETIVO_ACTIVO: return "OBJETIVO_ACTIVO";
    case V2_DIAG_NO_REARMADO: return "NO_REARMADO";
    case V2_DIAG_ESPERANDO_DESAPARICION: return "ESPERA_DESAPARICION";
    case V2_DIAG_ENCODER_NO_RECIBIDO: return "ENCODER_NO_RECIBIDO";
    case V2_DIAG_ENCODER_ANTIGUO: return "ENCODER_ANTIGUO";
    case V2_DIAG_ENCODER_HW: return "ENCODER_HW";
    case V2_DIAG_ENCODER_ESCALA: return "ENCODER_ESCALA";
    case V2_DIAG_ENCODER_SIN_PULSOS: return "ENCODER_SIN_PULSOS";
    case V2_DIAG_ENCODER_SATURADO: return "ENCODER_SATURADO";
    case V2_DIAG_ENCODER_SIGNO: return "ENCODER_SIGNO";
    case V2_DIAG_HUSKY_ERROR: return "HUSKY_ERROR";
    case V2_DIAG_SIN_RESULTADOS: return "SIN_RESULTADOS";
    case V2_DIAG_HOMOGRAFIA: return "HOMOGRAFIA";
    case V2_DIAG_FUERA_BANDA: return "FUERA_BANDA";
    case V2_DIAG_SIN_PIEZA: return "SIN_PIEZA";
    case V2_DIAG_CAMARA_REPETIDA: return "CAMARA_REPETIDA";
    case V2_DIAG_TRAYECTORIA: return "TRAYECTORIA";
    case V2_DIAG_DETECCIONES: return "DETECCIONES";
    case V2_DIAG_DESPLAZAMIENTO: return "DESPLAZAMIENTO";
    case V2_DIAG_OBJETIVO_LISTO: return "OBJETIVO_LISTO";
    case V2_DIAG_PUBLICADO: return "PUBLICADO";
    default: return "DESCONOCIDO";
  }
}

const char *nombreCortoDiagnosticoV2(uint8_t causa) {
  switch (causa) {
    case V2_DIAG_INACTIVO: return "INACTIVO";
    case V2_DIAG_BRAZO_OCUPADO: return "BRAZO OCUP.";
    case V2_DIAG_OBJETIVO_ACTIVO: return "OBJ ACTIVO";
    case V2_DIAG_NO_REARMADO: return "NO REARMADO";
    case V2_DIAG_ESPERANDO_DESAPARICION: return "ESPERA DESAP.";
    case V2_DIAG_ENCODER_NO_RECIBIDO: return "ENC SIN RX";
    case V2_DIAG_ENCODER_ANTIGUO: return "ENC ANTIGUO";
    case V2_DIAG_ENCODER_HW: return "ENC HW";
    case V2_DIAG_ENCODER_ESCALA: return "ENC ESCALA";
    case V2_DIAG_ENCODER_SIN_PULSOS: return "ENC SIN PULSO";
    case V2_DIAG_ENCODER_SATURADO: return "ENC SATURADO";
    case V2_DIAG_ENCODER_SIGNO: return "ENC SIGNO";
    case V2_DIAG_HUSKY_ERROR: return "HUSKY ERROR";
    case V2_DIAG_SIN_RESULTADOS: return "SIN RESULTADO";
    case V2_DIAG_HOMOGRAFIA: return "HOMOGRAFIA";
    case V2_DIAG_FUERA_BANDA: return "FUERA BANDA";
    case V2_DIAG_SIN_PIEZA: return "SIN PIEZA";
    case V2_DIAG_CAMARA_REPETIDA: return "CAM REPETIDA";
    case V2_DIAG_TRAYECTORIA: return "TRAYECTORIA";
    case V2_DIAG_DETECCIONES: return "DETECCIONES";
    case V2_DIAG_DESPLAZAMIENTO: return "DESPLAZAM.";
    case V2_DIAG_OBJETIVO_LISTO: return "OBJ LISTO";
    case V2_DIAG_PUBLICADO: return "PUBLICADO";
    default: return "---";
  }
}

uint8_t diagnosticarEncoderV2(const EstadoEncoderCompartido &encoder) {
  if (encoder.recibidoMs == 0) return V2_DIAG_ENCODER_NO_RECIBIDO;
  if (millis() - encoder.recibidoMs > TIMEOUT_MUESTRA_ENCODER_MS)
    return V2_DIAG_ENCODER_ANTIGUO;
  if ((encoder.flags & ENC_FLAG_HW_LISTO) == 0) return V2_DIAG_ENCODER_HW;
  if ((encoder.flags & ENC_FLAG_ESCALA_VALIDA) == 0 || encoder.nmPorCuenta == 0)
    return V2_DIAG_ENCODER_ESCALA;
  if ((encoder.flags & ENC_FLAG_PULSOS_VISTOS) == 0)
    return V2_DIAG_ENCODER_SIN_PULSOS;
  if ((encoder.flags & ENC_FLAG_SATURADO) != 0)
    return V2_DIAG_ENCODER_SATURADO;
  if (encoder.signo != 1 && encoder.signo != -1)
    return V2_DIAG_ENCODER_SIGNO;
  return V2_DIAG_SIN_BLOQUEO;
}

float escalaEncoderMm(const EstadoEncoderCompartido &encoder) {
  return static_cast<float>(encoder.nmPorCuenta) / 1000000.0f;
}

bool encoderRemotoVigente(const EstadoEncoderCompartido &encoder) {
  const uint8_t requeridos = ENC_FLAG_HW_LISTO |
                             ENC_FLAG_ESCALA_VALIDA |
                             ENC_FLAG_PULSOS_VISTOS;
  return encoder.recibidoMs != 0 &&
         millis() - encoder.recibidoMs <= TIMEOUT_MUESTRA_ENCODER_MS &&
         (encoder.flags & requeridos) == requeridos &&
         (encoder.flags & ENC_FLAG_SATURADO) == 0 &&
         encoder.nmPorCuenta != 0 &&
         (encoder.signo == 1 || encoder.signo == -1);
}

void actualizarEncoderDesdePortenta(const PaquetePortentaAESP &paquete) {
  EstadoEncoderCompartido nuevo = {};
  nuevo.conteo = paquete.conteoEncoder;
  nuevo.velocidadMmS = static_cast<float>(paquete.velocidadEncoderUmS) / 1000000.0f;
  nuevo.nmPorCuenta = extraerEscalaEncoderNm(paquete.nmPorCuentaEncoder);
  nuevo.secuencia = paquete.secuenciaEncoder;
  nuevo.flags = paquete.estadoEncoder;
  nuevo.signo = paquete.signoEncoder;
  nuevo.recibidoMs = millis();
  portENTER_CRITICAL(&encoderMux);
  estadoEncoder = nuevo;
  portEXIT_CRITICAL(&encoderMux);
}

void vigilarEncoderRemoto() {
  EstadoEncoderCompartido copia = copiarEstadoEncoder();
  if (copia.recibidoMs != 0 && millis() - copia.recibidoMs > TIMEOUT_PORTENTA_MS) {
    copia.flags = 0;
    portENTER_CRITICAL(&encoderMux);
    estadoEncoder = copia;
    portEXIT_CRITICAL(&encoderMux);
  }
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
  ++solicitudesLecturaI2C;
  ultimaActividadI2C = millis();
  PaqueteESPAPortenta copia;
  portENTER_CRITICAL(&txI2CMux);
  copia = paqueteTxSnapshot;
  portEXIT_CRITICAL(&txI2CMux);
  Wire.write(reinterpret_cast<const uint8_t *>(&copia), sizeof(copia));
}

void receiveEvent(int cantidadBytes) {
  ultimaActividadI2C = millis();
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

bool iniciarI2CEsclavo(bool reinicio) {
  if (i2cEsclavoIniciado) {
    Wire.end();
    i2cEsclavoIniciado = false;
  }

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Wire.setBufferSize(64);
  i2cEsclavoIniciado = Wire.begin(
    static_cast<uint8_t>(DIRECCION_ESP32),
    I2C_PORTENTA_SDA,
    I2C_PORTENTA_SCL,
    I2C_PORTENTA_HZ
  );
  ultimoIntentoI2C = millis();
  inicioInstanciaI2C = ultimoIntentoI2C;
  ultimaActividadI2C = 0;
  if (reinicio) ++reiniciosI2CEsclavo;

  Serial.print(reinicio ? F("[I2C][RECUPERACION] Esclavo reiniciado: ")
                        : F("[BOOT] I2C esclavo 0x40 GPIO27/GPIO14: "));
  Serial.println(i2cEsclavoIniciado ? F("OK") : F("ERROR"));
  return i2cEsclavoIniciado;
}

void mantenerI2CEsclavoRecuperable() {
  const uint32_t ahora = millis();
  if (!i2cEsclavoIniciado) {
    if (ahora - ultimoIntentoI2C >= PERIODO_REINTENTO_I2C_MS) {
      iniciarI2CEsclavo(true);
    }
    return;
  }

  const uint32_t ultima = ultimaActividadI2C;
  const bool nuncaHuboActividad = ultima == 0;
  const uint32_t referencia = nuncaHuboActividad ? inicioInstanciaI2C : ultima;
  if (sistemaBaseListo && ahora - referencia >= TIMEOUT_ACTIVIDAD_I2C_MS &&
      ahora - ultimoIntentoI2C >= PERIODO_REINTENTO_I2C_MS) {
    estadoPortentaValido = false;
    invalidarControlCamaraPorTimeout();
    iniciarI2CEsclavo(true);
  }
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
    if (ctl->b()) botonesControl |= BOTON_CIRCULO;
    if (ctl->y()) botonesControl |= BOTON_TRIANGULO;
    if (ctl->x()) botonesControl |= BOTON_CUADRADO;
    dpadRaw = ctl->dpad();

    const uint32_t ahora = millis();
    const bool circuloPresionado = ctl->b();
    const bool controlRotacionHabilitado = estadoPortentaValido &&
      millis() - ultimoEstadoPortenta <= TIMEOUT_PORTENTA_MS &&
      (estadoPortenta.estadoSistema == SISTEMA_MODO_MANUAL ||
       estadoPortenta.estadoSistema == SISTEMA_PRUEBA_SERVOS ||
        (estadoPortenta.estadoSistema == SISTEMA_ENTRENAMIENTO_ML &&
          estadoPortenta.faseCalibracionBrazo >= 1 &&
           (estadoPortenta.faseCalibracionBrazo <= 3 ||
            (estadoPortenta.opcionMenu == MENU_PRUEBA_SEGUIMIENTO &&
             estadoPortenta.faseCalibracionBrazo == 15))) ||
       (estadoPortenta.estadoSistema == SISTEMA_MODO_AUTOMATICO &&
        estadoPortenta.opcionMenu == MENU_REGISTRO_ANGULO &&
        estadoPortenta.faseCalibracionBrazo <= 3));
    const bool controlManualPinzaHabilitado = estadoPortentaValido &&
      (estadoPortenta.estadoSistema == SISTEMA_MODO_MANUAL ||
       estadoPortenta.estadoSistema == SISTEMA_PRUEBA_SERVOS);
    if (controlManualPinzaHabilitado &&
        circuloPresionado && !botonCirculoServoAnterior) {
      // En Manual y Prueba de servos, circulo alterna directamente entre los
      // dos pulsos fisicamente calibrados. La cruceta ya no gobierna la pinza.
      const bool cerrar = anguloServoPinza != ANGULO_PINZA_CERRADA;
      escribirPinzaCalibrada(cerrar);
    }
    botonCirculoServoAnterior = controlManualPinzaHabilitado
      ? circuloPresionado : false;

    if (controlRotacionHabilitado &&
        ahora - ultimoUpdateServo >= PERIODO_SERVO_MS) {
      ultimoUpdateServo = ahora;
      const int anguloAnterior = anguloServoRotacion;
      if (ejeDerechoX > 150) {
        anguloServoRotacion = min(180, anguloServoRotacion + 1);
      } else if (ejeDerechoX < -150) {
        anguloServoRotacion = max(0, anguloServoRotacion - 1);
      }
      servoRotacion.write(anguloServoRotacion);
      if (estadoPortenta.estadoSistema == SISTEMA_ENTRENAMIENTO_ML &&
          anguloServoRotacion != anguloAnterior) {
        if (!ajusteAnguloMLActivo) {
          ajusteAnguloMLActivo = true;
          inicioAjusteAnguloML = ahora;
          anguloAntesAjusteML = anguloAnterior;
          objetivoAjusteML = estadoPortenta.ackSecuenciaObjetivo;
        }
        ultimoCambioAnguloML = ahora;
      }
    }

  }

  bluetoothConectado = hayControlConectado;
  if (!hayControlConectado) {
    botonCirculoServoAnterior = false;
    ponerControlEnNeutro();
  }
  if (ajusteAnguloMLActivo &&
      (millis() - ultimoCambioAnguloML >= 120 ||
       !hayControlConectado ||
       estadoPortenta.estadoSistema != SISTEMA_ENTRENAMIENTO_ML ||
        (estadoPortenta.faseCalibracionBrazo > 3 &&
         estadoPortenta.faseCalibracionBrazo != 15))) {
    Serial.print(estadoPortenta.opcionMenu == MENU_PRUEBA_SEGUIMIENTO
      ? F("V2LOG|E|mode=ML_TRACK|event=ML_ANGLE_ADJUST|session=")
      : (estadoPortenta.opcionMenu == MENU_ENTRENAMIENTO_ML_V2
          ? F("V2LOG|E|mode=ML_V2|event=ML_ANGLE_ADJUST|session=")
          : F("V2LOG|E|mode=ML|event=ML_ANGLE_ADJUST|session=")));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(ultimoCambioAnguloML);
    Serial.print(F("|obj=")); Serial.print(objetivoAjusteML);
    Serial.print(F("|angle_origin=MANUAL|angle_before="));
    Serial.print(anguloAntesAjusteML);
    Serial.print(F("|angle_after=")); Serial.print(anguloServoRotacion);
    Serial.print(F("|adjust_start_ms=")); Serial.print(inicioAjusteAnguloML);
    Serial.print(F("|adjust_end_ms=")); Serial.print(ultimoCambioAnguloML);
    Serial.print(F("|correction_deg="));
    Serial.println(anguloServoRotacion - anguloAntesAjusteML);
    Serial.print(F("[ML][ANGULO] Manual obj="));
    Serial.print(objetivoAjusteML);
    Serial.print(F(" de ")); Serial.print(anguloAntesAjusteML);
    Serial.print(F(" a ")); Serial.print(anguloServoRotacion);
    Serial.print(F(" grados; ESP ms="));
    Serial.println(ultimoCambioAnguloML);
    ajusteAnguloMLActivo = false;
  }
}

void escribirPinzaCalibrada(bool cerrar) {
  anguloServoPinza = cerrar ? ANGULO_PINZA_CERRADA : ANGULO_PINZA_ABIERTA;
  servoPinza.writeMicroseconds(
    cerrar ? PULSO_PINZA_CERRADA_US : PULSO_PINZA_ABIERTA_US
  );
  Serial.print(F("[PINZA] "));
  Serial.print(cerrar ? F("CERRAR") : F("ABRIR"));
  Serial.print(F(" pulso_us="));
  Serial.println(cerrar ? PULSO_PINZA_CERRADA_US : PULSO_PINZA_ABIERTA_US);
}

void procesarPinzaAutomaticaV2() {
  const bool enlaceVigente = estadoPortentaValido &&
    millis() - ultimoEstadoPortenta <= TIMEOUT_PORTENTA_MS;
  const bool automaticoV2Activo = enlaceVigente && (
    (estadoPortenta.estadoSistema == SISTEMA_MODO_AUTOMATICO_V2 &&
     estadoPortenta.opcionMenu == MENU_MODO_AUTOMATICO_V2) ||
    (estadoPortenta.estadoSistema == SISTEMA_MODO_AUTOMATICO &&
     estadoPortenta.opcionMenu == MENU_REGISTRO_ANGULO) ||
    (estadoPortenta.estadoSistema == SISTEMA_ENTRENAMIENTO_ML &&
     (estadoPortenta.opcionMenu == MENU_ENTRENAMIENTO_ML ||
       estadoPortenta.opcionMenu == MENU_ENTRENAMIENTO_ML_V2 ||
       estadoPortenta.opcionMenu == MENU_PRUEBA_SEGUIMIENTO))
  );

  if (!automaticoV2Activo) {
    automaticoV2PinzaAnterior = false;
    return;
  }

  if (!automaticoV2PinzaAnterior) {
    // Toda entrada/reentrada a V2 empieza con la garra abierta. Esto tambien
    // recupera de forma determinista una sesion anterior interrumpida.
    escribirPinzaCalibrada(false);
    automaticoV2PinzaAnterior = true;
    ultimaSecuenciaPinzaV2 = estadoPortenta.ackSecuenciaObjetivo;
    ultimoCodigoPinzaV2 = estadoPortenta.codigoAckObjetivo;
    return;
  }

  const bool ordenNueva =
    estadoPortenta.ackSecuenciaObjetivo != ultimaSecuenciaPinzaV2 ||
    estadoPortenta.codigoAckObjetivo != ultimoCodigoPinzaV2;
  if (!ordenNueva) return;

  ultimaSecuenciaPinzaV2 = estadoPortenta.ackSecuenciaObjetivo;
  ultimoCodigoPinzaV2 = estadoPortenta.codigoAckObjetivo;
  if (ultimoCodigoPinzaV2 == ACK_OBJ_CERRAR_PINZA) {
    escribirPinzaCalibrada(true);
    if (estadoPortenta.estadoSistema == SISTEMA_ENTRENAMIENTO_ML) {
      Serial.print(estadoPortenta.opcionMenu == MENU_PRUEBA_SEGUIMIENTO
        ? F("V2LOG|E|mode=ML_TRACK|event=ML_GRIP_APPLIED|session=")
        : (estadoPortenta.opcionMenu == MENU_ENTRENAMIENTO_ML_V2
            ? F("V2LOG|E|mode=ML_V2|event=ML_GRIP_APPLIED|session=")
            : F("V2LOG|E|mode=ML|event=ML_GRIP_APPLIED|session=")));
      Serial.print(sesionArranque);
      Serial.print(F("|ms=")); Serial.print(millis());
      Serial.print(F("|obj=")); Serial.print(ultimaSecuenciaPinzaV2);
      Serial.print(F("|servo_rot_deg=")); Serial.print(anguloServoRotacion);
      Serial.print(F("|grip_deg=")); Serial.print(anguloServoPinza);
      Serial.print(F("|pulse_us=")); Serial.println(PULSO_PINZA_CERRADA_US);
    }
    Serial.print(F("ML_GRIP_APPLIED|seq="));
    Serial.print(ultimaSecuenciaPinzaV2);
    Serial.print(F("|servo_rot_deg="));
    Serial.print(anguloServoRotacion);
    Serial.print(F("|grip_deg="));
    Serial.print(anguloServoPinza);
    Serial.print(F("|pulse_us="));
    Serial.print(PULSO_PINZA_CERRADA_US);
    Serial.print(F("|esp_ms="));
    Serial.println(millis());
    if (estadoPortenta.estadoSistema == SISTEMA_MODO_AUTOMATICO &&
        estadoPortenta.opcionMenu == MENU_REGISTRO_ANGULO) {
      const EstadoCamaraPublicado camara = copiarEstadoCamara();
      Serial.print(F("V2LOG|E|mode=ANGLE_LABEL|event=ANGLE_FEEDBACK|session="));
      Serial.print(sesionArranque);
      Serial.print(F("|obj=")); Serial.print(ultimaSecuenciaPinzaV2);
      Serial.print(F("|suggested_rot="));
      if (camara.sugerenciaAngulo >= 0) {
        Serial.print(camara.sugerenciaAngulo);
      } else {
        Serial.print(F("NA"));
      }
      Serial.print(F("|servo_rot_deg=")); Serial.print(anguloServoRotacion);
      Serial.print(F("|correction_deg="));
      if (camara.sugerenciaAngulo >= 0) {
        Serial.print(abs(anguloServoRotacion - camara.sugerenciaAngulo));
      } else {
        Serial.print(F("NA"));
      }
      Serial.println();
    }
  } else if (ultimoCodigoPinzaV2 == ACK_OBJ_CANCELADO ||
             ultimoCodigoPinzaV2 == ACK_OBJ_COMPLETADO) {
    // CANCELADO abre de inmediato; COMPLETADO llega cuando Z ya regreso a la
    // posicion segura, dejando la garra lista para la siguiente pieza.
    escribirPinzaCalibrada(false);
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
  nuevo.pruebaEncoderActiva = paquete.estadoSistema == SISTEMA_PRUEBA_ENCODER &&
    paquete.opcionMenu == MENU_PRUEBA_ENCODER;
  // Automatico V2 y Ensenanza ML comparten el detector de piezas en
  // movimiento: ambos necesitan una deteccion estable y una instantanea del
  // encoder tomada en el mismo momento que las coordenadas de camara.
  nuevo.automaticoV2Activo = (nuevo.automaticoActivo &&
    (paquete.opcionMenu == MENU_MODO_AUTOMATICO_V2 ||
     paquete.opcionMenu == MENU_ENTRENAMIENTO_ML ||
      paquete.opcionMenu == MENU_ENTRENAMIENTO_ML_V2 ||
      paquete.opcionMenu == MENU_PRUEBA_SEGUIMIENTO));
  nuevo.registrarAngulo = nuevo.automaticoActivo &&
    paquete.estadoSistema == SISTEMA_MODO_AUTOMATICO &&
    paquete.opcionMenu == MENU_REGISTRO_ANGULO;
  nuevo.orientarGarraV2 = nuevo.automaticoV2Activo &&
    paquete.estadoSistema == SISTEMA_MODO_AUTOMATICO_V2 &&
    paquete.opcionMenu == MENU_MODO_AUTOMATICO_V2;
  nuevo.entrenamientoML = nuevo.automaticoV2Activo &&
    paquete.estadoSistema == SISTEMA_ENTRENAMIENTO_ML &&
    (paquete.opcionMenu == MENU_ENTRENAMIENTO_ML ||
      paquete.opcionMenu == MENU_ENTRENAMIENTO_ML_V2 ||
      paquete.opcionMenu == MENU_PRUEBA_SEGUIMIENTO);
  nuevo.entrenamientoMLV2 = nuevo.entrenamientoML &&
    (paquete.opcionMenu == MENU_ENTRENAMIENTO_ML_V2 ||
     paquete.opcionMenu == MENU_PRUEBA_SEGUIMIENTO);
  nuevo.pruebaSeguimiento = nuevo.entrenamientoML &&
    paquete.opcionMenu == MENU_PRUEBA_SEGUIMIENTO;
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
  controlCamara.automaticoV2Activo = false;
  controlCamara.registrarAngulo = false;
  controlCamara.entrenamientoML = false;
  controlCamara.entrenamientoMLV2 = false;
  controlCamara.pruebaSeguimiento = false;
  controlCamara.brazoOcupado = false;
  controlCamara.comando = CAM_CMD_NINGUNO;
  controlCamara.codigoAckObjetivo = ACK_OBJ_NINGUNO;
  portEXIT_CRITICAL(&controlCamaraMux);
}

bool paquetePortentaSemanticamenteValido(const PaquetePortentaAESP &p) {
  const uint8_t mascaraEncoder = ENC_FLAG_HW_LISTO |
                                 ENC_FLAG_ESCALA_VALIDA |
                                 ENC_FLAG_PULSOS_VISTOS |
                                 ENC_FLAG_EN_MOVIMIENTO |
                                 ENC_FLAG_DIRECCION_POSITIVA |
                                 ENC_FLAG_SATURADO;
  const bool flagsValidos =
    (p.estadoEncoder & static_cast<uint8_t>(~mascaraEncoder)) == 0;
  const bool signoValido = p.signoEncoder == 1 || p.signoEncoder == -1;
  const bool escalaCoherente =
    (p.estadoEncoder & ENC_FLAG_ESCALA_VALIDA) != 0
      ? extraerEscalaEncoderNm(p.nmPorCuentaEncoder) != 0
      : extraerEscalaEncoderNm(p.nmPorCuentaEncoder) == 0;
  const bool ackValido = p.codigoAckObjetivo <= ACK_OBJ_CERRAR_PINZA;
  return flagsValidos && signoValido && escalaCoherente && ackValido;
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
    } else if (!validarPaquete(copia.paquete) ||
               !paquetePortentaSemanticamenteValido(copia.paquete)) {
      ++rxI2CProtocoloIncorrecto;
    } else {
      estadoPortenta = copia.paquete;
      estadoPortentaValido = true;
      ultimoEstadoPortenta = millis();
      ++rxI2COk;
      actualizarEncoderDesdePortenta(copia.paquete);
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
  const EstadoEncoderCompartido encoder = copiarEstadoEncoder();
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
  empacarMuestrasTags(camara.muestras, paquete.muestrasTagEmpacadas);
  paquete.reservadoV2 = 0;
  paquete.claseObjetivo = camara.claseObjetivo;
  paquete.objetivoX10 = camara.objetivoX10;
  paquete.objetivoY10 = camara.objetivoY10;
  paquete.secuenciaObjetivo = camara.secuenciaObjetivo;
  paquete.conteoReferenciaObjetivo = camara.conteoReferenciaObjetivo;
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
// Modelo actualizado: indice 1, algoritmo 129.
constexpr uint8_t CUSTOM_MODEL_INDEX = VisionModelo129::INDICE_MODELO;

const eAlgorithm_t PIECE_MODEL = static_cast<eAlgorithm_t>(
  static_cast<uint8_t>(ALGORITHM_CUSTOM_BEGIN) + CUSTOM_MODEL_INDEX
);

struct Point2D {
  double x;
  double y;
};

struct CandidatoPiezaV2 {
  uint8_t clase;
  int8_t confianza;
  int16_t centroXpx;
  int16_t centroYpx;
  int16_t anchoPx;
  int16_t altoPx;
  Point2D posicion;
  uint8_t grupoDuplicado;
};

enum TipoRegistroLogV2 : uint8_t {
  LOG_V2_CONSULTA = 0,
  LOG_V2_CANDIDATO = 1
};

// La tarea de camara nunca imprime el registro estructurado directamente.
// Deposita estructuras de tamano fijo y loop() las serializa fuera del camino
// critico de HUSKYLENS. El conteo de perdidos permite detectar saturacion sin
// bloquear la adquisicion.
struct RegistroLogV2 {
  bool pruebaEncoder;
  uint8_t tipo;
  uint8_t causa;
  uint8_t indiceCandidato;
  uint8_t grupoCandidato;
  uint8_t grupoSeleccionado;
  uint32_t tiempoMs;
  uint32_t consulta;
  uint16_t sesion;
  uint16_t secuenciaObjetivo;
  uint16_t secuenciaEncoder;
  int32_t conteoEncoder;
  uint32_t edadEncoderMs;
  float velocidadEncoderMmS;
  uint8_t flagsEncoder;
  int8_t resultadosHusky;
  uint8_t candidatosValidos;
  uint8_t candidatosUnicos;
  uint8_t duplicadosDescartados;
  uint8_t rechazadosHomografia;
  uint8_t rechazadosFueraBanda;
  uint8_t clase;
  int8_t confianza;
  uint8_t deteccionesConsecutivas;
  int16_t centroXpx;
  int16_t centroYpx;
  int16_t anchoPx;
  int16_t altoPx;
  float xMm;
  float yMm;
  float yCompensadaMm;
  float dispersionXmm;
  float dispersionYmm;
  float desplazamientoMm;
  int8_t relacionYEncoder;
  uint32_t duracionConsultaMs;
  float recorridoDuranteConsultaMm;
  int32_t conteoAntesConsulta;
  int32_t conteoDespuesConsulta;
};

portMUX_TYPE colaLogV2Mux = portMUX_INITIALIZER_UNLOCKED;
RegistroLogV2 colaLogV2[CAPACIDAD_COLA_LOG_V2] = {};
volatile uint16_t cabezaColaLogV2 = 0;
volatile uint16_t colaColaLogV2 = 0;
volatile uint32_t registrosLogV2Perdidos = 0;
uint32_t consultasLogV2 = 0;

void encolarRegistroLogV2(const RegistroLogV2 &registro) {
  portENTER_CRITICAL(&colaLogV2Mux);
  const uint16_t siguiente = static_cast<uint16_t>(
    (cabezaColaLogV2 + 1U) % CAPACIDAD_COLA_LOG_V2
  );
  if (siguiente == colaColaLogV2) {
    ++registrosLogV2Perdidos;
  } else {
    colaLogV2[cabezaColaLogV2] = registro;
    cabezaColaLogV2 = siguiente;
  }
  portEXIT_CRITICAL(&colaLogV2Mux);
}

void registrarCandidatoLogV2(
  uint32_t consulta,
  const EstadoEncoderCompartido &encoder,
  const CandidatoPiezaV2 &candidato,
  uint8_t indice,
  uint8_t causa
) {
  RegistroLogV2 registro = {};
  registro.tipo = LOG_V2_CANDIDATO;
  registro.pruebaEncoder = copiarControlCamara().pruebaEncoderActiva;
  registro.causa = causa;
  registro.indiceCandidato = indice;
  registro.grupoCandidato = candidato.grupoDuplicado;
  registro.grupoSeleccionado = UINT8_MAX;
  registro.tiempoMs = millis();
  registro.consulta = consulta;
  registro.sesion = sesionArranque;
  registro.secuenciaEncoder = encoder.secuencia;
  registro.conteoEncoder = encoder.conteo;
  registro.edadEncoderMs = encoder.recibidoMs == 0
    ? UINT32_MAX : registro.tiempoMs - encoder.recibidoMs;
  registro.velocidadEncoderMmS = encoder.velocidadMmS;
  registro.flagsEncoder = encoder.flags;
  registro.clase = candidato.clase;
  registro.confianza = candidato.confianza;
  registro.centroXpx = candidato.centroXpx;
  registro.centroYpx = candidato.centroYpx;
  registro.anchoPx = candidato.anchoPx;
  registro.altoPx = candidato.altoPx;
  registro.xMm = static_cast<float>(candidato.posicion.x);
  registro.yMm = static_cast<float>(candidato.posicion.y);
  encolarRegistroLogV2(registro);
}

void registrarConsultaLogV2(
  const DiagnosticoDeteccionV2 &diag,
  const EstadoEncoderCompartido &encoder,
  uint32_t consulta,
  uint8_t grupoSeleccionado
) {
  RegistroLogV2 registro = {};
  registro.tipo = LOG_V2_CONSULTA;
  registro.pruebaEncoder = copiarControlCamara().pruebaEncoderActiva;
  registro.causa = diag.causa;
  registro.grupoSeleccionado = grupoSeleccionado;
  registro.tiempoMs = diag.actualizadoMs;
  registro.consulta = consulta;
  registro.sesion = sesionArranque;
  registro.secuenciaObjetivo = diag.secuenciaPublicada;
  registro.secuenciaEncoder = encoder.secuencia;
  registro.conteoEncoder = diag.conteoAsociadoCamara;
  registro.edadEncoderMs = diag.edadEncoderMs;
  registro.velocidadEncoderMmS = diag.velocidadEncoderMmS;
  registro.flagsEncoder = diag.flagsEncoder;
  registro.resultadosHusky = diag.resultadosHusky;
  registro.candidatosValidos = diag.candidatosValidos;
  registro.candidatosUnicos = diag.candidatosUnicos;
  registro.duplicadosDescartados = diag.duplicadosDescartados;
  registro.rechazadosHomografia = diag.rechazadosHomografia;
  registro.rechazadosFueraBanda = diag.rechazadosFueraBanda;
  registro.clase = diag.clase;
  registro.deteccionesConsecutivas = diag.deteccionesConsecutivas;
  registro.xMm = static_cast<float>(diag.xMm);
  registro.yMm = static_cast<float>(diag.yMm);
  registro.yCompensadaMm = static_cast<float>(diag.yCompensadaMm);
  registro.dispersionXmm = static_cast<float>(diag.dispersionXmm);
  registro.dispersionYmm = static_cast<float>(diag.dispersionYCompensadaMm);
  registro.desplazamientoMm = static_cast<float>(diag.desplazamientoMm);
  registro.relacionYEncoder = diag.relacionYEncoder;
  registro.duracionConsultaMs = diag.duracionConsultaMs;
  registro.recorridoDuranteConsultaMm =
    static_cast<float>(diag.recorridoDuranteConsultaMm);
  registro.conteoAntesConsulta = diag.conteoAntesConsulta;
  registro.conteoDespuesConsulta = diag.conteoDespuesConsulta;
  encolarRegistroLogV2(registro);
}

void publicarYRegistrarDiagnosticoV2(
  DiagnosticoDeteccionV2 &diag,
  uint8_t causa,
  const EstadoEncoderCompartido &encoder,
  uint32_t consulta,
  uint8_t grupoSeleccionado = UINT8_MAX
) {
  publicarCausaDiagnosticoV2(diag, causa);
  registrarConsultaLogV2(diag, encoder, consulta, grupoSeleccionado);
}

bool extraerRegistroLogV2(RegistroLogV2 &registro) {
  bool disponible = false;
  portENTER_CRITICAL(&colaLogV2Mux);
  if (colaColaLogV2 != cabezaColaLogV2) {
    registro = colaLogV2[colaColaLogV2];
    colaColaLogV2 = static_cast<uint16_t>(
      (colaColaLogV2 + 1U) % CAPACIDAD_COLA_LOG_V2
    );
    disponible = true;
  }
  portEXIT_CRITICAL(&colaLogV2Mux);
  return disponible;
}

void imprimirRegistroLogV2(const RegistroLogV2 &r) {
  Serial.print(F("V2LOG|E|ms=")); Serial.print(r.tiempoMs);
  Serial.print(F("|mode=")); Serial.print(r.pruebaEncoder ? F("ENCODER_TEST") : F("V2"));
  Serial.print(F("|session=")); Serial.print(r.sesion);
  Serial.print(F("|event="));
  Serial.print(r.tipo == LOG_V2_CONSULTA ? F("QUERY") : F("CANDIDATE"));
  Serial.print(F("|query=")); Serial.print(r.consulta);
  Serial.print(F("|obj=")); Serial.print(r.secuenciaObjetivo);
  Serial.print(F("|encseq=")); Serial.print(r.secuenciaEncoder);
  Serial.print(F("|enc=")); Serial.print(r.conteoEncoder);
  Serial.print(F("|age="));
  if (r.edadEncoderMs == UINT32_MAX) Serial.print(F("NA"));
  else Serial.print(r.edadEncoderMs);
  Serial.print(F("|vel=")); Serial.print(r.velocidadEncoderMmS, 3);
  Serial.print(F("|flags=")); Serial.print(r.flagsEncoder);
  Serial.print(F("|cause=")); Serial.print(nombreCausaDiagnosticoV2(r.causa));

  if (r.tipo == LOG_V2_CANDIDATO) {
    Serial.print(F("|candidate=")); Serial.print(r.indiceCandidato);
    Serial.print(F("|group=")); Serial.print(r.grupoCandidato);
    Serial.print(F("|class=")); Serial.print(r.clase);
    Serial.print(F("|confidence=")); Serial.print(r.confianza);
    Serial.print(F("|px=")); Serial.print(r.centroXpx);
    Serial.print(F("|py=")); Serial.print(r.centroYpx);
    Serial.print(F("|width=")); Serial.print(r.anchoPx);
    Serial.print(F("|height=")); Serial.print(r.altoPx);
    Serial.print(F("|x=")); Serial.print(r.xMm, 3);
    Serial.print(F("|y=")); Serial.print(r.yMm, 3);
  } else {
    Serial.print(F("|husky=")); Serial.print(r.resultadosHusky);
    Serial.print(F("|valid=")); Serial.print(r.candidatosValidos);
    Serial.print(F("|unique=")); Serial.print(r.candidatosUnicos);
    Serial.print(F("|dup=")); Serial.print(r.duplicadosDescartados);
    Serial.print(F("|rej_h=")); Serial.print(r.rechazadosHomografia);
    Serial.print(F("|rej_b=")); Serial.print(r.rechazadosFueraBanda);
    Serial.print(F("|selected=")); Serial.print(r.grupoSeleccionado);
    Serial.print(F("|class=")); Serial.print(r.clase);
    Serial.print(F("|n=")); Serial.print(r.deteccionesConsecutivas);
    Serial.print(F("|x=")); Serial.print(r.xMm, 3);
    // El encoder desplaza la pieza solo sobre Y; por eso X compensada coincide
    // con X fisica, pero se publica explicitamente para mantener el CSV fijo.
    Serial.print(F("|xc=")); Serial.print(r.xMm, 3);
    Serial.print(F("|y=")); Serial.print(r.yMm, 3);
    Serial.print(F("|yc=")); Serial.print(r.yCompensadaMm, 3);
    Serial.print(F("|dx=")); Serial.print(r.dispersionXmm, 3);
    Serial.print(F("|dy=")); Serial.print(r.dispersionYmm, 3);
    Serial.print(F("|travel=")); Serial.print(r.desplazamientoMm, 3);
    Serial.print(F("|rel_y=")); Serial.print(r.relacionYEncoder);
    Serial.print(F("|query_ms=")); Serial.print(r.duracionConsultaMs);
    Serial.print(F("|query_travel="));
    Serial.print(r.recorridoDuranteConsultaMm, 3);
    Serial.print(F("|enc_before=")); Serial.print(r.conteoAntesConsulta);
    Serial.print(F("|enc_after=")); Serial.print(r.conteoDespuesConsulta);
  }
  Serial.println();
}

void vaciarColaLogV2() {
  uint32_t perdidos = 0;
  portENTER_CRITICAL(&colaLogV2Mux);
  perdidos = registrosLogV2Perdidos;
  registrosLogV2Perdidos = 0;
  portEXIT_CRITICAL(&colaLogV2Mux);
  if (perdidos != 0) {
    Serial.print(F("V2LOG|E|ms=")); Serial.print(millis());
    Serial.print(F("|session=")); Serial.print(sesionArranque);
    Serial.print(F("|event=DROP|lost=")); Serial.println(perdidos);
  }

  RegistroLogV2 registro = {};
  for (uint8_t i = 0; i < 8 && extraerRegistroLogV2(registro); ++i) {
    imprimirRegistroLogV2(registro);
  }
}

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

// La caja del modelo es paralela a los ejes de imagen. Solo se distingue el
// eje dominante en el plano calibrado; no se inventa un angulo diagonal.
uint8_t estimarEjeCajaV2(const CandidatoPiezaV2 &candidato) {
  if (candidato.anchoPx <= 0 || candidato.altoPx <= 0) return 0;
  const double izquierda = candidato.centroXpx - candidato.anchoPx * 0.5;
  const double derecha = candidato.centroXpx + candidato.anchoPx * 0.5;
  const double arriba = candidato.centroYpx - candidato.altoPx * 0.5;
  const double abajo = candidato.centroYpx + candidato.altoPx * 0.5;
  Point2D esquinas[4];
  if (!pixelToMillimeters(izquierda, arriba, true, esquinas[0]) ||
      !pixelToMillimeters(derecha, arriba, true, esquinas[1]) ||
      !pixelToMillimeters(izquierda, abajo, true, esquinas[2]) ||
      !pixelToMillimeters(derecha, abajo, true, esquinas[3])) return 0;
  double minX = esquinas[0].x, maxX = minX;
  double minY = esquinas[0].y, maxY = minY;
  for (uint8_t i = 1; i < 4; ++i) {
    minX = fmin(minX, esquinas[i].x);
    maxX = fmax(maxX, esquinas[i].x);
    minY = fmin(minY, esquinas[i].y);
    maxY = fmax(maxY, esquinas[i].y);
  }
  const double anchoMm = maxX - minX;
  const double altoMm = maxY - minY;
  return VisionModelo129::ejePorDimensiones(
    anchoMm, altoMm, RELACION_MINIMA_ORIENTACION);
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
  uint8_t votosEjeX;
  uint8_t votosEjeY;
  int16_t centroXpx;
  int16_t centroYpx;
  int16_t anchoPx;
  int16_t altoPx;
  int8_t confianza;
};

struct FiltroDeteccionV2 {
  uint8_t clase;
  uint8_t consecutivas;
  int32_t conteoReferencia;
  int32_t ultimoConteo;
  double sumaX;
  double sumaYCompensada;
  double promedioX;
  double promedioYCompensada;
  double minimoX;
  double maximoX;
  double minimoYCompensada;
  double maximoYCompensada;
  int8_t relacionYEncoder;
  int16_t ultimoAnchoPx;
  int16_t ultimoAltoPx;
  double ultimoXRaw;
  double ultimoYRaw;
  uint8_t votosEjeX;
  uint8_t votosEjeY;
  uint32_t primerMs;
  uint32_t ultimoMs;
  double primerYRaw;
  uint8_t muestrasYDistintas;
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
  FiltroDeteccionV2 filtroV2;
  uint8_t ausenciasFiltroV2;
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
  bool objetivoV2;
  int32_t conteoReferenciaObjetivo;
  int16_t sugerenciaAngulo;
  bool pruebaEncoderAnterior;
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
  publicado.objetivoV2 = ctx.objetivoV2;
  publicado.sugerenciaAngulo = ctx.sugerenciaAngulo;
  publicado.conteoReferenciaObjetivo = ctx.conteoReferenciaObjetivo;

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
  ctx.filtro = {};
}

void orientarGarraAutomatica(uint8_t votosX, uint8_t votosY, bool modoV2) {
  const bool ejeX = votosX >= 2 && votosY == 0;
  const bool ejeY = votosY >= 2 && votosX == 0;
  anguloServoRotacion = ejeX ? ANGULO_GARRA_EJE_X :
    (ejeY ? ANGULO_GARRA_EJE_Y : ANGULO_SERVO_INICIAL);
  servoRotacion.write(anguloServoRotacion);
  Serial.print(modoV2 ? F("[AUTO V2]") : F("[AUTO]"));
  Serial.print(F(" Orientacion aprox eje="));
  Serial.print(ejeX ? F("X") : (ejeY ? F("Y") : F("INDETERMINADO")));
  Serial.print(F(" votos_x="));
  Serial.print(votosX);
  Serial.print(F(" votos_y="));
  Serial.print(votosY);
  Serial.print(F(" servo_deg="));
  Serial.println(anguloServoRotacion);
}

void reiniciarFiltroV2(ContextoCamara &ctx) {
  ctx.filtroV2 = {
    0, 0, 0, 0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0, 0, 0, 0.0, 0.0, 0, 0
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
  ctx.objetivoV2 = false;
  ctx.conteoReferenciaObjetivo = 0;
  ctx.sugerenciaAngulo = -1;
  reiniciarFiltro(ctx);
  reiniciarFiltroV2(ctx);
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
  bool brazoOcupado,
  uint32_t tiempoRearmeMs,
  bool automaticoV2
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

  if (ahora - ctx.inicioAusencia >= tiempoRearmeMs) {
    ctx.esperandoDesaparicion = false;
    ctx.inicioAusencia = 0;
    ctx.rearmada = true;
    ctx.claseEsperandoDesaparicion = 0;
    ctx.xEsperandoDesaparicion = 0.0;
    ctx.yEsperandoDesaparicion = 0.0;
    reiniciarFiltro(ctx);
    if (automaticoV2) {
      Serial.println(
        F("[AUTO V2] Detector liberado tras 500 ms; nueva secuencia permitida")
      );
    } else {
      Serial.println(F("[AUTO] Detector rearmado tras desaparicion"));
    }
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
  const Point2D &posicion,
  uint8_t ejeCaja,
  const CandidatoPiezaV2 &caja
) {
  FiltroDeteccion &filtro = ctx.filtro;
  filtro.centroXpx = caja.centroXpx;
  filtro.centroYpx = caja.centroYpx;
  filtro.anchoPx = caja.anchoPx;
  filtro.altoPx = caja.altoPx;
  filtro.confianza = caja.confianza;
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
    filtro.votosEjeX = static_cast<uint8_t>(ejeCaja == 1);
    filtro.votosEjeY = static_cast<uint8_t>(ejeCaja == 2);
    return;
  }

  if (filtro.consecutivas < UINT8_MAX) {
    ++filtro.consecutivas;
  }
  if (ejeCaja == 1 && filtro.votosEjeX < UINT8_MAX) ++filtro.votosEjeX;
  if (ejeCaja == 2 && filtro.votosEjeY < UINT8_MAX) ++filtro.votosEjeY;
  filtro.sumaX += posicion.x;
  filtro.sumaY += posicion.y;
  filtro.promedioX = filtro.sumaX / filtro.consecutivas;
  filtro.promedioY = filtro.sumaY / filtro.consecutivas;
  filtro.minimoX = fmin(filtro.minimoX, posicion.x);
  filtro.maximoX = fmax(filtro.maximoX, posicion.x);
  filtro.minimoY = fmin(filtro.minimoY, posicion.y);
  filtro.maximoY = fmax(filtro.maximoY, posicion.y);
}

// El modelo 129 no transmite giro. Usar el consenso de cajas en el plano
// calibrado tambien al registrar/ensenar; sin consenso conservar giro manual.
int16_t sugerirAnguloPorVotos(uint8_t votosX, uint8_t votosY) {
  if (votosX >= 2 && votosY == 0) return ANGULO_GARRA_EJE_X;
  if (votosY >= 2 && votosX == 0) return ANGULO_GARRA_EJE_Y;
  return -1;
}

int16_t sugerirAnguloRegistro(const FiltroDeteccion &filtro) {
  return sugerirAnguloPorVotos(filtro.votosEjeX, filtro.votosEjeY);
}

void publicarObjetivoEstable(ContextoCamara &ctx, bool registrarAngulo) {
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
  // El modo de etiquetado conserva la pieza hasta confirmar el angulo.
  ctx.objetivoV2 = registrarAngulo;
  ctx.conteoReferenciaObjetivo = 0;
  ctx.sugerenciaAngulo = registrarAngulo
    ? sugerirAnguloRegistro(ctx.filtro) : -1;
  ctx.rearmada = false;
  if (registrarAngulo) {
    if (ctx.sugerenciaAngulo >= 0) {
      anguloServoRotacion = ctx.sugerenciaAngulo;
      servoRotacion.write(anguloServoRotacion);
    }
    Serial.print(F("V2LOG|E|mode=ANGLE_LABEL|event=ANGLE_DETECTION|session="));
    Serial.print(sesionArranque);
    Serial.print(F("|obj=")); Serial.print(ctx.secuenciaObjetivo);
    Serial.print(F("|class=")); Serial.print(ctx.claseObjetivo);
    Serial.print(F("|cam_x=")); Serial.print(ctx.objetivoX10 / 10.0f, 1);
    Serial.print(F("|cam_y=")); Serial.print(ctx.objetivoY10 / 10.0f, 1);
    Serial.print(F("|px=")); Serial.print(ctx.filtro.centroXpx);
    Serial.print(F("|py=")); Serial.print(ctx.filtro.centroYpx);
    Serial.print(F("|width=")); Serial.print(ctx.filtro.anchoPx);
    Serial.print(F("|height=")); Serial.print(ctx.filtro.altoPx);
    Serial.print(F("|aspect_ratio="));
    if (ctx.filtro.altoPx > 0) {
      Serial.print(static_cast<float>(ctx.filtro.anchoPx) /
                   static_cast<float>(ctx.filtro.altoPx), 3);
    } else {
      Serial.print(F("NA"));
    }
    Serial.print(F("|suggested_rot="));
    if (ctx.sugerenciaAngulo >= 0) Serial.print(ctx.sugerenciaAngulo);
    else Serial.print(F("NA"));
    Serial.print(F("|suggestion_source=MODEL129_BOX_AXIS_MM"));
    // El modelo personalizado entrega -128 en el byte compartido con rfu1;
    // no presentarlo como una confianza de deteccion utilizable.
    Serial.print(F("|confidence="));
    if (ctx.filtro.confianza >= 0) Serial.print(ctx.filtro.confianza);
    else Serial.print(F("NA"));
    Serial.print(F("|n=")); Serial.print(ctx.filtro.consecutivas);
    Serial.print(F("|votes_x=")); Serial.print(ctx.filtro.votosEjeX);
    Serial.print(F("|votes_y=")); Serial.print(ctx.filtro.votosEjeY);
    const uint8_t anguloAproximado =
      ctx.filtro.votosEjeX >= 2 &&
      ctx.filtro.votosEjeY == 0
        ? ANGULO_GARRA_EJE_X
        : (ctx.filtro.votosEjeY >= 2 &&
           ctx.filtro.votosEjeX == 0
            ? ANGULO_GARRA_EJE_Y : ANGULO_SERVO_INICIAL);
    Serial.print(F("|approx_rot=")); Serial.print(anguloAproximado);
    Serial.print(F("|servo_rot_deg=")); Serial.println(anguloServoRotacion);
  } else {
    orientarGarraAutomatica(ctx.filtro.votosEjeX, ctx.filtro.votosEjeY, false);
  }
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

double desplazamientoEncoderMm(
  int32_t conteo,
  int32_t referencia,
  const EstadoEncoderCompartido &encoder
) {
  return static_cast<double>(encoder.signo) *
         static_cast<double>(diferenciaConteosConWrap(conteo, referencia)) *
         static_cast<double>(escalaEncoderMm(encoder));
}

double compensarYConEncoder(
  double y,
  int32_t conteo,
  int32_t referencia,
  const EstadoEncoderCompartido &encoder,
  int8_t relacionYEncoder
) {
  const int8_t relacion = relacionYEncoder == 0
    ? static_cast<int8_t>(1 / CAMERA_SIGNO_Y_LOCAL)
    : relacionYEncoder;
  return y - static_cast<double>(relacion) *
             desplazamientoEncoderMm(conteo, referencia, encoder);
}

int32_t conteoMedioConWrap(int32_t antes, int32_t despues) {
  return static_cast<int32_t>(
    static_cast<uint32_t>(antes) +
    static_cast<uint32_t>(diferenciaConteosConWrap(despues, antes) / 2)
  );
}

double areaCajaV2(const CandidatoPiezaV2 &candidato) {
  return static_cast<double>(abs(candidato.anchoPx)) *
         static_cast<double>(abs(candidato.altoPx));
}

bool cajasDuplicadasV2(
  const CandidatoPiezaV2 &a,
  const CandidatoPiezaV2 &b
) {
  if (a.clase != b.clase) return false;
  const double anchoA = fabs(static_cast<double>(a.anchoPx));
  const double altoA = fabs(static_cast<double>(a.altoPx));
  const double anchoB = fabs(static_cast<double>(b.anchoPx));
  const double altoB = fabs(static_cast<double>(b.altoPx));
  if (anchoA < 1.0 || altoA < 1.0 || anchoB < 1.0 || altoB < 1.0) {
    return false;
  }

  const double izquierda = fmax(
    a.centroXpx - anchoA * 0.5, b.centroXpx - anchoB * 0.5
  );
  const double derecha = fmin(
    a.centroXpx + anchoA * 0.5, b.centroXpx + anchoB * 0.5
  );
  const double arriba = fmax(
    a.centroYpx - altoA * 0.5, b.centroYpx - altoB * 0.5
  );
  const double abajo = fmin(
    a.centroYpx + altoA * 0.5, b.centroYpx + altoB * 0.5
  );
  const double interseccion = fmax(0.0, derecha - izquierda) *
                              fmax(0.0, abajo - arriba);
  const double areaMenor = fmin(anchoA * altoA, anchoB * altoB);
  if (areaMenor > 0.0 && interseccion / areaMenor >= SOLAPE_MINIMO_DUPLICADO) {
    return true;
  }

  const double dx = a.centroXpx - b.centroXpx;
  const double dy = a.centroYpx - b.centroYpx;
  const double radio = 0.25 * fmin(
    fmax(anchoA, altoA), fmax(anchoB, altoB)
  );
  return dx * dx + dy * dy <= radio * radio;
}

bool candidatoV2MejorQue(
  const CandidatoPiezaV2 &nuevo,
  const CandidatoPiezaV2 &actual
) {
  if (nuevo.confianza != actual.confianza) {
    return nuevo.confianza > actual.confianza;
  }
  return areaCajaV2(nuevo) > areaCajaV2(actual);
}

DiagnosticoDeteccionV2 crearDiagnosticoBaseV2(
  const EstadoEncoderCompartido &encoder,
  int8_t resultadosHusky
) {
  DiagnosticoDeteccionV2 diag = {};
  diag.causa = V2_DIAG_SIN_BLOQUEO;
  diag.actualizadoMs = millis();
  diag.edadEncoderMs = encoder.recibidoMs == 0
    ? UINT32_MAX : diag.actualizadoMs - encoder.recibidoMs;
  diag.conteoEncoder = encoder.conteo;
  diag.velocidadEncoderMmS = encoder.velocidadMmS;
  diag.flagsEncoder = encoder.flags;
  diag.signoEncoder = encoder.signo;
  diag.resultadosHusky = resultadosHusky;
  return diag;
}

void completarMetricasFiltroV2(
  DiagnosticoDeteccionV2 &diag,
  const FiltroDeteccionV2 &filtro,
  int32_t conteoActual,
  const EstadoEncoderCompartido &encoder
) {
  diag.clase = filtro.clase;
  diag.deteccionesConsecutivas = filtro.consecutivas;
  if (filtro.consecutivas == 0) return;
  diag.dispersionXmm = filtro.maximoX - filtro.minimoX;
  diag.dispersionYCompensadaMm =
    filtro.maximoYCompensada - filtro.minimoYCompensada;
  diag.desplazamientoMm = fabs(
    static_cast<double>(diferenciaConteosConWrap(
      conteoActual, filtro.conteoReferencia
    )) * static_cast<double>(escalaEncoderMm(encoder))
  );
}

void publicarCausaDiagnosticoV2(
  DiagnosticoDeteccionV2 &diag,
  uint8_t causa
) {
  diag.causa = causa;
  diag.actualizadoMs = millis();
  publicarDiagnosticoV2(diag);
}

int8_t elegirRelacionYEncoderV2(
  const FiltroDeteccionV2 &filtro,
  const Point2D &posicion,
  int32_t conteo,
  const EstadoEncoderCompartido &encoder
) {
  if (filtro.relacionYEncoder == 1 || filtro.relacionYEncoder == -1) {
    return filtro.relacionYEncoder;
  }
  const double yPositiva = compensarYConEncoder(
    posicion.y, conteo, filtro.conteoReferencia, encoder, 1
  );
  const double yNegativa = compensarYConEncoder(
    posicion.y, conteo, filtro.conteoReferencia, encoder, -1
  );
  return fabs(yPositiva - filtro.promedioYCompensada) <=
         fabs(yNegativa - filtro.promedioYCompensada) ? 1 : -1;
}

bool tamanoCompatibleV2(
  const FiltroDeteccionV2 &filtro,
  const CandidatoPiezaV2 &candidato
) {
  if (filtro.ultimoAnchoPx <= 0 || filtro.ultimoAltoPx <= 0 ||
      candidato.anchoPx <= 0 || candidato.altoPx <= 0) {
    return true;
  }
  const double areaAnterior = static_cast<double>(filtro.ultimoAnchoPx) *
                              static_cast<double>(filtro.ultimoAltoPx);
  const double areaActual = areaCajaV2(candidato);
  const double relacionArea = areaActual / areaAnterior;
  return relacionArea >= 0.35 && relacionArea <= 2.85;
}

bool mismaTrayectoriaV2(
  const FiltroDeteccionV2 &filtro,
  const CandidatoPiezaV2 &candidato,
  int32_t conteo,
  const EstadoEncoderCompartido &encoder
) {
  if (filtro.consecutivas == 0 || filtro.clase != candidato.clase ||
      !tamanoCompatibleV2(filtro, candidato)) return false;
  const int8_t relacion = elegirRelacionYEncoderV2(
    filtro, candidato.posicion, conteo, encoder
  );
  const double yCompensada = compensarYConEncoder(
    candidato.posicion.y,
    conteo,
    filtro.conteoReferencia,
    encoder,
    relacion
  );
  const double nuevoMinX = fmin(filtro.minimoX, candidato.posicion.x);
  const double nuevoMaxX = fmax(filtro.maximoX, candidato.posicion.x);
  const double nuevoMinY = fmin(filtro.minimoYCompensada, yCompensada);
  const double nuevoMaxY = fmax(filtro.maximoYCompensada, yCompensada);
  return nuevoMaxX - nuevoMinX <= TOLERANCIA_TRAYECTORIA_V2_MM &&
         nuevoMaxY - nuevoMinY <= TOLERANCIA_TRAYECTORIA_V2_MM;
}

void incorporarDeteccionV2(
  ContextoCamara &ctx,
  const CandidatoPiezaV2 &candidato,
  int32_t conteo,
  const EstadoEncoderCompartido &encoder,
  uint32_t instanteMs
) {
  FiltroDeteccionV2 &filtro = ctx.filtroV2;
  const uint8_t ejeCaja = estimarEjeCajaV2(candidato);
  if (!mismaTrayectoriaV2(filtro, candidato, conteo, encoder)) {
    filtro = {
      candidato.clase, 1, conteo, conteo,
      candidato.posicion.x, candidato.posicion.y,
      candidato.posicion.x, candidato.posicion.y,
      candidato.posicion.x, candidato.posicion.x,
      candidato.posicion.y, candidato.posicion.y,
      0, candidato.anchoPx, candidato.altoPx,
      candidato.posicion.x, candidato.posicion.y,
      static_cast<uint8_t>(ejeCaja == 1),
      static_cast<uint8_t>(ejeCaja == 2),
      instanteMs, instanteMs, candidato.posicion.y, 1U
    };
    ctx.ausenciasFiltroV2 = 0;
    return;
  }

  const int8_t relacion = elegirRelacionYEncoderV2(
    filtro, candidato.posicion, conteo, encoder
  );
  const double yCompensada = compensarYConEncoder(
    candidato.posicion.y,
    conteo,
    filtro.conteoReferencia,
    encoder,
    relacion
  );
  filtro.relacionYEncoder = relacion;
  if (filtro.consecutivas < UINT8_MAX) ++filtro.consecutivas;
  if (ejeCaja == 1 && filtro.votosEjeX < UINT8_MAX) ++filtro.votosEjeX;
  if (ejeCaja == 2 && filtro.votosEjeY < UINT8_MAX) ++filtro.votosEjeY;
  filtro.ultimoConteo = conteo;
  filtro.sumaX += candidato.posicion.x;
  filtro.sumaYCompensada += yCompensada;
  filtro.promedioX = filtro.sumaX / filtro.consecutivas;
  filtro.promedioYCompensada = filtro.sumaYCompensada / filtro.consecutivas;
  filtro.minimoX = fmin(filtro.minimoX, candidato.posicion.x);
  filtro.maximoX = fmax(filtro.maximoX, candidato.posicion.x);
  filtro.minimoYCompensada = fmin(filtro.minimoYCompensada, yCompensada);
  filtro.maximoYCompensada = fmax(filtro.maximoYCompensada, yCompensada);
  filtro.ultimoAnchoPx = candidato.anchoPx;
  filtro.ultimoAltoPx = candidato.altoPx;
  if (fabs(candidato.posicion.y - filtro.ultimoYRaw) >= 0.5 &&
      filtro.muestrasYDistintas < UINT8_MAX) {
    ++filtro.muestrasYDistintas;
  }
  filtro.ultimoXRaw = candidato.posicion.x;
  filtro.ultimoYRaw = candidato.posicion.y;
  filtro.ultimoMs = instanteMs;
  ctx.ausenciasFiltroV2 = 0;
}

bool publicarObjetivoV2(
  ContextoCamara &ctx,
  int32_t conteoActual,
  const EstadoEncoderCompartido &encoder,
  bool orientarGarra,
  bool entrenamientoML,
  bool entrenamientoMLV2,
  bool pruebaSeguimiento
) {
  FiltroDeteccionV2 &filtro = ctx.filtroV2;
  if (filtro.consecutivas < DETECCIONES_ESTABLES_V2) return false;
  const double desplazamiento = fabs(
    static_cast<double>(diferenciaConteosConWrap(
      conteoActual,
      filtro.conteoReferencia
    )) * static_cast<double>(escalaEncoderMm(encoder))
  );
  if (desplazamiento < DESPLAZAMIENTO_MINIMO_V2_MM) return false;

  const int8_t relacion = filtro.relacionYEncoder == 0
    ? static_cast<int8_t>(1 / CAMERA_SIGNO_Y_LOCAL)
    : filtro.relacionYEncoder;
  const double yActual = filtro.promedioYCompensada +
    static_cast<double>(relacion) * desplazamientoEncoderMm(
      conteoActual, filtro.conteoReferencia, encoder
    );
  const long x10 = lround(filtro.promedioX * 10.0);
  const long y10 = lround(yActual * 10.0);
  if (x10 < INT16_MIN || x10 > INT16_MAX ||
      y10 < INT16_MIN || y10 > INT16_MAX) {
    reiniciarFiltroV2(ctx);
    return false;
  }

  ++ctx.secuenciaObjetivo;
  if (ctx.secuenciaObjetivo == 0) ++ctx.secuenciaObjetivo;
  ctx.objetivoValido = true;
  ctx.objetivoV2 = true;
  ctx.claseObjetivo = filtro.clase;
  ctx.objetivoX10 = static_cast<int16_t>(x10);
  ctx.objetivoY10 = static_cast<int16_t>(y10);
  ctx.conteoReferenciaObjetivo = conteoActual;
  ctx.rearmada = false;
  if (entrenamientoMLV2) {
    const uint32_t intervaloMs = filtro.ultimoMs - filtro.primerMs;
    const double deltaCamaraY = filtro.ultimoYRaw - filtro.primerYRaw;
    const double recorridoCamaraMm = fabs(deltaCamaraY);
    const double recorridoEncoderMm = fabs(desplazamientoEncoderMm(
      filtro.ultimoConteo, filtro.conteoReferencia, encoder));
    const bool velocidadCamaraValida = intervaloMs >= 80U &&
      filtro.muestrasYDistintas >= 3U &&
      recorridoCamaraMm >= 2.0 && recorridoEncoderMm >= 2.0;
    const double velocidadCamaraMmS = velocidadCamaraValida
      ? 1000.0 * recorridoCamaraMm / intervaloMs : NAN;
    const double velocidadEncoderVentanaMmS = intervaloMs > 0
      ? 1000.0 * recorridoEncoderMm / intervaloMs : NAN;
    Serial.print(pruebaSeguimiento
      ? F("V2LOG|E|mode=ML_TRACK|event=CAMERA_SPEED|session=")
      : F("V2LOG|E|mode=ML_V2|event=CAMERA_SPEED|session="));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(filtro.ultimoMs);
    Serial.print(F("|obj=")); Serial.print(ctx.secuenciaObjetivo);
    Serial.print(F("|camera_speed_valid="));
    Serial.print(velocidadCamaraValida ? 1 : 0);
    Serial.print(F("|camera_speed_mm_s="));
    Serial.print(velocidadCamaraMmS, 3);
    Serial.print(F("|camera_vy_mm_s="));
    Serial.print(intervaloMs > 0
      ? 1000.0 * deltaCamaraY / intervaloMs : NAN, 3);
    Serial.print(F("|encoder_window_mm_s="));
    Serial.print(velocidadEncoderVentanaMmS, 3);
    Serial.print(F("|encoder_velocity_mm_s="));
    Serial.print(encoder.velocidadMmS, 3);
    Serial.print(F("|speed_error_mm_s="));
    Serial.print(velocidadCamaraValida
      ? velocidadCamaraMmS - velocidadEncoderVentanaMmS : NAN, 3);
    Serial.print(F("|camera_travel_mm="));
    Serial.print(recorridoCamaraMm, 3);
    Serial.print(F("|encoder_travel_mm="));
    Serial.print(recorridoEncoderMm, 3);
    Serial.print(F("|camera_interval_ms="));
    Serial.print(intervaloMs);
    Serial.print(F("|camera_unique_y="));
    Serial.print(filtro.muestrasYDistintas);
    Serial.print(F("|camera_y_first_mm="));
    Serial.print(filtro.primerYRaw, 3);
    Serial.print(F("|camera_y_last_mm="));
    Serial.println(filtro.ultimoYRaw, 3);
  }
  ctx.sugerenciaAngulo = entrenamientoML
    ? sugerirAnguloPorVotos(filtro.votosEjeX, filtro.votosEjeY) : -1;
  if (entrenamientoML) {
    if (ctx.sugerenciaAngulo >= 0) {
      anguloServoRotacion = ctx.sugerenciaAngulo;
      servoRotacion.write(anguloServoRotacion);
    }
    Serial.print(pruebaSeguimiento
      ? F("V2LOG|E|mode=ML_TRACK|event=ML_ANGLE_SUGGESTION|session=")
      : (entrenamientoMLV2
          ? F("V2LOG|E|mode=ML_V2|event=ML_ANGLE_SUGGESTION|session=")
          : F("V2LOG|E|mode=ML|event=ML_ANGLE_SUGGESTION|session=")));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(millis());
    Serial.print(F("|obj=")); Serial.print(ctx.secuenciaObjetivo);
    Serial.print(F("|class=")); Serial.print(filtro.clase);
    Serial.print(F("|width=")); Serial.print(filtro.ultimoAnchoPx);
    Serial.print(F("|height=")); Serial.print(filtro.ultimoAltoPx);
    Serial.print(F("|suggested_rot="));
    if (ctx.sugerenciaAngulo >= 0) Serial.print(ctx.sugerenciaAngulo);
    else Serial.print(F("NA"));
    Serial.print(F("|suggestion_source=MODEL129_BOX_AXIS_MM"));
    Serial.print(F("|servo_rot_deg="));
    Serial.println(anguloServoRotacion);
  }
  if (orientarGarra) {
    orientarGarraAutomatica(filtro.votosEjeX, filtro.votosEjeY, true);
  }
  reiniciarFiltroV2(ctx);

  Serial.print(F("[AUTO V2] Objetivo bloqueado seq="));
  Serial.print(ctx.secuenciaObjetivo);
  Serial.print(F(" clase="));
  Serial.print(ctx.claseObjetivo);
  Serial.print(F(" X="));
  Serial.print(ctx.objetivoX10 / 10.0f, 1);
  Serial.print(F(" Y="));
  Serial.print(ctx.objetivoY10 / 10.0f, 1);
  Serial.print(F(" encoder="));
  Serial.println(ctx.conteoReferenciaObjetivo);
  return true;
}

bool leerPiezasV2UnaVez(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control,
  uint32_t ahora
) {
  uint32_t consulta = ++consultasLogV2;
  if (consulta == 0) consulta = ++consultasLogV2;
  const EstadoEncoderCompartido encoderAntes = copiarEstadoEncoder();
  const uint32_t inicioConsulta = millis();
  const int8_t resultCount = huskylens.getResult(PIECE_MODEL);
  const uint32_t finConsulta = millis();
  const EstadoEncoderCompartido encoderDespues = copiarEstadoEncoder();
  EstadoEncoderCompartido encoder = encoderDespues;
  const bool muestrasCompatibles =
    encoderAntes.recibidoMs != 0 && encoderDespues.recibidoMs != 0 &&
    inicioConsulta - encoderAntes.recibidoMs <= TIMEOUT_MUESTRA_ENCODER_MS &&
    finConsulta - encoderDespues.recibidoMs <= TIMEOUT_MUESTRA_ENCODER_MS &&
    encoderAntes.nmPorCuenta == encoderDespues.nmPorCuenta &&
    encoderAntes.signo == encoderDespues.signo;
  if (muestrasCompatibles) {
    encoder.conteo = conteoMedioConWrap(
      encoderAntes.conteo, encoderDespues.conteo
    );
  }
  const bool encoderValido = encoderRemotoVigente(encoder);
  DiagnosticoDeteccionV2 diag = crearDiagnosticoBaseV2(
    encoder, resultCount
  );
  diag.duracionConsultaMs = finConsulta - inicioConsulta;
  diag.conteoAntesConsulta = encoderAntes.conteo;
  diag.conteoDespuesConsulta = encoderDespues.conteo;
  diag.conteoAsociadoCamara = encoder.conteo;
  if (muestrasCompatibles) {
    diag.recorridoDuranteConsultaMm = fabs(
      desplazamientoEncoderMm(
        encoderDespues.conteo, encoderAntes.conteo, encoderDespues
      )
    );
  }
  if (resultCount < 0) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_HUSKY_ERROR, encoder, consulta
    );
    return false;
  }

  CandidatoPiezaV2 candidatos[MAX_CANDIDATOS_V2] = {};
  CandidatoPiezaV2 unicos[MAX_CANDIDATOS_V2] = {};
  uint8_t cantidadCandidatos = 0;
  uint8_t cantidadUnicos = 0;
  uint8_t indiceResultado = 0;

  while (huskylens.available(PIECE_MODEL)) {
    Result *result = huskylens.popCachedResult(PIECE_MODEL);
    if (result == nullptr) continue;
    const uint8_t indiceActual = indiceResultado;
    if (indiceResultado < UINT8_MAX) ++indiceResultado;
    CandidatoPiezaV2 candidato = {};
    candidato.grupoDuplicado = UINT8_MAX;
    candidato.clase = VisionModelo129::clasePermitida(result->name.c_str());
    if (candidato.clase == 0 || result->type != COMMAND_RETURN_BLOCK ||
        result->width <= 0 || result->height <= 0) continue;
    candidato.confianza = result->confidence;
    candidato.centroXpx = result->xCenter;
    candidato.centroYpx = result->yCenter;
    candidato.anchoPx = result->width;
    candidato.altoPx = result->height;
    if (!pixelToMillimeters(result->xCenter, result->yCenter,
                            ctx.homografiaValida, candidato.posicion)) {
      if (diag.rechazadosHomografia < UINT8_MAX)
        ++diag.rechazadosHomografia;
      registrarCandidatoLogV2(
        consulta, encoder, candidato, indiceActual, V2_DIAG_HOMOGRAFIA
      );
      continue;
    }
    if (!isOverWhiteBelt(candidato.posicion)) {
      if (diag.rechazadosFueraBanda < UINT8_MAX)
        ++diag.rechazadosFueraBanda;
      registrarCandidatoLogV2(
        consulta, encoder, candidato, indiceActual, V2_DIAG_FUERA_BANDA
      );
      continue;
    }
    if (diag.candidatosValidos < UINT8_MAX) ++diag.candidatosValidos;
    if (cantidadCandidatos >= MAX_CANDIDATOS_V2) {
      registrarCandidatoLogV2(
        consulta, encoder, candidato, indiceActual, V2_DIAG_SIN_BLOQUEO
      );
      continue;
    }

    uint8_t grupo = cantidadUnicos;
    bool duplicado = false;
    for (uint8_t i = 0; i < cantidadUnicos; ++i) {
      if (!cajasDuplicadasV2(unicos[i], candidato)) continue;
      grupo = i;
      duplicado = true;
      if (candidatoV2MejorQue(candidato, unicos[i])) {
        unicos[i] = candidato;
      }
      break;
    }
    if (!duplicado && cantidadUnicos < MAX_CANDIDATOS_V2) {
      unicos[cantidadUnicos] = candidato;
      ++cantidadUnicos;
    }
    candidato.grupoDuplicado = grupo;
    candidatos[cantidadCandidatos++] = candidato;
    registrarCandidatoLogV2(
      consulta, encoder, candidato, indiceActual, V2_DIAG_SIN_BLOQUEO
    );
  }
  diag.candidatosUnicos = cantidadUnicos;
  diag.duplicadosDescartados = cantidadCandidatos >= cantidadUnicos
    ? cantidadCandidatos - cantidadUnicos : 0;

  // Observacion continua sin reservar objetivos ni esperar catch/ACK.
  // Se conservan TODOS los candidatos originales y la consulta en V2LOG.
  // Solo una pieza univoca y un encoder vigente se usan como referencia I2C.
  if (control.pruebaEncoderActiva) {
    ctx.objetivoValido = false;
    uint8_t seleccion = UINT8_MAX;
    if (cantidadUnicos == 1 && encoderValido) {
      const CandidatoPiezaV2 &c = unicos[0];
      const long x10 = lround(c.posicion.x * 10.0);
      const long y10 = lround(c.posicion.y * 10.0);
      if (x10 >= INT16_MIN && x10 <= INT16_MAX &&
          y10 >= INT16_MIN && y10 <= INT16_MAX) {
        ++ctx.secuenciaObjetivo;
        if (ctx.secuenciaObjetivo == 0) ++ctx.secuenciaObjetivo;
        ctx.objetivoValido = true;
        ctx.objetivoV2 = true;
        ctx.claseObjetivo = c.clase;
        ctx.objetivoX10 = static_cast<int16_t>(x10);
        ctx.objetivoY10 = static_cast<int16_t>(y10);
        ctx.conteoReferenciaObjetivo = encoder.conteo;
        diag.xMm = c.posicion.x;
        diag.yMm = c.posicion.y;
        diag.clase = c.clase;
        diag.secuenciaPublicada = ctx.secuenciaObjetivo;
        seleccion = 0;
      }
    }
    reiniciarFiltroV2(ctx);
    publicarYRegistrarDiagnosticoV2(diag,
      !encoderValido ? diagnosticarEncoderV2(encoder) :
        (cantidadUnicos == 0 ? V2_DIAG_SIN_PIEZA : V2_DIAG_SIN_BLOQUEO),
      encoder, consulta, seleccion);
    return true;
  }

  if (cantidadCandidatos > 1 &&
      ahora - ultimoReporteCandidatosV2 >= 1000UL) {
    ultimoReporteCandidatosV2 = ahora;
    Serial.print(F("[V2][CAND] total="));
    Serial.print(resultCount);
    Serial.print(F(" valid="));
    Serial.print(cantidadCandidatos);
    Serial.print(F(" unicos="));
    Serial.print(cantidadUnicos);
    Serial.print(F(" duplicados="));
    Serial.print(diag.duplicadosDescartados);
    Serial.print(F(" consulta="));
    Serial.print(diag.duracionConsultaMs);
    Serial.print(F("ms enc="));
    Serial.print(diag.conteoAntesConsulta);
    Serial.print('/');
    Serial.print(diag.conteoAsociadoCamara);
    Serial.print('/');
    Serial.print(diag.conteoDespuesConsulta);
    Serial.print(F(" recorridoConsulta="));
    Serial.println(diag.recorridoDuranteConsultaMm, 2);
    for (uint8_t i = 0; i < cantidadCandidatos; ++i) {
      const CandidatoPiezaV2 &c = candidatos[i];
      Serial.print(F("[V2][CAND] i="));
      Serial.print(i);
      Serial.print(F(" grupo="));
      Serial.print(c.grupoDuplicado);
      Serial.print(F(" id="));
      Serial.print(c.clase);
      Serial.print(F(" conf="));
      Serial.print(static_cast<int>(c.confianza));
      Serial.print(F(" px="));
      Serial.print(c.centroXpx);
      Serial.print(',');
      Serial.print(c.centroYpx);
      Serial.print(F(" caja="));
      Serial.print(c.anchoPx);
      Serial.print('x');
      Serial.print(c.altoPx);
      Serial.print(F(" mm="));
      Serial.print(c.posicion.x, 1);
      Serial.print(',');
      Serial.println(c.posicion.y, 1);
    }
  }

  uint8_t bloqueoPrevio = V2_DIAG_SIN_BLOQUEO;
  if (!control.portentaActiva || !control.automaticoV2Activo) {
    bloqueoPrevio = V2_DIAG_INACTIVO;
  } else if (control.brazoOcupado) {
    bloqueoPrevio = V2_DIAG_BRAZO_OCUPADO;
  } else if (ctx.objetivoValido) {
    bloqueoPrevio = V2_DIAG_OBJETIVO_ACTIVO;
  } else if (ctx.esperandoDesaparicion) {
    bloqueoPrevio = V2_DIAG_ESPERANDO_DESAPARICION;
  } else if (!ctx.rearmada) {
    bloqueoPrevio = V2_DIAG_NO_REARMADO;
  } else if (!encoderValido) {
    bloqueoPrevio = diagnosticarEncoderV2(encoder);
  }

  if (bloqueoPrevio != V2_DIAG_SIN_BLOQUEO) {
    publicarYRegistrarDiagnosticoV2(
      diag, bloqueoPrevio, encoder, consulta
    );
    reiniciarFiltroV2(ctx);
    ctx.ausenciasFiltroV2 = 0;
    return true;
  }

  if (cantidadUnicos == 0) {
    if (ctx.ausenciasFiltroV2 < UINT8_MAX) ++ctx.ausenciasFiltroV2;
    if (ctx.ausenciasFiltroV2 > 2) reiniciarFiltroV2(ctx);
    uint8_t causaSinPieza = V2_DIAG_SIN_RESULTADOS;
    if (resultCount == 0) {
      causaSinPieza = V2_DIAG_SIN_PIEZA;
    } else if (diag.rechazadosHomografia > 0 &&
               diag.rechazadosFueraBanda == 0) {
      causaSinPieza = V2_DIAG_HOMOGRAFIA;
    } else if (diag.rechazadosFueraBanda > 0) {
      causaSinPieza = V2_DIAG_FUERA_BANDA;
    }
    publicarYRegistrarDiagnosticoV2(
      diag, causaSinPieza, encoder, consulta
    );
    return true;
  }

  const bool habiaTrayectoria = ctx.filtroV2.consecutivas > 0;
  uint8_t indiceElegido = 0;
  bool hayCoincidente = false;
  double mejorPuntaje = HUGE_VAL;
  if (habiaTrayectoria) {
    for (uint8_t i = 0; i < cantidadUnicos; ++i) {
      const CandidatoPiezaV2 &candidato = unicos[i];
      if (!mismaTrayectoriaV2(
            ctx.filtroV2, candidato, encoder.conteo, encoder
          )) continue;
      const int8_t relacion = elegirRelacionYEncoderV2(
        ctx.filtroV2, candidato.posicion, encoder.conteo, encoder
      );
      const double yCompensada = compensarYConEncoder(
        candidato.posicion.y, encoder.conteo,
        ctx.filtroV2.conteoReferencia, encoder, relacion
      );
      const double dx = candidato.posicion.x - ctx.filtroV2.promedioX;
      const double dy = yCompensada - ctx.filtroV2.promedioYCompensada;
      const double areaAnterior = fmax(
        1.0, static_cast<double>(ctx.filtroV2.ultimoAnchoPx) *
             static_cast<double>(ctx.filtroV2.ultimoAltoPx)
      );
      const double penalizacionTamano = fabs(
        log(fmax(1.0, areaCajaV2(candidato)) / areaAnterior)
      );
      const double puntaje = dx * dx + dy * dy +
                              penalizacionTamano * penalizacionTamano;
      if (puntaje < mejorPuntaje) {
        mejorPuntaje = puntaje;
        indiceElegido = i;
        hayCoincidente = true;
      }
    }
  } else {
    for (uint8_t i = 1; i < cantidadUnicos; ++i) {
      if (candidatoV2MejorQue(unicos[i], unicos[indiceElegido])) {
        indiceElegido = i;
      }
    }
  }

  if (habiaTrayectoria && !hayCoincidente) {
    for (uint8_t i = 0; i < cantidadUnicos; ++i) {
      const CandidatoPiezaV2 &candidato = unicos[i];
      const double recorridoDesdeUltima = fabs(desplazamientoEncoderMm(
        encoder.conteo, ctx.filtroV2.ultimoConteo, encoder
      ));
      const double dxRaw = candidato.posicion.x - ctx.filtroV2.ultimoXRaw;
      const double dyRaw = candidato.posicion.y - ctx.filtroV2.ultimoYRaw;
      if (candidato.clase == ctx.filtroV2.clase &&
          recorridoDesdeUltima >= DESPLAZAMIENTO_CAMARA_REPETIDA_MM &&
          fabs(dxRaw) <= TOLERANCIA_CAMARA_REPETIDA_MM &&
          fabs(dyRaw) <= TOLERANCIA_CAMARA_REPETIDA_MM) {
        diag.clase = candidato.clase;
        diag.xMm = candidato.posicion.x;
        diag.yMm = candidato.posicion.y;
        diag.deteccionesConsecutivas = ctx.filtroV2.consecutivas;
        diag.desplazamientoMm = recorridoDesdeUltima;
        diag.relacionYEncoder = ctx.filtroV2.relacionYEncoder;
        publicarYRegistrarDiagnosticoV2(
          diag, V2_DIAG_CAMARA_REPETIDA, encoder, consulta, i
        );
        return true;
      }
    }
    for (uint8_t i = 1; i < cantidadUnicos; ++i) {
      if (candidatoV2MejorQue(unicos[i], unicos[indiceElegido])) {
        indiceElegido = i;
      }
    }
  }

  const CandidatoPiezaV2 &candidatoElegido = unicos[indiceElegido];
  if (habiaTrayectoria && !hayCoincidente) {
    const double yCompensadaRechazada = compensarYConEncoder(
      candidatoElegido.posicion.y,
      encoder.conteo,
      ctx.filtroV2.conteoReferencia,
      encoder,
      elegirRelacionYEncoderV2(
        ctx.filtroV2, candidatoElegido.posicion, encoder.conteo, encoder
      )
    );
    diag.clase = candidatoElegido.clase;
    diag.deteccionesConsecutivas = ctx.filtroV2.consecutivas;
    diag.xMm = candidatoElegido.posicion.x;
    diag.yMm = candidatoElegido.posicion.y;
    diag.yCompensadaMm = yCompensadaRechazada;
    diag.dispersionXmm =
      fmax(ctx.filtroV2.maximoX, candidatoElegido.posicion.x) -
      fmin(ctx.filtroV2.minimoX, candidatoElegido.posicion.x);
    diag.dispersionYCompensadaMm =
      fmax(ctx.filtroV2.maximoYCompensada, yCompensadaRechazada) -
      fmin(ctx.filtroV2.minimoYCompensada, yCompensadaRechazada);
    diag.desplazamientoMm = fabs(
      static_cast<double>(diferenciaConteosConWrap(
        encoder.conteo, ctx.filtroV2.conteoReferencia
      )) * static_cast<double>(escalaEncoderMm(encoder))
    );
  }

  incorporarDeteccionV2(ctx, candidatoElegido, encoder.conteo, encoder,
                        finConsulta);

  if (habiaTrayectoria && !hayCoincidente) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_TRAYECTORIA, encoder, consulta, indiceElegido
    );
    return true;
  }

  diag.clase = ctx.filtroV2.clase;
  diag.xMm = candidatoElegido.posicion.x;
  diag.yMm = candidatoElegido.posicion.y;
  diag.relacionYEncoder = ctx.filtroV2.relacionYEncoder;
  diag.yCompensadaMm = compensarYConEncoder(
    candidatoElegido.posicion.y,
    encoder.conteo,
    ctx.filtroV2.conteoReferencia,
    encoder,
    ctx.filtroV2.relacionYEncoder
  );
  completarMetricasFiltroV2(diag, ctx.filtroV2, encoder.conteo, encoder);

  if (ctx.filtroV2.consecutivas < DETECCIONES_ESTABLES_V2) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_DETECCIONES, encoder, consulta, indiceElegido
    );
    return true;
  }
  if (diag.desplazamientoMm < DESPLAZAMIENTO_MINIMO_V2_MM) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_DESPLAZAMIENTO, encoder, consulta, indiceElegido
    );
    return true;
  }

  publicarCausaDiagnosticoV2(diag, V2_DIAG_OBJETIVO_LISTO);
  if (publicarObjetivoV2(
        ctx, encoder.conteo, encoder, control.orientarGarraV2,
        control.entrenamientoML, control.entrenamientoMLV2,
        control.pruebaSeguimiento)) {
    diag.secuenciaPublicada = ctx.secuenciaObjetivo;
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_PUBLICADO, encoder, consulta, indiceElegido
    );
  } else {
    registrarConsultaLogV2(diag, encoder, consulta, indiceElegido);
  }
  return true;
}

bool leerPiezasUnaVez(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control,
  uint32_t ahora
) {
  if (control.automaticoV2Activo) {
    return leerPiezasV2UnaVez(ctx, control, ahora);
  }
  {
    const EstadoEncoderCompartido encoder = copiarEstadoEncoder();
    DiagnosticoDeteccionV2 diag = crearDiagnosticoBaseV2(encoder, 0);
    publicarCausaDiagnosticoV2(diag, V2_DIAG_INACTIVO);
  }
  const int8_t resultCount = huskylens.getResult(PIECE_MODEL);
  if (resultCount < 0) {
    return false;
  }

  bool hayPiezaValida = false;
  bool piezaProcesadaPresente = false;
  bool hayPrimera = false;
  uint8_t clasePrimera = 0;
  Point2D posicionPrimera = {};
  uint8_t ejePrimera = 0;
  CandidatoPiezaV2 cajaPrimera = {};
  bool hayCoincidente = false;
  uint8_t claseCoincidente = 0;
  Point2D posicionCoincidente = {};
  uint8_t ejeCoincidente = 0;
  CandidatoPiezaV2 cajaCoincidente = {};
  double distanciaCoincidente = HUGE_VAL;

  while (huskylens.available(PIECE_MODEL)) {
    Result *result = huskylens.popCachedResult(PIECE_MODEL);
    if (result == nullptr) {
      continue;
    }

    const uint8_t clase = VisionModelo129::clasePermitida(result->name.c_str());
    if (clase == 0 || result->type != COMMAND_RETURN_BLOCK ||
        result->width <= 0 || result->height <= 0) continue;

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
    CandidatoPiezaV2 caja = {};
    caja.centroXpx = result->xCenter;
    caja.centroYpx = result->yCenter;
    caja.anchoPx = result->width;
    caja.altoPx = result->height;
    caja.confianza = result->confidence;
    const uint8_t ejeCaja = estimarEjeCajaV2(caja);

    if (
      ctx.esperandoDesaparicion &&
      clase == ctx.claseEsperandoDesaparicion &&
      fabs(posicion.x - ctx.xEsperandoDesaparicion) <= TOLERANCIA_REARME_MM &&
      fabs(posicion.y - ctx.yEsperandoDesaparicion) <= TOLERANCIA_REARME_MM
    ) {
      piezaProcesadaPresente = true;
    }

    if (!hayPrimera) {
      hayPrimera = true;
      clasePrimera = clase;
      posicionPrimera = posicion;
      ejePrimera = ejeCaja;
      cajaPrimera = caja;
    }

    if (mismaDeteccionEstable(ctx.filtro, clase, posicion)) {
      const double dx = posicion.x - ctx.filtro.promedioX;
      const double dy = posicion.y - ctx.filtro.promedioY;
      const double distancia2 = dx * dx + dy * dy;
      if (
        distancia2 < distanciaCoincidente
      ) {
        distanciaCoincidente = distancia2;
        hayCoincidente = true;
        claseCoincidente = clase;
        posicionCoincidente = posicion;
        ejeCoincidente = ejeCaja;
        cajaCoincidente = caja;
      }
    }
  }

  actualizarEsperaDesaparicion(
    ctx,
    piezaProcesadaPresente,
    ahora,
    control.brazoOcupado,
    TIEMPO_DESAPARICION_MS,
    false
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
    incorporarDeteccion(
      ctx, claseCoincidente, posicionCoincidente, ejeCoincidente,
      cajaCoincidente);
  } else {
    incorporarDeteccion(ctx, clasePrimera, posicionPrimera, ejePrimera,
      cajaPrimera);
  }
  publicarObjetivoEstable(ctx, control.registrarAngulo);
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
  if (control.pruebaEncoderActiva != ctx.pruebaEncoderAnterior) {
    limpiarObjetivo(ctx, false);
    ctx.esperandoDesaparicion = false;
    ctx.rearmada = true;
    ctx.pruebaEncoderAnterior = control.pruebaEncoderActiva;
    ctx.proximaLectura = millis();
  }
  if (control.pruebaEncoderActiva) {
    ctx.portentaEstabaActiva = control.portentaActiva;
    ctx.autoEstabaActivo = false;
    return;
  }
  if (
    ctx.objetivoValido &&
    control.portentaActiva &&
    control.codigoAckObjetivo != ACK_OBJ_NINGUNO &&
    control.ackObjetivo == ctx.secuenciaObjetivo
  ) {
    // En V2 y en Registro Angulo, ACEPTADO reserva la pieza y CERRAR_PINZA
    // mantiene la reserva mientras Z se retira. Solo un resultado terminal
    // libera el objetivo y permite el rearme del detector.
    const bool objetivoReservado = ctx.objetivoV2 &&
      (control.codigoAckObjetivo == ACK_OBJ_ACEPTADO ||
       control.codigoAckObjetivo == ACK_OBJ_CERRAR_PINZA);
    if (!objetivoReservado) {
      if (control.registrarAngulo) {
        Serial.print(F("[ANGULO] Objetivo liberado seq="));
      } else if (ctx.objetivoV2) {
        Serial.print(F("[AUTO V2] Objetivo liberado seq="));
      } else {
        Serial.print(F("[AUTO] ACK objetivo seq="));
      }
      Serial.print(ctx.secuenciaObjetivo);
      Serial.print(F(" ack="));
      Serial.println(control.codigoAckObjetivo);
      limpiarObjetivo(ctx, true);
    }
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
      Serial.println(F("[CAM] Intentando conexion UART1 RX32/TX33"));
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
        // No se publica MODELO_LISTO solo por haber esperado. Una lectura
        // valida (tambien con cero detecciones) confirma que el modelo 129
        // termino de cargar y responde por UART.
        const int8_t resultados = huskylens.getResult(PIECE_MODEL);
        if (resultados < 0) {
          registrarErrorCamara(
            ctx,
            CAM_ERROR_ABRIR_MODELO,
            true,
            true
          );
          break;
        }
        ctx.modeloListo = true;
        ctx.solicitarModelo = false;
        ctx.error = CAM_ERROR_NINGUNO;
        ctx.proximaLectura = ahora;
        cambiarEstadoCamara(ctx, CAMARA_LISTA);
        Serial.println(F("[CAM] Modelo 129 confirmado; solo pieza6/pieza7 habilitadas"));
      }
      break;

    case CAMARA_LISTA: {
      // El rearme V2 se resuelve antes de decidir si se consulta la camara. De
      // este modo no se llama a HUSKYLENS durante el objetivo bloqueado ni en
      // los 500 ms posteriores a su resultado terminal.
      if (control.automaticoV2Activo && !control.pruebaEncoderActiva && ctx.esperandoDesaparicion) {
        actualizarEsperaDesaparicion(
          ctx,
          false,
          ahora,
          control.brazoOcupado,
          TIEMPO_REARME_V2_MS,
          true
        );
      }

      if (control.automaticoV2Activo && !control.pruebaEncoderActiva &&
          (ctx.objetivoValido || control.brazoOcupado ||
           ctx.esperandoDesaparicion || !ctx.rearmada)) {
        const EstadoEncoderCompartido encoder = copiarEstadoEncoder();
        DiagnosticoDeteccionV2 diag = crearDiagnosticoBaseV2(encoder, 0);
        uint8_t causaBloqueo = V2_DIAG_NO_REARMADO;
        if (control.brazoOcupado) {
          causaBloqueo = V2_DIAG_BRAZO_OCUPADO;
        } else if (ctx.objetivoValido) {
          causaBloqueo = V2_DIAG_OBJETIVO_ACTIVO;
        } else if (ctx.esperandoDesaparicion) {
          causaBloqueo = V2_DIAG_ESPERANDO_DESAPARICION;
        }
        publicarCausaDiagnosticoV2(
          diag,
          causaBloqueo
        );
        reiniciarFiltroV2(ctx);
        ctx.ausenciasFiltroV2 = 0;
        ctx.proximaLectura = ahora + CAM_READ_PERIOD_V2_MS;
        break;
      }

      const bool deteccionNecesaria =
        control.automaticoActivo ||
        ctx.esperandoDesaparicion ||
        ctx.objetivoValido;
      const uint32_t periodo = control.automaticoV2Activo
        ? CAM_READ_PERIOD_V2_MS
        : (deteccionNecesaria ? CAM_READ_PERIOD_MS : CAM_HEALTH_PERIOD_MS);

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
  // Replica la espera que usa el sketch minimo que ya fue validado en el
  // hardware. La tarea sigue siendo no bloqueante para el resto del sistema.
  camaraCtx.proximaConexion = millis() + CAM_UART_STARTUP_MS;
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

void dibujarChecklistConexiones(
  bool portentaOk,
  bool estabilizando,
  uint32_t restanteMs
) {
  const EstadoCamaraPublicado camara = copiarEstadoCamara();
  const bool camaraOk = camara.conectada &&
                        camara.estado != CAMARA_OFFLINE &&
                        camara.estado != CAMARA_CONECTANDO &&
                        camara.estado != CAMARA_ERROR &&
                        camara.error == CAM_ERROR_NINGUNO;

  dibujarTitulo(F("CHECK CONEXIONES"));
  pantalla.setCursor(0, 16);
  pantalla.print(F("PORTENTA: "));
  pantalla.println(portentaOk ? F("OK") : F("PENDIENTE"));
  pantalla.setCursor(0, 30);
  pantalla.print(F("CAMARA:   "));
  pantalla.println(camaraOk ? F("OK") : F("PENDIENTE"));
  pantalla.setCursor(0, 48);
  if (estabilizando) {
    pantalla.print(F("ESTABILIZANDO "));
    pantalla.print((restanteMs + 999U) / 1000U);
    pantalla.println(F(" s"));
  } else if (!portentaOk || !camaraOk) {
    pantalla.println(F("ESPERANDO ENLACES"));
  } else if (!bluetoothConectado) {
    pantalla.println(F("CONECTE EL CONTROL"));
  } else {
    pantalla.println(F("X: ENTRAR AL MENU"));
  }
}

void mostrarSinPortenta() {
  pantalla.clearDisplay();
  dibujarChecklistConexiones(false, false, 0);
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
  const char *opciones[11] = {
    "MANUAL", "AUTOMATICO", "AUTOMATICO V2",
    "REGISTRAR ANGULO", "CALIBRACIONES", "PRUEBA SERVOS",
    "ENSENANZA ML", "ENSENANZA ML V2", "SEGUIMIENTO Y",
    "PRUEBA DE ENCODER", "CHECKLIST"
  };
  const uint8_t valores[11] = {
    MENU_MODO_MANUAL, MENU_MODO_AUTOMATICO, MENU_MODO_AUTOMATICO_V2,
    MENU_REGISTRO_ANGULO, MENU_CALIBRACIONES, MENU_PRUEBA_SERVOS,
    MENU_ENTRENAMIENTO_ML, MENU_ENTRENAMIENTO_ML_V2,
    MENU_PRUEBA_SEGUIMIENTO,
    MENU_PRUEBA_ENCODER, MENU_DIAGNOSTICO
  };
  uint8_t seleccion = 0;
  for (uint8_t i = 0; i < 11; ++i) {
    if (p.opcionMenu == valores[i]) seleccion = i;
  }
  const uint8_t inicio = seleccion >= 5 ? seleccion - 4 : 0;
  for (uint8_t fila = 0; fila < 5; ++fila) {
    const uint8_t i = inicio + fila;
    pantalla.setCursor(0, 12 + fila * 10);
    pantalla.print(seleccion == i ? F(">") : F(" "));
    pantalla.println(opciones[i]);
  }
}

void mostrarMenuCalibraciones(const PaquetePortentaAESP &p) {
  dibujarTitulo(F("CALIBRACIONES"));
  const char *nombres[3] = {"BRAZO", "CAMARA", "ENCODER"};
  const bool listas[3] = {
    (p.flagsSistema & (SIS_FLAG_XY_CALIBRADO | SIS_FLAG_Z_CALIBRADO)) ==
      (SIS_FLAG_XY_CALIBRADO | SIS_FLAG_Z_CALIBRADO),
    (copiarEstadoCamara().estado == CAMARA_LISTA &&
     copiarEstadoCamara().conectada &&
     copiarEstadoCamara().homografiaValida &&
     copiarEstadoCamara().modeloListo),
    (p.flagsSistema & SIS_FLAG_ENCODER_CALIBRADO) != 0
  };
  for (uint8_t i = 0; i < 3; ++i) {
    pantalla.setCursor(0, 14 + i * 13);
    pantalla.print(p.opcionMenu == i ? F(">") : F(" "));
    pantalla.print(nombres[i]);
    pantalla.print(F("  "));
    pantalla.println(listas[i] ? F("OK") : F("PEND"));
  }
  pantalla.setCursor(0, 55);
  pantalla.print(F("X:INICIAR TRI:SALIR"));
}

void mostrarPruebaServos() {
  dibujarTitulo(F("PRUEBA SERVOS"));
  pantalla.setCursor(0, 15);
  pantalla.print(F("ROTACION: "));
  pantalla.println(anguloServoRotacion);
  pantalla.setCursor(0, 27);
  pantalla.print(F("PINZA:    "));
  pantalla.println(anguloServoPinza);
  pantalla.setCursor(0, 40);
  pantalla.println(F("STICK DER X: ROT"));
  pantalla.println(F("CIRCULO: ABRIR/CERRAR"));
  pantalla.setCursor(0, 56);
  pantalla.print(F("TRI: MENU"));
}

void mostrarDiagnostico(const PaquetePortentaAESP &p) {
  dibujarTitulo(F("CHECKLIST"));
  const bool paginaDos = p.opcionMenu != 0;
  if (!paginaDos) {
    pantalla.setCursor(0, 14);
    pantalla.println(F("PORTENTA I2C: OK"));
    pantalla.setCursor(0, 25);
    pantalla.print(F("CAMARA: "));
    pantalla.println(copiarEstadoCamara().conectada
      ? F("CONECTADA") : F("SIN ENLACE"));
    pantalla.setCursor(0, 36);
    pantalla.print(F("HOM:"));
    pantalla.print(copiarEstadoCamara().homografiaValida ? F("OK ") : F("-- "));
    pantalla.print(F("MOD:"));
    pantalla.println(copiarEstadoCamara().modeloListo ? F("OK") : F("--"));
    pantalla.setCursor(0, 47);
    pantalla.print(F("CONTROL: "));
    pantalla.println(bluetoothConectado ? F("OK") : F("SIN ENLACE"));
  } else {
    pantalla.setCursor(0, 14);
    pantalla.print(F("BRAZO: "));
    pantalla.println((p.flagsSistema & (SIS_FLAG_XY_CALIBRADO | SIS_FLAG_Z_CALIBRADO)) ==
      (SIS_FLAG_XY_CALIBRADO | SIS_FLAG_Z_CALIBRADO) ? F("OK") : F("PEND"));
    pantalla.setCursor(0, 25);
    pantalla.print(F("ENC CAL: "));
    pantalla.println((p.flagsSistema & SIS_FLAG_ENCODER_CALIBRADO)
      ? F("OK") : F("PEND"));
    pantalla.setCursor(0, 36);
    pantalla.print(F("ENC PULSOS: "));
    pantalla.println((p.estadoEncoder & ENC_FLAG_PULSOS_VISTOS)
      ? F("SI") : F("NO"));
    pantalla.setCursor(0, 47);
    pantalla.print(F("FINALES: "));
    pantalla.println((p.flagsLimites & LIM_FLAG_COHERENTES)
      ? F("OK") : F("ERROR"));
  }
  pantalla.setCursor(0, 56);
  pantalla.print(F("X:PAGINA TRI:MENU"));
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
  if (p.flagsSistema & SIS_FLAG_Z_CALIBRADO) {
    pantalla.print(F("Z-DIN04:"));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.print(F("p"));
  } else {
    pantalla.print(F("Z:CALIBRAR"));
  }
  pantalla.setCursor(106, 51);
  pantalla.print(F("TRI"));
}

void mostrarModoAutomatico(const PaquetePortentaAESP &p) {
  const EstadoCamaraPublicado camara = copiarEstadoCamara();
  const bool registro = p.opcionMenu == MENU_REGISTRO_ANGULO;
  dibujarTitulo(registro ? F("REGISTRAR ANGULO") : F("MODO AUTOMATICO"));
  pantalla.setCursor(0, 12);
  pantalla.print(F("CAM: "));
  pantalla.println(nombreEstadoCamara(camara.estado));

  pantalla.setCursor(0, 23);
  pantalla.print(F("OBJ X:"));
  pantalla.print(camara.objetivoX10 / 10.0f, 1);
  pantalla.print(F(" Y:"));
  pantalla.print(camara.objetivoY10 / 10.0f, 1);

  pantalla.setCursor(0, 34);
  if (registro) {
    pantalla.print(F("ROT: "));
    pantalla.print(anguloServoRotacion);
    if (camara.objetivoValido && camara.sugerenciaAngulo >= 0) {
      pantalla.print(F(" SUG:"));
      pantalla.print(camara.sugerenciaAngulo);
    } else if (camara.objetivoValido) {
      pantalla.print(F(" MANUAL"));
    } else {
      pantalla.print(F(" GRADOS"));
    }
  } else {
    pantalla.print(F("ENC:"));
    pantalla.print(p.conteoEncoder);
  }

  pantalla.setCursor(0, 45);
  if (registro) {
    const char *fase[] = {
      "PREPARANDO Z", "ESPERANDO PIEZA", "MOVIENDO XY",
      "AJUSTAR GIRO", "BAJANDO Z", "CERRANDO PINZA",
      "SUBIENDO Z", "CANCELANDO"
    };
    const uint8_t indice = p.faseCalibracionBrazo;
    pantalla.print(indice < 8 ? fase[indice] : "ESTADO DESCONOCIDO");
  } else if (p.flagsSistema & SIS_FLAG_BRAZO_OCUPADO) {
    pantalla.print(F("BRAZO MOVIENDO"));
  } else if (camara.objetivoValido) {
    pantalla.print(F("OBJETIVO LISTO"));
  } else {
    pantalla.print(F("ESPERANDO PIEZA"));
  }
  pantalla.setCursor(0, 56);
  if (registro) {
    pantalla.print(F("STICK DER  TRI:SALIR"));
  } else {
    pantalla.print(F("TRI:CANCELAR ACK:"));
    pantalla.print(p.ackSecuenciaObjetivo);
  }
}

void mostrarModoAutomaticoV2(const PaquetePortentaAESP &p) {
  const DiagnosticoDeteccionV2 diag = copiarDiagnosticoV2();
  dibujarTitulo(p.opcionMenu == MENU_REGISTRO_ANGULO
    ? F("REGISTRO ANGULO") : F("AUTOMATICO V2"));
  pantalla.setCursor(0, 12);
  pantalla.print(F("FASE: "));
  pantalla.println(p.faseCalibracionBrazo);
  if (p.faseCalibracionBrazo == 0) {
    pantalla.setCursor(0, 23);
    pantalla.print(F("BLOQ: "));
    pantalla.println(nombreCortoDiagnosticoV2(diag.causa));
    pantalla.setCursor(0, 35);
    pantalla.print(F("N:"));
    pantalla.print(diag.deteccionesConsecutivas);
    pantalla.print(F(" D:"));
    pantalla.print(diag.desplazamientoMm, 1);
    pantalla.println(F("mm"));
    pantalla.setCursor(0, 47);
    pantalla.print(F("AGE:"));
    if (diag.edadEncoderMs == UINT32_MAX) pantalla.print(F("---"));
    else pantalla.print(diag.edadEncoderMs);
    pantalla.print(F(" R:"));
    pantalla.print(diag.resultadosHusky);
    pantalla.print(F(" V:"));
    pantalla.print(diag.candidatosValidos);
    pantalla.setCursor(0, 57);
    if (p.opcionMenu == MENU_REGISTRO_ANGULO) {
      pantalla.print(F("ROT:"));
      pantalla.print(anguloServoRotacion);
      pantalla.print(F(" TRI:SALIR"));
    } else {
      pantalla.print(F("PRE:"));
      pantalla.print(Z_MARGEN_PRECAPTURA_PASOS);
      pantalla.print(F("p Z:"));
      pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
      pantalla.print(F("p"));
    }
    return;
  }
  pantalla.setCursor(0, 23);
  pantalla.print(F("ENC: "));
  pantalla.println(p.conteoEncoder);
  pantalla.setCursor(0, 35);
  pantalla.print(F("BANDA: "));
  pantalla.print(p.velocidadEncoderUmS / 1000000.0f, 1);
  pantalla.println(F(" mm/s"));
  pantalla.setCursor(0, 47);
  if (p.opcionMenu == MENU_REGISTRO_ANGULO) {
    pantalla.print(F("ENC:"));
    pantalla.print((copiarEstadoCamara().objetivoValido) ? F("OBJ ") : F("--- "));
    pantalla.print(F("ACK:"));
    pantalla.println(p.ackSecuenciaObjetivo);
  } else {
    pantalla.print(F("Z-DIN04 real: "));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.println(F("p"));
  }
  pantalla.setCursor(0, 57);
  if (p.opcionMenu == MENU_REGISTRO_ANGULO) {
    pantalla.print(F("ROT:"));
    pantalla.print(anguloServoRotacion);
    pantalla.print(F(" STICK DER X"));
  } else if (p.faseCalibracionBrazo == 6) {
    pantalla.print(F("ESPERA CATCH AUTO"));
  } else if (p.faseCalibracionBrazo == 9) {
    pantalla.print(F("PREPARANDO ESPERA"));
  } else if (p.faseCalibracionBrazo == 10) {
    pantalla.print(F("CERRANDO PINZA"));
  } else if (p.faseCalibracionBrazo == 11) {
    pantalla.print(F("Z FINAL + CATCH"));
  } else if (p.faseCalibracionBrazo == 7) {
    pantalla.print(F("CATCH AUTOMATICO"));
  } else {
    pantalla.print(F("TRI:CANCELAR"));
  }
}

void mostrarEntrenamientoML(const PaquetePortentaAESP &p) {
  const bool esSeguimiento = p.opcionMenu == MENU_PRUEBA_SEGUIMIENTO;
  const bool esV2 = p.opcionMenu == MENU_ENTRENAMIENTO_ML_V2 ||
    esSeguimiento;
  dibujarTitulo(esSeguimiento ? F("SEGUIMIENTO Y")
    : (esV2 ? F("ENSENANZA ML V2") : F("ENSENANZA ML")));
  pantalla.setCursor(0, 13);
  pantalla.print(F("FASE: "));
  pantalla.println(p.faseCalibracionBrazo);
  pantalla.setCursor(0, 25);
  if (p.faseCalibracionBrazo == 15) {
    pantalla.println(F("Y SIGUE LA PIEZA"));
    pantalla.println(F("DER X: ORIENTAR"));
    pantalla.setCursor(0, 40);
    pantalla.print(F("Z-DIN04: "));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.println(F("p"));
    pantalla.setCursor(0, 49);
    pantalla.print(F("ROT:"));
    pantalla.print(anguloServoRotacion);
    pantalla.print(F(" X:CATCH"));
  } else if (p.faseCalibracionBrazo == 3) {
    pantalla.println(F("IZQ: AJUSTAR X/Y"));
    pantalla.println(F("DER X: ORIENTAR"));
    pantalla.setCursor(0, 40);
    pantalla.print(F("Z-DIN04 real: "));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.print(F("p"));
    pantalla.setCursor(0, 49);
    pantalla.print(F("ROT:"));
    pantalla.print(anguloServoRotacion);
    pantalla.print(F(" X: CATCH"));
  } else if (esV2 && p.faseCalibracionBrazo == 14) {
    pantalla.println(F("AGARRO LA PIEZA?"));
    pantalla.println(F("X: SI"));
    pantalla.println(F("CUADRADO: NO"));
  } else if (p.faseCalibracionBrazo == 0) {
    pantalla.println(F("PIEZA EN MOVIMIENTO"));
    pantalla.println(F("CAMARA + ENCODER"));
    pantalla.setCursor(0, 47);
    pantalla.print(F("Z-DIN04 real: "));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.print(F("p"));
  } else if (p.faseCalibracionBrazo == 10) {
    pantalla.println(F("PIEZA ENTREGADA"));
    pantalla.println(F("PREPARANDO SIGUIENTE"));
    pantalla.setCursor(0, 47);
    pantalla.print(F("Z-DIN04 real: "));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.print(F("p"));
  } else {
    pantalla.println(F("SECUENCIA AUTOMATICA"));
    pantalla.print(F("Z-DIN04 real: "));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.println(F("p"));
  }
  pantalla.setCursor(0, 57);
  if (esV2 && p.faseCalibracionBrazo == 14) {
    pantalla.print(F("TRI:SALIR"));
  } else {
    pantalla.print(F("TRI:CANCELAR"));
  }
}

void procesarReporteDiagnosticoV2() {
  const ControlCamaraCompartido control = copiarControlCamara();
  if (!control.automaticoV2Activo) {
    ultimaCausaDiagnosticoV2Reportada = 0xFF;
    return;
  }

  const uint32_t ahora = millis();
  const DiagnosticoDeteccionV2 diag = copiarDiagnosticoV2();
  const bool cambio = diag.causa != ultimaCausaDiagnosticoV2Reportada;
  const bool bloqueoObjetivo =
    diag.causa == V2_DIAG_OBJETIVO_ACTIVO ||
    diag.causa == V2_DIAG_BRAZO_OCUPADO ||
    diag.causa == V2_DIAG_ESPERANDO_DESAPARICION ||
    diag.causa == V2_DIAG_NO_REARMADO;
  if (!cambio && (bloqueoObjetivo ||
                  ahora - ultimoReporteDiagnosticoV2 < 1000UL)) return;
  ultimaCausaDiagnosticoV2Reportada = diag.causa;
  ultimoReporteDiagnosticoV2 = ahora;

  Serial.print(F("[V2][DIAG] "));
  if (diag.causa == V2_DIAG_PUBLICADO) {
    Serial.print(F("PUBLICADO"));
  } else {
    Serial.print(F("bloqueo="));
    Serial.print(nombreCausaDiagnosticoV2(diag.causa));
  }
  Serial.print(F(" age="));
  if (diag.edadEncoderMs == UINT32_MAX) Serial.print(F("NA"));
  else Serial.print(diag.edadEncoderMs);
  Serial.print(F("ms flags=0x"));
  Serial.print(diag.flagsEncoder, HEX);
  Serial.print(F(" enc="));
  Serial.print(diag.conteoEncoder);
  Serial.print(F(" vel="));
  Serial.print(diag.velocidadEncoderMmS, 1);
  Serial.print(F(" husky="));
  Serial.print(diag.resultadosHusky);
  Serial.print(F(" valid="));
  Serial.print(diag.candidatosValidos);
  Serial.print(F(" unique="));
  Serial.print(diag.candidatosUnicos);
  Serial.print(F(" dup="));
  Serial.print(diag.duplicadosDescartados);
  Serial.print(F(" rejH="));
  Serial.print(diag.rechazadosHomografia);
  Serial.print(F(" rejB="));
  Serial.print(diag.rechazadosFueraBanda);
  Serial.print(F(" clase="));
  Serial.print(diag.clase);
  Serial.print(F(" n="));
  Serial.print(diag.deteccionesConsecutivas);
  Serial.print(F(" x="));
  Serial.print(diag.xMm, 1);
  Serial.print(F(" y="));
  Serial.print(diag.yMm, 1);
  Serial.print(F(" yc="));
  Serial.print(diag.yCompensadaMm, 1);
  Serial.print(F(" dx="));
  Serial.print(diag.dispersionXmm, 1);
  Serial.print(F(" dy="));
  Serial.print(diag.dispersionYCompensadaMm, 1);
  Serial.print(F(" d="));
  Serial.print(diag.desplazamientoMm, 1);
  Serial.print(F(" relY="));
  Serial.print(static_cast<int>(diag.relacionYEncoder));
  Serial.print(F(" query="));
  Serial.print(diag.duracionConsultaMs);
  Serial.print(F("ms dq="));
  Serial.print(diag.recorridoDuranteConsultaMm, 1);
  Serial.print(F(" encQ="));
  Serial.print(diag.conteoAntesConsulta);
  Serial.print('/');
  Serial.print(diag.conteoAsociadoCamara);
  Serial.print('/');
  Serial.print(diag.conteoDespuesConsulta);
  if (diag.secuenciaPublicada != 0) {
    Serial.print(F(" seq="));
    Serial.print(diag.secuenciaPublicada);
  }
  Serial.println();
}

void mostrarCalibracionEncoder(const PaquetePortentaAESP &p) {
  dibujarTitulo(F("CAL ENCODER"));
  pantalla.setCursor(0, 13);
  switch (p.faseCalibracionBrazo) {
    case 0:
      pantalla.println(F("PONGA BANDA AL 50%"));
      pantalla.setCursor(0, 25);
      pantalla.println(F("AVANCE CAMARA->BRAZO"));
      pantalla.setCursor(0, 38);
      pantalla.println(F("X: INICIAR MEDICION"));
      break;
    case 1:
      pantalla.println(F("ESTABILIZANDO 2 S"));
      pantalla.setCursor(0, 28);
      pantalla.println(F("MANTENGA EL 50%"));
      break;
    case 2:
      pantalla.println(F("MIDIENDO 5 S"));
      pantalla.setCursor(0, 28);
      pantalla.println(F("NO CAMBIE VELOCIDAD"));
      break;
    case 3:
      pantalla.println(F("MEDICION TERMINADA"));
      pantalla.setCursor(0, 28);
      pantalla.println(F("DETENGA LA BANDA"));
      break;
    case 4:
      pantalla.println(F("CALIBRACION OK"));
      pantalla.setCursor(0, 28);
      pantalla.println(F("BANDA DETENIDA"));
      break;
    case 5:
      pantalla.println(F("MEDICION INVALIDA"));
      pantalla.setCursor(0, 28);
      pantalla.println(F("X: REINTENTAR"));
      break;
    default:
      pantalla.println(F("FASE DESCONOCIDA"));
      break;
  }
  pantalla.setCursor(0, 51);
  pantalla.print(F("V:"));
  pantalla.print(fabsf(p.velocidadEncoderUmS / 1000000.0f), 1);
  pantalla.print(F(" mm/s C:"));
  pantalla.print(p.conteoEncoder);
}

void actualizarPantallaESP32(const PaquetePortentaAESP &p) {
  if (p.estadoSistema != estadoRemotoAnterior) {
    estadoRemotoAnterior = p.estadoSistema;
    inicioEstadoRemoto = millis();
  }
  pantalla.clearDisplay();

  switch (p.estadoSistema) {
    case SISTEMA_ARRANQUE_SEGURO:
      dibujarChecklistConexiones(true, false, 0);
      break;

    case SISTEMA_ESPERANDO_I2C:
      dibujarChecklistConexiones(true, false, 0);
      break;

    case SISTEMA_ESPERA_5S: {
      const uint32_t transcurrido = millis() - inicioEstadoRemoto;
      const uint32_t restante = transcurrido < 5000 ? 5000 - transcurrido : 0;
      dibujarChecklistConexiones(true, true, restante);
      break;
    }

    case SISTEMA_CALIBRANDO_CAMARA:
      mostrarCalibracionCamara();
      break;

    case SISTEMA_CALIBRANDO_BRAZO:
      dibujarTitulo(F("CALIBRACION BRAZO"));
      pantalla.setCursor(0, 14);
      pantalla.println(nombreFaseBrazo(p.faseCalibracionBrazo));
      pantalla.setCursor(0, 28);
      pantalla.print(F("LIM:")); pantalla.print(p.flagsLimites, HEX);
      pantalla.print(F(" ENC:")); pantalla.print(p.conteoEncoder);
      pantalla.setCursor(0, 41);
      pantalla.print(F("FASE:")); pantalla.print(p.faseCalibracionBrazo);
      pantalla.setCursor(0, 55);
      pantalla.print(F("TRI: CANCELAR"));
      break;

    case SISTEMA_CALIBRANDO_ENCODER:
      mostrarCalibracionEncoder(p);
      break;

    case SISTEMA_ESPERANDO_CONTROL:
      dibujarTitulo(F("ESPERANDO CONTROL"));
      pantalla.setCursor(0, 19);
      pantalla.println(F("ENLACE I2C ESTABLE"));
      pantalla.setCursor(0, 34);
      pantalla.println(F("CONECTE EL CONTROL"));
      break;

    case SISTEMA_CHECKLIST:
      dibujarChecklistConexiones(true, false, 0);
      break;

    case SISTEMA_MENU_PRINCIPAL:
      mostrarMenuPrincipal(p);
      break;

    case SISTEMA_MENU_CALIBRACIONES:
      mostrarMenuCalibraciones(p);
      break;

    case SISTEMA_PRUEBA_SERVOS:
      mostrarPruebaServos();
      break;

    case SISTEMA_DIAGNOSTICO:
      mostrarDiagnostico(p);
      break;

    case SISTEMA_MODO_MANUAL:
      mostrarModoManual(p);
      break;

    case SISTEMA_MODO_AUTOMATICO:
      mostrarModoAutomatico(p);
      break;

    case SISTEMA_MODO_AUTOMATICO_V2:
      mostrarModoAutomaticoV2(p);
      break;

    case SISTEMA_ENTRENAMIENTO_ML:
      mostrarEntrenamientoML(p);
      break;
    case SISTEMA_PRUEBA_ENCODER:
      dibujarTitulo(F("PRUEBA DE ENCODER"));
      pantalla.setCursor(0, 14);
      pantalla.print(F("ESTADO: "));
      pantalla.println(p.faseCalibracionBrazo == 2 ? F("RESULTADO") :
        (p.faseCalibracionBrazo == 1 ? F("MIDIENDO") : F("LISTO")));
      pantalla.setCursor(0, 25);
      pantalla.print(F("PULSOS: "));
      pantalla.println(p.conteoEncoder);
      pantalla.setCursor(0, 36);
      pantalla.print(F("TIEMPO: "));
      pantalla.print(p.velocidadEncoderUmS / 1000.0f, 3);
      pantalla.println(F(" s"));
      pantalla.setCursor(0, 47);
      pantalla.println(p.faseCalibracionBrazo == 1 ? F("X: DETENER") :
        (p.faseCalibracionBrazo == 2 ? F("RESULTADO GUARDADO") : F("X: INICIAR")));
      pantalla.setCursor(0, 56);
      pantalla.println(F("O: CERO TRI: SALIR"));
      break;

    case SISTEMA_ERROR:
      dibujarTitulo(F("ERROR DEL SISTEMA"));
      pantalla.setCursor(0, 17);
      pantalla.println(textoErrorSistema(p.errorSistema));
      pantalla.setCursor(0, 32);
      pantalla.print(F("CODIGO DETALLE: "));
      pantalla.println(p.errorSistema);
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
  Serial.begin(BAUD_LOG_ESP);
  Serial.println();
  Serial.println(F("[BOOT] ESP32 integrada iniciando"));

  sesionArranque = static_cast<uint16_t>(esp_random());
  if (sesionArranque == 0) {
    sesionArranque = 1;
  }
  inicializarSnapshotSeguro();

  Serial.println(F("[ENC] Telemetria ABZ recibida desde Portenta por I2C"));

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

  // UART1 no consulta la camara; por tanto no bloquea setup().
  HuskyUART.begin(
    HUSKY_BAUDRATE,
    SERIAL_8N1,
    HUSKY_RX_PIN,
    HUSKY_TX_PIN
  );
  Serial.println(F("[BOOT] HUSKYLENS UART1 RX32/TX33 a 115200"));

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
  // perifericos a medio inicializar. Si el primer begin coincide con el
  // arranque de la Portenta, loop() vuelve a levantarlo automaticamente.
  iniciarI2CEsclavo(false);
  sistemaBaseListo = true;
  prepararSnapshotI2C();

  Serial.print(F("[BOOT] Sesion ESP: "));
  Serial.println(sesionArranque);
  Serial.println(F("[BOOT] Sistema base listo"));
}

void loop() {
  // Bluepad32 conserva prioridad funcional en cada iteracion.
  BP32.update();
  processControllers();
  vigilarEncoderRemoto();
  mantenerI2CEsclavoRecuperable();

  procesarRecepcionI2C();
  procesarPinzaAutomaticaV2();
  vaciarColaLogV2();
  intentarInicializarOLEDNoBloqueante();
  prepararSnapshotI2C();
  procesarReporteDiagnosticoV2();
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
    Serial.print(rxI2CProtocoloIncorrecto);
    Serial.print(F(" requests="));
    Serial.print(solicitudesLecturaI2C);
    Serial.print(F(" reinicios="));
    Serial.println(reiniciosI2CEsclavo);
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

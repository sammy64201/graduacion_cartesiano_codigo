// Coordinador general para Arduino Portenta H7 + Machine Control.
// Mantiene el nucleo mecanico del sketch funcional MAster/MAster.ino e integra
// arranque autonomo, camara, checklist, menu de cuatro opciones y modo automatico.

#include <Arduino_MachineControl.h>
#include <Wire.h>
#include "mbed.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ProtocoloI2C.h"

using namespace machinecontrol;
using namespace ProtocoloI2C;

//-------------------------------------------------------------------------------------------------
// CONFIGURACION MECANICA CONSERVADA
//-------------------------------------------------------------------------------------------------
// Canales de salida de Machine Control (no son GPIO Arduino convencionales).
const int pP_Z = 0;
const int pP_X = 4;
const int pP_Y = 2;

const int pD_Z = 1;
const int pD_X = 5;
const int pD_Y = 3;

// El ticker conmuta STEP cada 100 us. Un paso se cuenta en el flanco ascendente.
const float velocidadMotores = 0.0001f;
mbed::Ticker motorTicker;

const uint16_t DIV_MANUAL = 1;
const uint16_t DIV_CAL_RAPIDA = 2;
const uint16_t DIV_CAL_LENTA = 8;
const uint16_t DIV_HOME = 2;
const uint16_t DIV_POSICION = 1;

const long PASOS_SEPARACION = 300;
const long RANGO_MINIMO_VALIDO = 100;
const unsigned long TIMEOUT_FASE_MS = 180000UL;
const unsigned long TIMEOUT_MOVIMIENTO_MS = 180000UL;

const float RANGO_FISICO_X_MM = 496.0f;
const float RANGO_FISICO_Y_MM = 337.0f;
const float MARGEN_SEGURIDAD_MM = 2.0f;

// Ajuste fisico camara -> brazo. Se aplica swap, luego signo y finalmente offset.
constexpr bool CAMERA_SWAP_XY = false;
constexpr int8_t CAMERA_SIGN_X = -1;
constexpr int8_t CAMERA_SIGN_Y = -1;
constexpr float CAMERA_OFFSET_X_MM = 0.0f;
constexpr float CAMERA_OFFSET_Y_MM = 0.0f;

static_assert(CAMERA_SIGN_X == 1 || CAMERA_SIGN_X == -1, "CAMERA_SIGN_X debe ser +/-1");
static_assert(CAMERA_SIGN_Y == 1 || CAMERA_SIGN_Y == -1, "CAMERA_SIGN_Y debe ser +/-1");

//-------------------------------------------------------------------------------------------------
// PERIODOS Y TIMEOUTS DEL COORDINADOR
//-------------------------------------------------------------------------------------------------
const unsigned long RETARDO_ARRANQUE_ESP32_MS = 3000UL;
const unsigned long PERIODO_CONTROL_MS = 5UL;
const unsigned long PERIODO_ESTADO_ESP_MS = 100UL;  // OLED <= 10 Hz.
const unsigned long TIMEOUT_I2C_MS = 150UL;
// Un callback I2C todavia podria responder aunque el loop de la ESP32 se
// hubiera congelado. La secuencia de su snapshot debe avanzar periodicamente.
const unsigned long TIMEOUT_SECUENCIA_ESP_MS = 150UL;
const unsigned long TIEMPO_ESTABILIZACION_I2C_MS = 5000UL;
const unsigned long TIMEOUT_CAMARA_MS = 240000UL;
const uint8_t MAX_PAQUETES_INVALIDOS_CONSECUTIVOS = 10;

//-------------------------------------------------------------------------------------------------
// ESTADOS LOCALES
//-------------------------------------------------------------------------------------------------
enum EstadoGeneral : uint8_t {
    EST_BOOT_SAFE = 0,
    EST_WAIT_I2C,
    EST_I2C_SETTLE,
    EST_CAMERA_CALIBRATION,
    EST_ARM_CALIBRATION,
    EST_WAIT_CONTROLLER,
    EST_FINAL_CHECKLIST,
    EST_MAIN_MENU,
    EST_MANUAL,
    EST_AUTOMATICO,
    EST_USER_ARM_CALIBRATION,
    EST_USER_CAMERA_CALIBRATION,
    EST_SYSTEM_ERROR
};

enum FaseCalibracion : uint8_t {
    CAL_ESPERA = 0,
    CAL_X_MIN_1,
    CAL_X_MIN_LIBERAR,
    CAL_X_MIN_SEPARAR,
    CAL_X_MIN_2,
    CAL_X_MAX_1,
    CAL_X_MAX_LIBERAR,
    CAL_X_MAX_SEPARAR,
    CAL_X_MAX_2,
    CAL_Y_MIN_1,
    CAL_Y_MIN_LIBERAR,
    CAL_Y_MIN_SEPARAR,
    CAL_Y_MIN_2,
    CAL_Y_MAX_1,
    CAL_Y_MAX_LIBERAR,
    CAL_Y_MAX_SEPARAR,
    CAL_Y_MAX_2,
    CAL_Z_MIN_1,
    CAL_Z_MIN_LIBERAR,
    CAL_Z_MIN_SEPARAR,
    CAL_Z_MIN_2,
    CAL_Z_MAX_1,
    CAL_Z_MAX_LIBERAR,
    CAL_Z_MAX_SEPARAR,
    CAL_Z_MAX_2,
    CAL_HOME,
    CAL_COMPLETA,
    CAL_ERROR
};

enum PropietarioMovimiento : uint8_t {
    MOV_SIN_PROPIETARIO = 0,
    MOV_TERMINAL,
    MOV_AUTOMATICO
};

enum CodigoErrorLocal : uint8_t {
    ERROR_NINGUNO = 0,
    ERROR_TIMEOUT_I2C = 1,
    ERROR_PROTOCOLO_INVALIDO = 2,
    ERROR_TIMEOUT_CAMARA = 3,
    ERROR_CAMARA = 4,
    ERROR_CALIBRACION_BRAZO = 5,
    ERROR_FINALES_INCOHERENTES = 6,
    ERROR_TIMEOUT_MOVIMIENTO = 7,
    ERROR_FINAL_INESPERADO = 8,
    ERROR_CHECKLIST = 9,
    ERROR_OBJETIVO_INVALIDO = 10,
    ERROR_REINICIO_ESP32 = 11,
    ERROR_CANCELADO = 12
};

enum ResultadoChecklist : uint8_t {
    CHECK_OK = 0,
    CHECK_I2C,
    CHECK_PROTOCOLO,
    CHECK_CAMARA,
    CHECK_HOMOGRAFIA,
    CHECK_MODELO,
    CHECK_XY,
    CHECK_Z,
    CHECK_HOME,
    CHECK_MOTORES,
    CHECK_BT,
    CHECK_CAL_ERROR,
    CHECK_FINALES
};

//-------------------------------------------------------------------------------------------------
// VARIABLES COMPARTIDAS CON EL TICKER
//-------------------------------------------------------------------------------------------------
volatile int8_t movX = 0;
volatile int8_t movY = 0;
volatile int8_t movZ = 0;

volatile bool pulsoX = false;
volatile bool pulsoY = false;
volatile bool pulsoZ = false;

volatile uint16_t cuentaX = 0;
volatile uint16_t cuentaY = 0;
volatile uint16_t cuentaZ = 0;

volatile uint16_t divisorX = DIV_MANUAL;
volatile uint16_t divisorY = DIV_MANUAL;
volatile uint16_t divisorZ = DIV_MANUAL;

volatile long pasosX = 0;
volatile long pasosY = 0;
volatile long pasosZ = 0;

volatile bool objetivoXActivo = false;
volatile bool objetivoYActivo = false;
volatile bool objetivoZActivo = false;
volatile long objetivoX = 0;
volatile long objetivoY = 0;
volatile long objetivoZ = 0;

//-------------------------------------------------------------------------------------------------
// ESTADO MECANICO Y DE CALIBRACION
//-------------------------------------------------------------------------------------------------
bool limiteXmas = false;
bool limiteXmenos = false;
bool limiteYmenos = false;
bool limiteYmas = false;
bool limiteZarriba = false;
bool limiteZabajo = false;

FaseCalibracion faseCal = CAL_ESPERA;
unsigned long inicioFase = 0;
bool homeIniciado = false;
bool calibracionXYValida = false;
bool calibracionZValida = false;
const char *mensajeErrorCalibracion = "";

long rangoXPasos = 0;
long rangoYPasos = 0;
long rangoZPasos = 0;
float pasosPorMmX = 0.0f;
float pasosPorMmY = 0.0f;

bool movimientoPosicionadoActivo = false;
PropietarioMovimiento propietarioMovimiento = MOV_SIN_PROPIETARIO;
unsigned long inicioMovimientoPosicionado = 0;
float objetivoXmm = 0.0f;
float objetivoYmm = 0.0f;
float objetivoZmm = 0.0f;
long objetivoCartesianoXPasos = 0;
long objetivoCartesianoYPasos = 0;

//-------------------------------------------------------------------------------------------------
// ESTADO DEL COORDINADOR Y DEL ENLACE
//-------------------------------------------------------------------------------------------------
EstadoGeneral estadoGeneral = EST_BOOT_SAFE;
unsigned long inicioEstadoGeneral = 0;
CodigoErrorLocal errorSistema = ERROR_NINGUNO;
const char *mensajeErrorSistema = "";
ResultadoChecklist ultimoResultadoChecklist = CHECK_OK;
bool checklistInicialCompletado = false;

uint8_t opcionMenu = 0;
int8_t entradaMenuYAnterior = 0;

PaqueteESPAPortenta paqueteESP = {};
bool existePaqueteValido = false;
bool protocoloValido = false;
unsigned long ultimoPaqueteValidoMs = 0;
uint8_t fallosPaqueteConsecutivos = 0;
uint16_t sesionArranqueESP = 0;
bool sesionESPConocida = false;
bool cambioSesionESPPendiente = false;
uint8_t ultimaSecuenciaPaqueteESP = 0;
bool secuenciaPaqueteESPConocida = false;
unsigned long ultimoCambioSecuenciaESPMs = 0;

int8_t joystickX = 0;
int8_t joystickY = 0;
int8_t joystickZ = 0;
bool btConectado = false;
bool botonX = false;
bool botonTriangulo = false;
bool botonXAnterior = false;
bool botonTrianguloAnterior = false;
bool eventoBotonX = false;
bool eventoBotonTriangulo = false;
uint8_t posServoRot = 90;
uint8_t posServoPin = 90;

uint8_t estadoCamara = 0;
uint8_t errorCamara = 0;
uint8_t flagsCamara = 0;
uint8_t muestrasTag[4] = {0, 0, 0, 0};
uint8_t claseObjetivo = 0;
int16_t objetivoCamaraX10 = 0;
int16_t objetivoCamaraY10 = 0;
uint16_t secuenciaObjetivoRecibida = 0;

uint8_t comandoCamaraActual = 0;
uint8_t secuenciaComandoCamara = 0;
bool inicioCalibracionCamaraObservado = false;
bool volverChecklistTrasCalibracionCamara = false;

uint16_t ackSecuenciaObjetivo = 0;
uint8_t codigoAckObjetivo = 0;
uint16_t secuenciaObjetivoEnMovimiento = 0;

unsigned long tiempoEncendidoSistema = 0;
unsigned long tAnteriorI2C = 0;
unsigned long tAnteriorEstadoESP = 0;
bool comunicacionI2CHabilitada = false;
uint32_t lecturasI2COk = 0;
uint32_t lecturasI2CError = 0;
uint32_t enviosI2COk = 0;
uint32_t enviosI2CError = 0;
unsigned long ultimoReporteI2C = 0;

String lineaTerminal = "";

// Declaraciones de funciones que cruzan secciones.
void entrarErrorSistema(CodigoErrorLocal codigo, const char *mensaje);
void cambiarEstadoGeneral(EstadoGeneral nuevoEstado);
void cancelarMovimientoPosicionado(const char *motivo, bool posicionPerdida);

//-------------------------------------------------------------------------------------------------
// ACCESO ATOMICO A CONTADORES Y OBJETIVOS
//-------------------------------------------------------------------------------------------------
long leerPasosX() {
    noInterrupts();
    long valor = pasosX;
    interrupts();
    return valor;
}

long leerPasosY() {
    noInterrupts();
    long valor = pasosY;
    interrupts();
    return valor;
}

long leerPasosZ() {
    noInterrupts();
    long valor = pasosZ;
    interrupts();
    return valor;
}

void fijarPasosX(long valor) {
    noInterrupts();
    pasosX = valor;
    interrupts();
}

void fijarPasosY(long valor) {
    noInterrupts();
    pasosY = valor;
    interrupts();
}

void fijarPasosZ(long valor) {
    noInterrupts();
    pasosZ = valor;
    interrupts();
}

bool objetivoXEnCurso() {
    noInterrupts();
    bool valor = objetivoXActivo;
    interrupts();
    return valor;
}

bool objetivoYEnCurso() {
    noInterrupts();
    bool valor = objetivoYActivo;
    interrupts();
    return valor;
}

bool objetivoZEnCurso() {
    noInterrupts();
    bool valor = objetivoZActivo;
    interrupts();
    return valor;
}

bool motoresEnMovimiento() {
    noInterrupts();
    bool valor = (movX != 0 || movY != 0 || movZ != 0 ||
                  objetivoXActivo || objetivoYActivo || objetivoZActivo);
    interrupts();
    return valor;
}

//-------------------------------------------------------------------------------------------------
// FINALES DE CARRERA: LOGICA NC CONSERVADA
//-------------------------------------------------------------------------------------------------
void leerFinalesCarrera() {
    // true = presionado o circuito NC abierto/desconectado.
    limiteXmas = !digital_inputs.read(DIN_READ_CH_PIN_01);
    limiteXmenos = !digital_inputs.read(DIN_READ_CH_PIN_00);
    limiteYmenos = !digital_inputs.read(DIN_READ_CH_PIN_02);
    limiteYmas = !digital_inputs.read(DIN_READ_CH_PIN_03);
    limiteZarriba = !digital_inputs.read(DIN_READ_CH_PIN_04);
    limiteZabajo = !digital_inputs.read(DIN_READ_CH_PIN_05);
}

bool finalesCoherentes() {
    return !(limiteXmas && limiteXmenos) &&
           !(limiteYmas && limiteYmenos) &&
           !(limiteZarriba && limiteZabajo);
}

bool algunFinalActivo() {
    return limiteXmas || limiteXmenos || limiteYmas ||
           limiteYmenos || limiteZarriba || limiteZabajo;
}

//-------------------------------------------------------------------------------------------------
// TICKER DE GENERACION DE PULSOS
//-------------------------------------------------------------------------------------------------
void generarPulsoMotor() {
    if (movX != 0) {
        cuentaX++;
        if (cuentaX >= divisorX) {
            cuentaX = 0;
            pulsoX = !pulsoX;
            digital_outputs.set(pP_X, pulsoX ? HIGH : LOW);
            if (pulsoX) {
                pasosX += movX;
                if (objetivoXActivo &&
                    ((movX > 0 && pasosX >= objetivoX) ||
                     (movX < 0 && pasosX <= objetivoX))) {
                    pasosX = objetivoX;
                    movX = 0;
                    objetivoXActivo = false;
                    digital_outputs.set(pP_X, LOW);
                }
            }
        }
    } else {
        cuentaX = 0;
        pulsoX = false;
        digital_outputs.set(pP_X, LOW);
    }

    if (movY != 0) {
        cuentaY++;
        if (cuentaY >= divisorY) {
            cuentaY = 0;
            pulsoY = !pulsoY;
            digital_outputs.set(pP_Y, pulsoY ? HIGH : LOW);
            if (pulsoY) {
                pasosY += movY;
                if (objetivoYActivo &&
                    ((movY > 0 && pasosY >= objetivoY) ||
                     (movY < 0 && pasosY <= objetivoY))) {
                    pasosY = objetivoY;
                    movY = 0;
                    objetivoYActivo = false;
                    digital_outputs.set(pP_Y, LOW);
                }
            }
        }
    } else {
        cuentaY = 0;
        pulsoY = false;
        digital_outputs.set(pP_Y, LOW);
    }

    if (movZ != 0) {
        cuentaZ++;
        if (cuentaZ >= divisorZ) {
            cuentaZ = 0;
            pulsoZ = !pulsoZ;
            digital_outputs.set(pP_Z, pulsoZ ? HIGH : LOW);
            if (pulsoZ) {
                pasosZ += movZ;
                if (objetivoZActivo &&
                    ((movZ > 0 && pasosZ >= objetivoZ) ||
                     (movZ < 0 && pasosZ <= objetivoZ))) {
                    pasosZ = objetivoZ;
                    movZ = 0;
                    objetivoZActivo = false;
                    digital_outputs.set(pP_Z, LOW);
                }
            }
        }
    } else {
        cuentaZ = 0;
        pulsoZ = false;
        digital_outputs.set(pP_Z, LOW);
    }
}

//-------------------------------------------------------------------------------------------------
// PRIMITIVAS DE MOVIMIENTO CONSERVADAS
//-------------------------------------------------------------------------------------------------
void detenerX() {
    noInterrupts();
    movX = 0;
    objetivoXActivo = false;
    cuentaX = 0;
    pulsoX = false;
    interrupts();
    digital_outputs.set(pP_X, LOW);
}

void detenerY() {
    noInterrupts();
    movY = 0;
    objetivoYActivo = false;
    cuentaY = 0;
    pulsoY = false;
    interrupts();
    digital_outputs.set(pP_Y, LOW);
}

void detenerZ() {
    noInterrupts();
    movZ = 0;
    objetivoZActivo = false;
    cuentaZ = 0;
    pulsoZ = false;
    interrupts();
    digital_outputs.set(pP_Z, LOW);
}

void detenerTodos() {
    detenerX();
    detenerY();
    detenerZ();
}

void moverXContinuo(int8_t direccion, uint16_t divisor) {
    if (direccion == 0) {
        detenerX();
        return;
    }
    noInterrupts();
    bool igual = movX == direccion && !objetivoXActivo && divisorX == divisor;
    interrupts();
    if (igual) return;
    detenerX();
    delayMicroseconds(150);
    digital_outputs.set(pD_X, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    divisorX = divisor;
    cuentaX = 0;
    pulsoX = false;
    objetivoXActivo = false;
    movX = direccion;
    interrupts();
}

void moverYContinuo(int8_t direccion, uint16_t divisor) {
    if (direccion == 0) {
        detenerY();
        return;
    }
    noInterrupts();
    bool igual = movY == direccion && !objetivoYActivo && divisorY == divisor;
    interrupts();
    if (igual) return;
    detenerY();
    delayMicroseconds(150);
    digital_outputs.set(pD_Y, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    divisorY = divisor;
    cuentaY = 0;
    pulsoY = false;
    objetivoYActivo = false;
    movY = direccion;
    interrupts();
}

void moverZContinuo(int8_t direccion, uint16_t divisor) {
    if (direccion == 0) {
        detenerZ();
        return;
    }
    noInterrupts();
    bool igual = movZ == direccion && !objetivoZActivo && divisorZ == divisor;
    interrupts();
    if (igual) return;
    detenerZ();
    delayMicroseconds(150);
    digital_outputs.set(pD_Z, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    divisorZ = divisor;
    cuentaZ = 0;
    pulsoZ = false;
    objetivoZActivo = false;
    movZ = direccion;
    interrupts();
}

void moverXHasta(long destino, uint16_t divisor) {
    long actual = leerPasosX();
    if (actual == destino) {
        detenerX();
        return;
    }
    int8_t direccion = destino > actual ? 1 : -1;
    detenerX();
    delayMicroseconds(150);
    digital_outputs.set(pD_X, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    objetivoX = destino;
    objetivoXActivo = true;
    divisorX = divisor;
    cuentaX = 0;
    pulsoX = false;
    movX = direccion;
    interrupts();
}

void moverYHasta(long destino, uint16_t divisor) {
    long actual = leerPasosY();
    if (actual == destino) {
        detenerY();
        return;
    }
    int8_t direccion = destino > actual ? 1 : -1;
    detenerY();
    delayMicroseconds(150);
    digital_outputs.set(pD_Y, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    objetivoY = destino;
    objetivoYActivo = true;
    divisorY = divisor;
    cuentaY = 0;
    pulsoY = false;
    movY = direccion;
    interrupts();
}

void moverZHasta(long destino, uint16_t divisor) {
    long actual = leerPasosZ();
    if (actual == destino) {
        detenerZ();
        return;
    }
    int8_t direccion = destino > actual ? 1 : -1;
    detenerZ();
    delayMicroseconds(150);
    digital_outputs.set(pD_Z, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    objetivoZ = destino;
    objetivoZActivo = true;
    divisorZ = divisor;
    cuentaZ = 0;
    pulsoZ = false;
    movZ = direccion;
    interrupts();
}

void aplicarBloqueoPorFinales() {
    if (movX > 0 && limiteXmas) detenerX();
    if (movX < 0 && limiteXmenos) detenerX();
    if (movY > 0 && limiteYmas) detenerY();
    if (movY < 0 && limiteYmenos) detenerY();
    if (movZ > 0 && limiteZarriba) detenerZ();
    if (movZ < 0 && limiteZabajo) detenerZ();
}

//-------------------------------------------------------------------------------------------------
// ESCALA, POSICION Y CINEMATICA CARTESIANA
//-------------------------------------------------------------------------------------------------
bool escalaConfigurada() {
    return pasosPorMmX > 0.0f && pasosPorMmY > 0.0f;
}

float posicionXmm() {
    return pasosPorMmX > 0.0f ? (float)leerPasosX() / pasosPorMmX : 0.0f;
}

float posicionYmm() {
    return pasosPorMmY > 0.0f ? (float)leerPasosY() / pasosPorMmY : 0.0f;
}

float rangoXmm() {
    return pasosPorMmX > 0.0f ? (float)rangoXPasos / pasosPorMmX : 0.0f;
}

float rangoYmm() {
    return pasosPorMmY > 0.0f ? (float)rangoYPasos / pasosPorMmY : 0.0f;
}

long limiteMinimoXPasos() { return -(rangoXPasos / 2); }
long limiteMaximoXPasos() { return rangoXPasos - rangoXPasos / 2; }
long limiteMinimoYPasos() { return -(rangoYPasos / 2); }
long limiteMaximoYPasos() { return rangoYPasos - rangoYPasos / 2; }

bool calcularEscalaAutomatica() {
    if (rangoXPasos <= 0 || rangoYPasos <= 0) {
        pasosPorMmX = 0.0f;
        pasosPorMmY = 0.0f;
        Serial.println(F("[CAL][ERROR] Rangos invalidos para calcular escala"));
        return false;
    }
    pasosPorMmX = (float)rangoXPasos / RANGO_FISICO_X_MM;
    pasosPorMmY = (float)rangoYPasos / RANGO_FISICO_Y_MM;
    Serial.print(F("[CAL] Escala X="));
    Serial.print(pasosPorMmX, 6);
    Serial.print(F(" pasos/mm, Y="));
    Serial.print(pasosPorMmY, 6);
    Serial.println(F(" pasos/mm"));
    return true;
}

bool cinematicaInversaCartesiana(float xMm, float yMm, float zMm,
                                 long &xPasosDestino, long &yPasosDestino) {
    if (!isfinite(xMm) || !isfinite(yMm) || !isfinite(zMm)) {
        Serial.println(F("[ERROR] Coordenada no finita rechazada"));
        return false;
    }
    if (!calibracionXYValida || !escalaConfigurada()) {
        Serial.println(F("[ERROR] X/Y no estan calibrados"));
        return false;
    }

    xPasosDestino = lroundf(xMm * pasosPorMmX);
    yPasosDestino = lroundf(yMm * pasosPorMmY);
    const long margenX = lroundf(MARGEN_SEGURIDAD_MM * pasosPorMmX);
    const long margenY = lroundf(MARGEN_SEGURIDAD_MM * pasosPorMmY);
    const long xMin = limiteMinimoXPasos() + margenX;
    const long xMax = limiteMaximoXPasos() - margenX;
    const long yMin = limiteMinimoYPasos() + margenY;
    const long yMax = limiteMaximoYPasos() - margenY;

    if (xPasosDestino < xMin || xPasosDestino > xMax ||
        yPasosDestino < yMin || yPasosDestino > yMax) {
        Serial.print(F("[ERROR] Objetivo fuera de rango seguro X="));
        Serial.print(xMm, 2);
        Serial.print(F(" Y="));
        Serial.println(yMm, 2);
        return false;
    }
    return true;
}

bool transformarCamaraABrazo(float camXmm, float camYmm,
                             float &brazoXmm, float &brazoYmm) {
    if (!isfinite(camXmm) || !isfinite(camYmm)) return false;
    const float baseX = CAMERA_SWAP_XY ? camYmm : camXmm;
    const float baseY = CAMERA_SWAP_XY ? camXmm : camYmm;
    brazoXmm = (float)CAMERA_SIGN_X * baseX + CAMERA_OFFSET_X_MM;
    brazoYmm = (float)CAMERA_SIGN_Y * baseY + CAMERA_OFFSET_Y_MM;
    return isfinite(brazoXmm) && isfinite(brazoYmm);
}

bool brazoEnHome() {
    return calibracionXYValida && calibracionZValida &&
           leerPasosX() == 0 && leerPasosY() == 0 && leerPasosZ() == 0 &&
           !motoresEnMovimiento() && !movimientoPosicionadoActivo;
}

//-------------------------------------------------------------------------------------------------
// MOVIMIENTO POSICIONADO XY (TERMINAL Y AUTOMATICO)
//-------------------------------------------------------------------------------------------------
bool iniciarMovimientoXY(float xMm, float yMm, float zMm,
                         PropietarioMovimiento propietario) {
    long destinoX = 0;
    long destinoY = 0;
    if (!cinematicaInversaCartesiana(xMm, yMm, zMm, destinoX, destinoY)) {
        return false;
    }

    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;

    objetivoXmm = xMm;
    objetivoYmm = yMm;
    objetivoZmm = zMm;
    objetivoCartesianoXPasos = destinoX;
    objetivoCartesianoYPasos = destinoY;

    Serial.print(propietario == MOV_AUTOMATICO ? F("[AUTO] Objetivo X=") : F("[XYZ] Objetivo X="));
    Serial.print(xMm, 2);
    Serial.print(F(" mm, Y="));
    Serial.print(yMm, 2);
    Serial.print(F(" mm; Z="));
    Serial.print(zMm, 2);
    Serial.println(F(" mm recibido pero no se movera"));

    moverXHasta(destinoX, DIV_POSICION);
    moverYHasta(destinoY, DIV_POSICION);

    movimientoPosicionadoActivo = objetivoXEnCurso() || objetivoYEnCurso();
    propietarioMovimiento = movimientoPosicionadoActivo ? propietario : MOV_SIN_PROPIETARIO;
    inicioMovimientoPosicionado = millis();

    if (!movimientoPosicionadoActivo) {
        Serial.println(propietario == MOV_AUTOMATICO
                           ? F("[AUTO] Brazo ya estaba en el objetivo")
                           : F("[XYZ] Brazo ya estaba en el objetivo"));
    }
    return true;
}

void cancelarMovimientoPosicionado(const char *motivo, bool posicionPerdida) {
    PropietarioMovimiento propietarioAnterior = propietarioMovimiento;
    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;

    if (posicionPerdida) calibracionXYValida = false;
    if (propietarioAnterior == MOV_AUTOMATICO && secuenciaObjetivoEnMovimiento != 0) {
        ackSecuenciaObjetivo = secuenciaObjetivoEnMovimiento;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
    }

    Serial.print(F("[XYZ] Movimiento detenido: "));
    Serial.println(motivo);
}

void actualizarMovimientoPosicionado() {
    if (!movimientoPosicionadoActivo) return;

    const bool finalInesperado =
        (movX > 0 && limiteXmas) || (movX < 0 && limiteXmenos) ||
        (movY > 0 && limiteYmas) || (movY < 0 && limiteYmenos);

    if (finalInesperado) {
        cancelarMovimientoPosicionado("final de carrera inesperado", true);
        entrarErrorSistema(ERROR_FINAL_INESPERADO,
                           "Final de carrera durante movimiento XY");
        return;
    }

    if (millis() - inicioMovimientoPosicionado > TIMEOUT_MOVIMIENTO_MS) {
        cancelarMovimientoPosicionado("timeout", true);
        entrarErrorSistema(ERROR_TIMEOUT_MOVIMIENTO,
                           "Timeout de movimiento XY");
        return;
    }

    if (!objetivoXEnCurso() && !objetivoYEnCurso() && movX == 0 && movY == 0) {
        PropietarioMovimiento propietarioFinal = propietarioMovimiento;
        movimientoPosicionadoActivo = false;
        propietarioMovimiento = MOV_SIN_PROPIETARIO;
        Serial.println(propietarioFinal == MOV_AUTOMATICO
                           ? F("[AUTO] Objetivo alcanzado")
                           : F("[XYZ] Objetivo alcanzado"));
    }
}

//-------------------------------------------------------------------------------------------------
// CALIBRACION X/Y/Z CONSERVADA, AHORA PROCESADA DIRECTAMENTE DESDE loop()
//-------------------------------------------------------------------------------------------------
const char *nombreFaseCalibracion() {
    switch (faseCal) {
        case CAL_ESPERA: return "ESPERA";
        case CAL_X_MIN_1: return "BUSCANDO X-";
        case CAL_X_MIN_LIBERAR: return "LIBERANDO X-";
        case CAL_X_MIN_SEPARAR: return "SEPARANDO X-";
        case CAL_X_MIN_2: return "VERIFICANDO X-";
        case CAL_X_MAX_1: return "BUSCANDO X+";
        case CAL_X_MAX_LIBERAR: return "LIBERANDO X+";
        case CAL_X_MAX_SEPARAR: return "SEPARANDO X+";
        case CAL_X_MAX_2: return "VERIFICANDO X+";
        case CAL_Y_MIN_1: return "BUSCANDO Y-";
        case CAL_Y_MIN_LIBERAR: return "LIBERANDO Y-";
        case CAL_Y_MIN_SEPARAR: return "SEPARANDO Y-";
        case CAL_Y_MIN_2: return "VERIFICANDO Y-";
        case CAL_Y_MAX_1: return "BUSCANDO Y+";
        case CAL_Y_MAX_LIBERAR: return "LIBERANDO Y+";
        case CAL_Y_MAX_SEPARAR: return "SEPARANDO Y+";
        case CAL_Y_MAX_2: return "VERIFICANDO Y+";
        case CAL_Z_MIN_1: return "BUSCANDO Z ABAJO";
        case CAL_Z_MIN_LIBERAR: return "LIBERANDO Z ABAJO";
        case CAL_Z_MIN_SEPARAR: return "SEPARANDO Z ABAJO";
        case CAL_Z_MIN_2: return "VERIFICANDO Z ABAJO";
        case CAL_Z_MAX_1: return "BUSCANDO Z ARRIBA";
        case CAL_Z_MAX_LIBERAR: return "LIBERANDO Z ARRIBA";
        case CAL_Z_MAX_SEPARAR: return "SEPARANDO Z ARRIBA";
        case CAL_Z_MAX_2: return "VERIFICANDO Z ARRIBA";
        case CAL_HOME: return "YENDO A HOME";
        case CAL_COMPLETA: return "CALIBRACION OK";
        case CAL_ERROR: return "ERROR";
        default: return "DESCONOCIDA";
    }
}

void cambiarFaseCalibracion(FaseCalibracion nuevaFase) {
    faseCal = nuevaFase;
    inicioFase = millis();
    Serial.print(F("[CAL] "));
    Serial.println(nombreFaseCalibracion());
}

uint8_t codigoErrorCalibracion() {
    if (faseCal != CAL_ERROR) return 0;
    if (strcmp(mensajeErrorCalibracion, "Tiempo maximo excedido") == 0) return 1;
    if (strcmp(mensajeErrorCalibracion, "Rango X invalido") == 0) return 2;
    if (strcmp(mensajeErrorCalibracion, "Rango Y invalido") == 0) return 3;
    if (strcmp(mensajeErrorCalibracion, "Rango Z invalido") == 0) return 4;
    if (strcmp(mensajeErrorCalibracion, "No se pudo calcular la escala") == 0) return 5;
    if (strcmp(mensajeErrorCalibracion, "HOME interrumpido") == 0) return 6;
    return 255;
}

void detenerPorErrorCalibracion(const char *texto) {
    detenerTodos();
    calibracionXYValida = false;
    calibracionZValida = false;
    mensajeErrorCalibracion = texto;
    cambiarFaseCalibracion(CAL_ERROR);
    Serial.print(F("[CAL][ERROR] "));
    Serial.println(texto);
    entrarErrorSistema(ERROR_CALIBRACION_BRAZO, texto);
}

void iniciarCalibracionBrazo() {
    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    calibracionXYValida = false;
    calibracionZValida = false;
    pasosPorMmX = 0.0f;
    pasosPorMmY = 0.0f;
    rangoXPasos = 0;
    rangoYPasos = 0;
    rangoZPasos = 0;
    homeIniciado = false;
    mensajeErrorCalibracion = "";
    Serial.println(F("[CAL] Inicio autonomo de calibracion X/Y/Z"));
    cambiarFaseCalibracion(CAL_X_MIN_1);
}

void procesarCalibracionBrazo() {
    if (faseCal == CAL_ESPERA || faseCal == CAL_COMPLETA || faseCal == CAL_ERROR) {
        return;
    }

    if (millis() - inicioFase > TIMEOUT_FASE_MS) {
        detenerPorErrorCalibracion("Tiempo maximo excedido");
        return;
    }

    switch (faseCal) {
        case CAL_X_MIN_1:
            if (limiteXmenos) {
                detenerX();
                cambiarFaseCalibracion(CAL_X_MIN_LIBERAR);
            } else moverXContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MIN_LIBERAR:
            if (!limiteXmenos) {
                detenerX();
                moverXHasta(leerPasosX() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_X_MIN_SEPARAR);
            } else moverXContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MIN_SEPARAR:
            if (!objetivoXEnCurso()) cambiarFaseCalibracion(CAL_X_MIN_2);
            break;

        case CAL_X_MIN_2:
            if (limiteXmenos) {
                detenerX();
                fijarPasosX(0);
                cambiarFaseCalibracion(CAL_X_MAX_1);
            } else moverXContinuo(-1, DIV_CAL_LENTA);
            break;

        case CAL_X_MAX_1:
            if (limiteXmas) {
                detenerX();
                cambiarFaseCalibracion(CAL_X_MAX_LIBERAR);
            } else moverXContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MAX_LIBERAR:
            if (!limiteXmas) {
                detenerX();
                moverXHasta(leerPasosX() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_X_MAX_SEPARAR);
            } else moverXContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MAX_SEPARAR:
            if (!objetivoXEnCurso()) cambiarFaseCalibracion(CAL_X_MAX_2);
            break;

        case CAL_X_MAX_2:
            if (limiteXmas) {
                detenerX();
                rangoXPasos = leerPasosX();
                if (rangoXPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorErrorCalibracion("Rango X invalido");
                    return;
                }
                Serial.print(F("[CAL][X] Rango="));
                Serial.println(rangoXPasos);
                cambiarFaseCalibracion(CAL_Y_MIN_1);
            } else moverXContinuo(1, DIV_CAL_LENTA);
            break;

        case CAL_Y_MIN_1:
            if (limiteYmenos) {
                detenerY();
                cambiarFaseCalibracion(CAL_Y_MIN_LIBERAR);
            } else moverYContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MIN_LIBERAR:
            if (!limiteYmenos) {
                detenerY();
                moverYHasta(leerPasosY() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Y_MIN_SEPARAR);
            } else moverYContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MIN_SEPARAR:
            if (!objetivoYEnCurso()) cambiarFaseCalibracion(CAL_Y_MIN_2);
            break;

        case CAL_Y_MIN_2:
            if (limiteYmenos) {
                detenerY();
                fijarPasosY(0);
                cambiarFaseCalibracion(CAL_Y_MAX_1);
            } else moverYContinuo(-1, DIV_CAL_LENTA);
            break;

        case CAL_Y_MAX_1:
            if (limiteYmas) {
                detenerY();
                cambiarFaseCalibracion(CAL_Y_MAX_LIBERAR);
            } else moverYContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MAX_LIBERAR:
            if (!limiteYmas) {
                detenerY();
                moverYHasta(leerPasosY() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Y_MAX_SEPARAR);
            } else moverYContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MAX_SEPARAR:
            if (!objetivoYEnCurso()) cambiarFaseCalibracion(CAL_Y_MAX_2);
            break;

        case CAL_Y_MAX_2:
            if (limiteYmas) {
                detenerY();
                rangoYPasos = leerPasosY();
                if (rangoYPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorErrorCalibracion("Rango Y invalido");
                    return;
                }
                Serial.print(F("[CAL][Y] Rango="));
                Serial.println(rangoYPasos);
                cambiarFaseCalibracion(CAL_Z_MIN_1);
            } else moverYContinuo(1, DIV_CAL_LENTA);
            break;

        case CAL_Z_MIN_1:
            if (limiteZabajo) {
                detenerZ();
                cambiarFaseCalibracion(CAL_Z_MIN_LIBERAR);
            } else moverZContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MIN_LIBERAR:
            if (!limiteZabajo) {
                detenerZ();
                moverZHasta(leerPasosZ() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Z_MIN_SEPARAR);
            } else moverZContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MIN_SEPARAR:
            if (!objetivoZEnCurso()) cambiarFaseCalibracion(CAL_Z_MIN_2);
            break;

        case CAL_Z_MIN_2:
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(0);
                cambiarFaseCalibracion(CAL_Z_MAX_1);
            } else moverZContinuo(-1, DIV_CAL_LENTA);
            break;

        case CAL_Z_MAX_1:
            if (limiteZarriba) {
                detenerZ();
                cambiarFaseCalibracion(CAL_Z_MAX_LIBERAR);
            } else moverZContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MAX_LIBERAR:
            if (!limiteZarriba) {
                detenerZ();
                moverZHasta(leerPasosZ() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Z_MAX_SEPARAR);
            } else moverZContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MAX_SEPARAR:
            if (!objetivoZEnCurso()) cambiarFaseCalibracion(CAL_Z_MAX_2);
            break;

        case CAL_Z_MAX_2:
            if (limiteZarriba) {
                detenerZ();
                rangoZPasos = leerPasosZ();
                if (rangoZPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorErrorCalibracion("Rango Z invalido");
                    return;
                }
                Serial.print(F("[CAL][Z] Rango="));
                Serial.println(rangoZPasos);
                homeIniciado = false;
                cambiarFaseCalibracion(CAL_HOME);
            } else moverZContinuo(1, DIV_CAL_LENTA);
            break;

        case CAL_HOME: {
            const long centroX = rangoXPasos / 2;
            const long centroY = rangoYPasos / 2;
            const long centroZ = rangoZPasos / 2;
            if (!homeIniciado) {
                homeIniciado = true;
                moverXHasta(centroX, DIV_HOME);
                moverYHasta(centroY, DIV_HOME);
                moverZHasta(centroZ, DIV_HOME);
            }

            if (!objetivoXEnCurso() && !objetivoYEnCurso() && !objetivoZEnCurso() &&
                movX == 0 && movY == 0 && movZ == 0) {
                if (leerPasosX() != centroX || leerPasosY() != centroY ||
                    leerPasosZ() != centroZ) {
                    detenerPorErrorCalibracion("HOME interrumpido");
                    return;
                }
                fijarPasosX(0);
                fijarPasosY(0);
                fijarPasosZ(0);
                if (!calcularEscalaAutomatica()) {
                    detenerPorErrorCalibracion("No se pudo calcular la escala");
                    return;
                }
                calibracionXYValida = true;
                calibracionZValida = true;
                cambiarFaseCalibracion(CAL_COMPLETA);
                Serial.println(F("[CAL] Calibracion completa; HOME X=0 Y=0 Z=0"));
            }
            break;
        }

        case CAL_ESPERA:
        case CAL_COMPLETA:
        case CAL_ERROR:
            break;
    }
}

//-------------------------------------------------------------------------------------------------
// ADAPTACION DEL ESTADO LOCAL AL PROTOCOLO COMPARTIDO
//-------------------------------------------------------------------------------------------------
uint8_t estadoGeneralWire() {
    switch (estadoGeneral) {
        case EST_BOOT_SAFE: return SISTEMA_ARRANQUE_SEGURO;
        case EST_WAIT_I2C: return SISTEMA_ESPERANDO_I2C;
        case EST_I2C_SETTLE: return SISTEMA_ESPERA_5S;
        case EST_CAMERA_CALIBRATION:
        case EST_USER_CAMERA_CALIBRATION:
            return SISTEMA_CALIBRANDO_CAMARA;
        case EST_ARM_CALIBRATION:
        case EST_USER_ARM_CALIBRATION:
            return SISTEMA_CALIBRANDO_BRAZO;
        case EST_WAIT_CONTROLLER: return SISTEMA_ESPERANDO_CONTROL;
        case EST_FINAL_CHECKLIST: return SISTEMA_CHECKLIST;
        case EST_MAIN_MENU: return SISTEMA_MENU_PRINCIPAL;
        case EST_MANUAL: return SISTEMA_MODO_MANUAL;
        case EST_AUTOMATICO: return SISTEMA_MODO_AUTOMATICO;
        case EST_SYSTEM_ERROR: return SISTEMA_ERROR;
        default: return SISTEMA_ERROR;
    }
}

uint8_t errorSistemaWire() {
    switch (errorSistema) {
        case ERROR_NINGUNO: return SISTEMA_ERROR_NINGUNO;
        case ERROR_TIMEOUT_I2C:
        case ERROR_PROTOCOLO_INVALIDO:
        case ERROR_REINICIO_ESP32:
            return SISTEMA_ERROR_I2C;
        case ERROR_TIMEOUT_CAMARA:
        case ERROR_CAMARA:
            return SISTEMA_ERROR_CAMARA;
        case ERROR_CALIBRACION_BRAZO:
            return SISTEMA_ERROR_CALIBRACION_BRAZO;
        case ERROR_CHECKLIST:
            return SISTEMA_ERROR_CHECKLIST;
        case ERROR_OBJETIVO_INVALIDO:
            return SISTEMA_ERROR_OBJETIVO_FUERA_RANGO;
        case ERROR_TIMEOUT_MOVIMIENTO:
            return SISTEMA_ERROR_TIMEOUT_MOVIMIENTO;
        case ERROR_FINALES_INCOHERENTES:
        case ERROR_FINAL_INESPERADO:
            return SISTEMA_ERROR_FINALES_INCOHERENTES;
        case ERROR_CANCELADO:
            return SISTEMA_ERROR_CANCELADO;
        default:
            return SISTEMA_ERROR_CANCELADO;
    }
}

bool enlaceI2CVigente() {
    return existePaqueteValido &&
           secuenciaPaqueteESPConocida &&
           millis() - ultimoPaqueteValidoMs <= TIMEOUT_I2C_MS &&
           millis() - ultimoCambioSecuenciaESPMs <= TIMEOUT_SECUENCIA_ESP_MS;
}

bool baseESPLista() {
    return (paqueteESP.flags & ESP_FLAG_BASE_LISTA) != 0;
}

bool camaraConectada() {
    return (flagsCamara & ESP_FLAG_CAMARA_CONECTADA) != 0;
}

bool homografiaValida() {
    return (flagsCamara & ESP_FLAG_HOMOGRAFIA_VALIDA) != 0;
}

bool modeloListo() {
    return (flagsCamara & ESP_FLAG_MODELO_LISTO) != 0;
}

bool objetivoCamaraValido() {
    return (flagsCamara & ESP_FLAG_OBJETIVO_VALIDO) != 0;
}

bool camaraOcupada() {
    return (flagsCamara & ESP_FLAG_CAMARA_OCUPADA) != 0;
}

bool camaraListaCompleta() {
    return camaraConectada() && homografiaValida() && modeloListo() &&
           estadoCamara == CAMARA_LISTA && !camaraOcupada() && errorCamara == 0;
}

bool paqueteSemanticamenteValido(const PaqueteESPAPortenta &p) {
    const bool joystickValido =
        p.joystickX >= -1 && p.joystickX <= 1 &&
        p.joystickY >= -1 && p.joystickY <= 1 &&
        p.joystickZ >= -1 && p.joystickZ <= 1;
    const bool botonesValidos =
        (p.botones & static_cast<uint8_t>(~(BOTON_X | BOTON_TRIANGULO))) == 0;
    const bool servosValidos = p.servoRotacion <= 180 && p.servoPinza <= 180;
    const bool camaraValida = p.estadoCamara <= CAMARA_ERROR;
    return joystickValido && botonesValidos && servosValidos && camaraValida;
}

void registrarPaqueteValido(const PaqueteESPAPortenta &nuevo) {
    if (!sesionESPConocida) {
        sesionArranqueESP = nuevo.sesionArranque;
        sesionESPConocida = true;
        secuenciaPaqueteESPConocida = false;
        ackSecuenciaObjetivo = 0;
        codigoAckObjetivo = ACK_OBJ_NINGUNO;
        secuenciaObjetivoEnMovimiento = 0;
        Serial.print(F("[I2C] Sesion ESP32 inicial="));
        Serial.println(sesionArranqueESP);
    } else if (nuevo.sesionArranque != sesionArranqueESP) {
        Serial.print(F("[I2C] Reinicio ESP32: sesion "));
        Serial.print(sesionArranqueESP);
        Serial.print(F(" -> "));
        Serial.println(nuevo.sesionArranque);
        sesionArranqueESP = nuevo.sesionArranque;
        secuenciaPaqueteESPConocida = false;
        cambioSesionESPPendiente = true;
        ackSecuenciaObjetivo = 0;
        codigoAckObjetivo = ACK_OBJ_NINGUNO;
        secuenciaObjetivoEnMovimiento = 0;
        if (movimientoPosicionadoActivo) {
            cancelarMovimientoPosicionado("reinicio de ESP32", true);
        }
    }

    if (!secuenciaPaqueteESPConocida ||
        nuevo.secuenciaPaquete != ultimaSecuenciaPaqueteESP) {
        ultimaSecuenciaPaqueteESP = nuevo.secuenciaPaquete;
        secuenciaPaqueteESPConocida = true;
        ultimoCambioSecuenciaESPMs = millis();
    }

    paqueteESP = nuevo;
    joystickX = nuevo.joystickX;
    joystickY = nuevo.joystickY;
    joystickZ = nuevo.joystickZ;
    btConectado = (nuevo.flags & ESP_FLAG_BT_CONECTADO) != 0;
    posServoRot = nuevo.servoRotacion;
    posServoPin = nuevo.servoPinza;

    botonX = (nuevo.botones & BOTON_X) != 0;
    botonTriangulo = (nuevo.botones & BOTON_TRIANGULO) != 0;
    if (botonX && !botonXAnterior) eventoBotonX = true;
    if (botonTriangulo && !botonTrianguloAnterior) eventoBotonTriangulo = true;
    botonXAnterior = botonX;
    botonTrianguloAnterior = botonTriangulo;

    estadoCamara = nuevo.estadoCamara;
    errorCamara = nuevo.errorCamara;
    flagsCamara = nuevo.flags;
    for (uint8_t i = 0; i < 4; ++i) muestrasTag[i] = nuevo.muestrasTag[i];
    claseObjetivo = nuevo.claseObjetivo;
    objetivoCamaraX10 = nuevo.objetivoX10;
    objetivoCamaraY10 = nuevo.objetivoY10;
    secuenciaObjetivoRecibida = nuevo.secuenciaObjetivo;

    existePaqueteValido = true;
    protocoloValido = true;
    ultimoPaqueteValidoMs = millis();
    fallosPaqueteConsecutivos = 0;
}

bool leerPaqueteESP32() {
    const int recibidos = Wire.requestFrom(
        static_cast<uint8_t>(DIRECCION_ESP32),
        static_cast<uint8_t>(sizeof(PaqueteESPAPortenta))
    );

    if (recibidos != static_cast<int>(sizeof(PaqueteESPAPortenta)) ||
        Wire.available() < static_cast<int>(sizeof(PaqueteESPAPortenta))) {
        while (Wire.available()) Wire.read();
        lecturasI2CError++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }

    PaqueteESPAPortenta recibido = {};
    uint8_t *destino = reinterpret_cast<uint8_t *>(&recibido);
    for (size_t i = 0; i < sizeof(recibido); ++i) {
        destino[i] = static_cast<uint8_t>(Wire.read());
    }
    while (Wire.available()) Wire.read();

    if (!validarPaquete(recibido) || !paqueteSemanticamenteValido(recibido)) {
        lecturasI2CError++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }

    registrarPaqueteValido(recibido);
    lecturasI2COk++;
    return true;
}

uint8_t construirFlagsSistema() {
    uint8_t flags = 0;
    if (calibracionXYValida) flags |= SIS_FLAG_XY_CALIBRADO;
    if (calibracionZValida) flags |= SIS_FLAG_Z_CALIBRADO;
    if (brazoEnHome()) flags |= SIS_FLAG_EN_HOME;
    if (estadoGeneral == EST_AUTOMATICO) flags |= SIS_FLAG_AUTO_ACTIVO;
    if (motoresEnMovimiento() || movimientoPosicionadoActivo ||
        (faseCal > CAL_ESPERA && faseCal < CAL_COMPLETA)) {
        flags |= SIS_FLAG_BRAZO_OCUPADO;
    }
    if (estadoGeneral == EST_SYSTEM_ERROR) flags |= SIS_FLAG_ERROR_CRITICO;
    if (checklistInicialCompletado && estadoGeneral != EST_SYSTEM_ERROR) {
        flags |= SIS_FLAG_CHECKLIST_OK;
    }
    if (enlaceI2CVigente() && estadoGeneral != EST_SYSTEM_ERROR &&
        (estadoGeneral == EST_MANUAL || estadoGeneral == EST_AUTOMATICO ||
         estadoGeneral == EST_ARM_CALIBRATION ||
         estadoGeneral == EST_USER_ARM_CALIBRATION ||
         movimientoPosicionadoActivo)) {
        flags |= SIS_FLAG_MOTORES_HABILITADOS;
    }
    return flags;
}

uint8_t construirFlagsLimites() {
    uint8_t flags = 0;
    if (limiteXmas) flags |= LIM_FLAG_X_MAS;
    if (limiteXmenos) flags |= LIM_FLAG_X_MENOS;
    if (limiteYmas) flags |= LIM_FLAG_Y_MAS;
    if (limiteYmenos) flags |= LIM_FLAG_Y_MENOS;
    if (limiteZarriba) flags |= LIM_FLAG_Z_ARRIBA;
    if (limiteZabajo) flags |= LIM_FLAG_Z_ABAJO;
    if (finalesCoherentes()) flags |= LIM_FLAG_COHERENTES;
    return flags;
}

void llenarValoresPantalla(PaquetePortentaAESP &p) {
    if (estadoGeneral == EST_CAMERA_CALIBRATION ||
        estadoGeneral == EST_USER_CAMERA_CALIBRATION) {
        p.valorPantalla1 = estadoCamara;
        p.valorPantalla2 = errorCamara;
        p.valorPantalla3 = static_cast<int32_t>(muestrasTag[0]) |
                           (static_cast<int32_t>(muestrasTag[1]) << 8);
        p.valorPantalla4 = static_cast<int32_t>(muestrasTag[2]) |
                           (static_cast<int32_t>(muestrasTag[3]) << 8);
    } else if (estadoGeneral == EST_ARM_CALIBRATION ||
               estadoGeneral == EST_USER_ARM_CALIBRATION) {
        p.valorPantalla1 = leerPasosX();
        p.valorPantalla2 = leerPasosY();
        p.valorPantalla3 = rangoXPasos;
        p.valorPantalla4 = rangoYPasos;
    } else if (estadoGeneral == EST_AUTOMATICO || movimientoPosicionadoActivo) {
        p.valorPantalla1 = lroundf(posicionXmm() * 10.0f);
        p.valorPantalla2 = lroundf(posicionYmm() * 10.0f);
        p.valorPantalla3 = lroundf(objetivoXmm * 10.0f);
        p.valorPantalla4 = lroundf(objetivoYmm * 10.0f);
    } else if (estadoGeneral == EST_SYSTEM_ERROR) {
        p.valorPantalla1 = static_cast<int32_t>(errorSistema);
        p.valorPantalla2 = static_cast<int32_t>(ultimoResultadoChecklist);
        p.valorPantalla3 = static_cast<int32_t>(errorCamara);
        p.valorPantalla4 = static_cast<int32_t>(codigoErrorCalibracion());
    } else {
        p.valorPantalla1 = leerPasosX();
        p.valorPantalla2 = leerPasosY();
        p.valorPantalla3 = leerPasosZ();
        p.valorPantalla4 = sesionArranqueESP;
    }
}

void construirPaquetePortenta(PaquetePortentaAESP &p) {
    memset(&p, 0, sizeof(p));
    p.estadoSistema = estadoGeneralWire();
    p.opcionMenu = opcionMenu;
    p.faseCalibracionBrazo = static_cast<uint8_t>(faseCal);
    p.flagsSistema = construirFlagsSistema();
    p.flagsLimites = construirFlagsLimites();
    noInterrupts();
    p.movimientos = codificarMovimientos(movX, movY, movZ);
    interrupts();
    p.errorSistema = errorSistemaWire();
    p.comandoCamara = comandoCamaraActual;
    p.secuenciaComandoCamara = secuenciaComandoCamara;
    p.ackSecuenciaObjetivo = ackSecuenciaObjetivo;
    p.codigoAckObjetivo = codigoAckObjetivo;
    llenarValoresPantalla(p);
    prepararPaquete(p);
}

bool enviarPaquetePortenta() {
    PaquetePortentaAESP salida = {};
    construirPaquetePortenta(salida);
    Wire.beginTransmission(static_cast<uint8_t>(DIRECCION_ESP32));
    const size_t escritos = Wire.write(
        reinterpret_cast<const uint8_t *>(&salida), sizeof(salida)
    );
    const uint8_t error = Wire.endTransmission(true);
    if (escritos == sizeof(salida) && error == 0) {
        enviosI2COk++;
        return true;
    }
    enviosI2CError++;
    return false;
}

//-------------------------------------------------------------------------------------------------
// CAMBIOS DE ESTADO, COMANDOS DE CAMARA Y ERRORES
//-------------------------------------------------------------------------------------------------
void solicitarComandoCamara(uint8_t comando) {
    comandoCamaraActual = comando;
    secuenciaComandoCamara++;
    if (secuenciaComandoCamara == 0) secuenciaComandoCamara = 1;
    inicioCalibracionCamaraObservado = false;
    Serial.print(F("[CAM] Comando="));
    Serial.print(comandoCamaraActual);
    Serial.print(F(" secuencia="));
    Serial.println(secuenciaComandoCamara);
}

void cambiarEstadoGeneral(EstadoGeneral nuevoEstado) {
    if (estadoGeneral == nuevoEstado) return;
    estadoGeneral = nuevoEstado;
    inicioEstadoGeneral = millis();
    entradaMenuYAnterior = joystickY;

    Serial.print(F("[BOOT] Estado general -> "));
    Serial.println(estadoGeneralWire());

    switch (nuevoEstado) {
        case EST_CAMERA_CALIBRATION:
            volverChecklistTrasCalibracionCamara = false;
            detenerTodos();
            solicitarComandoCamara(CAM_CMD_CALIBRAR);
            break;
        case EST_USER_CAMERA_CALIBRATION:
            detenerTodos();
            solicitarComandoCamara(CAM_CMD_CALIBRAR);
            break;
        case EST_ARM_CALIBRATION:
        case EST_USER_ARM_CALIBRATION:
            detenerTodos();
            iniciarCalibracionBrazo();
            break;
        case EST_MAIN_MENU:
        case EST_WAIT_CONTROLLER:
        case EST_FINAL_CHECKLIST:
        case EST_BOOT_SAFE:
        case EST_WAIT_I2C:
        case EST_I2C_SETTLE:
        case EST_SYSTEM_ERROR:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            break;
        case EST_MANUAL:
        case EST_AUTOMATICO:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            break;
    }
}

void entrarErrorSistema(CodigoErrorLocal codigo, const char *mensaje) {
    detenerTodos();
    if (propietarioMovimiento == MOV_AUTOMATICO && secuenciaObjetivoEnMovimiento != 0) {
        ackSecuenciaObjetivo = secuenciaObjetivoEnMovimiento;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
    }
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;

    if (estadoGeneral == EST_ARM_CALIBRATION ||
        estadoGeneral == EST_USER_ARM_CALIBRATION) {
        calibracionXYValida = false;
        calibracionZValida = false;
        if (faseCal != CAL_ERROR) {
            mensajeErrorCalibracion = mensaje;
            faseCal = CAL_ERROR;
        }
    }

    errorSistema = codigo;
    mensajeErrorSistema = mensaje;
    eventoBotonX = false;
    eventoBotonTriangulo = false;
    cambiarEstadoGeneral(EST_SYSTEM_ERROR);
    Serial.print(F("[ERROR] codigo="));
    Serial.print(static_cast<uint8_t>(codigo));
    Serial.print(F(" "));
    Serial.println(mensaje);
}

//-------------------------------------------------------------------------------------------------
// CHECKLIST FINAL
//-------------------------------------------------------------------------------------------------
ResultadoChecklist evaluarChecklistFinal() {
    if (!enlaceI2CVigente()) return CHECK_I2C;
    if (!protocoloValido || !baseESPLista()) return CHECK_PROTOCOLO;
    if (!camaraConectada() || estadoCamara != CAMARA_LISTA || errorCamara != 0)
        return CHECK_CAMARA;
    if (!homografiaValida()) return CHECK_HOMOGRAFIA;
    if (!modeloListo()) return CHECK_MODELO;
    if (!calibracionXYValida || !escalaConfigurada()) return CHECK_XY;
    if (!calibracionZValida) return CHECK_Z;
    if (!brazoEnHome()) return CHECK_HOME;
    if (motoresEnMovimiento() || movimientoPosicionadoActivo) return CHECK_MOTORES;
    if (!btConectado) return CHECK_BT;
    if (faseCal == CAL_ERROR || mensajeErrorCalibracion[0] != '\0') return CHECK_CAL_ERROR;
    if (!finalesCoherentes() || algunFinalActivo()) return CHECK_FINALES;
    return CHECK_OK;
}

const char *textoChecklist(ResultadoChecklist resultado) {
    switch (resultado) {
        case CHECK_OK: return "Checklist completo";
        case CHECK_I2C: return "Comunicacion I2C inactiva";
        case CHECK_PROTOCOLO: return "Protocolo/base ESP32 invalido";
        case CHECK_CAMARA: return "Camara no conectada o con error";
        case CHECK_HOMOGRAFIA: return "Homografia invalida";
        case CHECK_MODELO: return "Modelo de piezas no listo";
        case CHECK_XY: return "Calibracion X/Y invalida";
        case CHECK_Z: return "Calibracion Z invalida";
        case CHECK_HOME: return "Brazo fuera de HOME";
        case CHECK_MOTORES: return "Motores no detenidos";
        case CHECK_BT: return "Control Bluetooth desconectado";
        case CHECK_CAL_ERROR: return "Error de calibracion pendiente";
        case CHECK_FINALES: return "Finales de carrera incoherentes";
        default: return "Fallo de checklist desconocido";
    }
}

//-------------------------------------------------------------------------------------------------
// MODOS DE USUARIO
//-------------------------------------------------------------------------------------------------
void procesarMenuPrincipal() {
    if (!btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (movimientoPosicionadoActivo || motoresEnMovimiento()) return;

    if (joystickY != 0 && entradaMenuYAnterior == 0) {
        int nueva = static_cast<int>(opcionMenu) - joystickY;
        if (nueva < 0) nueva = 3;
        if (nueva > 3) nueva = 0;
        opcionMenu = static_cast<uint8_t>(nueva);
        Serial.print(F("[MENU] Opcion="));
        Serial.println(opcionMenu);
    }
    entradaMenuYAnterior = joystickY;

    if (!eventoBotonX) return;
    eventoBotonX = false;

    switch (opcionMenu) {
        case MENU_MODO_MANUAL:
            cambiarEstadoGeneral(EST_MANUAL);
            break;
        case MENU_MODO_AUTOMATICO:
            if (!calibracionXYValida || !calibracionZValida ||
                !camaraListaCompleta()) {
                Serial.println(F("[AUTO] No disponible: camara o brazo sin calibrar"));
                return;
            }
            cambiarEstadoGeneral(EST_AUTOMATICO);
            break;
        case MENU_CALIBRACION_BRAZO:
            cambiarEstadoGeneral(EST_USER_ARM_CALIBRATION);
            break;
        case MENU_CALIBRACION_CAMARA:
            volverChecklistTrasCalibracionCamara = false;
            cambiarEstadoGeneral(EST_USER_CAMERA_CALIBRATION);
            break;
        default:
            opcionMenu = MENU_MODO_MANUAL;
            break;
    }
}

void procesarModoManual() {
    if (!btConectado) {
        detenerTodos();
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        detenerTodos();
        cambiarEstadoGeneral(EST_MAIN_MENU);
        return;
    }

    int8_t x = joystickX;
    int8_t y = joystickY;
    int8_t z = joystickZ;
    if ((x > 0 && limiteXmas) || (x < 0 && limiteXmenos)) x = 0;
    if ((y > 0 && limiteYmas) || (y < 0 && limiteYmenos)) y = 0;
    if ((z > 0 && limiteZarriba) || (z < 0 && limiteZabajo)) z = 0;
    moverXContinuo(x, DIV_MANUAL);
    moverYContinuo(y, DIV_MANUAL);
    moverZContinuo(z, DIV_MANUAL);
}

void rechazarObjetivoFueraDeRango(uint16_t secuencia) {
    ackSecuenciaObjetivo = secuencia;
    codigoAckObjetivo = ACK_OBJ_RECHAZADO_RANGO;
    secuenciaObjetivoEnMovimiento = 0;
    entrarErrorSistema(ERROR_OBJETIVO_INVALIDO,
                       "Objetivo de camara fuera del espacio seguro");
}

void procesarModoAutomatico() {
    // Z nunca se mueve en automatico.
    moverZContinuo(0, DIV_MANUAL);

    if (!btConectado) {
        if (movimientoPosicionadoActivo)
            cancelarMovimientoPosicionado("control Bluetooth desconectado", false);
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }

    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        if (movimientoPosicionadoActivo)
            cancelarMovimientoPosicionado("cancelado por usuario", false);
        cambiarEstadoGeneral(EST_MAIN_MENU);
        return;
    }

    if (!camaraListaCompleta()) {
        if (movimientoPosicionadoActivo)
            cancelarMovimientoPosicionado("camara dejo de estar lista", false);
        entrarErrorSistema(ERROR_CAMARA, "Camara no disponible en automatico");
        return;
    }

    if (movimientoPosicionadoActivo || motoresEnMovimiento()) return;
    if (!objetivoCamaraValido() || secuenciaObjetivoRecibida == 0) return;
    if (secuenciaObjetivoRecibida == ackSecuenciaObjetivo) return;

    const float camX = static_cast<float>(objetivoCamaraX10) / 10.0f;
    const float camY = static_cast<float>(objetivoCamaraY10) / 10.0f;
    float brazoX = 0.0f;
    float brazoY = 0.0f;
    if (!transformarCamaraABrazo(camX, camY, brazoX, brazoY)) {
        rechazarObjetivoFueraDeRango(secuenciaObjetivoRecibida);
        return;
    }

    long pruebaX = 0;
    long pruebaY = 0;
    if (!cinematicaInversaCartesiana(brazoX, brazoY, 0.0f, pruebaX, pruebaY)) {
        rechazarObjetivoFueraDeRango(secuenciaObjetivoRecibida);
        return;
    }

    secuenciaObjetivoEnMovimiento = secuenciaObjetivoRecibida;
    ackSecuenciaObjetivo = secuenciaObjetivoRecibida;
    codigoAckObjetivo = ACK_OBJ_ACEPTADO;

    Serial.print(F("[AUTO] Pieza clase="));
    Serial.print(claseObjetivo);
    Serial.print(F(" camara=("));
    Serial.print(camX, 1);
    Serial.print(F(","));
    Serial.print(camY, 1);
    Serial.print(F(") brazo=("));
    Serial.print(brazoX, 1);
    Serial.print(F(","));
    Serial.print(brazoY, 1);
    Serial.println(F(")"));

    if (!iniciarMovimientoXY(brazoX, brazoY, 0.0f, MOV_AUTOMATICO)) {
        rechazarObjetivoFueraDeRango(secuenciaObjetivoRecibida);
    }
}

void procesarCalibracionCamaraEnCurso(bool iniciadaDesdeArranque) {
    const bool comandoAplicado =
        paqueteESP.ackSecuenciaComandoCamara == secuenciaComandoCamara;

    if (comandoAplicado &&
        (estadoCamara != CAMARA_LISTA || camaraOcupada() ||
         !homografiaValida() || !modeloListo())) {
        inicioCalibracionCamaraObservado = true;
    }

    // Antes del ACK todavia puede estar visible el error del intento anterior.
    // El ESP limpia ese error al aceptar el CALIBRAR nuevo; solo entonces debe
    // evaluarse como resultado de esta solicitud.
    if (comandoAplicado &&
        (estadoCamara == CAMARA_ERROR || errorCamara != CAM_ERROR_NINGUNO)) {
        entrarErrorSistema(ERROR_CAMARA, "La camara reporto un error");
        return;
    }
    if (millis() - inicioEstadoGeneral > TIMEOUT_CAMARA_MS) {
        entrarErrorSistema(ERROR_TIMEOUT_CAMARA, "Timeout calibrando camara");
        return;
    }

    if (comandoAplicado && inicioCalibracionCamaraObservado &&
        camaraListaCompleta()) {
        Serial.println(F("[CAM] Homografia valida y modelo listo"));
        if (iniciadaDesdeArranque) {
            cambiarEstadoGeneral(EST_ARM_CALIBRATION);
        } else if (volverChecklistTrasCalibracionCamara) {
            volverChecklistTrasCalibracionCamara = false;
            cambiarEstadoGeneral(btConectado
                                     ? EST_FINAL_CHECKLIST
                                     : EST_WAIT_CONTROLLER);
        } else {
            cambiarEstadoGeneral(btConectado ? EST_MAIN_MENU : EST_WAIT_CONTROLLER);
        }
        return;
    }

    if (!iniciadaDesdeArranque && eventoBotonTriangulo && btConectado) {
        eventoBotonTriangulo = false;
        volverChecklistTrasCalibracionCamara = false;
        solicitarComandoCamara(CAM_CMD_STANDBY);
        cambiarEstadoGeneral(EST_MAIN_MENU);
    }
}

void procesarCalibracionBrazoEnCurso(bool iniciadaDesdeArranque) {
    procesarCalibracionBrazo();
    if (estadoGeneral == EST_SYSTEM_ERROR) return;

    if (faseCal == CAL_COMPLETA) {
        if (iniciadaDesdeArranque) {
            cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        } else {
            cambiarEstadoGeneral(btConectado ? EST_MAIN_MENU : EST_WAIT_CONTROLLER);
        }
        return;
    }

    if (!iniciadaDesdeArranque && eventoBotonTriangulo && btConectado) {
        eventoBotonTriangulo = false;
        detenerTodos();
        calibracionXYValida = false;
        calibracionZValida = false;
        faseCal = CAL_ESPERA;
        mensajeErrorCalibracion = "";
        cambiarEstadoGeneral(EST_MAIN_MENU);
    }
}

//-------------------------------------------------------------------------------------------------
// SEGURIDAD DE ENLACE Y SESION
//-------------------------------------------------------------------------------------------------
void vigilarSeguridadComunicacion() {
    if (cambioSesionESPPendiente) {
        cambioSesionESPPendiente = false;
        if (estadoGeneral == EST_WAIT_I2C || estadoGeneral == EST_I2C_SETTLE ||
            estadoGeneral == EST_BOOT_SAFE) {
            cambiarEstadoGeneral(EST_WAIT_I2C);
        } else if (estadoGeneral != EST_SYSTEM_ERROR) {
            entrarErrorSistema(ERROR_REINICIO_ESP32,
                               "ESP32 se reinicio; objetivos anteriores descartados");
        }
        return;
    }

    if (fallosPaqueteConsecutivos >= MAX_PAQUETES_INVALIDOS_CONSECUTIVOS &&
        estadoGeneral != EST_BOOT_SAFE && estadoGeneral != EST_WAIT_I2C &&
        estadoGeneral != EST_SYSTEM_ERROR) {
        entrarErrorSistema(ERROR_PROTOCOLO_INVALIDO,
                           "Paquetes I2C invalidos persistentes");
        return;
    }

    if (estadoGeneral == EST_BOOT_SAFE || estadoGeneral == EST_WAIT_I2C ||
        estadoGeneral == EST_SYSTEM_ERROR) {
        return;
    }

    if (!enlaceI2CVigente()) {
        detenerTodos();
        if (estadoGeneral == EST_I2C_SETTLE) {
            Serial.println(F("[I2C] Enlace perdido durante espera de 5 s"));
            cambiarEstadoGeneral(EST_WAIT_I2C);
        } else {
            entrarErrorSistema(ERROR_TIMEOUT_I2C, "Timeout de comunicacion I2C");
        }
    }
}

bool resultadoChecklistRelacionadoConCamara() {
    return ultimoResultadoChecklist == CHECK_CAMARA ||
           ultimoResultadoChecklist == CHECK_HOMOGRAFIA ||
           ultimoResultadoChecklist == CHECK_MODELO;
}

bool errorActualRelacionadoConCamara() {
    return errorSistema == ERROR_TIMEOUT_CAMARA ||
           errorSistema == ERROR_CAMARA ||
           (errorSistema == ERROR_CHECKLIST &&
            resultadoChecklistRelacionadoConCamara());
}

void reiniciarSecuenciaCompleta() {
    detenerTodos();
    calibracionXYValida = false;
    calibracionZValida = false;
    pasosPorMmX = 0.0f;
    pasosPorMmY = 0.0f;
    faseCal = CAL_ESPERA;
    mensajeErrorCalibracion = "";
    errorSistema = ERROR_NINGUNO;
    mensajeErrorSistema = "";
    ultimoResultadoChecklist = CHECK_OK;
    checklistInicialCompletado = false;
    ackSecuenciaObjetivo = 0;
    codigoAckObjetivo = ACK_OBJ_NINGUNO;
    secuenciaObjetivoEnMovimiento = 0;
    volverChecklistTrasCalibracionCamara = false;
    if (enlaceI2CVigente()) solicitarComandoCamara(CAM_CMD_REINICIAR_ERROR);
    cambiarEstadoGeneral(EST_BOOT_SAFE);
}

void reintentarDesdeEstadoError() {
    if (!errorActualRelacionadoConCamara()) {
        Serial.println(F("[ERROR] Reintento completo por error no relacionado con camara"));
        reiniciarSecuenciaCompleta();
        return;
    }

    if (!enlaceI2CVigente()) {
        Serial.println(F("[ERROR] Sin enlace I2C; no es posible reintentar solo la camara"));
        reiniciarSecuenciaCompleta();
        return;
    }

    const bool brazoCalibrado = calibracionXYValida &&
                                calibracionZValida &&
                                escalaConfigurada();
    const bool regresarAlChecklist =
        brazoCalibrado &&
        (volverChecklistTrasCalibracionCamara ||
         errorSistema == ERROR_CHECKLIST);

    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    secuenciaObjetivoEnMovimiento = 0;
    errorSistema = ERROR_NINGUNO;
    mensajeErrorSistema = "";
    ultimoResultadoChecklist = CHECK_OK;
    eventoBotonX = false;
    eventoBotonTriangulo = false;
    volverChecklistTrasCalibracionCamara = regresarAlChecklist;

    if (brazoCalibrado) {
        Serial.println(F("[CAM] Reintento selectivo; calibracion del brazo conservada"));
        cambiarEstadoGeneral(EST_USER_CAMERA_CALIBRATION);
    } else {
        Serial.println(F("[CAM] Reintento de camara; el brazo aun requiere calibracion"));
        cambiarEstadoGeneral(EST_CAMERA_CALIBRATION);
    }
}

//-------------------------------------------------------------------------------------------------
// MAQUINA GENERAL DEL SISTEMA
//-------------------------------------------------------------------------------------------------
void procesarMaquinaGeneral() {
    switch (estadoGeneral) {
        case EST_BOOT_SAFE:
            detenerTodos();
            if (millis() - inicioEstadoGeneral >= RETARDO_ARRANQUE_ESP32_MS) {
                cambiarEstadoGeneral(EST_WAIT_I2C);
            }
            break;

        case EST_WAIT_I2C:
            detenerTodos();
            if (enlaceI2CVigente() && protocoloValido && baseESPLista()) {
                Serial.println(F("[I2C] Primer paquete completo y valido"));
                cambiarEstadoGeneral(EST_I2C_SETTLE);
            }
            break;

        case EST_I2C_SETTLE:
            detenerTodos();
            if (millis() - inicioEstadoGeneral >= TIEMPO_ESTABILIZACION_I2C_MS) {
                cambiarEstadoGeneral(EST_CAMERA_CALIBRATION);
            }
            break;

        case EST_CAMERA_CALIBRATION:
            procesarCalibracionCamaraEnCurso(true);
            break;

        case EST_ARM_CALIBRATION:
            procesarCalibracionBrazoEnCurso(true);
            break;

        case EST_WAIT_CONTROLLER:
            detenerTodos();
            if (btConectado) {
                cambiarEstadoGeneral(checklistInicialCompletado
                                         ? EST_MAIN_MENU
                                         : EST_FINAL_CHECKLIST);
            }
            break;

        case EST_FINAL_CHECKLIST:
            detenerTodos();
            ultimoResultadoChecklist = evaluarChecklistFinal();
            if (ultimoResultadoChecklist == CHECK_OK) {
                checklistInicialCompletado = true;
                Serial.println(F("[BOOT] Checklist final OK"));
                cambiarEstadoGeneral(EST_MAIN_MENU);
            } else {
                const char *texto = textoChecklist(ultimoResultadoChecklist);
                Serial.print(F("[BOOT][CHECKLIST] Falla: "));
                Serial.println(texto);
                entrarErrorSistema(ERROR_CHECKLIST, texto);
            }
            break;

        case EST_MAIN_MENU:
            procesarMenuPrincipal();
            break;

        case EST_MANUAL:
            procesarModoManual();
            break;

        case EST_AUTOMATICO:
            procesarModoAutomatico();
            break;

        case EST_USER_ARM_CALIBRATION:
            procesarCalibracionBrazoEnCurso(false);
            break;

        case EST_USER_CAMERA_CALIBRATION:
            procesarCalibracionCamaraEnCurso(false);
            break;

        case EST_SYSTEM_ERROR:
            detenerTodos();
            if (btConectado && eventoBotonX) {
                eventoBotonX = false;
                Serial.println(F("[ERROR] Reintento seguro solicitado"));
                reintentarDesdeEstadoError();
            }
            break;
    }
}

//-------------------------------------------------------------------------------------------------
// DIAGNOSTICO Y TERMINAL (COMANDOS CONSERVADOS)
//-------------------------------------------------------------------------------------------------
void imprimirPosicionActual() {
    Serial.println(F("----- POSICION ACTUAL -----"));
    Serial.print(F("X="));
    if (escalaConfigurada()) {
        Serial.print(posicionXmm(), 2);
        Serial.print(F(" mm | "));
    }
    Serial.print(leerPasosX());
    Serial.println(F(" pasos"));

    Serial.print(F("Y="));
    if (escalaConfigurada()) {
        Serial.print(posicionYmm(), 2);
        Serial.print(F(" mm | "));
    }
    Serial.print(leerPasosY());
    Serial.println(F(" pasos"));

    Serial.print(F("Z="));
    Serial.print(leerPasosZ());
    Serial.print(F(" pasos | rango="));
    Serial.println(rangoZPasos);
}

void imprimirRangoTrabajo() {
    Serial.println(F("===== ESPACIO DE TRABAJO ====="));
    Serial.print(F("Rango X="));
    Serial.print(rangoXPasos);
    Serial.print(F(" pasos"));
    if (escalaConfigurada()) {
        Serial.print(F(" = "));
        Serial.print(rangoXmm(), 2);
        Serial.print(F(" mm"));
    }
    Serial.println();

    Serial.print(F("Rango Y="));
    Serial.print(rangoYPasos);
    Serial.print(F(" pasos"));
    if (escalaConfigurada()) {
        Serial.print(F(" = "));
        Serial.print(rangoYmm(), 2);
        Serial.print(F(" mm"));
    }
    Serial.println();
    Serial.print(F("Rango Z="));
    Serial.print(rangoZPasos);
    Serial.println(F(" pasos"));
    Serial.print(F("X pasos permitidos: "));
    Serial.print(limiteMinimoXPasos());
    Serial.print(F(" a "));
    Serial.println(limiteMaximoXPasos());
    Serial.print(F("Y pasos permitidos: "));
    Serial.print(limiteMinimoYPasos());
    Serial.print(F(" a "));
    Serial.println(limiteMaximoYPasos());
    Serial.println(F("Origen: centro fisico X=0, Y=0, Z=0"));
}

void mostrarAyudaTerminal() {
    Serial.println(F("===== COMANDOS DISPONIBLES ====="));
    Serial.println(F("GOTO X Y Z  -> posicion absoluta en mm (solo mueve X/Y)"));
    Serial.println(F("X Y Z       -> forma abreviada de GOTO"));
    Serial.println(F("HOME        -> X=0, Y=0; Z no se mueve"));
    Serial.println(F("POS         -> posicion actual"));
    Serial.println(F("RANGO       -> espacio de trabajo"));
    Serial.println(F("STOP        -> parada inmediata"));
    Serial.println(F("REINTENTAR  -> reinicio seguro desde estado de error"));
    Serial.println(F("AYUDA       -> esta ayuda"));
    Serial.println(F("Escala automatica: X=496 mm, Y=337 mm"));
}

bool movimientoTerminalPermitido() {
    return estadoGeneral == EST_MAIN_MENU && enlaceI2CVigente() &&
           calibracionXYValida && escalaConfigurada() &&
           (propietarioMovimiento == MOV_SIN_PROPIETARIO ||
            propietarioMovimiento == MOV_TERMINAL);
}

void ejecutarStopTerminal() {
    const bool estabaCalibrando =
        estadoGeneral == EST_ARM_CALIBRATION ||
        estadoGeneral == EST_USER_ARM_CALIBRATION;
    if (movimientoPosicionadoActivo) {
        cancelarMovimientoPosicionado("orden STOP", false);
    } else {
        detenerTodos();
    }

    if (estabaCalibrando) {
        calibracionXYValida = false;
        calibracionZValida = false;
        mensajeErrorCalibracion = "STOP por terminal";
        faseCal = CAL_ERROR;
        entrarErrorSistema(ERROR_CANCELADO, "Calibracion cancelada por STOP");
    } else if (estadoGeneral == EST_MANUAL || estadoGeneral == EST_AUTOMATICO) {
        cambiarEstadoGeneral(btConectado ? EST_MAIN_MENU : EST_WAIT_CONTROLLER);
    }
    Serial.println(F("[SEGURIDAD] STOP ejecutado"));
}

void procesarComandoTerminal(String comando) {
    comando.trim();
    if (comando.length() == 0) return;

    String mayuscula = comando;
    mayuscula.toUpperCase();
    if (mayuscula == "AYUDA") {
        mostrarAyudaTerminal();
        return;
    }
    if (mayuscula == "POS") {
        imprimirPosicionActual();
        return;
    }
    if (mayuscula == "RANGO") {
        imprimirRangoTrabajo();
        return;
    }
    if (mayuscula == "STOP") {
        ejecutarStopTerminal();
        return;
    }
    if (mayuscula == "REINTENTAR") {
        if (estadoGeneral != EST_SYSTEM_ERROR) {
            Serial.println(F("[ERROR] REINTENTAR solo aplica en estado de error"));
            return;
        }
        Serial.println(F("[ERROR] Reintento seguro solicitado por terminal"));
        reintentarDesdeEstadoError();
        return;
    }
    if (mayuscula == "HOME") {
        if (!movimientoTerminalPermitido()) {
            Serial.println(F("[ERROR] HOME requiere menu, enlace y X/Y calibrados"));
            return;
        }
        iniciarMovimientoXY(0.0f, 0.0f, 0.0f, MOV_TERMINAL);
        return;
    }

    if (!movimientoTerminalPermitido()) {
        Serial.println(F("[ERROR] GOTO requiere menu, enlace y X/Y calibrados"));
        return;
    }

    String datos = comando;
    if (mayuscula.startsWith("GOTO ")) datos = comando.substring(5);
    datos.replace(',', ' ');
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    char sobrante = '\0';
    const int leidos = sscanf(datos.c_str(), "%f %f %f %c", &x, &y, &z, &sobrante);
    if (leidos == 3 && isfinite(x) && isfinite(y) && isfinite(z)) {
        iniciarMovimientoXY(x, y, z, MOV_TERMINAL);
        return;
    }

    Serial.print(F("[ERROR] Comando no reconocido: "));
    Serial.println(comando);
    Serial.println(F("Usa: GOTO X Y Z"));
}

void leerTerminal() {
    while (Serial.available()) {
        const char caracter = static_cast<char>(Serial.read());
        if (caracter == '\n' || caracter == '\r') {
            if (lineaTerminal.length() > 0) {
                procesarComandoTerminal(lineaTerminal);
                lineaTerminal = "";
            }
        } else if (lineaTerminal.length() < 80) {
            lineaTerminal += caracter;
        } else {
            lineaTerminal = "";
            Serial.println(F("[ERROR] Comando demasiado largo"));
        }
    }
}

//-------------------------------------------------------------------------------------------------
// SETUP Y LOOP
//-------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(100000);

    digital_inputs.init();
    digital_outputs.set(pP_X, LOW);
    digital_outputs.set(pP_Y, LOW);
    digital_outputs.set(pP_Z, LOW);
    digital_outputs.set(pD_X, LOW);
    digital_outputs.set(pD_Y, LOW);
    digital_outputs.set(pD_Z, LOW);
    detenerTodos();
    leerFinalesCarrera();

    motorTicker.attach(&generarPulsoMotor, velocidadMotores);

    tiempoEncendidoSistema = millis();
    inicioEstadoGeneral = tiempoEncendidoSistema;
    tAnteriorI2C = tiempoEncendidoSistema;
    tAnteriorEstadoESP = tiempoEncendidoSistema;
    comunicacionI2CHabilitada = false;
    comandoCamaraActual = CAM_CMD_NINGUNO;
    secuenciaComandoCamara = 0;

    Serial.println(F("[BOOT] Portenta coordinadora iniciada en estado seguro"));
    Serial.println(F("[BOOT] I2C maestro 0x40 a 100 kHz; retencion inicial 3000 ms"));
    Serial.println(F("[BOOT] Finales Z: DIN04=arriba, DIN05=abajo"));
    Serial.println(F("[BOOT] Monitor serial: 115200 baudios"));
    mostrarAyudaTerminal();
}

void loop() {
    leerTerminal();
    leerFinalesCarrera();

    const unsigned long ahora = millis();
    if (!comunicacionI2CHabilitada &&
        ahora - tiempoEncendidoSistema >= RETARDO_ARRANQUE_ESP32_MS) {
        comunicacionI2CHabilitada = true;
        tAnteriorI2C = ahora;
        tAnteriorEstadoESP = ahora;
        Serial.println(F("[I2C] Inicio de sondeo versionado a ESP32"));
    }

    bool huboLectura = false;
    // if (comunicacionI2CHabilitada) {
    //     if (ahora - tAnteriorI2C >= PERIODO_CONTROL_MS) {
    //         tAnteriorI2C = ahora;
    //         leerPaqueteESP32();
    //         huboLectura = true;
    //     }
    //     // Evita solicitar y escribir al esclavo en la misma vuelta del loop.
    //     if (!huboLectura && ahora - tAnteriorEstadoESP >= PERIODO_ESTADO_ESP_MS) {
    //         tAnteriorEstadoESP = ahora;
    //         enviarPaquetePortenta();
    //     }
    // }
    if (comunicacionI2CHabilitada) {
    const bool tocaEnviar =
        ahora - tAnteriorEstadoESP >= PERIODO_ESTADO_ESP_MS;

    const bool tocaLeer =
        ahora - tAnteriorI2C >= PERIODO_CONTROL_MS;

    // El envío tiene prioridad cuando se cumplen los 100 ms.
    // Así se evita que las lecturas cada 5 ms lo bloqueen indefinidamente.
    if (tocaEnviar) {
        tAnteriorEstadoESP = ahora;
        enviarPaquetePortenta();
    }
    else if (tocaLeer) {
        tAnteriorI2C = ahora;
        leerPaqueteESP32();
    }
}

    if (!finalesCoherentes() && estadoGeneral != EST_SYSTEM_ERROR) {
        entrarErrorSistema(ERROR_FINALES_INCOHERENTES,
                           "Ambos finales de un eje estan activos");
    }

    actualizarMovimientoPosicionado();
    vigilarSeguridadComunicacion();
    procesarMaquinaGeneral();
    aplicarBloqueoPorFinales();

    // Los clicks son eventos de una sola iteracion, nunca quedan latched al cambiar de estado.
    eventoBotonX = false;
    eventoBotonTriangulo = false;

    if (ahora - ultimoReporteI2C >= 1000UL) {
        ultimoReporteI2C = ahora;
        Serial.print(F("[I2C] rxOK="));
        Serial.print(lecturasI2COk);
        Serial.print(F(" rxError="));
        Serial.print(lecturasI2CError);
        Serial.print(F(" txOK="));
        Serial.print(enviosI2COk);
        Serial.print(F(" txError="));
        Serial.print(enviosI2CError);
        Serial.print(F(" estado="));
        Serial.println(estadoGeneralWire());
    }
}

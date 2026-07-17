//------------------------------------------------------------------------------------------------
// DECLARACION DE LIBRERIAS
//------------------------------------------------------------------------------------------------
#include <Arduino_MachineControl.h>
#include <Wire.h>
#include "mbed.h"
#include <math.h>
#include <stdio.h>

using namespace machinecontrol;

//------------------------------------------------------------------------------------------------
// CONFIGURACION MOTORES STEPPER
//------------------------------------------------------------------------------------------------
// Segun tu codigo funcional actual:
//
// STEP / PUL
// pP_Z = 0
// pP_X = 4
// pP_Y = 2
//
// DIR
// pD_Z = 1
// pD_X = 5
// pD_Y = 3

const int pP_Z = 0;
const int pP_X = 4;
const int pP_Y = 2;

const int pD_Z = 1;
const int pD_X = 5;
const int pD_Y = 3;

// El Ticker corre cada 100 us.
const float velocidadMotores = 0.0001f;
mbed::Ticker motorTicker;

//------------------------------------------------------------------------------------------------
// VELOCIDADES
//------------------------------------------------------------------------------------------------
// Mientras mayor sea el divisor, menor es la velocidad.
// divisor 1 = mas rapido.
// divisor 8 = mas lento.

const uint16_t DIV_MANUAL = 1;
const uint16_t DIV_CAL_RAPIDA = 2;
const uint16_t DIV_CAL_LENTA = 8;
const uint16_t DIV_HOME = 2;
const uint16_t DIV_POSICION = 1;

//------------------------------------------------------------------------------------------------
// PARAMETROS DE CALIBRACION
//------------------------------------------------------------------------------------------------
const long PASOS_SEPARACION = 300;
const long RANGO_MINIMO_VALIDO = 100;
const unsigned long TIMEOUT_FASE_MS = 180000UL;

//------------------------------------------------------------------------------------------------
// CONVERSION A MILIMETROS
//------------------------------------------------------------------------------------------------
// Puedes configurar estos valores de dos formas:
//
// 1) Directamente aqui, si ya sabes los pasos/mm.
// 2) Por terminal despues de calibrar, usando:
//    ESCALA rangoXmm rangoYmm
//
// Ejemplo:
// Si despues de calibrar el rango fisico total de X es 500 mm
// y el rango fisico total de Y es 400 mm:
// ESCALA 500 400
//
// El sistema calculara:
// pasosPorMmX = rangoXPasos / 500
// pasosPorMmY = rangoYPasos / 400

//------------------------------------------------------------------------------------------------
// DIMENSIONES FISICAS DEL ESPACIO DE TRABAJO
//------------------------------------------------------------------------------------------------
// Estas medidas corresponden al recorrido total entre los dos
// finales de carrera verificados durante la calibracion.

const float RANGO_FISICO_X_MM = 496.0f;
const float RANGO_FISICO_Y_MM = 337.0f;

// Se calculan automaticamente al terminar la calibracion.
float pasosPorMmX = 0.0f;
float pasosPorMmY = 0.0f;

const float MARGEN_SEGURIDAD_MM = 2.0f;



//------------------------------------------------------------------------------------------------
// VARIABLES COMPARTIDAS CON INTERRUPCION
//------------------------------------------------------------------------------------------------
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

// Posicion estimada por conteo de pulsos.
volatile long pasosX = 0;
volatile long pasosY = 0;
volatile long pasosZ = 0;

// Movimiento hacia una posicion especifica.
volatile bool objetivoXActivo = false;
volatile bool objetivoYActivo = false;
volatile bool objetivoZActivo = false;

volatile long objetivoX = 0;
volatile long objetivoY = 0;
volatile long objetivoZ = 0;

//------------------------------------------------------------------------------------------------
// MAQUINA DE ESTADOS
//------------------------------------------------------------------------------------------------
enum EstadoSistema {
    MENU_PRINCIPAL,
    MODO_MANUAL,
    MODO_HOMING,
    MODO_CARTESIANO
};

EstadoSistema estadoActual = MENU_PRINCIPAL;

int opcionMenu = 0;

//------------------------------------------------------------------------------------------------
// FASES DE CALIBRACION
//------------------------------------------------------------------------------------------------
enum FaseCalibracion {
    CAL_ESPERA,

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

FaseCalibracion faseCal = CAL_ESPERA;

unsigned long inicioFase = 0;

bool homeIniciado = false;
bool calibracionXYValida = false;
bool calibracionZValida = false;

const char *mensajeError = "";

long rangoXPasos = 0;
long rangoYPasos = 0;
long rangoZPasos = 0;

//------------------------------------------------------------------------------------------------
// FINALES DE CARRERA
//------------------------------------------------------------------------------------------------
// OJO:
// Mantengo el mapeo como estaba en tu codigo funcional:
//
// limiteXmas   = DIN01
// limiteXmenos = DIN00
//
// Si fisicamente DIN00 es X+ y DIN01 es X-,
// solo intercambia esas dos lineas dentro de leerFinalesCarrera().
//
// Logica NC:
// read() == true  -> final NO presionado
// read() == false -> final presionado o cable desconectado

const int PIN_LIMITE_Z_ARRIBA = DIN_READ_CH_PIN_04;
const int PIN_LIMITE_Z_ABAJO = DIN_READ_CH_PIN_05;

const int8_t DIR_Z_ARRIBA = 1;
const int8_t DIR_Z_ABAJO = -1;

const uint8_t FLAG_Z_CALIBRADO = 0x20;
const uint8_t LIMITE_Z_ARRIBA_BIT = 0x10;
const uint8_t LIMITE_Z_ABAJO_BIT = 0x20;

bool limiteXmas = false;
bool limiteXmenos = false;
bool limiteYmenos = false;
bool limiteYmas = false;
bool limiteZarriba = false;
bool limiteZabajo = false;

//------------------------------------------------------------------------------------------------
// ACTUADORES
//------------------------------------------------------------------------------------------------
uint8_t posServoRot = 90;
uint8_t posServoPin = 90;

//------------------------------------------------------------------------------------------------
// COMUNICACION
//------------------------------------------------------------------------------------------------
#define DIRECCION_ESP32 0x40

const unsigned long PERIODO_CONTROL_MS = 5;
const unsigned long PERIODO_ESTADO_PANTALLA_MS = 150;
const unsigned long TIMEOUT_CONTROL_MS = 150;
const unsigned long RETARDO_ARRANQUE_ESP32_MS = 3000;

uint8_t ultimoErrorEnvioPantalla = 255;
unsigned long ultimoReporteI2C = 0;
uint32_t enviosPantallaOk = 0;
uint32_t enviosPantallaError = 0;

unsigned long tAnteriorI2C = 0;
unsigned long tAnteriorEstadoPantalla = 0;
unsigned long ultimoPaqueteControlMs = 0;

bool esp32Conectado = false;
bool btConectado = false;
bool comunicacionI2CHabilitada = false;
unsigned long tiempoEncendidoSistema = 0;

int8_t prevInputY = 0;
uint8_t prevBtnX = 0;
uint8_t prevBtnTri = 0;

// Paquete que se envia desde la Portenta hacia el ESP32 para que el ESP32 dibuje la OLED.
struct __attribute__((packed)) PaquetePantalla {
    uint8_t magic;
    uint8_t version;
    uint8_t estado;
    uint8_t opcionMenu;
    uint8_t faseCal;
    uint8_t flags;
    uint8_t limites;
    int8_t movX;
    int8_t movY;
    int8_t movZ;
    uint8_t servoRot;
    uint8_t servoPin;
    uint8_t errorCal;
    int32_t valor1;
    int32_t valor2;
    int32_t valor3;
    int32_t valor4;
    uint8_t checksum;
};

static_assert(sizeof(PaquetePantalla) <= 32, "PaquetePantalla excede el buffer I2C");

const uint8_t MAGIC_PANTALLA = 0xA5;
const uint8_t VERSION_PANTALLA = 1;

//------------------------------------------------------------------------------------------------
// VARIABLES MODO CARTESIANO
//------------------------------------------------------------------------------------------------
bool movimientoCartesianoActivo = false;

float objetivoXmm = 0.0f;
float objetivoYmm = 0.0f;
float objetivoZmm = 0.0f;

long objetivoCartesianoXPasos = 0;
long objetivoCartesianoYPasos = 0;

String lineaTerminal = "";

//------------------------------------------------------------------------------------------------
// FUNCIONES ATOMICAS PARA CONTADORES
//------------------------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------------------------
// LECTURA DE FINALES DE CARRERA
//------------------------------------------------------------------------------------------------
void leerFinalesCarrera() {
    limiteXmas   = !digital_inputs.read(DIN_READ_CH_PIN_01); // X+
    limiteXmenos = !digital_inputs.read(DIN_READ_CH_PIN_00); // X-
    limiteYmenos = !digital_inputs.read(DIN_READ_CH_PIN_02); // Y-
    limiteYmas   = !digital_inputs.read(DIN_READ_CH_PIN_03); // Y+
    limiteZarriba = !digital_inputs.read(PIN_LIMITE_Z_ARRIBA); // Z arriba
    limiteZabajo  = !digital_inputs.read(PIN_LIMITE_Z_ABAJO); // Z abajo
}

//------------------------------------------------------------------------------------------------
// INTERRUPCION: GENERACION DE PULSOS
//------------------------------------------------------------------------------------------------
void generarPulsoMotor() {
    // EJE X
    if (movX != 0) {
        cuentaX++;

        if (cuentaX >= divisorX) {
            cuentaX = 0;
            pulsoX = !pulsoX;

            digital_outputs.set(pP_X, pulsoX ? HIGH : LOW);

            if (pulsoX) {
                pasosX += movX;

                if (
                    objetivoXActivo &&
                    (
                        (movX > 0 && pasosX >= objetivoX) ||
                        (movX < 0 && pasosX <= objetivoX)
                    )
                ) {
                    pasosX = objetivoX;
                    movX = 0;
                    objetivoXActivo = false;
                    digital_outputs.set(pP_X, LOW);
                }
            }
        }
    }
    else {
        cuentaX = 0;
        pulsoX = false;
        digital_outputs.set(pP_X, LOW);
    }

    // EJE Y
    if (movY != 0) {
        cuentaY++;

        if (cuentaY >= divisorY) {
            cuentaY = 0;
            pulsoY = !pulsoY;

            digital_outputs.set(pP_Y, pulsoY ? HIGH : LOW);

            if (pulsoY) {
                pasosY += movY;

                if (
                    objetivoYActivo &&
                    (
                        (movY > 0 && pasosY >= objetivoY) ||
                        (movY < 0 && pasosY <= objetivoY)
                    )
                ) {
                    pasosY = objetivoY;
                    movY = 0;
                    objetivoYActivo = false;
                    digital_outputs.set(pP_Y, LOW);
                }
            }
        }
    }
    else {
        cuentaY = 0;
        pulsoY = false;
        digital_outputs.set(pP_Y, LOW);
    }

    // EJE Z
    if (movZ != 0) {
        cuentaZ++;

        if (cuentaZ >= divisorZ) {
            cuentaZ = 0;
            pulsoZ = !pulsoZ;

            digital_outputs.set(pP_Z, pulsoZ ? HIGH : LOW);

            if (pulsoZ) {
                pasosZ += movZ;

                if (
                    objetivoZActivo &&
                    (
                        (movZ > 0 && pasosZ >= objetivoZ) ||
                        (movZ < 0 && pasosZ <= objetivoZ)
                    )
                ) {
                    pasosZ = objetivoZ;
                    movZ = 0;
                    objetivoZActivo = false;
                    digital_outputs.set(pP_Z, LOW);
                }
            }
        }
    }
    else {
        cuentaZ = 0;
        pulsoZ = false;
        digital_outputs.set(pP_Z, LOW);
    }
}

//------------------------------------------------------------------------------------------------
// DETENER MOTORES
//------------------------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------------------------
// MOVIMIENTO CONTINUO
//------------------------------------------------------------------------------------------------
void moverXContinuo(int8_t direccion, uint16_t divisor) {
    if (direccion == 0) {
        detenerX();
        return;
    }

    noInterrupts();
    bool mismaConfiguracion =
        (
            movX == direccion &&
            !objetivoXActivo &&
            divisorX == divisor
        );
    interrupts();

    if (mismaConfiguracion) return;

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
    bool mismaConfiguracion =
        (
            movY == direccion &&
            !objetivoYActivo &&
            divisorY == divisor
        );
    interrupts();

    if (mismaConfiguracion) return;

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

    if (
        (direccion == DIR_Z_ARRIBA && limiteZarriba) ||
        (direccion == DIR_Z_ABAJO && limiteZabajo)
    ) {
        detenerZ();
        return;
    }

    noInterrupts();
    bool mismaConfiguracion =
        (
            movZ == direccion &&
            !objetivoZActivo &&
            divisorZ == divisor
        );
    interrupts();

    if (mismaConfiguracion) return;

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

//------------------------------------------------------------------------------------------------
// MOVIMIENTO A POSICION
//------------------------------------------------------------------------------------------------
void moverXHasta(long destino, uint16_t divisor) {
    long posicionActual = leerPasosX();

    if (posicionActual == destino) {
        detenerX();
        return;
    }

    int8_t direccion = destino > posicionActual ? 1 : -1;

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
    long posicionActual = leerPasosY();

    if (posicionActual == destino) {
        detenerY();
        return;
    }

    int8_t direccion = destino > posicionActual ? 1 : -1;

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
    long posicionActual = leerPasosZ();

    if (posicionActual == destino) {
        detenerZ();
        return;
    }

    int8_t direccion = destino > posicionActual ? 1 : -1;

    if (
        (direccion == DIR_Z_ARRIBA && limiteZarriba) ||
        (direccion == DIR_Z_ABAJO && limiteZabajo)
    ) {
        detenerZ();
        return;
    }

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

//------------------------------------------------------------------------------------------------
// SEGURIDAD POR FINALES
//------------------------------------------------------------------------------------------------
void aplicarBloqueoPorFinales() {
    if (movX > 0 && limiteXmas) detenerX();
    if (movX < 0 && limiteXmenos) detenerX();

    if (movY > 0 && limiteYmas) detenerY();
    if (movY < 0 && limiteYmenos) detenerY();

    if (movZ > 0 && limiteZarriba) detenerZ();
    if (movZ < 0 && limiteZabajo) detenerZ();
}

//------------------------------------------------------------------------------------------------
// FUNCIONES DE ESCALA Y POSICION
//------------------------------------------------------------------------------------------------
bool escalaConfigurada() {
    return (
        pasosPorMmX > 0.0f &&
        pasosPorMmY > 0.0f
    );
}

float posicionXmm() {
    if (pasosPorMmX <= 0.0f) return 0.0f;
    return (float)leerPasosX() / pasosPorMmX;
}

float posicionYmm() {
    if (pasosPorMmY <= 0.0f) return 0.0f;
    return (float)leerPasosY() / pasosPorMmY;
}

float rangoXmm() {
    if (pasosPorMmX <= 0.0f) return 0.0f;
    return (float)rangoXPasos / pasosPorMmX;
}

float rangoYmm() {
    if (pasosPorMmY <= 0.0f) return 0.0f;
    return (float)rangoYPasos / pasosPorMmY;
}

long limiteMinimoXPasos() {
    return -(rangoXPasos / 2);
}

long limiteMaximoXPasos() {
    return rangoXPasos - rangoXPasos / 2;
}

long limiteMinimoYPasos() {
    return -(rangoYPasos / 2);
}

long limiteMaximoYPasos() {
    return rangoYPasos - rangoYPasos / 2;
}

//------------------------------------------------------------------------------------------------
// IMPRESION EN TERMINAL
//------------------------------------------------------------------------------------------------
void imprimirPosicionActual() {
    Serial.println();
    Serial.println(F("----- POSICION ACTUAL -----"));

    Serial.print(F("X = "));
    if (escalaConfigurada()) {
        Serial.print(posicionXmm(), 2);
        Serial.print(F(" mm | "));
    }
    Serial.print(leerPasosX());
    Serial.println(F(" pasos"));

    Serial.print(F("Y = "));
    if (escalaConfigurada()) {
        Serial.print(posicionYmm(), 2);
        Serial.print(F(" mm | "));
    }
    Serial.print(leerPasosY());
    Serial.println(F(" pasos"));

    Serial.print(F("Z = "));
    Serial.print(leerPasosZ());
    if (calibracionZValida) {
        Serial.print(F(" pasos | rango Z = "));
        Serial.print(rangoZPasos);
        Serial.println(F(" pasos"));
    }
    else {
        Serial.println(F(" pasos | SIN CALIBRACION"));
    }

    Serial.println(F("---------------------------"));
}
//------------------------------------------------------------------------------------------------
// CALCULAR ESCALA AUTOMATICAMENTE
//------------------------------------------------------------------------------------------------
bool calcularEscalaAutomatica() {
    if (
        rangoXPasos <= 0 ||
        rangoYPasos <= 0
    ) {
        pasosPorMmX = 0.0f;
        pasosPorMmY = 0.0f;

        Serial.println(
            F("[ERROR] No se puede calcular la escala: rangos invalidos.")
        );

        return false;
    }

    pasosPorMmX =
        (float)rangoXPasos /
        RANGO_FISICO_X_MM;

    pasosPorMmY =
        (float)rangoYPasos /
        RANGO_FISICO_Y_MM;

    Serial.println();
    Serial.println(
        F("===== ESCALA AUTOMATICA =====")
    );

    Serial.print(
        F("Recorrido fisico X: ")
    );
    Serial.print(
        RANGO_FISICO_X_MM,
        2
    );
    Serial.println(
        F(" mm")
    );

    Serial.print(
        F("Recorrido medido X: ")
    );
    Serial.print(
        rangoXPasos
    );
    Serial.println(
        F(" pasos")
    );

    Serial.print(
        F("Escala X: ")
    );
    Serial.print(
        pasosPorMmX,
        6
    );
    Serial.println(
        F(" pasos/mm")
    );

    Serial.println();

    Serial.print(
        F("Recorrido fisico Y: ")
    );
    Serial.print(
        RANGO_FISICO_Y_MM,
        2
    );
    Serial.println(
        F(" mm")
    );

    Serial.print(
        F("Recorrido medido Y: ")
    );
    Serial.print(
        rangoYPasos
    );
    Serial.println(
        F(" pasos")
    );

    Serial.print(
        F("Escala Y: ")
    );
    Serial.print(
        pasosPorMmY,
        6
    );
    Serial.println(
        F(" pasos/mm")
    );

    Serial.println(
        F("============================")
    );

    return true;
}

void imprimirRangoTrabajo() {
    Serial.println();
    Serial.println(F("===== ESPACIO DE TRABAJO ====="));

    Serial.print(F("Rango total X: "));
    Serial.print(rangoXPasos);
    Serial.println(F(" pasos"));

    Serial.print(F("Rango total Y: "));
    Serial.print(rangoYPasos);
    Serial.println(F(" pasos"));

    Serial.print(F("Rango total Z: "));
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

    if (!escalaConfigurada()) {
        Serial.println(F("Escala en mm NO configurada."));
        Serial.println(F("Usa: ESCALA rangoXmm rangoYmm"));
        Serial.println(F("Ejemplo: ESCALA 500 400"));
        Serial.println(F("=============================="));
        return;
    }

    float xMin = (float)limiteMinimoXPasos() / pasosPorMmX;
    float xMax = (float)limiteMaximoXPasos() / pasosPorMmX;

    float yMin = (float)limiteMinimoYPasos() / pasosPorMmY;
    float yMax = (float)limiteMaximoYPasos() / pasosPorMmY;

    Serial.print(F("pasos/mm X: "));
    Serial.println(pasosPorMmX, 4);

    Serial.print(F("pasos/mm Y: "));
    Serial.println(pasosPorMmY, 4);

    Serial.print(F("Rango total X: "));
    Serial.print(rangoXmm(), 2);
    Serial.println(F(" mm"));

    Serial.print(F("X permitido: "));
    Serial.print(xMin, 2);
    Serial.print(F(" mm a "));
    Serial.print(xMax, 2);
    Serial.println(F(" mm"));

    Serial.print(F("Rango total Y: "));
    Serial.print(rangoYmm(), 2);
    Serial.println(F(" mm"));

    Serial.print(F("Y permitido: "));
    Serial.print(yMin, 2);
    Serial.print(F(" mm a "));
    Serial.print(yMax, 2);
    Serial.println(F(" mm"));

    Serial.println(F("Origen: centro fisico X=0, Y=0"));
    Serial.println(F("=============================="));
}

//------------------------------------------------------------------------------------------------
// CINEMATICA INVERSA CARTESIANA
//------------------------------------------------------------------------------------------------
// Brazo cartesiano:
// qX = X
// qY = Y
// qZ = Z
//
// Por ahora:
// X e Y se convierten a pasos.
// Z se recibe, pero no se mueve.

bool cinematicaInversaCartesiana(
    float xMm,
    float yMm,
    float zMm,
    long &xPasos,
    long &yPasos
) {
    if (!calibracionXYValida) {
        Serial.println(F("[ERROR] Primero debes calibrar X/Y."));
        return false;
    }

    if (!escalaConfigurada()) {
        Serial.println(F("[ERROR] Escala no configurada."));
        Serial.println(F("Usa: ESCALA rangoXmm rangoYmm"));
        return false;
    }

    xPasos = lroundf(xMm * pasosPorMmX);
    yPasos = lroundf(yMm * pasosPorMmY);

    long margenXPasos = lroundf(MARGEN_SEGURIDAD_MM * pasosPorMmX);
    long margenYPasos = lroundf(MARGEN_SEGURIDAD_MM * pasosPorMmY);

    long xMinSeguro = limiteMinimoXPasos() + margenXPasos;
    long xMaxSeguro = limiteMaximoXPasos() - margenXPasos;

    long yMinSeguro = limiteMinimoYPasos() + margenYPasos;
    long yMaxSeguro = limiteMaximoYPasos() - margenYPasos;

    if (xPasos < xMinSeguro || xPasos > xMaxSeguro) {
        Serial.print(F("[ERROR] X fuera del espacio de trabajo: "));
        Serial.print(xMm, 2);
        Serial.println(F(" mm"));
        return false;
    }

    if (yPasos < yMinSeguro || yPasos > yMaxSeguro) {
        Serial.print(F("[ERROR] Y fuera del espacio de trabajo: "));
        Serial.print(yMm, 2);
        Serial.println(F(" mm"));
        return false;
    }

    (void)zMm;

    return true;
}

//------------------------------------------------------------------------------------------------
// MOVIMIENTO CARTESIANO
//------------------------------------------------------------------------------------------------
void cancelarMovimientoCartesiano(const char *motivo) {
    detenerTodos();
    movimientoCartesianoActivo = false;

    Serial.print(F("[XYZ] Movimiento detenido: "));
    Serial.println(motivo);
}

void irAPosicionXYZ(float xMm, float yMm, float zMm) {
    if (estadoActual != MODO_CARTESIANO) {
        Serial.println(F("[ERROR] Entra primero al menu POSICION XYZ."));
        return;
    }

    long destinoXPasos = 0;
    long destinoYPasos = 0;

    if (
        !cinematicaInversaCartesiana(
            xMm,
            yMm,
            zMm,
            destinoXPasos,
            destinoYPasos
        )
    ) {
        return;
    }

    detenerTodos();

    objetivoXmm = xMm;
    objetivoYmm = yMm;
    objetivoZmm = zMm;

    objetivoCartesianoXPasos = destinoXPasos;
    objetivoCartesianoYPasos = destinoYPasos;

    Serial.println();
    Serial.println(F("===== NUEVO OBJETIVO XYZ ====="));

    Serial.print(F("X objetivo: "));
    Serial.print(objetivoXmm, 2);
    Serial.print(F(" mm -> "));
    Serial.print(objetivoCartesianoXPasos);
    Serial.println(F(" pasos"));

    Serial.print(F("Y objetivo: "));
    Serial.print(objetivoYmm, 2);
    Serial.print(F(" mm -> "));
    Serial.print(objetivoCartesianoYPasos);
    Serial.println(F(" pasos"));

    Serial.print(F("Z recibido: "));
    Serial.print(objetivoZmm, 2);
    Serial.println(F(" mm, pero NO se movera."));

    Serial.println(F("=============================="));

    moverXHasta(objetivoCartesianoXPasos, DIV_POSICION);
    moverYHasta(objetivoCartesianoYPasos, DIV_POSICION);

    movimientoCartesianoActivo =
        objetivoXEnCurso() ||
        objetivoYEnCurso();

    if (!movimientoCartesianoActivo) {
        Serial.println(F("[XYZ] Ya esta en esa posicion."));
        imprimirPosicionActual();
    }
}

void actualizarMovimientoCartesiano() {
    if (!movimientoCartesianoActivo) return;

    if (
        (movX > 0 && limiteXmas) ||
        (movX < 0 && limiteXmenos) ||
        (movY > 0 && limiteYmas) ||
        (movY < 0 && limiteYmenos)
    ) {
        cancelarMovimientoCartesiano("Se activo un final de carrera");

        calibracionXYValida = false;

        Serial.println(F("[SEGURIDAD] Posicion perdida. Debes volver a calibrar."));
        return;
    }

    if (
        !objetivoXEnCurso() &&
        !objetivoYEnCurso() &&
        movX == 0 &&
        movY == 0
    ) {
        movimientoCartesianoActivo = false;

        Serial.println();
        Serial.println(F("[XYZ] OBJETIVO ALCANZADO"));
        imprimirPosicionActual();
    }
}

//------------------------------------------------------------------------------------------------
// NOMBRE DE FASE DE CALIBRACION
//------------------------------------------------------------------------------------------------
const char *nombreFase() {
    switch (faseCal) {
        case CAL_ESPERA: return "PRESIONA X";

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

        default: return "";
    }
}

void cambiarFase(FaseCalibracion nuevaFase) {
    faseCal = nuevaFase;
    inicioFase = millis();

    Serial.print(F("[CAL] "));
    Serial.println(nombreFase());
}

void detenerPorError(const char *texto) {
    detenerTodos();

    mensajeError = texto;

    cambiarFase(CAL_ERROR);

    Serial.print(F("[CAL][ERROR] "));
    Serial.println(texto);
}

//------------------------------------------------------------------------------------------------
// INICIAR CALIBRACION
//------------------------------------------------------------------------------------------------
void iniciarCalibracion() {
    detenerTodos();

    calibracionXYValida = false;
    calibracionZValida = false;
    movimientoCartesianoActivo = false;

    rangoXPasos = 0;
    rangoYPasos = 0;
    rangoZPasos = 0;

    homeIniciado = false;
    mensajeError = "";

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("INICIO CALIBRACION X/Y/Z"));
    Serial.println(F("================================"));

    cambiarFase(CAL_X_MIN_1);
}

//------------------------------------------------------------------------------------------------
// PROCESAR CALIBRACION
//------------------------------------------------------------------------------------------------
void procesarCalibracion() {
    if (
        faseCal != CAL_ESPERA &&
        faseCal != CAL_COMPLETA &&
        faseCal != CAL_ERROR
    ) {
        if (millis() - inicioFase > TIMEOUT_FASE_MS) {
            detenerPorError("Tiempo maximo excedido");
            return;
        }
    }

    switch (faseCal) {
        case CAL_ESPERA:
            detenerTodos();
            break;

        //----------------------------------------------------------------------------------------
        // X-
        //----------------------------------------------------------------------------------------
        case CAL_X_MIN_1:
            if (limiteXmenos) {
                detenerX();
                cambiarFase(CAL_X_MIN_LIBERAR);
            }
            else {
                moverXContinuo(-1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_X_MIN_LIBERAR:
            if (!limiteXmenos) {
                detenerX();
                moverXHasta(leerPasosX() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFase(CAL_X_MIN_SEPARAR);
            }
            else {
                moverXContinuo(1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_X_MIN_SEPARAR:
            if (!objetivoXEnCurso()) {
                cambiarFase(CAL_X_MIN_2);
            }
            break;

        case CAL_X_MIN_2:
            if (limiteXmenos) {
                detenerX();
                fijarPasosX(0);

                Serial.println(F("[CAL][X] X- verificado. X temporal = 0 pasos"));

                cambiarFase(CAL_X_MAX_1);
            }
            else {
                moverXContinuo(-1, DIV_CAL_LENTA);
            }
            break;

        //----------------------------------------------------------------------------------------
        // X+
        //----------------------------------------------------------------------------------------
        case CAL_X_MAX_1:
            if (limiteXmas) {
                detenerX();
                cambiarFase(CAL_X_MAX_LIBERAR);
            }
            else {
                moverXContinuo(1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_X_MAX_LIBERAR:
            if (!limiteXmas) {
                detenerX();
                moverXHasta(leerPasosX() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFase(CAL_X_MAX_SEPARAR);
            }
            else {
                moverXContinuo(-1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_X_MAX_SEPARAR:
            if (!objetivoXEnCurso()) {
                cambiarFase(CAL_X_MAX_2);
            }
            break;

        case CAL_X_MAX_2:
            if (limiteXmas) {
                detenerX();

                rangoXPasos = leerPasosX();

                if (rangoXPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorError("Rango X invalido");
                    return;
                }

                Serial.print(F("[CAL][X] Rango = "));
                Serial.print(rangoXPasos);
                Serial.println(F(" pasos"));

                cambiarFase(CAL_Y_MIN_1);
            }
            else {
                moverXContinuo(1, DIV_CAL_LENTA);
            }
            break;

        //----------------------------------------------------------------------------------------
        // Y-
        //----------------------------------------------------------------------------------------
        case CAL_Y_MIN_1:
            if (limiteYmenos) {
                detenerY();
                cambiarFase(CAL_Y_MIN_LIBERAR);
            }
            else {
                moverYContinuo(-1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Y_MIN_LIBERAR:
            if (!limiteYmenos) {
                detenerY();
                moverYHasta(leerPasosY() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFase(CAL_Y_MIN_SEPARAR);
            }
            else {
                moverYContinuo(1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Y_MIN_SEPARAR:
            if (!objetivoYEnCurso()) {
                cambiarFase(CAL_Y_MIN_2);
            }
            break;

        case CAL_Y_MIN_2:
            if (limiteYmenos) {
                detenerY();
                fijarPasosY(0);

                Serial.println(F("[CAL][Y] Y- verificado. Y temporal = 0 pasos"));

                cambiarFase(CAL_Y_MAX_1);
            }
            else {
                moverYContinuo(-1, DIV_CAL_LENTA);
            }
            break;

        //----------------------------------------------------------------------------------------
        // Y+
        //----------------------------------------------------------------------------------------
        case CAL_Y_MAX_1:
            if (limiteYmas) {
                detenerY();
                cambiarFase(CAL_Y_MAX_LIBERAR);
            }
            else {
                moverYContinuo(1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Y_MAX_LIBERAR:
            if (!limiteYmas) {
                detenerY();
                moverYHasta(leerPasosY() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFase(CAL_Y_MAX_SEPARAR);
            }
            else {
                moverYContinuo(-1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Y_MAX_SEPARAR:
            if (!objetivoYEnCurso()) {
                cambiarFase(CAL_Y_MAX_2);
            }
            break;

        case CAL_Y_MAX_2:
            if (limiteYmas) {
                detenerY();

                rangoYPasos = leerPasosY();

                if (rangoYPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorError("Rango Y invalido");
                    return;
                }

                Serial.print(F("[CAL][Y] Rango = "));
                Serial.print(rangoYPasos);
                Serial.println(F(" pasos"));

                cambiarFase(CAL_Z_MIN_1);
            }
            else {
                moverYContinuo(1, DIV_CAL_LENTA);
            }
            break;

        //----------------------------------------------------------------------------------------
        // Z ABAJO
        //----------------------------------------------------------------------------------------
        case CAL_Z_MIN_1:
            if (limiteZabajo) {
                detenerZ();
                cambiarFase(CAL_Z_MIN_LIBERAR);
            }
            else {
                moverZContinuo(-1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Z_MIN_LIBERAR:
            if (!limiteZabajo) {
                detenerZ();
                moverZHasta(leerPasosZ() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFase(CAL_Z_MIN_SEPARAR);
            }
            else {
                moverZContinuo(1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Z_MIN_SEPARAR:
            if (!objetivoZEnCurso()) {
                cambiarFase(CAL_Z_MIN_2);
            }
            break;

        case CAL_Z_MIN_2:
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(0);

                Serial.println(F("[CAL][Z] Z abajo verificado. Z temporal = 0 pasos"));

                cambiarFase(CAL_Z_MAX_1);
            }
            else {
                moverZContinuo(-1, DIV_CAL_LENTA);
            }
            break;

        //----------------------------------------------------------------------------------------
        // Z ARRIBA
        //----------------------------------------------------------------------------------------
        case CAL_Z_MAX_1:
            if (limiteZarriba) {
                detenerZ();
                cambiarFase(CAL_Z_MAX_LIBERAR);
            }
            else {
                moverZContinuo(1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Z_MAX_LIBERAR:
            if (!limiteZarriba) {
                detenerZ();
                moverZHasta(leerPasosZ() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFase(CAL_Z_MAX_SEPARAR);
            }
            else {
                moverZContinuo(-1, DIV_CAL_RAPIDA);
            }
            break;

        case CAL_Z_MAX_SEPARAR:
            if (!objetivoZEnCurso()) {
                cambiarFase(CAL_Z_MAX_2);
            }
            break;

        case CAL_Z_MAX_2:
            if (limiteZarriba) {
                detenerZ();

                rangoZPasos = leerPasosZ();

                if (rangoZPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorError("Rango Z invalido");
                    return;
                }

                Serial.print(F("[CAL][Z] Rango = "));
                Serial.print(rangoZPasos);
                Serial.println(F(" pasos"));

                homeIniciado = false;

                cambiarFase(CAL_HOME);
            }
            else {
                moverZContinuo(1, DIV_CAL_LENTA);
            }
            break;

        //----------------------------------------------------------------------------------------
        // HOME AL CENTRO
        //----------------------------------------------------------------------------------------
        case CAL_HOME:
            if (!homeIniciado) {
                homeIniciado = true;

                long centroX = rangoXPasos / 2;
                long centroY = rangoYPasos / 2;
                long centroZ = rangoZPasos / 2;

                Serial.print(F("[CAL] Centro X = "));
                Serial.print(centroX);
                Serial.print(F(" pasos | Centro Y = "));
                Serial.print(centroY);
                Serial.print(F(" pasos | Centro Z = "));
                Serial.print(centroZ);
                Serial.println(F(" pasos"));

                moverXHasta(centroX, DIV_HOME);
                moverYHasta(centroY, DIV_HOME);
                moverZHasta(centroZ, DIV_HOME);
            }

            if (
                !objetivoXEnCurso() &&
                !objetivoYEnCurso() &&
                !objetivoZEnCurso() &&
                movX == 0 &&
                movY == 0 &&
                movZ == 0
            ) {
                fijarPasosX(0);
                fijarPasosY(0);
                fijarPasosZ(0);

                if (!calcularEscalaAutomatica()) {
                    calibracionXYValida = false;

                    detenerPorError(
                        "No se pudo calcular la escala"
                    );

                    return;
                }

                calibracionXYValida = true;
                calibracionZValida = true;

                cambiarFase(
                    CAL_COMPLETA
                );

                Serial.println(F("[CAL] HOME alcanzado"));
                Serial.println(F("[CAL] Centro fisico definido como X=0, Y=0, Z=0"));
                Serial.println(F("================================"));
                Serial.println(F("CALIBRACION COMPLETA"));
                Serial.println(F("================================"));

                imprimirRangoTrabajo();
            }
            break;

        case CAL_COMPLETA:
            detenerTodos();
            break;

        case CAL_ERROR:
            detenerTodos();
            break;
    }
}

//------------------------------------------------------------------------------------------------
// TERMINAL
//------------------------------------------------------------------------------------------------
void mostrarAyudaTerminal() {
    Serial.println();
    Serial.println(
        F("===== COMANDOS DISPONIBLES =====")
    );

    Serial.println(
        F("GOTO X Y Z       -> Ir a posicion en mm")
    );

    Serial.println(
        F("Ejemplo: GOTO 100 -50 30")
    );

    Serial.println(
        F("Tambien: 100 -50 30")
    );

    Serial.println(
        F("HOME             -> Ir a X=0 Y=0")
    );

    Serial.println(
        F("POS              -> Mostrar posicion actual")
    );

    Serial.println(
        F("RANGO            -> Mostrar rango de trabajo")
    );

    Serial.println(
        F("STOP             -> Detener movimiento")
    );

    Serial.println(
        F("AYUDA            -> Mostrar ayuda")
    );

    Serial.println(
        F("Escala automatica: X=496 mm, Y=337 mm")
    );

    Serial.println(
        F("Z esta protegido por finales en manual; en XYZ aun no se mueve.")
    );

    Serial.println(
        F("===============================")
    );
}



void procesarComandoTerminal(String comando) {
    comando.trim();

    if (comando.length() == 0) return;

    String comandoMayuscula = comando;
    comandoMayuscula.toUpperCase();

    if (comandoMayuscula == "AYUDA") {
        mostrarAyudaTerminal();
        return;
    }

    if (comandoMayuscula == "POS") {
        imprimirPosicionActual();
        return;
    }

    if (comandoMayuscula == "RANGO") {
        imprimirRangoTrabajo();
        return;
    }

    if (comandoMayuscula == "STOP") {
        cancelarMovimientoCartesiano("Orden STOP");
        return;
    }

    if (comandoMayuscula == "HOME") {
        irAPosicionXYZ(0.0f, 0.0f, 0.0f);
        return;
    }

    

    String datos = comando;

    if (comandoMayuscula.startsWith("GOTO ")) {
        datos = comando.substring(5);
    }

    datos.replace(',', ' ');

    float x;
    float y;
    float z;

    int valoresLeidos = sscanf(
        datos.c_str(),
        "%f %f %f",
        &x,
        &y,
        &z
    );

    if (valoresLeidos == 3) {
        irAPosicionXYZ(x, y, z);
        return;
    }

    Serial.print(F("[ERROR] Comando no reconocido: "));
    Serial.println(comando);
    Serial.println(F("Usa: GOTO X Y Z"));
}

void leerTerminal() {
    while (Serial.available()) {
        char caracter = Serial.read();

        if (caracter == '\n' || caracter == '\r') {
            if (lineaTerminal.length() > 0) {
                procesarComandoTerminal(lineaTerminal);
                lineaTerminal = "";
            }
        }
        else {
            if (lineaTerminal.length() < 80) {
                lineaTerminal += caracter;
            }
            else {
                lineaTerminal = "";
                Serial.println(F("[ERROR] Comando demasiado largo."));
            }
        }
    }
}

//------------------------------------------------------------------------------------------------
// COMUNICACION: PORTENTA -> ESP32 PARA LA PANTALLA
//------------------------------------------------------------------------------------------------
uint8_t calcularChecksum(const uint8_t *datos, size_t longitud) {
    uint8_t resultado = 0;

    for (size_t i = 0; i < longitud; i++) {
        resultado ^= datos[i];
    }

    return resultado;
}

uint8_t codigoErrorCalibracion() {
    if (faseCal != CAL_ERROR) return 0;

    if (strcmp(mensajeError, "Tiempo maximo excedido") == 0) return 1;
    if (strcmp(mensajeError, "Rango X invalido") == 0) return 2;
    if (strcmp(mensajeError, "Rango Y invalido") == 0) return 3;
    if (strcmp(mensajeError, "Rango Z invalido") == 0) return 4;
    if (strcmp(mensajeError, "No se pudo calcular la escala") == 0) return 5;

    return 255;
}

void construirPaquetePantalla(PaquetePantalla &p) {
    memset(&p, 0, sizeof(p));

    p.magic = MAGIC_PANTALLA;
    p.version = VERSION_PANTALLA;
    p.estado = (uint8_t)estadoActual;
    p.opcionMenu = (uint8_t)opcionMenu;
    p.faseCal = (uint8_t)faseCal;

    if (calibracionXYValida) p.flags |= 0x01;
    if (calibracionZValida) p.flags |= FLAG_Z_CALIBRADO;
    if (movimientoCartesianoActivo) p.flags |= 0x02;
    if (escalaConfigurada()) p.flags |= 0x04;
    if (esp32Conectado) p.flags |= 0x08;
    if (btConectado) p.flags |= 0x10;

    if (limiteXmas) p.limites |= 0x01;
    if (limiteXmenos) p.limites |= 0x02;
    if (limiteYmas) p.limites |= 0x04;
    if (limiteYmenos) p.limites |= 0x08;
    if (limiteZarriba) p.limites |= LIMITE_Z_ARRIBA_BIT;
    if (limiteZabajo) p.limites |= LIMITE_Z_ABAJO_BIT;

    noInterrupts();
    p.movX = movX;
    p.movY = movY;
    p.movZ = movZ;
    interrupts();

    p.servoRot = posServoRot;
    p.servoPin = posServoPin;
    p.errorCal = codigoErrorCalibracion();

    if (estadoActual == MODO_HOMING) {
        p.valor1 = leerPasosX();
        p.valor2 = leerPasosY();
        p.valor3 = rangoXPasos;
        p.valor4 = rangoYPasos;
    }
    else if (estadoActual == MODO_CARTESIANO) {
        p.valor1 = lroundf(posicionXmm() * 10.0f);
        p.valor2 = lroundf(posicionYmm() * 10.0f);
        p.valor3 = lroundf(objetivoXmm * 10.0f);
        p.valor4 = lroundf(objetivoYmm * 10.0f);
    }

    p.checksum = calcularChecksum(
        (const uint8_t *)&p,
        sizeof(PaquetePantalla) - 1
    );
}

bool enviarEstadoPantalla() {
    PaquetePantalla paquete;
    construirPaquetePantalla(paquete);

    Wire.beginTransmission((uint8_t)DIRECCION_ESP32);

    const size_t bytesEnBuffer = Wire.write(
        (const uint8_t *)&paquete,
        sizeof(paquete)
    );

    const uint8_t error = Wire.endTransmission(true);
    ultimoErrorEnvioPantalla = error;

    if (bytesEnBuffer == sizeof(paquete) && error == 0) {
        enviosPantallaOk++;
        return true;
    }

    enviosPantallaError++;
    return false;
}

//------------------------------------------------------------------------------------------------
// COMUNICACION: ESP32 -> PORTENTA PARA EL CONTROL
//------------------------------------------------------------------------------------------------
void procesarControlRecibido(
    int8_t jX,
    int8_t jY,
    int8_t jZ,
    uint8_t rBtnX,
    uint8_t rBtnTri
) {
    bool clickX = (rBtnX == 1 && prevBtnX == 0);
    bool clickTri = (rBtnTri == 1 && prevBtnTri == 0);

    prevBtnX = rBtnX;
    prevBtnTri = rBtnTri;

    if (!btConectado) {
        if (estadoActual != MODO_CARTESIANO) {
            detenerTodos();
            estadoActual = MENU_PRINCIPAL;
            faseCal = CAL_ESPERA;
        }

        return;
    }

    //--------------------------------------------------------------------------------------------
    // MENU PRINCIPAL
    //--------------------------------------------------------------------------------------------
    if (estadoActual == MENU_PRINCIPAL) {
        detenerTodos();

        if (jY != 0 && prevInputY == 0) {
            opcionMenu -= jY;

            if (opcionMenu < 0) opcionMenu = 2;
            if (opcionMenu > 2) opcionMenu = 0;
        }

        prevInputY = jY;

        if (clickX) {
            if (opcionMenu == 0) {
                estadoActual = MODO_MANUAL;
            }
            else if (opcionMenu == 1) {
                estadoActual = MODO_HOMING;
                faseCal = CAL_ESPERA;
                mensajeError = "";
            }
            else if (opcionMenu == 2) {
                if (!calibracionXYValida) {
                    Serial.println(F("[ERROR] Debes calibrar X/Y antes de usar POSICION XYZ."));
                }
                else {
                    estadoActual = MODO_CARTESIANO;
                    detenerTodos();
                    movimientoCartesianoActivo = false;

                    Serial.println();
                    Serial.println(F("MODO POSICION XYZ"));

                    imprimirRangoTrabajo();
                    mostrarAyudaTerminal();
                }
            }
        }

        return;
    }

    //--------------------------------------------------------------------------------------------
    // MODO MANUAL
    //--------------------------------------------------------------------------------------------
    if (estadoActual == MODO_MANUAL) {
        if ((jX > 0 && limiteXmas) || (jX < 0 && limiteXmenos)) {
            jX = 0;
        }

        if ((jY > 0 && limiteYmas) || (jY < 0 && limiteYmenos)) {
            jY = 0;
        }

        if (
            (jZ == DIR_Z_ARRIBA && limiteZarriba) ||
            (jZ == DIR_Z_ABAJO && limiteZabajo)
        ) {
            jZ = 0;
        }

        moverXContinuo(jX, DIV_MANUAL);
        moverYContinuo(jY, DIV_MANUAL);
        moverZContinuo(jZ, DIV_MANUAL);

        aplicarBloqueoPorFinales();

        if (clickTri) {
            detenerTodos();
            estadoActual = MENU_PRINCIPAL;
            prevInputY = 0;
        }

        return;
    }

    //--------------------------------------------------------------------------------------------
    // MODO CALIBRACION
    //--------------------------------------------------------------------------------------------
    if (estadoActual == MODO_HOMING) {
        if (clickTri) {
            detenerTodos();
            estadoActual = MENU_PRINCIPAL;
            faseCal = CAL_ESPERA;
            prevInputY = 0;
        }
        else {
            if (faseCal == CAL_ESPERA && clickX) {
                iniciarCalibracion();
            }

            procesarCalibracion();
            aplicarBloqueoPorFinales();
        }

        return;
    }

    //--------------------------------------------------------------------------------------------
    // MODO CARTESIANO
    //--------------------------------------------------------------------------------------------
    if (estadoActual == MODO_CARTESIANO) {
        moverZContinuo(0, DIV_MANUAL);

        if (clickTri) {
            cancelarMovimientoCartesiano("Salida al menu");
            estadoActual = MENU_PRINCIPAL;
            prevInputY = 0;
        }
    }
}

bool leerControlESP32() {
    int bytes = Wire.requestFrom(
        (uint8_t)DIRECCION_ESP32,
        (uint8_t)8
    );

    if (bytes != 8 || Wire.available() < 8) {
        while (Wire.available()) Wire.read();
        return false;
    }

    int8_t rawX = (int8_t)Wire.read();
    int8_t rawY = (int8_t)Wire.read();
    int8_t rawZ = (int8_t)Wire.read();

    uint8_t rBT = (uint8_t)Wire.read();
    uint8_t rBtnX = (uint8_t)Wire.read();
    uint8_t rBtnTri = (uint8_t)Wire.read();

    posServoRot = (uint8_t)Wire.read();
    posServoPin = (uint8_t)Wire.read();

    int8_t jX = (rawX == 0) ? 0 : (rawX == 1 ? 1 : -1);
    int8_t jY = (rawY == 0) ? 0 : (rawY == 1 ? 1 : -1);
    int8_t jZ = (rawZ == 0) ? 0 : (rawZ == 1 ? 1 : -1);

    btConectado = (rBT == 1);
    esp32Conectado = true;
    ultimoPaqueteControlMs = millis();

    procesarControlRecibido(jX, jY, jZ, rBtnX, rBtnTri);

    return true;
}

void aplicarTimeoutComunicacion() {
    if (millis() - ultimoPaqueteControlMs <= TIMEOUT_CONTROL_MS) {
        return;
    }

    esp32Conectado = false;
    btConectado = false;
    prevBtnX = 0;
    prevBtnTri = 0;

    if (estadoActual != MODO_CARTESIANO) {
        detenerTodos();
        estadoActual = MENU_PRINCIPAL;
        faseCal = CAL_ESPERA;
    }
}

//------------------------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // La Portenta es el unico maestro de este bus.
    // La OLED ya no esta conectada aqui.
    // Se usa 100 kHz porque era la velocidad del codigo que ya funcionaba
    // y es mas tolerante para el ESP32 actuando como esclavo.
    Wire.begin();
    Wire.setClock(100000);

    // No se transmite nada al ESP32 durante sus primeros 3 segundos de encendido.
    // Esto evita callbacks I2C mientras Bluepad32 y los buses del ESP32 arrancan.
    tiempoEncendidoSistema = millis();
    comunicacionI2CHabilitada = false;

    Serial.println(F("I2C hacia ESP32 retenido durante 3000 ms"));

    digital_inputs.init();

    digital_outputs.set(pP_X, LOW);
    digital_outputs.set(pP_Y, LOW);
    digital_outputs.set(pP_Z, LOW);

    digital_outputs.set(pD_X, LOW);
    digital_outputs.set(pD_Y, LOW);
    digital_outputs.set(pD_Z, LOW);

    leerFinalesCarrera();

    motorTicker.attach(&generarPulsoMotor, velocidadMotores);

    ultimoPaqueteControlMs = millis();

    Serial.println();
    Serial.println(F("Sistema listo"));
    Serial.println(F("Finales Z: DIN04=arriba, DIN05=abajo"));
    Serial.println(F("Monitor serial: 115200 baudios"));
    Serial.println(F("Control I2C: 5 ms"));
    Serial.println(F("Estado de pantalla: 150 ms"));

    mostrarAyudaTerminal();
}

//------------------------------------------------------------------------------------------------
// LOOP PRINCIPAL
//------------------------------------------------------------------------------------------------
void loop() {
    leerTerminal();

    // Estas tareas ya no dependen de la actualizacion de la OLED.
    leerFinalesCarrera();
    actualizarMovimientoCartesiano();
    aplicarBloqueoPorFinales();

    unsigned long ahora = millis();

    // Habilitar la comunicacion solo cuando el ESP32 ya tuvo tiempo de completar su arranque.
    if (
        !comunicacionI2CHabilitada &&
        ahora - tiempoEncendidoSistema >= RETARDO_ARRANQUE_ESP32_MS
    ) {
        comunicacionI2CHabilitada = true;
        tAnteriorI2C = ahora;
        tAnteriorEstadoPantalla = ahora;

        Wire.beginTransmission((uint8_t)DIRECCION_ESP32);
        const uint8_t errorDeteccionESP32 = Wire.endTransmission(true);

        Serial.print(F("Deteccion tardia ESP32 I2C 0x40: "));
        if (errorDeteccionESP32 == 0) {
            Serial.println(F("OK"));
        }
        else {
            Serial.print(F("ERROR "));
            Serial.println(errorDeteccionESP32);
        }
    }

    bool huboLecturaI2C = false;

    if (comunicacionI2CHabilitada) {
        // Lectura rapida del joystick y botones.
        if (ahora - tAnteriorI2C >= PERIODO_CONTROL_MS) {
            tAnteriorI2C = ahora;
            leerControlESP32();
            huboLecturaI2C = true;
        }

        // No se hace lectura y escritura en la misma vuelta del loop.
        if (
            !huboLecturaI2C &&
            ahora - tAnteriorEstadoPantalla >= PERIODO_ESTADO_PANTALLA_MS
        ) {
            tAnteriorEstadoPantalla = ahora;
            enviarEstadoPantalla();
        }
    }

    if (ahora - ultimoReporteI2C >= 1000) {
        ultimoReporteI2C = ahora;

        Serial.print(F("[I2C TX OLED] ok="));
        Serial.print(enviosPantallaOk);
        Serial.print(F(" error="));
        Serial.print(enviosPantallaError);
        Serial.print(F(" ultimoCodigo="));
        Serial.println(ultimoErrorEnvioPantalla);
    }

    aplicarTimeoutComunicacion();
}

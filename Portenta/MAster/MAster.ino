//------------------------------------------------------------------------------------------------
// DECLARACION DE LIBRERIAS
//------------------------------------------------------------------------------------------------
#include <Arduino_MachineControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "mbed.h"
#include <math.h>
#include <stdio.h>

using namespace machinecontrol;

//------------------------------------------------------------------------------------------------
// CONFIGURACION PANTALLA OLED
//------------------------------------------------------------------------------------------------
Adafruit_SH1106G pantalla(128, 64, &Wire, -1);

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
const uint16_t DIV_POSICION = 2;

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

    CAL_HOME,
    CAL_COMPLETA,
    CAL_ERROR
};

FaseCalibracion faseCal = CAL_ESPERA;

unsigned long inicioFase = 0;

bool homeIniciado = false;
bool calibracionXYValida = false;

const char *mensajeError = "";

long rangoXPasos = 0;
long rangoYPasos = 0;

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

bool limiteXmas = false;
bool limiteXmenos = false;
bool limiteYmenos = false;
bool limiteYmas = false;

//------------------------------------------------------------------------------------------------
// ACTUADORES
//------------------------------------------------------------------------------------------------
uint8_t posServoRot = 90;
uint8_t posServoPin = 90;

//------------------------------------------------------------------------------------------------
// COMUNICACION
//------------------------------------------------------------------------------------------------
#define DIRECCION_ESP32 0x40

unsigned long tAnteriorI2C = 0;
unsigned long tAnteriorTerminal = 0;

bool esp32Conectado = false;
bool btConectado = false;

int8_t prevInputY = 0;
uint8_t prevBtnX = 0;
uint8_t prevBtnTri = 0;

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

//------------------------------------------------------------------------------------------------
// LECTURA DE FINALES DE CARRERA
//------------------------------------------------------------------------------------------------
void leerFinalesCarrera() {
    limiteXmas   = !digital_inputs.read(DIN_READ_CH_PIN_01); // X+
    limiteXmenos = !digital_inputs.read(DIN_READ_CH_PIN_00); // X-
    limiteYmenos = !digital_inputs.read(DIN_READ_CH_PIN_02); // Y-
    limiteYmas   = !digital_inputs.read(DIN_READ_CH_PIN_03); // Y+
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

//------------------------------------------------------------------------------------------------
// SEGURIDAD POR FINALES
//------------------------------------------------------------------------------------------------
void aplicarBloqueoPorFinales() {
    if (movX > 0 && limiteXmas) detenerX();
    if (movX < 0 && limiteXmenos) detenerX();

    if (movY > 0 && limiteYmas) detenerY();
    if (movY < 0 && limiteYmenos) detenerY();

    // Z no se bloquea todavia porque no tiene finales implementados.
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
    Serial.println(F(" pasos | SIN CALIBRACION"));

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
    movimientoCartesianoActivo = false;

    rangoXPasos = 0;
    rangoYPasos = 0;

    homeIniciado = false;
    mensajeError = "";

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("INICIO CALIBRACION X/Y"));
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

                homeIniciado = false;

                cambiarFase(CAL_HOME);
            }
            else {
                moverYContinuo(1, DIV_CAL_LENTA);
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

                Serial.print(F("[CAL] Centro X = "));
                Serial.print(centroX);
                Serial.print(F(" pasos | Centro Y = "));
                Serial.print(centroY);
                Serial.println(F(" pasos"));

                moverXHasta(centroX, DIV_HOME);
                moverYHasta(centroY, DIV_HOME);
            }

            if (
                !objetivoXEnCurso() &&
                !objetivoYEnCurso() &&
                movX == 0 &&
                movY == 0
            ) {
                fijarPasosX(0);
                fijarPasosY(0);

                if (!calcularEscalaAutomatica()) {
                    calibracionXYValida = false;

                    detenerPorError(
                        "No se pudo calcular la escala"
                    );

                    return;
                }

                calibracionXYValida = true;

                cambiarFase(
                    CAL_COMPLETA
                );

                Serial.println(F("[CAL] HOME alcanzado"));
                Serial.println(F("[CAL] Centro fisico definido como X=0, Y=0"));
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
        F("Z se recibe, pero todavia no se mueve.")
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
// PANTALLA
//------------------------------------------------------------------------------------------------
void actualizarPantalla() {
    pantalla.clearDisplay();
    pantalla.setCursor(0, 0);

    if (estadoActual == MENU_PRINCIPAL) {
        pantalla.println(F("--- MENU PRINCIPAL ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        pantalla.setCursor(4, 15);
        pantalla.print(opcionMenu == 0 ? "-> " : "   ");
        pantalla.println(F("MODO MANUAL"));

        pantalla.setCursor(4, 29);
        pantalla.print(opcionMenu == 1 ? "-> " : "   ");
        pantalla.println(F("CALIBRACION"));

        pantalla.setCursor(4, 43);
        pantalla.print(opcionMenu == 2 ? "-> " : "   ");
        pantalla.println(F("POSICION XYZ"));

        pantalla.setCursor(0, 56);
        pantalla.print(calibracionXYValida ? F("XY CALIBRADO") : F("XY SIN CALIBRAR"));
    }

    else if (estadoActual == MODO_MANUAL) {
        pantalla.println(F("CONTROL MANUAL"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        pantalla.setCursor(0, 14);
        pantalla.print(F("X:"));
        pantalla.print(movX == 0 ? " 0" : (movX > 0 ? "+1" : "-1"));
        pantalla.print(F(" Y:"));
        pantalla.print(movY == 0 ? " 0" : (movY > 0 ? "+1" : "-1"));
        pantalla.print(F(" Z:"));
        pantalla.println(movZ == 0 ? " 0" : (movZ > 0 ? "+1" : "-1"));

        pantalla.setCursor(0, 28);
        pantalla.print(F("X+:"));
        pantalla.print(limiteXmas ? "ON " : "OFF");
        pantalla.print(F(" X-:"));
        pantalla.println(limiteXmenos ? "ON" : "OFF");

        pantalla.setCursor(0, 42);
        pantalla.print(F("Y+:"));
        pantalla.print(limiteYmas ? "ON " : "OFF");
        pantalla.print(F(" Y-:"));
        pantalla.println(limiteYmenos ? "ON" : "OFF");

        pantalla.setCursor(0, 56);
        pantalla.print(F("S1:"));
        pantalla.print(posServoRot);
        pantalla.print(F(" S2:"));
        pantalla.print(posServoPin);
    }

    else if (estadoActual == MODO_HOMING) {
        pantalla.println(F("--- CALIBRACION ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        if (faseCal == CAL_ESPERA) {
            pantalla.setCursor(0, 20);
            pantalla.println(F("PRESIONA X"));
            pantalla.println(F("PARA INICIAR"));
            pantalla.println(F("TRI: SALIR"));
        }

        else if (faseCal == CAL_COMPLETA) {
            pantalla.setCursor(0, 18);
            pantalla.println(F("CALIBRACION OK"));
            pantalla.println(F("HOME X=0 Y=0"));
            pantalla.println(F("TRI: MENU"));
        }

        else if (faseCal == CAL_ERROR) {
            pantalla.setCursor(0, 18);
            pantalla.println(F("ERROR CALIBRACION"));
            pantalla.println(mensajeError);
            pantalla.println(F("TRI: MENU"));
        }

        else {
            pantalla.setCursor(0, 16);
            pantalla.println(nombreFase());

            pantalla.setCursor(0, 32);
            pantalla.print(F("X:"));
            pantalla.print(leerPasosX());
            pantalla.print(F(" Y:"));
            pantalla.println(leerPasosY());

            pantalla.setCursor(0, 46);
            pantalla.print(F("RX:"));
            pantalla.print(rangoXPasos);
            pantalla.print(F(" RY:"));
            pantalla.println(rangoYPasos);

            pantalla.setCursor(0, 57);
            pantalla.print(F("TRI: CANCELAR"));
        }
    }

    else if (estadoActual == MODO_CARTESIANO) {
        pantalla.println(F("--- POSICION XYZ ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        pantalla.setCursor(0, 15);

        if (escalaConfigurada()) {
            pantalla.print(F("X:"));
            pantalla.print(posicionXmm(), 1);
            pantalla.print(F(" Y:"));
            pantalla.println(posicionYmm(), 1);
        }
        else {
            pantalla.println(F("SIN ESCALA MM"));
        }

        pantalla.setCursor(0, 29);
        pantalla.print(F("TX:"));
        pantalla.print(objetivoXmm, 1);
        pantalla.print(F(" TY:"));
        pantalla.println(objetivoYmm, 1);

        pantalla.setCursor(0, 43);

        if (movimientoCartesianoActivo) {
            pantalla.println(F("ESTADO: MOVIENDO"));
        }
        else {
            pantalla.println(F("ESTADO: LISTO"));
        }

        pantalla.setCursor(0, 56);
        pantalla.print(F("SERIAL 115200"));
    }

    pantalla.display();
}

//------------------------------------------------------------------------------------------------
// SETUP
//------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    Wire.begin();

    delay(100);

    if (pantalla.begin(0x3C, true)) {
        pantalla.clearDisplay();
        pantalla.display();
        delay(50);
        pantalla.setTextSize(1);
        pantalla.setTextColor(SH110X_WHITE);
    }

    digital_inputs.init();

    digital_outputs.set(pP_X, LOW);
    digital_outputs.set(pP_Y, LOW);
    digital_outputs.set(pP_Z, LOW);

    digital_outputs.set(pD_X, LOW);
    digital_outputs.set(pD_Y, LOW);
    digital_outputs.set(pD_Z, LOW);

    leerFinalesCarrera();

    motorTicker.attach(&generarPulsoMotor, velocidadMotores);

    Serial.println();
    Serial.println(F("Sistema listo"));
    Serial.println(F("Monitor serial: 115200 baudios"));

    mostrarAyudaTerminal();
}

//------------------------------------------------------------------------------------------------
// LOOP PRINCIPAL
//------------------------------------------------------------------------------------------------
void loop() {
    leerTerminal();

    unsigned long tActual = millis();

    if (tActual - tAnteriorI2C >= 15) {
        tAnteriorI2C = tActual;

        leerFinalesCarrera();

        actualizarMovimientoCartesiano();

        int bytes = Wire.requestFrom((uint8_t)DIRECCION_ESP32, (uint8_t)8);

        if (bytes == 8) {
            esp32Conectado = true;

            int8_t rawX = Wire.read();
            int8_t rawY = Wire.read();
            int8_t rawZ = Wire.read();

            uint8_t rBT = Wire.read();
            uint8_t rBtnX = Wire.read();
            uint8_t rBtnTri = Wire.read();

            posServoRot = Wire.read();
            posServoPin = Wire.read();

            btConectado = (rBT == 1);

            int8_t jX = (rawX == 0) ? 0 : (rawX == 1 ? 1 : -1);
            int8_t jY = (rawY == 0) ? 0 : (rawY == 1 ? 1 : -1);
            int8_t jZ = (rawZ == 0) ? 0 : (rawZ == 1 ? 1 : -1);

            bool clickX = (rBtnX == 1 && prevBtnX == 0);
            bool clickTri = (rBtnTri == 1 && prevBtnTri == 0);

            prevBtnX = rBtnX;
            prevBtnTri = rBtnTri;

            if (btConectado) {
                //--------------------------------------------------------------------------------
                // MENU PRINCIPAL
                //--------------------------------------------------------------------------------
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
                }

                //--------------------------------------------------------------------------------
                // MODO MANUAL
                //--------------------------------------------------------------------------------
                else if (estadoActual == MODO_MANUAL) {
                    if ((jX > 0 && limiteXmas) || (jX < 0 && limiteXmenos)) {
                        jX = 0;
                    }

                    if ((jY > 0 && limiteYmas) || (jY < 0 && limiteYmenos)) {
                        jY = 0;
                    }

                    moverXContinuo(jX, DIV_MANUAL);
                    moverYContinuo(jY, DIV_MANUAL);
                    moverZContinuo(jZ, DIV_MANUAL);

                    aplicarBloqueoPorFinales();

                    if (clickTri) {
                        detenerTodos();
                        estadoActual = MENU_PRINCIPAL;
                    }
                }

                //--------------------------------------------------------------------------------
                // MODO CALIBRACION
                //--------------------------------------------------------------------------------
                else if (estadoActual == MODO_HOMING) {
                    if (clickTri) {
                        detenerTodos();
                        estadoActual = MENU_PRINCIPAL;
                        faseCal = CAL_ESPERA;
                    }
                    else {
                        if (faseCal == CAL_ESPERA && clickX) {
                            iniciarCalibracion();
                        }

                        procesarCalibracion();
                        aplicarBloqueoPorFinales();
                    }
                }

                //--------------------------------------------------------------------------------
                // MODO CARTESIANO
                //--------------------------------------------------------------------------------
                else if (estadoActual == MODO_CARTESIANO) {
                    // En este modo X/Y se controlan desde la terminal.
                    // No se debe copiar jX ni jY a los motores.

                    moverZContinuo(0, DIV_MANUAL);

                    if (clickTri) {
                        cancelarMovimientoCartesiano("Salida al menu");
                        estadoActual = MENU_PRINCIPAL;
                    }
                }
            }

            else {
                // Si el control Bluetooth se desconecta:
                // - En manual/calibracion se detiene todo.
                // - En modo cartesiano se permite seguir usando terminal.

                if (estadoActual != MODO_CARTESIANO) {
                    detenerTodos();
                    estadoActual = MENU_PRINCIPAL;
                }
            }
        }

        else {
            esp32Conectado = false;
            btConectado = false;

            // Si se pierde I2C:
            // - En manual/calibracion se detiene.
            // - En modo cartesiano se permite seguir usando terminal.

            if (estadoActual != MODO_CARTESIANO) {
                detenerTodos();
                estadoActual = MENU_PRINCIPAL;
            }
        }

        actualizarPantalla();
    }
}
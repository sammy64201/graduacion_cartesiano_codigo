//------------------------------------------------------------------------------------------------DELCARACION-DE-LIBRERIAS----------------------------------------------------------------------------------------------------
#include <Arduino_MachineControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "mbed.h"

using namespace machinecontrol;

// --- CONFIGURACIÓN PANTALLA OLED ---
Adafruit_SH1106G pantalla(128, 64, &Wire, -1);

// --- CONFIGURACIÓN MOTORES STEPPER ---
// DIR+ = 4  X
// PIL+ = 5  X
// DIR+ = 2  Y
// PIL+ = 3  Y
// DIR+ = 1  Z
// PIL+ = 0  Z

const int pP_Z = 0;
const int pP_X = 4;
const int pP_Y = 2;

const int pD_Z = 1;
const int pD_X = 5;
const int pD_Y = 3;

const float velocidadMotores = 0.0001; // segundos
mbed::Ticker motorTicker;

volatile bool estadoPulso = false;
volatile int8_t movX = 0, movY = 0, movZ = 0;

// --- MÁQUINA DE ESTADOS Y NAVEGACIÓN ---
enum EstadoSistema { MENU_PRINCIPAL, MODO_MANUAL, MODO_HOMING };
EstadoSistema estadoActual = MENU_PRINCIPAL;

int opcionMenu = 0;
int faseHoming = 0;

// --- FINALES DE CARRERA ---
// Nueva conexión NC + COM:
//
// DIN 00 = X+
// DIN 01 = X-
// DIN 02 = Y-
// DIN 03 = Y+
//
// Como son NC:
// read() == true  -> final NO presionado
// read() == false -> final presionado o cable desconectado
//
// Por eso se usa !digital_inputs.read(...)
bool limiteXmas   = false;
bool limiteXmenos = false;
bool limiteYmenos = false;
bool limiteYmas   = false;

// --- VARIABLES ACTUADORES ---
uint8_t posServoRot = 90; // SERVO 01
uint8_t posServoPin = 90; // SERVO 02

// --- VARIABLES COMUNICACIÓN Y CONTROL ---
#define DIRECCION_ESP32 0x40

unsigned long tAnteriorI2C = 0;
bool esp32Conectado = false;
bool btConectado = false;

int8_t prevInputY = 0;
uint8_t prevBtnX = 0;
uint8_t prevBtnTri = 0;

// ------------------------------------------------------------------------------------------------
// FUNCIÓN PARA LEER FINALES DE CARRERA CON LÓGICA NC
// ------------------------------------------------------------------------------------------------
void leerFinalesCarrera() {
    limiteXmas   = !digital_inputs.read(DIN_READ_CH_PIN_01); // X+
    limiteXmenos = !digital_inputs.read(DIN_READ_CH_PIN_00); // X-
    limiteYmenos = !digital_inputs.read(DIN_READ_CH_PIN_02); // Y-
    limiteYmas   = !digital_inputs.read(DIN_READ_CH_PIN_03); // Y+
}

// ------------------------------------------------------------------------------------------------
// FUNCIÓN DE SEGURIDAD POR DIRECCIÓN
// ------------------------------------------------------------------------------------------------
void aplicarBloqueoPorFinales() {
    // X+
    if (movX > 0 && limiteXmas) {
        movX = 0;
    }

    // X-
    if (movX < 0 && limiteXmenos) {
        movX = 0;
    }

    // Y+
    if (movY > 0 && limiteYmas) {
        movY = 0;
    }

    // Y-
    if (movY < 0 && limiteYmenos) {
        movY = 0;
    }

    // Por ahora no se bloquea Z porque no indicaste finales de carrera para Z.
}

// ------------------------------------------------------------------------------------------------
// INTERRUPCIÓN HARDWARE: GENERACIÓN DE PULSOS PARA MOTORES
// ------------------------------------------------------------------------------------------------
void generarPulsoMotor() {
    estadoPulso = !estadoPulso;

    if (movX != 0) digital_outputs.set(pP_X, estadoPulso);
    else digital_outputs.set(pP_X, LOW);

    if (movY != 0) digital_outputs.set(pP_Y, estadoPulso);
    else digital_outputs.set(pP_Y, LOW);

    if (movZ != 0) digital_outputs.set(pP_Z, estadoPulso);
    else digital_outputs.set(pP_Z, LOW);
}

// ------------------------------------------------------------------------------------------------
// INTERFAZ GRÁFICA
// ------------------------------------------------------------------------------------------------
void actualizarPantalla() {
    pantalla.clearDisplay();
    pantalla.setCursor(0, 0);

    if (estadoActual == MENU_PRINCIPAL) {
        pantalla.println(F("--- MENU PRINCIPAL ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        pantalla.setCursor(10, 25);
        pantalla.print(opcionMenu == 0 ? "-> " : "   ");
        pantalla.println(F("MODO MANUAL"));

        pantalla.setCursor(10, 40);
        pantalla.print(opcionMenu == 1 ? "-> " : "   ");
        pantalla.println(F("CALIBRACION"));

        pantalla.setCursor(0, 56);
        pantalla.print(F("MOTORES BLOQUEADOS"));
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

        if (faseHoming == 0) {
            pantalla.setCursor(0, 25);
            pantalla.println(F("PRESIONA X"));
            pantalla.println(F("PARA INICIAR"));
        } else {
            pantalla.setCursor(0, 18);
            pantalla.print(F("X-: "));
            pantalla.print(limiteXmenos ? "ON " : "OFF");
            pantalla.println(faseHoming > 1 ? " [OK]" : " BUSCANDO");

            pantalla.setCursor(0, 34);
            pantalla.print(F("Y-: "));
            pantalla.print(limiteYmenos ? "ON " : "OFF");
            pantalla.println(faseHoming > 2 ? " [OK]" : " BUSCANDO");

            pantalla.setCursor(0, 50);
            if (faseHoming == 3) {
                pantalla.println(F("HOMING COMPLETO"));
            } else {
                pantalla.println(F("TRI: SALIR"));
            }
        }
    }

    pantalla.display();
}

// ------------------------------------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------------------------------------
void setup() {
    Wire.begin();

    delay(100);

    if (pantalla.begin(0x3C, true)) {
        pantalla.clearDisplay();
        pantalla.display();
        delay(50);
        pantalla.setTextSize(1);
        pantalla.setTextColor(SH110X_WHITE);
    }

    // Entradas digitales industriales 24 V
    digital_inputs.init();

    // Generación de pulsos
    motorTicker.attach(&generarPulsoMotor, velocidadMotores);
}

// ------------------------------------------------------------------------------------------------
// LOOP PRINCIPAL
// ------------------------------------------------------------------------------------------------
void loop() {
    unsigned long tActual = millis();

    // Ciclo de lectura cada 15 ms
    if (tActual - tAnteriorI2C >= 15) {
        tAnteriorI2C = tActual;

        // 1. Leer finales de carrera
        leerFinalesCarrera();

        // 2. Solicitar 8 bytes al ESP32
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

            // Mapeo de joystick a -1, 0, 1
            int8_t jX = (rawX == 0) ? 0 : (rawX == 1 ? 1 : -1);
            int8_t jY = (rawY == 0) ? 0 : (rawY == 1 ? 1 : -1);
            int8_t jZ = (rawZ == 0) ? 0 : (rawZ == 1 ? 1 : -1);

            // Botones con detección de flanco
            bool clickX = (rBtnX == 1 && prevBtnX == 0);
            bool clickTri = (rBtnTri == 1 && prevBtnTri == 0);

            prevBtnX = rBtnX;
            prevBtnTri = rBtnTri;

            if (btConectado) {
                // ==================================================================================
                // MENÚ PRINCIPAL
                // ==================================================================================
                if (estadoActual == MENU_PRINCIPAL) {
                    movX = 0;
                    movY = 0;
                    movZ = 0;

                    if (jY != 0 && prevInputY == 0) {
                        opcionMenu -= jY;

                        if (opcionMenu < 0) opcionMenu = 1;
                        if (opcionMenu > 1) opcionMenu = 0;
                    }

                    prevInputY = jY;

                    if (clickX) {
                        if (opcionMenu == 0) {
                            estadoActual = MODO_MANUAL;
                        }

                        if (opcionMenu == 1) {
                            estadoActual = MODO_HOMING;
                            faseHoming = 0;
                        }
                    }
                }

                // ==================================================================================
                // MODO MANUAL CON BLOQUEO POR FINAL DE CARRERA
                // ==================================================================================
                else if (estadoActual == MODO_MANUAL) {
                    movX = jX;
                    movY = jY;
                    movZ = jZ;

                    // Aquí se aplica la seguridad:
                    // Si toca X+, bloquea solo X+.
                    // Si toca X-, bloquea solo X-.
                    // Si toca Y+, bloquea solo Y+.
                    // Si toca Y-, bloquea solo Y-.
                    aplicarBloqueoPorFinales();

                    if (clickTri) {
                        estadoActual = MENU_PRINCIPAL;
                        movX = 0;
                        movY = 0;
                        movZ = 0;
                    }
                }

                // ==================================================================================
                // MODO HOMING
                // ==================================================================================
                else if (estadoActual == MODO_HOMING) {
                    if (clickTri) {
                        estadoActual = MENU_PRINCIPAL;
                        movX = 0;
                        movY = 0;
                        movZ = 0;
                    } else {
                        // Iniciar homing con botón X
                        if (faseHoming == 0 && clickX) {
                            faseHoming = 1;
                        }

                        // Homing eje X hacia X-
                        if (faseHoming == 1) {
                            movY = 0;
                            movZ = 0;

                            if (limiteXmenos) {
                                movX = 0;
                                faseHoming = 2;
                            } else {
                                movX = -1;
                            }
                        }

                        // Homing eje Y hacia Y-
                        else if (faseHoming == 2) {
                            movX = 0;
                            movZ = 0;

                            if (limiteYmenos) {
                                movY = 0;
                                faseHoming = 3;
                            } else {
                                movY = -1;
                            }
                        }

                        // Homing terminado
                        else if (faseHoming == 3) {
                            movX = 0;
                            movY = 0;
                            movZ = 0;
                        }

                        // Seguridad adicional también durante homing
                        aplicarBloqueoPorFinales();
                    }
                }

                // Aplicar dirección a los drivers físicos
                digital_outputs.set(pD_X, movX > 0 ? HIGH : LOW);
                digital_outputs.set(pD_Y, movY > 0 ? HIGH : LOW);
                digital_outputs.set(pD_Z, movZ > 0 ? HIGH : LOW);
            }

            else {
                // Si el control se desconecta, modo seguro
                estadoActual = MENU_PRINCIPAL;
                movX = 0;
                movY = 0;
                movZ = 0;
            }
        }

        else {
            esp32Conectado = false;
            movX = 0;
            movY = 0;
            movZ = 0;
        }

        actualizarPantalla();
    }
}
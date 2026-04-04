#include <Arduino_MachineControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "mbed.h"

using namespace machinecontrol;

// --- OLED ---
Adafruit_SH1106G pantalla(128, 64, &Wire, -1);

// --- MOTORES ---
const int pP_Z = 0; const int pP_X = 2; const int pP_Y = 4;
const int pD_Z = 1; const int pD_X = 3; const int pD_Y = 5;

mbed::Ticker motorTicker;
volatile bool estadoPulso = false;
volatile int8_t movX = 0, movY = 0, movZ = 0; 

// --- MÁQUINA DE ESTADOS ---
enum EstadoSistema { MENU_PRINCIPAL, MODO_MANUAL, MODO_HOMING };
EstadoSistema estadoActual = MENU_PRINCIPAL;

int opcionMenu = 0; 
int faseHoming = 0; // 0: Espera, 1: EjeX, 2: EjeY, 3: EjeZ, 4: Terminado

// --- LECTURAS DE SENSORES FINALES DE CARRERA ---
bool sensorX = false;
bool sensorY = false;
bool sensorZ = false;

// --- VARIABLES I2C Y BOTONES ---
#define DIRECCION_ESP32 0x40
unsigned long tAnteriorI2C = 0;
bool esp32Conectado = false;
bool btConectado = false;

int8_t prevY = 0;
uint8_t prevBtnX = 0;
uint8_t prevBtnTri = 0;

void generarPulsoMotor() {
    estadoPulso = !estadoPulso;
    if (movX != 0) digital_outputs.set(pP_X, estadoPulso);
    else digital_outputs.set(pP_X, LOW);
    
    if (movY != 0) digital_outputs.set(pP_Y, estadoPulso);
    else digital_outputs.set(pP_Y, LOW);
    
    if (movZ != 0) digital_outputs.set(pP_Z, estadoPulso);
    else digital_outputs.set(pP_Z, LOW);
}

void actualizarPantalla() {
    pantalla.clearDisplay();
    pantalla.setCursor(0,0);             
    
    if (estadoActual == MENU_PRINCIPAL) {
        pantalla.println(F("--- MENU PRINCIPAL ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
        
        pantalla.setCursor(10, 25);
        if(opcionMenu == 0) pantalla.print(F("-> ")); else pantalla.print(F("   "));
        pantalla.println(F("MODO MANUAL"));
        
        pantalla.setCursor(10, 40);
        if(opcionMenu == 1) pantalla.print(F("-> ")); else pantalla.print(F("   "));
        pantalla.println(F("CALIBRACION (HOMING)"));
        
    } else if (estadoActual == MODO_MANUAL) {
        pantalla.println(F("--- MODO MANUAL ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
        pantalla.setCursor(0,25);
        pantalla.print(F("X: ")); pantalla.println(movX == 0 ? "STOP" : (movX > 0 ? "ADELANTE" : "ATRAS"));
        pantalla.setCursor(0,38);
        pantalla.print(F("Y: ")); pantalla.println(movY == 0 ? "STOP" : (movY > 0 ? "ADELANTE" : "ATRAS"));
        pantalla.setCursor(0,50);
        pantalla.print(F("Z: ")); pantalla.println(movZ == 0 ? "STOP" : (movZ > 0 ? "ARRIBA" : "ABAJO"));
        
    } else if (estadoActual == MODO_HOMING) {
        pantalla.println(F("--- CALIBRACION ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
        
        if (faseHoming == 0) {
            pantalla.setCursor(0,25);
            pantalla.println(F("PRESIONA 'X'"));
            pantalla.println(F("PARA INICIAR ZEROS"));
        } else {
            // Interfaz de depuración: Muestra estado físico del sensor
            pantalla.setCursor(0,22);
            pantalla.print(F("X:")); pantalla.print(sensorX ? "(ON) " : "(OFF) ");
            pantalla.println(faseHoming > 1 ? "[OK]" : (faseHoming == 1 ? "BUSCANDO" : "[ ]"));
            
            pantalla.setCursor(0,36);
            pantalla.print(F("Y:")); pantalla.print(sensorY ? "(ON) " : "(OFF) ");
            pantalla.println(faseHoming > 2 ? "[OK]" : (faseHoming == 2 ? "BUSCANDO" : "[ ]"));
            
            pantalla.setCursor(0,50);
            pantalla.print(F("Z:")); pantalla.print(sensorZ ? "(ON) " : "(OFF) ");
            pantalla.println(faseHoming > 3 ? "[OK]" : (faseHoming == 3 ? "BUSCANDO" : "[ ]"));
        }
    }
    pantalla.display(); 
}

void setup() {
    Wire.begin();
    Wire.setClock(400000); 
    if(pantalla.begin(0x3C)) {
        pantalla.setTextSize(1);
        pantalla.setTextColor(SH110X_WHITE);
    }
    
    digital_inputs.init(); // Inicia el módulo de Entradas de 24V

    motorTicker.attach(&generarPulsoMotor, 0.0006);
}

void loop() {
    unsigned long tActual = millis();

    if (tActual - tAnteriorI2C >= 15) {
        tAnteriorI2C = tActual;
        
        // 1. LEER LOS SENSORES FÍSICOS USANDO LAS CONSTANTES OFICIALES
        sensorX = digital_inputs.read(DIN_READ_CH_PIN_00);
        sensorY = digital_inputs.read(DIN_READ_CH_PIN_01);
        sensorZ = digital_inputs.read(DIN_READ_CH_PIN_02);
        
        int bytes = Wire.requestFrom((uint8_t)DIRECCION_ESP32, (uint8_t)6);
        
        if (bytes == 6) {
            esp32Conectado = true;
            uint8_t rX = Wire.read();
            uint8_t rY = Wire.read();
            uint8_t rZ = Wire.read();
            uint8_t rBT = Wire.read();
            uint8_t rBtnX = Wire.read();
            uint8_t rBtnTri = Wire.read();

            btConectado = (rBT == 1);

            int8_t jX = (rX == 0) ? 0 : (rX == 1 ? 1 : -1);
            int8_t jY = (rY == 0) ? 0 : (rY == 1 ? 1 : -1);
            int8_t jZ = (rZ == 0) ? 0 : (rZ == 1 ? 1 : -1);
            
            bool clickX = (rBtnX == 1 && prevBtnX == 0);
            bool clickTri = (rBtnTri == 1 && prevBtnTri == 0);
            prevBtnX = rBtnX; prevBtnTri = rBtnTri;

            if (btConectado) {
                if (estadoActual == MENU_PRINCIPAL) {
                    movX = 0; movY = 0; movZ = 0; 
                    
                    if (jY != 0 && prevY == 0) {
                        opcionMenu -= jY; 
                        if (opcionMenu < 0) opcionMenu = 1;
                        if (opcionMenu > 1) opcionMenu = 0;
                    }
                    prevY = jY;

                    if (clickX) {
                        if (opcionMenu == 0) estadoActual = MODO_MANUAL;
                        if (opcionMenu == 1) { estadoActual = MODO_HOMING; faseHoming = 0; }
                    }
                } 
                else if (estadoActual == MODO_MANUAL) {
                    movX = jX; movY = jY; movZ = jZ;
                    if (clickTri) estadoActual = MENU_PRINCIPAL;
                } 
                else if (estadoActual == MODO_HOMING) {
                    if (clickTri) { 
                        estadoActual = MENU_PRINCIPAL; 
                        movX = 0; movY = 0; movZ = 0; 
                    } else {
                        // LÓGICA DE HOMING BLINDADA
                        if (faseHoming == 0 && clickX) {
                            faseHoming = 1; 
                        }
                        
                        if (faseHoming == 1) { // HOMING X
                            movY = 0; movZ = 0; // Seguridad: Congela los otros ejes
                            if (sensorX == true) { 
                                movX = 0; 
                                faseHoming = 2; // Pasa al siguiente
                            } else {
                                movX = -1; // Ajusta a 1 si el motor gira al revés
                            }
                        } 
                        else if (faseHoming == 2) { // HOMING Y
                            movX = 0; movZ = 0; 
                            if (sensorY == true) { 
                                movY = 0; 
                                faseHoming = 3;
                            } else {
                                movY = -1; 
                            }
                        } 
                        else if (faseHoming == 3) { // HOMING Z
                            movX = 0; movY = 0; 
                            if (sensorZ == true) { 
                                movZ = 0; 
                                faseHoming = 4; // Terminado
                            } else {
                                movZ = -1; 
                            }
                        }
                        else if (faseHoming == 4) { // RUTINA COMPLETADA
                            movX = 0; movY = 0; movZ = 0;
                        }
                    }
                }
                
                digital_outputs.set(pD_X, movX > 0 ? HIGH : LOW);
                digital_outputs.set(pD_Y, movY > 0 ? HIGH : LOW);
                digital_outputs.set(pD_Z, movZ > 0 ? HIGH : LOW);

            } else {
                estadoActual = MENU_PRINCIPAL;
                movX = 0; movY = 0; movZ = 0;
            }
        } else {
            esp32Conectado = false;
        }
        
        actualizarPantalla();
    }
}
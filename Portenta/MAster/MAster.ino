//------------------------------------------------------------------------------------------------DELCARACION-DE-LIBRERIAS**********----------------------------------------------------------------------------------------------------
#include <Arduino_MachineControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "mbed.h"

using namespace machinecontrol;

// --- CONFIGURACIÓN PANTALLA OLED ---
Adafruit_SH1106G pantalla(128, 64, &Wire, -1);

// --- CONFIGURACIÓN MOTORES STEPPER ---
const int pP_Z = 0; const int pP_X = 2; const int pP_Y = 4;
const int pD_Z = 1; const int pD_X = 3; const int pD_Y = 5;

const float velocidadMotores = 0.0003; // MS
mbed::Ticker motorTicker;
volatile bool estadoPulso = false;
volatile int8_t movX = 0, movY = 0, movZ = 0; 

// --- MÁQUINA DE ESTADOS Y NAVEGACIÓN ---
enum EstadoSistema { MENU_PRINCIPAL, MODO_MANUAL, MODO_HOMING };
EstadoSistema estadoActual = MENU_PRINCIPAL;

int opcionMenu = 0; 
int faseHoming = 0; 

// --- VARIABLES SENSORES Y ACTUADORES ---
bool sensorX = false, sensorY = false, sensorZ = false;
uint8_t posServoRot = 90; // SERVO 01
uint8_t posServoPin = 90; // SERVO 02 

// --- VARIABLES COMUNICACIÓN Y CONTROL ---
#define DIRECCION_ESP32 0x40    // DIRECCION DE ESTA    
unsigned long tAnteriorI2C = 0;
bool esp32Conectado = false;
bool btConectado = false;

int8_t prevInputY = 0; // Para detectar el "clic" direccional de navegación
uint8_t prevBtnX = 0;
uint8_t prevBtnTri = 0;

// --- INTERRUPCIÓN HARDWARE (TICKER MOTORES) ---
void generarPulsoMotor() {
    estadoPulso = !estadoPulso; 
    if (movX != 0) digital_outputs.set(pP_X, estadoPulso);
    else digital_outputs.set(pP_X, LOW);
    
    if (movY != 0) digital_outputs.set(pP_Y, estadoPulso);
    else digital_outputs.set(pP_Y, LOW);
    
    if (movZ != 0) digital_outputs.set(pP_Z, estadoPulso);
    else digital_outputs.set(pP_Z, LOW);
}

// --- INTERFAZ GRÁFICA ---
void actualizarPantalla() {
    pantalla.clearDisplay();
    pantalla.setCursor(0,0);
    
    if (estadoActual == MENU_PRINCIPAL) {
        pantalla.println(F("--- MENU PRINCIPAL ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
        
        pantalla.setCursor(10, 25);
        pantalla.print(opcionMenu == 0 ? "-> " : "   "); 
        pantalla.println(F("MODO MANUAL"));
        
        pantalla.setCursor(10, 40);
        pantalla.print(opcionMenu == 1 ? "-> " : "   "); 
        pantalla.println(F("CALIBRACION (HOMING)"));
        
        // Indicador de seguridad
        pantalla.setCursor(0, 56);
        pantalla.print(F("MOTORES BLOQUEADOS"));
        
    } else if (estadoActual == MODO_MANUAL) {
        pantalla.println(F("CONTROL TOTAL"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
        
        // Ejes X, Y, Z
        pantalla.setCursor(0,18);
        pantalla.print(F("X:")); pantalla.print(movX == 0 ? " 0" : (movX > 0 ? "+1" : "-1"));
        pantalla.print(F(" Y:")); pantalla.print(movY == 0 ? " 0" : (movY > 0 ? "+1" : "-1"));
        pantalla.print(F(" Z:")); pantalla.println(movZ == 0 ? " 0" : (movZ > 0 ? "+1" : "-1"));

        // Servos Actuadores
        pantalla.setCursor(0,35);
        pantalla.print(F("S1 (ROT): ")); 
        pantalla.print(posServoRot); 
        pantalla.println(F(" deg"));

        pantalla.setCursor(0,50);
        pantalla.print(F("S2 (PIN): "));
        pantalla.print(posServoPin); 
        pantalla.println(F(" deg"));
        
    } else if (estadoActual == MODO_HOMING) {
        pantalla.println(F("--- CALIBRACION ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
        
        if (faseHoming == 0) {
            pantalla.setCursor(0,25);
            pantalla.println(F("PRESIONA 'X'"));
            pantalla.println(F("PARA INICIAR ZEROS"));
        } else {
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
    
    // Inicialización segura de la pantalla OLED
    delay(100);
    if(pantalla.begin(0x3C, true)) {
        pantalla.clearDisplay();
        pantalla.display(); 
        delay(50);
        pantalla.setTextSize(1);
        pantalla.setTextColor(SH110X_WHITE);
    }
    
    // Iniciar Entradas Digitales a 24V para Homing
    digital_inputs.init(); 

    // Activar generación de pulsos
    motorTicker.attach(&generarPulsoMotor, velocidadMotores);
}

void loop() {
    unsigned long tActual = millis();

    // Ciclo de lectura cada 15 milisegundos
    if (tActual - tAnteriorI2C >= 15) {
        tAnteriorI2C = tActual;
        
        // 1. LEER SENSORES FÍSICOS (Microswitches 24V)
        sensorX = digital_inputs.read(DIN_READ_CH_PIN_00);
        sensorY = digital_inputs.read(DIN_READ_CH_PIN_01);
        sensorZ = digital_inputs.read(DIN_READ_CH_PIN_02);
        
        // 2. SOLICITAR 8 BYTES AL ESP32
        int bytes = Wire.requestFrom((uint8_t)DIRECCION_ESP32, (uint8_t)8);
        
        if (bytes == 8) {
            esp32Conectado = true;
            
            // Extracción limpia del paquete de datos
            int8_t rawX = Wire.read();
            int8_t rawY = Wire.read();
            int8_t rawZ = Wire.read(); 
            uint8_t rBT = Wire.read();
            uint8_t rBtnX = Wire.read();
            uint8_t rBtnTri = Wire.read();
            posServoRot = Wire.read(); // Servo 1 (0-180)    
            posServoPin = Wire.read(); // Servo 2 (0-180)

            btConectado = (rBT == 1);

            // Mapeo de ejes a lógica discreta (-1, 0, 1)
            int8_t jX = (rawX == 0) ? 0 : (rawX == 1 ? 1 : -1);
            int8_t jY = (rawY == 0) ? 0 : (rawY == 1 ? 1 : -1);
            int8_t jZ = (rawZ == 0) ? 0 : (rawZ == 1 ? 1 : -1);
            
            // Detección de botones "Un Solo Clic"
            bool clickX = (rBtnX == 1 && prevBtnX == 0);
            bool clickTri = (rBtnTri == 1 && prevBtnTri == 0);
            prevBtnX = rBtnX; prevBtnTri = rBtnTri;

            if (btConectado) {
                // ==========================================
                // MÁQUINA DE ESTADOS
                // ==========================================
                if (estadoActual == MENU_PRINCIPAL) {
                    
                    // SEGURIDAD: Bloqueo incondicional de los 3 motores
                    movX = 0; movY = 0; movZ = 0; 
                    
                    // NAVEGACIÓN: Detecta un movimiento hacia arriba o abajo (Joystick Y)
                    if (jY != 0 && prevInputY == 0) {
                        opcionMenu -= jY; 
                        if (opcionMenu < 0) opcionMenu = 1;
                        if (opcionMenu > 1) opcionMenu = 0;
                    }
                    prevInputY = jY;

                    if (clickX) {
                        if (opcionMenu == 0) estadoActual = MODO_MANUAL;
                        if (opcionMenu == 1) { estadoActual = MODO_HOMING; faseHoming = 0; }
                    }
                } 
                else if (estadoActual == MODO_MANUAL) {
                    
                    // Se libera el control a los joysticks
                    movX = jX; 
                    movY = jY; 
                    movZ = jZ;
                    
                    if (clickTri) estadoActual = MENU_PRINCIPAL;
                } 
                else if (estadoActual == MODO_HOMING) {
                    if (clickTri) { 
                        estadoActual = MENU_PRINCIPAL; 
                        movX = 0; movY = 0; movZ = 0; 
                    } else {
                        // SECUENCIA DE HOMING (Un motor a la vez por seguridad)
                        if (faseHoming == 0 && clickX) faseHoming = 1; 
                        
                        if (faseHoming == 1) { 
                            movY = 0; movZ = 0; 
                            if (sensorX) { movX = 0; faseHoming = 2; } else { movX = -1; }
                        } 
                        else if (faseHoming == 2) { 
                            movX = 0; movZ = 0; 
                            if (sensorY) { movY = 0; faseHoming = 3; } else { movY = -1; }
                        } 
                        else if (faseHoming == 3) { 
                            movX = 0; movY = 0; 
                            if (sensorZ) { movZ = 0; faseHoming = 4; } else { movZ = -1; }
                        }
                        else if (faseHoming == 4) { 
                            movX = 0; movY = 0; movZ = 0; 
                        }
                    }
                }
                
                // Aplicar dirección a los drivers físicos
                digital_outputs.set(pD_X, movX > 0 ? HIGH : LOW);
                digital_outputs.set(pD_Y, movY > 0 ? HIGH : LOW);
                digital_outputs.set(pD_Z, movZ > 0 ? HIGH : LOW);

            } else {
                // Si el control se apaga, entra en modo seguro
                estadoActual = MENU_PRINCIPAL;
                movX = 0; movY = 0; movZ = 0;
            }
        } else {
            esp32Conectado = false;
        }
        
        actualizarPantalla();
    }
}
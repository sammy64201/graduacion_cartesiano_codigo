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

void generarPulsoMotor() {
    estadoPulso = !estadoPulso;
    
    if (movX != 0) digital_outputs.set(pP_X, estadoPulso);
    else digital_outputs.set(pP_X, LOW);
    
    if (movY != 0) digital_outputs.set(pP_Y, estadoPulso);
    else digital_outputs.set(pP_Y, LOW);
    
    if (movZ != 0) digital_outputs.set(pP_Z, estadoPulso);
    else digital_outputs.set(pP_Z, LOW);
}

// --- VARIABLES I2C Y ESTADOS ---
#define DIRECCION_ESP32 0x40
unsigned long tAnteriorI2C = 0;
bool esp32Conectado = false;
bool btConectado = false;

// --- INTERFAZ GRÁFICA ---
void actualizarPantalla() {
    pantalla.clearDisplay();
    
    pantalla.setCursor(0,0);             
    pantalla.println(F("SISTEMA CARTESIANO"));
    pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
    
    // Mostramos estado de conexión
    pantalla.setCursor(0,14);
    if (!esp32Conectado) {
        pantalla.println(F("SYS: ESP32 OFFLINE"));
    } else if (!btConectado) {
        pantalla.println(F("BT: ESPERANDO MANDO"));
    } else {
        pantalla.println(F("BT: CONECTADO [OK]"));
    }
    
    // Mostramos estado de los ejes (Solo si todo está conectado)
    pantalla.setCursor(0,26);
    pantalla.print(F("X: ")); pantalla.println(movX == 0 ? "STOP" : (movX > 0 ? "ADELANTE" : "ATRAS"));
    pantalla.setCursor(0,38);
    pantalla.print(F("Y: ")); pantalla.println(movY == 0 ? "STOP" : (movY > 0 ? "ADELANTE" : "ATRAS"));
    pantalla.setCursor(0,50);
    pantalla.print(F("Z: ")); pantalla.println(movZ == 0 ? "STOP" : (movZ > 0 ? "ARRIBA" : "ABAJO"));
    
    pantalla.display(); 
}

void setup() {
    Wire.begin();
    
    Wire.setClock(400000); // Sube la velocidad del bus I2C a 400 kHz
    if(pantalla.begin(0x3C)) {
        pantalla.setTextSize(1);
        pantalla.setTextColor(SH110X_WHITE);
    }

    digital_outputs.set(pD_Z, HIGH);
    digital_outputs.set(pD_X, HIGH);
    digital_outputs.set(pD_Y, HIGH);

    motorTicker.attach(&generarPulsoMotor, 0.0006);
}

void loop() {
    unsigned long tActual = millis();

    // Polleo rápido (50ms)
    if (tActual - tAnteriorI2C >= 15) {
        tAnteriorI2C = tActual;
        
        // Ahora pedimos 4 bytes
        int bytes = Wire.requestFrom((uint8_t)DIRECCION_ESP32, (uint8_t)4);
        
        if (bytes == 4) {
            esp32Conectado = true;
            
            uint8_t rX = Wire.read();
            uint8_t rY = Wire.read();
            uint8_t rZ = Wire.read();
            uint8_t rBT = Wire.read(); // Leemos el cuarto byte (Estado del mando)

            btConectado = (rBT == 1);

            // Si el mando está conectado, aplicamos los movimientos
            if (btConectado) {
                movX = (rX == 0) ? 0 : (rX == 1 ? 1 : -1);
                digital_outputs.set(pD_X, movX > 0 ? HIGH : LOW);

                movY = (rY == 0) ? 0 : (rY == 1 ? 1 : -1);
                digital_outputs.set(pD_Y, movY > 0 ? HIGH : LOW);

                movZ = (rZ == 0) ? 0 : (rZ == 1 ? 1 : -1);
                digital_outputs.set(pD_Z, movZ > 0 ? HIGH : LOW);
            } else {
                // Si el mando se apaga, forzamos STOP físico y lógico
                movX = 0; movY = 0; movZ = 0;
            }
        } else {
            // Si el ESP32 se desconecta (cable I2C)
            esp32Conectado = false;
            btConectado = false;
            movX = 0; movY = 0; movZ = 0;
        }
        
        actualizarPantalla();
    }
}
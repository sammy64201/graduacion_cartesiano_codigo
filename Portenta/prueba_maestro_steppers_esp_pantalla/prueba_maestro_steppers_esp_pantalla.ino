#include <Arduino_MachineControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "mbed.h" // Necesario para usar Ticker

using namespace machinecontrol;

// --- CONFIGURACIÓN OLED ---
#define DIRECCION_OLED 0x3C
Adafruit_SH1106G pantalla(128, 64, &Wire, -1);

// --- CONFIGURACIÓN MOTORES ---
const int pinPasoz = 0; const int pinPasox = 2; const int pinPasoy = 4;
const int pinDirz = 1;  const int pinDirx = 3;  const int pinDiry = 5;

// Ticker de Mbed para los motores
mbed::Ticker motorTicker;

// Variables de estado del motor (deben ser volátiles por usarse en interrupciones)
volatile bool estadoPulso = false;

// --- FUNCIÓN DE INTERRUPCIÓN (ISR) ---
// Esta función se ejecuta EXACTAMENTE cada 600us, pase lo que pase en el loop
void generarPulsoMotor() {
    estadoPulso = !estadoPulso;
    
    if (estadoPulso) {
        digital_outputs.set(pinPasoz, HIGH);
        digital_outputs.set(pinPasox, HIGH);
        digital_outputs.set(pinPasoy, HIGH);
    } else {
        digital_outputs.set(pinPasoz, LOW);
        digital_outputs.set(pinPasox, LOW);
        digital_outputs.set(pinPasoy, LOW);
    }
}

// --- COMUNICACIÓN I2C ---
#define DIRECCION_ESP32 0x40
unsigned long tiempoAnteriorI2C = 0;
int contadorOK = 0;

void actualizarPantalla(String status) {
    pantalla.clearDisplay();
    
    // Título ajustado para no exceder los 21 caracteres
    pantalla.setCursor(0,0);             
    pantalla.println(F("SISTEMA CARTESIANO")); 
    
    // Línea divisoria limpia
    pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
    
    pantalla.setCursor(0,18);
    pantalla.print(F("Status: ")); 
    pantalla.println(status);
    
    pantalla.setCursor(0,34);
    pantalla.print(F("Pings: ")); 
    pantalla.println(contadorOK);
    
    pantalla.display(); 
}

void setup() {
    Wire.begin();
    if(pantalla.begin(DIRECCION_OLED)) {
        pantalla.setTextSize(1);
        pantalla.setTextColor(SH110X_WHITE);
    }

    // Configurar direcciones
    digital_outputs.set(pinDirz, HIGH);
    digital_outputs.set(pinDirx, HIGH);
    digital_outputs.set(pinDiry, HIGH);

    // --- ACTIVAR EL TIMER ---
    // Adjuntamos la función 'generarPulsoMotor' para que corra cada 600 microsegundos
    // 0.0006 segundos = 600 us
    motorTicker.attach(&generarPulsoMotor, 0.0006);
}

void loop() {
    // El loop ahora SOLO se encarga de la comunicación lenta
    unsigned long tiempoActual = millis();

    if (tiempoActual - tiempoAnteriorI2C >= 10000) {
        tiempoAnteriorI2C = tiempoActual;
        
        int bytes = Wire.requestFrom((uint8_t)DIRECCION_ESP32, (uint8_t)2);
        String res = "";
        if (bytes == 2) {
            while (Wire.available()) res += (char)Wire.read();
        }

        if (res == "OK") {
            contadorOK++;
            actualizarPantalla("CONECTADO");
        } else {
            actualizarPantalla("ERROR COM.");
        }
    }
}
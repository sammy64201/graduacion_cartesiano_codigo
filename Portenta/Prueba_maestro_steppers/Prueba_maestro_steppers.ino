#include <Arduino_MachineControl.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h> // <--- LIBRERÍA NUEVA

using namespace machinecontrol;

// --- Configuración de la Pantalla OLED SH1106 ---
#define ANCHO_PANTALLA 128 // Ancho de la pantalla OLED, en píxeles
#define ALTO_PANTALLA 64   // Alto de la pantalla OLED, en píxeles
#define RESET_OLED -1      // Pin de reset (o -1 si comparte el pin de reset del Arduino)
#define DIRECCION_I2C 0x3C // Dirección 0x78 desplazada a 7-bits (0x3C)

// Inicializamos el objeto de la pantalla usando SH1106G para 128x64
Adafruit_SH1106G pantalla(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, RESET_OLED);

// --- Configuración de los Motores Stepper (SIN CAMBIOS) ---
const int pinPasoz = 0;       // PUL+ Z
const int pinDireccionz = 1;  // DIR+ Z
const int pinPasox = 2;       // PUL+ X
const int pinDireccionx = 3;  // DIR+ X
const int pinPasoy = 4;       // PUL+ Y
const int pinDirecciony = 5;  // DIR+ Y

// Parámetros de velocidad
const long intervaloVelocidad = 400; 

// Variables para el control no bloqueante del tiempo (SIN CAMBIOS)
unsigned long tiempoAnteriorPaso = 0;
bool estadoPinPaso = false; // Rastrea si el pulso está en ALTO o BAJO

void setup() {
  // 1. Inicializar I2C y Pantalla OLED (MODIFICADO)
  Wire.begin();
  
  // Para SH1106G, el primer parámetro no existe, se inicializa diferente
  if(!pantalla.begin(DIRECCION_I2C)) {
    // Si la pantalla no se inicializa, el código continuará, pero los motores seguirán funcionando.
  } else {
    // Limpiamos el buffer de la pantalla (igual que antes)
    pantalla.clearDisplay();
    
    // Configuramos texto (igual que antes)
    pantalla.setTextSize(1);             // Escala de texto 1:1
    pantalla.setTextColor(SH110X_WHITE); // Texto blanco (encendido). Usamos la constante de SH110X
    
    // Posicionamos el cursor e imprimimos el estado
    pantalla.setCursor(0,0);             
    pantalla.println(F("Estado: SH1106 I2C")); // Mensaje actualizado
    
    pantalla.setCursor(0,16);
    pantalla.print(F("Direccion (7b): 0x"));
    pantalla.println(DIRECCION_I2C, HEX); // Imprime 3C

    pantalla.setCursor(0,32);
    pantalla.print(F("Direccion (8b): 0x78"));
    
    // Enviamos el buffer a la pantalla para que se muestre
    pantalla.display();
  }

  // 2. Configurar Direcciones de Motores (SIN CAMBIOS)
  digital_outputs.set(pinDireccionz, HIGH);
  digital_outputs.set(pinDireccionx, HIGH);
  digital_outputs.set(pinDirecciony, HIGH);
}

void loop() {
  // --- Arquitectura de Máquina de Estados (SIN CAMBIOS) ---
  // Capturamos el tiempo actual en microsegundos
  unsigned long tiempoActual = micros();

  // Verificamos si ha pasado el tiempo necesario (600 microsegundos)
  if (tiempoActual - tiempoAnteriorPaso >= intervaloVelocidad) {
    // Guardamos el tiempo actual para la próxima comparación
    tiempoAnteriorPaso = tiempoActual;
    
    // Invertimos el estado del pulso lógicamente
    estadoPinPaso = !estadoPinPaso;

    // Aplicamos el estado físico a los drivers DM542
    if (estadoPinPaso == true) {
      digital_outputs.set(pinPasoz, HIGH);
      digital_outputs.set(pinPasox, HIGH);
      digital_outputs.set(pinPasoy, HIGH);
    } else {
      digital_outputs.set(pinPasoz, LOW);
      digital_outputs.set(pinPasox, LOW);
      digital_outputs.set(pinPasoy, LOW);
    }
  }
}
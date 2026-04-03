#include <Arduino_MachineControl.h>

// Usamos el espacio de nombres de la librería para no escribir prefijos
using namespace machinecontrol;

// Definimos las salidas que conectamos al driver DM542
const int pinPasoz = 0;       // Conectado a PUL+ (Salida 00)
const int pinDireccionz = 1;  // Conectado a DIR+ (Salida 01)
const int pinPasox = 2;
const int pinDireccionx = 3;
const int pinPasoy = 4;
const int pinDirecciony = 5;

// Ajuste de velocidad: MENOR número = MAYOR velocidad
int velocidadMicrosegundos = 600; 

void setup() {
  // En esta versión de la librería, no es necesario el begin()
  
  // Establecemos la dirección de giro estática (una sola dirección).
  // Si en el futuro quieres que gire al revés, solo cambia HIGH por LOW.
  digital_outputs.set(pinDireccionz, HIGH);
  digital_outputs.set(pinDireccionx, HIGH);
  digital_outputs.set(pinDirecciony, HIGH);
}

void loop() {
  // --- Secuencia de un "Micro-paso" ---
  
  // 1. Encendemos la señal de pulso (envía 24V al PUL+)
  digital_outputs.set(pinPasoz, HIGH);
  digital_outputs.set(pinPasox, HIGH);
  digital_outputs.set(pinPasoy, HIGH);
  
  // 2. Esperamos un instante
  delayMicroseconds(velocidadMicrosegundos);
  
  // 3. Apagamos la señal de pulso (cae a 0V)
  digital_outputs.set(pinPasoz, LOW);
  digital_outputs.set(pinPasox, LOW);
  digital_outputs.set(pinPasoy, LOW);
  
  // 4. Esperamos el mismo instante antes del siguiente paso
  delayMicroseconds(velocidadMicrosegundos);
}
// Diagnostico estatico del MAX485 para Nano clasico ATmega328P de 5 V.
// MAX485 separado de ESP, convertidor y Portenta. DE y /RE NO puenteados.
// No usa UART hacia el MAX485 ni controla actuadores.
#include <Arduino.h>

#if !defined(__AVR_ATmega328P__)
#error "Seleccionar Arduino Nano clasico ATmega328P (logica 5 V)."
#endif

constexpr uint8_t PIN_RO = 2;
constexpr uint8_t PIN_DI = 3;
constexpr uint8_t PIN_DE = 4;
constexpr uint8_t PIN_RE = 5;
constexpr uint32_t INTERVALO_MS = 5000;

enum class Modo : uint8_t { DETENIDO, SALIDA, RETORNO };
Modo modo = Modo::DETENIDO;
bool nivelAlto = false;
uint32_t ultimoCambio = 0;
uint32_t lecturas = 0;
uint32_t fallos = 0;

void mostrarMenu() {
  Serial.println(F("\n[NANO] MAX485 aislado. D2=RO D3=DI D4=DE D5=/RE."));
  Serial.println(F("[NANO] Quitar puente DE-/RE; A/B sin Portenta."));
  Serial.println(F("[NANO] 1: salida A/B, DI alterna cada 5 s; medir A-B."));
  Serial.println(F("[NANO] 2: retorno estatico, compara RO con DI cada 5 s."));
  Serial.println(F("[NANO] 0: detener y deshabilitar modulo. ?: menu."));
}

void aplicarNivel(bool alto) {
  nivelAlto = alto;
  digitalWrite(PIN_DI, alto ? HIGH : LOW);
  delay(2); // Asentamiento antes de leer; esto no es una prueba UART.
  Serial.print(F("[NANO] DI ORDENADO="));
  Serial.print(alto ? F("HIGH (~5 V)") : F("LOW (~0 V)"));
  Serial.print(alto ? F("; A-B esperado POSITIVO") : F("; A-B esperado NEGATIVO"));
  if (modo == Modo::RETORNO) {
    const bool recibidoAlto = digitalRead(PIN_RO) == HIGH;
    ++lecturas;
    if (recibidoAlto != alto) ++fallos;
    Serial.print(F("; RO LEIDO="));
    Serial.print(recibidoAlto ? F("HIGH") : F("LOW"));
    Serial.print(recibidoAlto == alto ? F("; OK_ESTATICO") : F("; NO_COINCIDE"));
    Serial.print(F("; lecturas="));
    Serial.print(lecturas);
    Serial.print(F("; fallos="));
    Serial.print(fallos);
  }
  Serial.println();
}

void seleccionarModo(Modo nuevo) {
  // Primero soltar el bus y deshabilitar el receptor.
  digitalWrite(PIN_DE, LOW);
  digitalWrite(PIN_RE, HIGH);
  digitalWrite(PIN_DI, LOW);
  modo = nuevo;
  nivelAlto = false;
  lecturas = 0;
  fallos = 0;
  ultimoCambio = millis();
  if (modo == Modo::DETENIDO) {
    Serial.println(F("[NANO] DETENIDO: DE=LOW /RE=HIGH DI=LOW; RO alta impedancia."));
    return;
  }
  digitalWrite(PIN_RE, modo == Modo::RETORNO ? LOW : HIGH);
  digitalWrite(PIN_DE, HIGH);
  Serial.println(modo == Modo::RETORNO ? F("[NANO] RETORNO: DE=HIGH /RE=LOW.") :
                                       F("[NANO] SALIDA: DE=HIGH /RE=HIGH; no evaluar RO."));
  aplicarNivel(false);
}

void setup() {
  digitalWrite(PIN_DE, LOW);
  digitalWrite(PIN_RE, HIGH);
  digitalWrite(PIN_DI, LOW);
  pinMode(PIN_DE, OUTPUT);
  pinMode(PIN_RE, OUTPUT);
  pinMode(PIN_DI, OUTPUT);
  pinMode(PIN_RO, INPUT);
  Serial.begin(115200);
  delay(300);
  mostrarMenu();
  seleccionarModo(Modo::DETENIDO);
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case '1': seleccionarModo(Modo::SALIDA); break;
      case '2': seleccionarModo(Modo::RETORNO); break;
      case '0': seleccionarModo(Modo::DETENIDO); break;
      case '?': mostrarMenu(); break;
      default: break;
    }
  }
  const uint32_t ahora = millis();
  if (modo != Modo::DETENIDO && ahora - ultimoCambio >= INTERVALO_MS) {
    ultimoCambio = ahora;
    aplicarNivel(!nivelAlto);
  }
  delay(1);
}

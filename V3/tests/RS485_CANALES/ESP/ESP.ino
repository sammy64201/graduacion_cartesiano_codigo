// Diagnostico estatico de GPIO y convertidor de nivel. NO usa UART RS485.
// Desconectar MAX485 de B4/DI, B5/RO y B2/DE/RE antes de usar este sketch.
// No inicia motores, servos, camara, OLED ni Bluetooth.
#include <Arduino.h>

constexpr int PIN_TX = 14;
constexpr int PIN_RX = 27;
constexpr int PIN_DIRECCION = 18;
constexpr uint32_t INTERVALO_NIVEL_MS = 5000;
constexpr uint32_t INTERVALO_LECTURA_MS = 1000;

enum class Modo : uint8_t { DETENIDO, TX, DIRECCION, RX };
Modo modo = Modo::DETENIDO;
bool nivelAlto = false;
uint32_t ultimoCambio = 0;
uint32_t ultimaLectura = 0;

void mostrarMenu() {
  Serial.println("\n[CANALES] MAX485 separado de DI, RO y DE/RE; Portenta desconectada.");
  Serial.println("[CANALES] 1: D14 -> A4/B4, LOW/HIGH cada 5 s.");
  Serial.println("[CANALES] 2: D18 -> A2/B2, LOW/HIGH cada 5 s.");
  Serial.println("[CANALES] 3: B5 -> A5/D27, entrada manual desde GND o 5 V SOLO en B5.");
  Serial.println("[CANALES] 0: detener, D14 y D18 en LOW. ?: mostrar menu.");
  Serial.println("[CANALES] No aplicar 5 V a A5 ni a D27. GND comun, VA=3.3 V, VB=5 V.");
}

void informarSalida() {
  const bool esTX = modo == Modo::TX;
  Serial.print(esTX ? "[CANALES] D14 ORDENADO=" : "[CANALES] D18 ORDENADO=");
  Serial.print(nivelAlto ? "HIGH" : "LOW");
  Serial.print(esTX ? "; medir D14, A4 y B4" : "; medir D18, A2 y B2");
  Serial.println(nivelAlto ? "; esperado lado A~3.3 V y B~5 V" :
                           "; esperado lado A~0 V y B~0 V");
  // ORDENADO indica el nivel pedido por software, no un voltaje medido.
}

void seleccionarModo(Modo nuevo) {
  digitalWrite(PIN_TX, LOW);
  digitalWrite(PIN_DIRECCION, LOW);
  nivelAlto = false;
  modo = nuevo;
  ultimoCambio = millis();
  ultimaLectura = millis() - INTERVALO_LECTURA_MS;
  if (modo == Modo::TX || modo == Modo::DIRECCION) {
    informarSalida();
  } else if (modo == Modo::RX) {
    Serial.println("[CANALES] D27 solo INPUT, sin pull-up. RO debe estar desconectado de B5.");
    Serial.println("[CANALES] Conectar B5 a GND o a 5 V; medir tambien A5 y D27.");
  } else {
    Serial.println("[CANALES] DETENIDO: D14=LOW, D18=LOW, D27=INPUT.");
  }
}

void setup() {
  // Preestablecer LOW antes de habilitar las salidas.
  digitalWrite(PIN_TX, LOW);
  digitalWrite(PIN_DIRECCION, LOW);
  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_DIRECCION, OUTPUT);
  pinMode(PIN_RX, INPUT);
  Serial.begin(115200);
  delay(300);
  mostrarMenu();
  seleccionarModo(Modo::DETENIDO);
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case '1': seleccionarModo(Modo::TX); break;
      case '2': seleccionarModo(Modo::DIRECCION); break;
      case '3': seleccionarModo(Modo::RX); break;
      case '0': seleccionarModo(Modo::DETENIDO); break;
      case '?': mostrarMenu(); break;
      default: break; // Ignorar espacios y fin de linea del monitor.
    }
  }

  const uint32_t ahora = millis();
  if ((modo == Modo::TX || modo == Modo::DIRECCION) &&
      ahora - ultimoCambio >= INTERVALO_NIVEL_MS) {
    ultimoCambio = ahora;
    nivelAlto = !nivelAlto;
    digitalWrite(modo == Modo::TX ? PIN_TX : PIN_DIRECCION,
                 nivelAlto ? HIGH : LOW);
    informarSalida();
  } else if (modo == Modo::RX && ahora - ultimaLectura >= INTERVALO_LECTURA_MS) {
    ultimaLectura = ahora;
    Serial.print("[CANALES] D27 LEIDO=");
    Serial.println(digitalRead(PIN_RX) == HIGH ? "HIGH" : "LOW");
  }
  delay(1);
}

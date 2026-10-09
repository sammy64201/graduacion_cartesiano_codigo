#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\ESP32\\ESP32.ino"
// Banco sin actuadores: solo UART2 y pin DE del MAX485.
#include <Arduino.h>
#include "FinalRS485.h"
using namespace FinalRS485;
HardwareSerial enlace(2);
EnlaceRS485::Receptor receptor;
BancoESP banco;
uint16_t arranque;
uint32_t txOK = 0, txError = 0, ultimoResumen = 0;
bool respuestaPendiente = false;
uint32_t responderDesde = 0;
EnlaceRS485::Mensaje pendiente;

#line 14 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\ESP32\\ESP32.ino"
void enviar(const EnlaceRS485::Mensaje &m);
#line 26 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\ESP32\\ESP32.ino"
void setup();
#line 36 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\ESP32\\ESP32.ino"
void loop();
#line 14 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\ESP32\\ESP32.ino"
void enviar(const EnlaceRS485::Mensaje &m) {
    uint8_t trama[EnlaceRS485::TRAMA];
    const size_t n = EnlaceRS485::codificar(m, trama);
    delayMicroseconds(EnlaceRS485::GIRO_BUS_US);
    digitalWrite(FinalRS485::DE, HIGH);
    delayMicroseconds(EnlaceRS485::PRE_TX_US);
    const size_t escritos = enlace.write(trama, n);
    enlace.flush();
    delayMicroseconds(EnlaceRS485::POST_TX_US);
    digitalWrite(FinalRS485::DE, LOW);
    if (escritos == n) ++txOK; else ++txError;
}
void setup() {
    Serial.begin(115200);
    digitalWrite(FinalRS485::DE, LOW);
    pinMode(FinalRS485::DE, OUTPUT);
    enlace.begin(EnlaceRS485::BAUD, SERIAL_8N1, FinalRS485::RX, FinalRS485::TX);
    arranque = static_cast<uint16_t>(esp_random());
    if (!arranque) arranque = 1;
    Serial.print("[FINAL ESP] v17; RX14 TX27 DE18; bus");
    Serial.print(EnlaceRS485::BAUD); Serial.println("; SIN ACTUADORES");
}
void loop() {
    EnlaceRS485::Mensaje m;
    while (enlace.available()) {
        if (!receptor.agregar(static_cast<uint8_t>(enlace.read()), millis(), m)) continue;
        if (!banco.aceptar(m)) continue;
        // Respuestas artificiales exclusivas de los casos del banco.
        if (m.solicitud == 20018) continue; // Perdida deliberada.
        pendiente = m;
        const auto p = respuesta(m.solicitud, arranque);
        memcpy(pendiente.datos, &p, sizeof(p));
        if (m.solicitud == 20014) pendiente.sesion ^= 0x80000000UL;
        responderDesde = millis() + (m.solicitud == 20016 ? EnlaceRS485::TIMEOUT_RESPUESTA_MS + 15 : 0);
        respuestaPendiente = true;
    }
    receptor.vencerFragmento(millis());
    if (respuestaPendiente && static_cast<int32_t>(millis() - responderDesde) >= 0) {
        enviar(pendiente);
        respuestaPendiente = false;
    }
    if (millis() - ultimoResumen >= 1000) {
        ultimoResumen = millis();
        Serial.print("[FINAL ESP] nuevas="); Serial.print(banco.aplicadas);
        Serial.print(" duplicadas="); Serial.print(banco.duplicadas);
        Serial.print(" descartadas="); Serial.print(banco.descartadas);
        Serial.print(" sem="); Serial.print(banco.invalidas);
        Serial.print(" crc="); Serial.print(receptor.crcIncorrecto);
        Serial.print(" len="); Serial.print(receptor.longitudIncorrecta);
        Serial.print(" fragmentos="); Serial.print(receptor.fragmentos);
        Serial.print(" txOK="); Serial.print(txOK);
        Serial.print(" txError="); Serial.println(txError);
    }
}


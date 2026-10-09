#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\MAX485_UN_MODULO_NANO_UNO\\NANO\\NANO.ino"
// Nano clasico 5 V: TX D3->DI del unico MAX485; RX D2<-TX D3 del Uno.
// MAX485 DE a 5 V y /RE a GND, separados. A/B sin cables externos. Solo diagnostico.
#include <Arduino.h>
#include <SoftwareSerial.h>

#if !defined(__AVR_ATmega328P__)
#error "Esta prueba requiere Nano clasico ATmega328P de 5 V."
#endif
#include "PruebaMAX485.h"

constexpr int RS485_RX = 2;
constexpr int RS485_TX = 3;

SoftwareSerial enlace(RS485_RX, RS485_TX);
PruebaMAX485::Receptor receptor;
uint32_t recibidosUno = 0;
uint32_t enviadosNANO = 0;
uint32_t confirmadosNANO = 0;
uint32_t timeoutsNANO = 0;
uint32_t inesperados = 0;
uint32_t ultimoResumen = 0;
uint32_t numeroNANO = 0;
uint32_t valorEnviado = 0;
uint32_t turnoEnviado = 0;
uint32_t inicioEspera = 0;
bool esperandoACK = false;

#line 28 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\MAX485_UN_MODULO_NANO_UNO\\NANO\\NANO.ino"
void enviarPaquete(uint8_t tipo, uint32_t numero, uint32_t valor, uint32_t turno);
#line 37 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\MAX485_UN_MODULO_NANO_UNO\\NANO\\NANO.ino"
void setup();
#line 45 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\MAX485_UN_MODULO_NANO_UNO\\NANO\\NANO.ino"
void loop();
#line 28 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\MAX485_UN_MODULO_NANO_UNO\\NANO\\NANO.ino"
void enviarPaquete(uint8_t tipo, uint32_t numero, uint32_t valor, uint32_t turno) {
  uint8_t paquete[PruebaMAX485::TAMANO];
  PruebaMAX485::crear(tipo, numero, valor, turno, paquete);
  // Margen entre turnos para que el receptor procese la trama anterior.
  delayMicroseconds(2000);
  // SoftwareSerial::write espera el bit de parada. DE y /RE son fijos.
  enlace.write(paquete, sizeof(paquete));
}

void setup() {
  Serial.begin(115200);
  enlace.begin(PruebaMAX485::BAUD);
  enlace.listen();
  Serial.println(F("[MAX485 NANO] UN MODULO v4; bus=9600; monitor=115200; 32 bytes"));
  Serial.println(F("[MAX485 NANO] TX D3->DI; RX D2<-Uno D3; DE=5V /RE=GND"));
}

void loop() {
  uint8_t paquete[PruebaMAX485::TAMANO];
  while (enlace.available()) {
    if (!receptor.agregar(static_cast<uint8_t>(enlace.read()), paquete)) continue;
    const uint32_t numero = PruebaMAX485::secuencia(paquete);
    const uint32_t valor = PruebaMAX485::valor(paquete);
    const uint32_t turno = PruebaMAX485::turno(paquete);
    if (paquete[5] == PruebaMAX485::DATOS_UNO && turno == numero) {
      // Un nuevo ciclo permite recuperar el enlace tras perder el ACK anterior.
      if (esperandoACK) {
        esperandoACK = false;
        ++timeoutsNANO;
      }
      ++recibidosUno;
      enviarPaquete(PruebaMAX485::ACK_NANO, numero, valor, turno);
      Serial.print(F("[MAX485 NANO] RX UNO->NANO OK seq=")); Serial.print(numero);
      Serial.print(F(" uno_ms=")); Serial.print(valor);
      Serial.println(F(" ACK enviado"));
    } else if (paquete[5] == PruebaMAX485::TURNO_NANO && turno == numero &&
               valor == 0 && !esperandoACK) {
      // Mensaje propio: secuencia independiente y reloj local del Nano.
      valorEnviado = millis();
      turnoEnviado = turno;
      enviarPaquete(PruebaMAX485::DATOS_NANO, ++numeroNANO, valorEnviado, turnoEnviado);
      ++enviadosNANO;
      inicioEspera = millis();
      esperandoACK = true;
      Serial.print(F("[MAX485 NANO] TX NANO->UNO seq=")); Serial.print(numeroNANO);
      Serial.print(F(" nano_ms=")); Serial.println(valorEnviado);
    } else if (paquete[5] == PruebaMAX485::ACK_UNO && esperandoACK &&
               numero == numeroNANO && valor == valorEnviado && turno == turnoEnviado &&
               millis() - inicioEspera < PruebaMAX485::TIMEOUT_MS) {
      esperandoACK = false;
      ++confirmadosNANO;
      Serial.print(F("[MAX485 NANO] TX NANO->UNO CONFIRMADO seq="));
      Serial.print(numeroNANO);
      Serial.print(F(" ida_vuelta_ms=")); Serial.println(millis() - inicioEspera);
    } else {
      ++inesperados;
    }
  }
  if (esperandoACK && millis() - inicioEspera >= PruebaMAX485::TIMEOUT_MS) {
    esperandoACK = false;
    ++timeoutsNANO;
    Serial.print(F("[MAX485 NANO] TIMEOUT ACK NANO->UNO seq=")); Serial.println(numeroNANO);
  }
  if (millis() - ultimoResumen >= 10000) {
    ultimoResumen = millis();
    Serial.print(F("[MAX485 NANO] rx_uno=")); Serial.print(recibidosUno);
    Serial.print(F(" tx_nano=")); Serial.print(enviadosNANO);
    Serial.print(F(" confirmado_nano=")); Serial.print(confirmadosNANO);
    Serial.print(F(" timeout_nano=")); Serial.print(timeoutsNANO);
    Serial.print(F(" rx_bytes=")); Serial.print(receptor.bytes);
    Serial.print(F(" invalidos=")); Serial.print(receptor.invalidos);
    Serial.print(F(" inesperados=")); Serial.println(inesperados);
  }
  delay(1);
}


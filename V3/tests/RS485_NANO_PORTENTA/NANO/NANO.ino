// Nano clasico + MAX485: solo comunicacion: sin servos, camara, OLED, control ni motores.
#include <Arduino.h>
#include <SoftwareSerial.h>

#if !defined(__AVR_ATmega328P__)
#error "Esta prueba requiere Nano clasico ATmega328P de 5 V."
#endif
#include "PruebaRS485.h"

constexpr int RS485_RX = 2;
constexpr int RS485_TX = 3;
constexpr int RS485_DIRECCION = 4;
// true: DE y /RE unidos a D4. false: modulo de direccion automatica.
constexpr bool DIRECCION_MANUAL = true;

SoftwareSerial enlace(RS485_RX, RS485_TX);
PruebaRS485::Receptor receptor;
uint32_t recibidosPortenta = 0;
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

void enviarPaquete(uint8_t tipo, uint32_t numero, uint32_t valor, uint32_t turno) {
  uint8_t paquete[PruebaRS485::TAMANO];
  PruebaRS485::crear(tipo, numero, valor, turno, paquete);
  // La Portenta mantiene DE 500 us tras TX. Dar margen antes de responder.
  delayMicroseconds(2000);
  if (DIRECCION_MANUAL) {
    digitalWrite(RS485_DIRECCION, HIGH);
    delayMicroseconds(50);
  }
  enlace.write(paquete, sizeof(paquete));
  // SoftwareSerial::write transmite sin buffer y espera el bit de parada.
  if (DIRECCION_MANUAL) {
    delayMicroseconds(50);
    digitalWrite(RS485_DIRECCION, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  if (DIRECCION_MANUAL) {
    digitalWrite(RS485_DIRECCION, LOW);
    pinMode(RS485_DIRECCION, OUTPUT);
  }
  enlace.begin(PruebaRS485::BAUD);
  enlace.listen();
  Serial.println(F("[RS485 NANO] Prueba BIDIRECCIONAL v3; RX=D2 TX=D3; bus=9600; monitor=115200; 32 bytes"));
  Serial.println(DIRECCION_MANUAL ? "[RS485 NANO] DE+/RE=D4" :
                                 "[RS485 NANO] Direccion automatica");
}

void loop() {
  uint8_t paquete[PruebaRS485::TAMANO];
  while (enlace.available()) {
    if (!receptor.agregar(static_cast<uint8_t>(enlace.read()), paquete)) continue;
    const uint32_t numero = PruebaRS485::secuencia(paquete);
    const uint32_t valor = PruebaRS485::valor(paquete);
    const uint32_t turno = PruebaRS485::turno(paquete);
    if (paquete[5] == PruebaRS485::DATOS_PORTENTA && turno == numero) {
      // Un nuevo ciclo permite recuperar el enlace tras perder el ACK anterior.
      if (esperandoACK) {
        esperandoACK = false;
        ++timeoutsNANO;
      }
      ++recibidosPortenta;
      enviarPaquete(PruebaRS485::ACK_NANO, numero, valor, turno);
      Serial.print(F("[RS485 NANO] RX PORTENTA->NANO OK seq=")); Serial.print(numero);
      Serial.print(F(" portenta_ms=")); Serial.print(valor);
      Serial.println(F(" ACK enviado"));
    } else if (paquete[5] == PruebaRS485::TURNO_NANO && turno == numero &&
               valor == 0 && !esperandoACK) {
      // Mensaje propio: secuencia independiente y reloj local de el Nano.
      valorEnviado = millis();
      turnoEnviado = turno;
      enviarPaquete(PruebaRS485::DATOS_NANO, ++numeroNANO, valorEnviado, turnoEnviado);
      ++enviadosNANO;
      inicioEspera = millis();
      esperandoACK = true;
      Serial.print(F("[RS485 NANO] TX NANO->PORTENTA seq=")); Serial.print(numeroNANO);
      Serial.print(F(" nano_ms=")); Serial.println(valorEnviado);
    } else if (paquete[5] == PruebaRS485::ACK_PORTENTA && esperandoACK &&
               numero == numeroNANO && valor == valorEnviado && turno == turnoEnviado &&
               millis() - inicioEspera < PruebaRS485::TIMEOUT_MS) {
      esperandoACK = false;
      ++confirmadosNANO;
      Serial.print(F("[RS485 NANO] TX NANO->PORTENTA CONFIRMADO seq="));
      Serial.print(numeroNANO);
      Serial.print(F(" ida_vuelta_ms=")); Serial.println(millis() - inicioEspera);
    } else {
      ++inesperados;
    }
  }
  if (esperandoACK && millis() - inicioEspera >= PruebaRS485::TIMEOUT_MS) {
    esperandoACK = false;
    ++timeoutsNANO;
    Serial.print(F("[RS485 NANO] TIMEOUT ACK NANO->PORTENTA seq=")); Serial.println(numeroNANO);
  }
  if (millis() - ultimoResumen >= 10000) {
    ultimoResumen = millis();
    Serial.print(F("[RS485 NANO] rx_portenta=")); Serial.print(recibidosPortenta);
    Serial.print(F(" tx_nano=")); Serial.print(enviadosNANO);
    Serial.print(F(" confirmado_nano=")); Serial.print(confirmadosNANO);
    Serial.print(F(" timeout_nano=")); Serial.print(timeoutsNANO);
    Serial.print(F(" rx_bytes=")); Serial.print(receptor.bytes);
    Serial.print(F(" invalidos=")); Serial.print(receptor.invalidos);
    Serial.print(F(" inesperados=")); Serial.println(inesperados);
  }
  delay(1);
}

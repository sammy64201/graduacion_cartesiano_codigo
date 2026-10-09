// Nano clasico + UART TTL: solo comunicacion: sin servos, camara, OLED, control ni motores.
#include <Arduino.h>
#include <SoftwareSerial.h>

#if !defined(__AVR_ATmega328P__)
#error "Esta prueba requiere Nano clasico ATmega328P de 5 V."
#endif
#include "PruebaUART.h"

constexpr int UART_RX = 2;
constexpr int UART_TX = 3;

SoftwareSerial enlace(UART_RX, UART_TX);
PruebaUART::Receptor receptor;
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
  uint8_t paquete[PruebaUART::TAMANO];
  PruebaUART::crear(tipo, numero, valor, turno, paquete);
  // Margen antes de responder para SoftwareSerial del Nano.
  delayMicroseconds(2000);
  enlace.write(paquete, sizeof(paquete));
  // SoftwareSerial::write transmite sin buffer y espera el bit de parada.
}

void setup() {
  Serial.begin(115200);
  enlace.begin(PruebaUART::BAUD);
  enlace.listen();
  Serial.println(F("[UART NANO] Prueba BIDIRECCIONAL v1; RX=D2 TX=D3; bus=9600; monitor=115200; 32 bytes"));
  Serial.println(F("[UART NANO] TTL con adaptacion de nivel; D4/D5 sin uso."));
}

void loop() {
  uint8_t paquete[PruebaUART::TAMANO];
  while (enlace.available()) {
    if (!receptor.agregar(static_cast<uint8_t>(enlace.read()), paquete)) continue;
    const uint32_t numero = PruebaUART::secuencia(paquete);
    const uint32_t valor = PruebaUART::valor(paquete);
    const uint32_t turno = PruebaUART::turno(paquete);
    if (paquete[5] == PruebaUART::DATOS_PORTENTA && turno == numero) {
      // Un nuevo ciclo permite recuperar el enlace tras perder el ACK anterior.
      if (esperandoACK) {
        esperandoACK = false;
        ++timeoutsNANO;
      }
      ++recibidosPortenta;
      enviarPaquete(PruebaUART::ACK_NANO, numero, valor, turno);
      Serial.print(F("[UART NANO] RX PORTENTA->NANO OK seq=")); Serial.print(numero);
      Serial.print(F(" portenta_ms=")); Serial.print(valor);
      Serial.println(F(" ACK enviado"));
    } else if (paquete[5] == PruebaUART::TURNO_NANO && turno == numero &&
               valor == 0 && !esperandoACK) {
      // Mensaje propio: secuencia independiente y reloj local de el Nano.
      valorEnviado = millis();
      turnoEnviado = turno;
      enviarPaquete(PruebaUART::DATOS_NANO, ++numeroNANO, valorEnviado, turnoEnviado);
      ++enviadosNANO;
      inicioEspera = millis();
      esperandoACK = true;
      Serial.print(F("[UART NANO] TX NANO->PORTENTA seq=")); Serial.print(numeroNANO);
      Serial.print(F(" nano_ms=")); Serial.println(valorEnviado);
    } else if (paquete[5] == PruebaUART::ACK_PORTENTA && esperandoACK &&
               numero == numeroNANO && valor == valorEnviado && turno == turnoEnviado &&
               millis() - inicioEspera < PruebaUART::TIMEOUT_MS) {
      esperandoACK = false;
      ++confirmadosNANO;
      Serial.print(F("[UART NANO] TX NANO->PORTENTA CONFIRMADO seq="));
      Serial.print(numeroNANO);
      Serial.print(F(" ida_vuelta_ms=")); Serial.println(millis() - inicioEspera);
    } else {
      ++inesperados;
    }
  }
  if (esperandoACK && millis() - inicioEspera >= PruebaUART::TIMEOUT_MS) {
    esperandoACK = false;
    ++timeoutsNANO;
    Serial.print(F("[UART NANO] TIMEOUT ACK NANO->PORTENTA seq=")); Serial.println(numeroNANO);
  }
  if (millis() - ultimoResumen >= 10000) {
    ultimoResumen = millis();
    Serial.print(F("[UART NANO] rx_portenta=")); Serial.print(recibidosPortenta);
    Serial.print(F(" tx_nano=")); Serial.print(enviadosNANO);
    Serial.print(F(" confirmado_nano=")); Serial.print(confirmadosNANO);
    Serial.print(F(" timeout_nano=")); Serial.print(timeoutsNANO);
    Serial.print(F(" rx_bytes=")); Serial.print(receptor.bytes);
    Serial.print(F(" invalidos=")); Serial.print(receptor.invalidos);
    Serial.print(F(" inesperados=")); Serial.println(inesperados);
  }
  delay(1);
}

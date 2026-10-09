// Uno R3 5 V: RX D2<-RO del unico MAX485; TX D3->Nano D2 directo.
// Solo diagnostico local: no es un enlace RS485 entre dos extremos.
#include <Arduino.h>
#include <SoftwareSerial.h>
#if !defined(__AVR_ATmega328P__)
#error "Esta prueba requiere Uno R3 ATmega328P de 5 V."
#endif
#include "PruebaMAX485.h"
constexpr int RS485_RX = 2;
constexpr int RS485_TX = 3;
SoftwareSerial enlace(RS485_RX, RS485_TX);

PruebaMAX485::Receptor receptor;
enum FasePrueba : uint8_t { REPOSO, ESPERA_ACK_NANO, ESPERA_DATOS_NANO };
FasePrueba fase = REPOSO;
uint32_t numeroUno = 0;
uint32_t valorEnviado = 0;
uint32_t enviados = 0;
uint32_t confirmadosUno = 0;
uint32_t recibidosNANO = 0;
uint32_t timeoutsUno = 0;
uint32_t timeoutsTurnoNANO = 0;
uint32_t inesperados = 0;
uint32_t inicioEspera = 0;
uint32_t ultimoEnvio = 0;
uint32_t ultimoResumen = 0;

void enviarPaquete(uint8_t tipo, uint32_t numero, uint32_t valor, uint32_t turno) {
  uint8_t paquete[PruebaMAX485::TAMANO];
  PruebaMAX485::crear(tipo, numero, valor, turno, paquete);
  delayMicroseconds(2000);
  // Retorno directo al Nano, sin MAX485; escritura completa y bloqueante.
  enlace.write(paquete, sizeof(paquete));
}

void setup() {
  Serial.begin(115200);
  enlace.begin(PruebaMAX485::BAUD);
  enlace.listen();
  ultimoEnvio = millis();
  Serial.println(F("[MAX485 UNO] UN MODULO v4; RX D2<-RO; TX D3->Nano D2; bus=9600; monitor=115200"));
  Serial.println(F("[MAX485 UNO] Turnos UNO->NANO y NANO->UNO; ciclo=1 s"));
}

void loop() {
  uint8_t paquete[PruebaMAX485::TAMANO];
  while (enlace.available()) {
    if (!receptor.agregar(static_cast<uint8_t>(enlace.read()), paquete))
      continue;
    const uint32_t numero = PruebaMAX485::secuencia(paquete);
    const uint32_t valor = PruebaMAX485::valor(paquete);
    const uint32_t turno = PruebaMAX485::turno(paquete);
    if (fase == ESPERA_ACK_NANO && paquete[5] == PruebaMAX485::ACK_NANO &&
        numero == numeroUno && valor == valorEnviado && turno == numeroUno &&
        millis() - inicioEspera < PruebaMAX485::TIMEOUT_MS) {
      ++confirmadosUno;
      Serial.print(F("[MAX485 UNO] TX UNO->NANO CONFIRMADO seq="));
      Serial.print(numeroUno);
      Serial.print(F(" ida_vuelta_ms=")); Serial.println(millis() - inicioEspera);
      enviarPaquete(PruebaMAX485::TURNO_NANO, numeroUno, 0, numeroUno);
      inicioEspera = millis();
      fase = ESPERA_DATOS_NANO;
    } else if (fase == ESPERA_DATOS_NANO && paquete[5] == PruebaMAX485::DATOS_NANO &&
               turno == numeroUno &&
               millis() - inicioEspera < PruebaMAX485::TIMEOUT_MS) {
      ++recibidosNANO;
      enviarPaquete(PruebaMAX485::ACK_UNO, numero, valor, turno);
      fase = REPOSO;
      Serial.print(F("[MAX485 UNO] RX NANO->UNO OK seq=")); Serial.print(numero);
      Serial.print(F(" nano_ms=")); Serial.print(valor);
      Serial.println(F(" ACK enviado"));
    } else {
      ++inesperados;
    }
  }
  const uint32_t ahora = millis();
  if (fase != REPOSO && ahora - inicioEspera >= PruebaMAX485::TIMEOUT_MS) {
    if (fase == ESPERA_ACK_NANO) {
      ++timeoutsUno;
      Serial.print(F("[MAX485 UNO] TIMEOUT ACK UNO->NANO seq="));
    } else {
      ++timeoutsTurnoNANO;
      Serial.print(F("[MAX485 UNO] TIMEOUT DATOS NANO->UNO turno="));
    }
    Serial.println(numeroUno);
    fase = REPOSO;
  }
  if (fase == REPOSO && ahora - ultimoEnvio >= PruebaMAX485::INTERVALO_MS) {
    valorEnviado = millis();
    ++numeroUno;
    enviarPaquete(PruebaMAX485::DATOS_UNO, numeroUno, valorEnviado, numeroUno);
    inicioEspera = millis();
    ultimoEnvio = inicioEspera;
    fase = ESPERA_ACK_NANO;
    ++enviados;
    Serial.print(F("[MAX485 UNO] TX UNO->NANO seq=")); Serial.print(numeroUno);
    Serial.print(F(" uno_ms=")); Serial.println(valorEnviado);
  }
  if (ahora - ultimoResumen >= 10000) {
    ultimoResumen = ahora;
    Serial.print(F("[MAX485 UNO] enviados=")); Serial.print(enviados);
    Serial.print(F(" confirmado_uno=")); Serial.print(confirmadosUno);
    Serial.print(F(" rx_nano=")); Serial.print(recibidosNANO);
    Serial.print(F(" timeout_uno=")); Serial.print(timeoutsUno);
    Serial.print(F(" timeout_turno_nano=")); Serial.print(timeoutsTurnoNANO);
    Serial.print(F(" rx_bytes=")); Serial.print(receptor.bytes);
    Serial.print(F(" invalidos=")); Serial.print(receptor.invalidos);
    Serial.print(F(" inesperados=")); Serial.println(inesperados);
  }
  delay(1);
}

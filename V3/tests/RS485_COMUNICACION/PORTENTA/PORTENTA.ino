// Solo prueba RS485 integrada de Portenta H7 + Machine Control.
#include <Arduino_MachineControl.h>
#include "PruebaRS485.h"

using namespace machinecontrol;
constexpr bool TERMINACION_120_OHM = true;

PruebaRS485::Receptor receptor;
enum FasePrueba : uint8_t { REPOSO, ESPERA_ACK_ESP, ESPERA_DATOS_ESP };
FasePrueba fase = REPOSO;
uint32_t numeroPortenta = 0;
uint32_t valorEnviado = 0;
uint32_t enviados = 0;
uint32_t confirmadosPortenta = 0;
uint32_t recibidosESP = 0;
uint32_t timeoutsPortenta = 0;
uint32_t timeoutsTurnoESP = 0;
uint32_t inesperados = 0;
uint32_t inicioEspera = 0;
uint32_t ultimoEnvio = 0;
uint32_t ultimoResumen = 0;

void enviarPaquete(uint8_t tipo, uint32_t numero, uint32_t valor, uint32_t turno) {
  uint8_t paquete[PruebaRS485::TAMANO];
  PruebaRS485::crear(tipo, numero, valor, turno, paquete);
  // Permitir que la ESP libere DE antes de responder por el mismo par.
  delayMicroseconds(2000);
  comm_protocols.rs485.noReceive();
  comm_protocols.rs485.beginTransmission();
  comm_protocols.rs485.write(paquete, sizeof(paquete));
  comm_protocols.rs485.endTransmission();
  comm_protocols.rs485.receive();
}

void setup() {
  Serial.begin(115200);
  const uint32_t inicioUSB = millis();
  while (!Serial && millis() - inicioUSB < 2500) delay(10);
  comm_protocols.init();
  comm_protocols.rs485ModeRS232(false);
  comm_protocols.rs485FullDuplex(false);
  comm_protocols.rs485ABTerm(TERMINACION_120_OHM);
  comm_protocols.rs485Enable(true);
  comm_protocols.rs485.begin(PruebaRS485::BAUD, SERIAL_8N1, 50, 500);
  comm_protocols.rs485.receive();
  ultimoEnvio = millis();
  Serial.println("[RS485 PORTENTA] Prueba BIDIRECCIONAL v2; TX P/TX N; 115200; 32 bytes");
  Serial.println("[RS485 PORTENTA] Turnos PORTENTA->ESP y ESP->PORTENTA; ciclo=1 s");
}

void loop() {
  uint8_t paquete[PruebaRS485::TAMANO];
  while (comm_protocols.rs485.available()) {
    if (!receptor.agregar(static_cast<uint8_t>(comm_protocols.rs485.read()), paquete))
      continue;
    const uint32_t numero = PruebaRS485::secuencia(paquete);
    const uint32_t valor = PruebaRS485::valor(paquete);
    const uint32_t turno = PruebaRS485::turno(paquete);
    if (fase == ESPERA_ACK_ESP && paquete[5] == PruebaRS485::ACK_ESP &&
        numero == numeroPortenta && valor == valorEnviado && turno == numeroPortenta &&
        millis() - inicioEspera < PruebaRS485::TIMEOUT_MS) {
      ++confirmadosPortenta;
      Serial.print("[RS485 PORTENTA] TX PORTENTA->ESP CONFIRMADO seq=");
      Serial.print(numeroPortenta);
      Serial.print(" ida_vuelta_ms="); Serial.println(millis() - inicioEspera);
      enviarPaquete(PruebaRS485::TURNO_ESP, numeroPortenta, 0, numeroPortenta);
      inicioEspera = millis();
      fase = ESPERA_DATOS_ESP;
    } else if (fase == ESPERA_DATOS_ESP && paquete[5] == PruebaRS485::DATOS_ESP &&
               turno == numeroPortenta &&
               millis() - inicioEspera < PruebaRS485::TIMEOUT_MS) {
      ++recibidosESP;
      enviarPaquete(PruebaRS485::ACK_PORTENTA, numero, valor, turno);
      fase = REPOSO;
      Serial.print("[RS485 PORTENTA] RX ESP->PORTENTA OK seq="); Serial.print(numero);
      Serial.print(" esp_ms="); Serial.print(valor);
      Serial.println(" ACK enviado");
    } else {
      ++inesperados;
    }
  }
  const uint32_t ahora = millis();
  if (fase != REPOSO && ahora - inicioEspera >= PruebaRS485::TIMEOUT_MS) {
    if (fase == ESPERA_ACK_ESP) {
      ++timeoutsPortenta;
      Serial.print("[RS485 PORTENTA] TIMEOUT ACK PORTENTA->ESP seq=");
    } else {
      ++timeoutsTurnoESP;
      Serial.print("[RS485 PORTENTA] TIMEOUT DATOS ESP->PORTENTA turno=");
    }
    Serial.println(numeroPortenta);
    fase = REPOSO;
  }
  if (fase == REPOSO && ahora - ultimoEnvio >= PruebaRS485::INTERVALO_MS) {
    valorEnviado = millis();
    ++numeroPortenta;
    enviarPaquete(PruebaRS485::DATOS_PORTENTA, numeroPortenta, valorEnviado, numeroPortenta);
    inicioEspera = millis();
    ultimoEnvio = inicioEspera;
    fase = ESPERA_ACK_ESP;
    ++enviados;
    Serial.print("[RS485 PORTENTA] TX PORTENTA->ESP seq="); Serial.print(numeroPortenta);
    Serial.print(" portenta_ms="); Serial.println(valorEnviado);
  }
  if (ahora - ultimoResumen >= 10000) {
    ultimoResumen = ahora;
    Serial.print("[RS485 PORTENTA] enviados="); Serial.print(enviados);
    Serial.print(" confirmado_portenta="); Serial.print(confirmadosPortenta);
    Serial.print(" rx_esp="); Serial.print(recibidosESP);
    Serial.print(" timeout_portenta="); Serial.print(timeoutsPortenta);
    Serial.print(" timeout_turno_esp="); Serial.print(timeoutsTurnoESP);
    Serial.print(" rx_bytes="); Serial.print(receptor.bytes);
    Serial.print(" invalidos="); Serial.print(receptor.invalidos);
    Serial.print(" inesperados="); Serial.println(inesperados);
  }
  delay(1);
}

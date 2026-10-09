#include <Arduino.h>
#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\UART_NANO_PORTENTA\\PORTENTA\\PORTENTA.ino"
// UART TTL de Machine Control: TX TP78/PA0 y RX TP83/PI9. NO usar TX P/TX N.
#include <Arduino_MachineControl.h>
#include "PruebaUART.h"

using namespace machinecontrol;
arduino::UART& enlace = comm_protocols._UART4_;

PruebaUART::Receptor receptor;
enum FasePrueba : uint8_t { REPOSO, ESPERA_ACK_NANO, ESPERA_DATOS_NANO };
FasePrueba fase = REPOSO;
uint32_t numeroPortenta = 0;
uint32_t valorEnviado = 0;
uint32_t enviados = 0;
uint32_t confirmadosPortenta = 0;
uint32_t recibidosNANO = 0;
uint32_t timeoutsPortenta = 0;
uint32_t timeoutsTurnoNANO = 0;
uint32_t inesperados = 0;
uint32_t inicioEspera = 0;
uint32_t ultimoEnvio = 0;
uint32_t ultimoResumen = 0;

#line 23 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\UART_NANO_PORTENTA\\PORTENTA\\PORTENTA.ino"
void enviarPaquete(uint8_t tipo, uint32_t numero, uint32_t valor, uint32_t turno);
#line 32 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\UART_NANO_PORTENTA\\PORTENTA\\PORTENTA.ino"
void setup();
#line 55 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\UART_NANO_PORTENTA\\PORTENTA\\PORTENTA.ino"
void loop();
#line 23 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\UART_NANO_PORTENTA\\PORTENTA\\PORTENTA.ino"
void enviarPaquete(uint8_t tipo, uint32_t numero, uint32_t valor, uint32_t turno) {
  uint8_t paquete[PruebaUART::TAMANO];
  PruebaUART::crear(tipo, numero, valor, turno, paquete);
  // Margen para SoftwareSerial del Nano; UART tiene TX/RX separados.
  delayMicroseconds(2000);
  enlace.write(paquete, sizeof(paquete));
  enlace.flush();
}

void setup() {
  Serial.begin(115200);
  const uint32_t inicioUSB = millis();
  while (!Serial && millis() - inicioUSB < 2500) delay(10);
  // Modo RS485, /RE=HIGH, DE=LOW y SHDN=LOW: SP335 sin manejar RX/TX.
  // No llamar comm_protocols.init(): toca tambien las lineas UART como GPIO.
  digitalWrite(PinNameToIndex(PG_9), LOW);
  pinMode(PinNameToIndex(PG_9), OUTPUT);
  comm_protocols.rs485Enable(false);
  digitalWrite(PinNameToIndex(PA_10), HIGH);
  pinMode(PinNameToIndex(PA_10), OUTPUT);
  comm_protocols.rs485ModeRS232(false);
  digitalWrite(PinNameToIndex(PI_10), HIGH);
  pinMode(PinNameToIndex(PI_10), OUTPUT);
  digitalWrite(PinNameToIndex(PI_13), LOW);
  pinMode(PinNameToIndex(PI_13), OUTPUT);
  delayMicroseconds(5);
  enlace.begin(PruebaUART::BAUD, SERIAL_8N1);
  ultimoEnvio = millis();
  Serial.println("[UART PORTENTA] Prueba BIDIRECCIONAL v1; TX=TP78 RX=TP83; bus=9600; monitor=115200; 32 bytes");
  Serial.println("[UART PORTENTA] Turnos PORTENTA->NANO y NANO->PORTENTA; ciclo=1 s");
}

void loop() {
  uint8_t paquete[PruebaUART::TAMANO];
  while (enlace.available()) {
    if (!receptor.agregar(static_cast<uint8_t>(enlace.read()), paquete))
      continue;
    const uint32_t numero = PruebaUART::secuencia(paquete);
    const uint32_t valor = PruebaUART::valor(paquete);
    const uint32_t turno = PruebaUART::turno(paquete);
    if (fase == ESPERA_ACK_NANO && paquete[5] == PruebaUART::ACK_NANO &&
        numero == numeroPortenta && valor == valorEnviado && turno == numeroPortenta &&
        millis() - inicioEspera < PruebaUART::TIMEOUT_MS) {
      ++confirmadosPortenta;
      Serial.print("[UART PORTENTA] TX PORTENTA->NANO CONFIRMADO seq=");
      Serial.print(numeroPortenta);
      Serial.print(" ida_vuelta_ms="); Serial.println(millis() - inicioEspera);
      enviarPaquete(PruebaUART::TURNO_NANO, numeroPortenta, 0, numeroPortenta);
      inicioEspera = millis();
      fase = ESPERA_DATOS_NANO;
    } else if (fase == ESPERA_DATOS_NANO && paquete[5] == PruebaUART::DATOS_NANO &&
               turno == numeroPortenta &&
               millis() - inicioEspera < PruebaUART::TIMEOUT_MS) {
      ++recibidosNANO;
      enviarPaquete(PruebaUART::ACK_PORTENTA, numero, valor, turno);
      fase = REPOSO;
      Serial.print("[UART PORTENTA] RX NANO->PORTENTA OK seq="); Serial.print(numero);
      Serial.print(" nano_ms="); Serial.print(valor);
      Serial.println(" ACK enviado");
    } else {
      ++inesperados;
    }
  }
  const uint32_t ahora = millis();
  if (fase != REPOSO && ahora - inicioEspera >= PruebaUART::TIMEOUT_MS) {
    if (fase == ESPERA_ACK_NANO) {
      ++timeoutsPortenta;
      Serial.print("[UART PORTENTA] TIMEOUT ACK PORTENTA->NANO seq=");
    } else {
      ++timeoutsTurnoNANO;
      Serial.print("[UART PORTENTA] TIMEOUT DATOS NANO->PORTENTA turno=");
    }
    Serial.println(numeroPortenta);
    fase = REPOSO;
  }
  if (fase == REPOSO && ahora - ultimoEnvio >= PruebaUART::INTERVALO_MS) {
    valorEnviado = millis();
    ++numeroPortenta;
    enviarPaquete(PruebaUART::DATOS_PORTENTA, numeroPortenta, valorEnviado, numeroPortenta);
    inicioEspera = millis();
    ultimoEnvio = inicioEspera;
    fase = ESPERA_ACK_NANO;
    ++enviados;
    Serial.print("[UART PORTENTA] TX PORTENTA->NANO seq="); Serial.print(numeroPortenta);
    Serial.print(" portenta_ms="); Serial.println(valorEnviado);
  }
  if (ahora - ultimoResumen >= 10000) {
    ultimoResumen = ahora;
    Serial.print("[UART PORTENTA] enviados="); Serial.print(enviados);
    Serial.print(" confirmado_portenta="); Serial.print(confirmadosPortenta);
    Serial.print(" rx_nano="); Serial.print(recibidosNANO);
    Serial.print(" timeout_portenta="); Serial.print(timeoutsPortenta);
    Serial.print(" timeout_turno_nano="); Serial.print(timeoutsTurnoNANO);
    Serial.print(" rx_bytes="); Serial.print(receptor.bytes);
    Serial.print(" invalidos="); Serial.print(receptor.invalidos);
    Serial.print(" inesperados="); Serial.println(inesperados);
  }
  delay(1);
}


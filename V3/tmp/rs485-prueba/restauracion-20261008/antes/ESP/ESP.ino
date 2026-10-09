// Solo prueba de comunicacion: sin servos, camara, OLED, control ni motores.
#include <Arduino.h>
#include "PruebaRS485.h"

constexpr int RS485_RX = 14;
constexpr int RS485_TX = 27;
constexpr int RS485_DIRECCION = 18;
// true: DE y /RE unidos a GPIO18. false: modulo de direccion automatica.
constexpr bool DIRECCION_MANUAL = true;

HardwareSerial enlace(2);
PruebaRS485::Receptor receptor;
uint32_t recibidosPortenta = 0;
uint32_t enviadosESP = 0;
uint32_t confirmadosESP = 0;
uint32_t timeoutsESP = 0;
uint32_t inesperados = 0;
uint32_t ultimoResumen = 0;
uint32_t numeroESP = 0;
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
  enlace.flush(); // Esperar el ultimo bit antes de liberar el bus.
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
  enlace.begin(PruebaRS485::BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  Serial.println("[RS485 ESP] Prueba BIDIRECCIONAL v2; RX=27 TX=14; 115200; 32 bytes");
  Serial.println(DIRECCION_MANUAL ? "[RS485 ESP] DE+/RE=GPIO18" :
                                 "[RS485 ESP] Direccion automatica");
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
        ++timeoutsESP;
      }
      ++recibidosPortenta;
      enviarPaquete(PruebaRS485::ACK_ESP, numero, valor, turno);
      Serial.print("[RS485 ESP] RX PORTENTA->ESP OK seq="); Serial.print(numero);
      Serial.print(" portenta_ms="); Serial.print(valor);
      Serial.println(" ACK enviado");
    } else if (paquete[5] == PruebaRS485::TURNO_ESP && turno == numero &&
               valor == 0 && !esperandoACK) {
      // Mensaje propio: secuencia independiente y reloj local de la ESP.
      valorEnviado = millis();
      turnoEnviado = turno;
      enviarPaquete(PruebaRS485::DATOS_ESP, ++numeroESP, valorEnviado, turnoEnviado);
      ++enviadosESP;
      inicioEspera = millis();
      esperandoACK = true;
      Serial.print("[RS485 ESP] TX ESP->PORTENTA seq="); Serial.print(numeroESP);
      Serial.print(" esp_ms="); Serial.println(valorEnviado);
    } else if (paquete[5] == PruebaRS485::ACK_PORTENTA && esperandoACK &&
               numero == numeroESP && valor == valorEnviado && turno == turnoEnviado &&
               millis() - inicioEspera < PruebaRS485::TIMEOUT_MS) {
      esperandoACK = false;
      ++confirmadosESP;
      Serial.print("[RS485 ESP] TX ESP->PORTENTA CONFIRMADO seq=");
      Serial.print(numeroESP);
      Serial.print(" ida_vuelta_ms="); Serial.println(millis() - inicioEspera);
    } else {
      ++inesperados;
    }
  }
  if (esperandoACK && millis() - inicioEspera >= PruebaRS485::TIMEOUT_MS) {
    esperandoACK = false;
    ++timeoutsESP;
    Serial.print("[RS485 ESP] TIMEOUT ACK ESP->PORTENTA seq="); Serial.println(numeroESP);
  }
  if (millis() - ultimoResumen >= 10000) {
    ultimoResumen = millis();
    Serial.print("[RS485 ESP] rx_portenta="); Serial.print(recibidosPortenta);
    Serial.print(" tx_esp="); Serial.print(enviadosESP);
    Serial.print(" confirmado_esp="); Serial.print(confirmadosESP);
    Serial.print(" timeout_esp="); Serial.print(timeoutsESP);
    Serial.print(" rx_bytes="); Serial.print(receptor.bytes);
    Serial.print(" invalidos="); Serial.print(receptor.invalidos);
    Serial.print(" inesperados="); Serial.println(inesperados);
  }
  delay(1);
}

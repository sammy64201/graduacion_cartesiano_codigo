#include <initializer_list>
#include <assert.h>
#include <stdint.h>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/ESP/ProtocoloI2C.h"
using namespace ProtocoloI2C;
struct Terminal { template<class T> void print(const T&) {};
template<class T> void println(const T&) {}; } Serial;
unsigned long millis() { return 100; }
struct ContextoCamara { bool pruebaEncoderAnterior=false, esperandoDesaparicion=false,
rearmada=true, portentaEstabaActiva=true, autoEstabaActivo=true, objetivoValido=true,
objetivoV2=true; uint16_t secuenciaObjetivo=42, ultimaSecuenciaComando=0, ackComando=0;
unsigned long proximaLectura=0; };
struct ControlCamaraCompartido { bool pruebaEncoderActiva=false, portentaActiva=true,
automaticoActivo=true, registrarAngulo=false; uint8_t codigoAckObjetivo=0;
uint16_t ackObjetivo=42; };
void limpiarObjetivo(ContextoCamara& ctx, bool) { ctx.objetivoValido=false; }
void procesarHandshakeObjetivo(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control
) {
  if (control.pruebaEncoderActiva != ctx.pruebaEncoderAnterior) {
    limpiarObjetivo(ctx, false);
    ctx.esperandoDesaparicion = false;
    ctx.rearmada = true;
    ctx.pruebaEncoderAnterior = control.pruebaEncoderActiva;
    ctx.proximaLectura = millis();
  }
  if (control.pruebaEncoderActiva) {
    ctx.portentaEstabaActiva = control.portentaActiva;
    ctx.autoEstabaActivo = false;
    return;
  }
  if (
    ctx.objetivoValido &&
    control.portentaActiva &&
    control.codigoAckObjetivo != ACK_OBJ_NINGUNO &&
    control.ackObjetivo == ctx.secuenciaObjetivo
  ) {
    // ACEPTADO, CERRAR_PINZA y ABRIR_PINZA mantienen la reserva durante
    // captura, entrega y retirada. Solo un resultado terminal
    // libera el objetivo y permite el rearme del detector.
    const bool objetivoReservado = ctx.objetivoV2 &&
      (control.codigoAckObjetivo == ACK_OBJ_ACEPTADO ||
       control.codigoAckObjetivo == ACK_OBJ_CERRAR_PINZA ||
       control.codigoAckObjetivo == ACK_OBJ_ABRIR_PINZA);
    if (!objetivoReservado) {
      if (control.registrarAngulo) {
        Serial.print(F("[ANGULO] Objetivo liberado seq="));
      } else if (ctx.objetivoV2) {
        Serial.print(F("[AUTO V2] Objetivo liberado seq="));
      } else {
        Serial.print(F("[AUTO] ACK objetivo seq="));
      }
      Serial.print(ctx.secuenciaObjetivo);
      Serial.print(F(" ack="));
      Serial.println(control.codigoAckObjetivo);
      limpiarObjetivo(ctx, true);
    }
  }

  if (!control.portentaActiva && ctx.portentaEstabaActiva) {
    Serial.println(F("[AUTO] Objetivo invalidado por perdida I2C"));
    limpiarObjetivo(ctx, true);

    // La Portenta puede reiniciarse sin que la ESP32 lo haga. Su contador de
    // comandos volvera a cero; olvidar el dominio anterior evita confundir un
    // CALIBRAR nuevo con una retransmision ya atendida antes del reinicio.
    ctx.ultimaSecuenciaComando = 0;
    ctx.ackComando = 0;
  }

  if (!control.automaticoActivo && ctx.autoEstabaActivo) {
    Serial.println(F("[AUTO] Filtro limpiado al salir de modo automatico"));
    limpiarObjetivo(ctx, true);
  }

  ctx.portentaEstabaActiva = control.portentaActiva;
  ctx.autoEstabaActivo = control.automaticoActivo;
}int main() {
ContextoCamara ctx; ControlCamaraCompartido control;
for (uint8_t ack : {ACK_OBJ_ACEPTADO, ACK_OBJ_CERRAR_PINZA, ACK_OBJ_ABRIR_PINZA}) {
  control.codigoAckObjetivo=ack; procesarHandshakeObjetivo(ctx,control);
  assert(ctx.objetivoValido);
}
control.codigoAckObjetivo=ACK_OBJ_COMPLETADO; procesarHandshakeObjetivo(ctx,control);
assert(!ctx.objetivoValido);
ctx.objetivoValido=true; control.codigoAckObjetivo=ACK_OBJ_CANCELADO;
procesarHandshakeObjetivo(ctx,control); assert(!ctx.objetivoValido);
}
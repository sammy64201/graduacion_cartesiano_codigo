#include <initializer_list>
#include <assert.h>
#include <stdint.h>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/ESP/ProtocoloI2C.h"
using namespace ProtocoloI2C;
struct Terminal { template<class T> void print(const T&) {};
template<class T> void println(const T&) {}; void println() {} } Serial;
unsigned long millis() { return 100; }
struct ContextoCamara { bool pruebaEncoderAnterior=false, esperandoDesaparicion=false,
rearmada=true, portentaEstabaActiva=true, autoEstabaActivo=true, objetivoValido=true,
objetivoV2=true; uint16_t secuenciaObjetivo=42, ultimaSecuenciaComando=0, ackComando=0;
unsigned long proximaLectura=0; };
struct ControlCamaraCompartido { bool pruebaEncoderActiva=false, portentaActiva=true,
automaticoActivo=true, registrarAngulo=false; uint8_t codigoAckObjetivo=0;
uint16_t ackObjetivo=42; };
void limpiarObjetivo(ContextoCamara& ctx, bool) { ctx.objetivoValido=false; }
PaquetePortentaAESP estadoPortenta={};
bool estadoPortentaValido=true, automaticoV2PinzaAnterior=false;
unsigned long ultimoEstadoPortenta=100;
constexpr unsigned long TIMEOUT_PORTENTA_MS=150;
uint16_t ultimaSecuenciaPinzaV2=0, sesionArranque=1;
uint8_t ultimoCodigoPinzaV2=0;
int anguloServoRotacion=59, anguloServoPinza=0, aperturas=0, cierres=0;
constexpr int PULSO_PINZA_CERRADA_US=1816;
struct EstadoCamaraPublicado { int sugerenciaAngulo=-1; };
EstadoCamaraPublicado copiarEstadoCamara() { return {}; }
void escribirPinzaCalibrada(bool cerrar) {
  if (cerrar) ++cierres; else ++aperturas;
}
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
struct EstadoEncoderCompartido {
  int32_t conteo;
  float velocidadMmS;
  uint32_t nmPorCuenta;
  uint16_t secuencia;
  uint8_t flags;
  int8_t signo;
  uint32_t recibidoMs;
};
EstadoEncoderCompartido estadoEncoder;
void actualizarEncoderDesdePortenta(const PaquetePortentaAESP &paquete) {
  EstadoEncoderCompartido nuevo = {};
  if (paquete.estadoSistema == SISTEMA_CAMBIOS_CATCH) {
    // Datos de resumen: invalidar encoder remoto, sin publicar offsets como velocidad.
    nuevo.recibidoMs = millis();
    portENTER_CRITICAL(&encoderMux); estadoEncoder = nuevo; portEXIT_CRITICAL(&encoderMux);
    return;
  }
  nuevo.conteo = paquete.conteoEncoder;
  nuevo.velocidadMmS = static_cast<float>(paquete.velocidadEncoderUmS) / 1000000.0f;
  nuevo.nmPorCuenta = extraerEscalaEncoderNm(paquete.nmPorCuentaEncoder);
  nuevo.secuencia = paquete.secuenciaEncoder;
  nuevo.flags = paquete.estadoEncoder;
  nuevo.signo = paquete.signoEncoder;
  nuevo.recibidoMs = millis();
  portENTER_CRITICAL(&encoderMux);
  estadoEncoder = nuevo;
  portEXIT_CRITICAL(&encoderMux);
}bool paquetePortentaSemanticamenteValido(const PaquetePortentaAESP &p) {
  if (p.errorSistema > SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA) return false;
  if (p.errorSistema == SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA &&
      (p.estadoSistema != SISTEMA_MODO_AUTOMATICO_V2 ||
       p.opcionMenu != MENU_MODO_AUTOMATICO_V2)) return false;
  if (p.estadoSistema == SISTEMA_CAMBIOS_CATCH) {
    // El resumen usa campos del encoder con unidades/flags propios.
    return p.opcionMenu == MENU_CAMBIOS_CATCH && p.faseCalibracionBrazo <= 1 &&
      (p.estadoEncoder & 0xF0U) == 0 &&
      ((p.estadoEncoder & 1U) != 0) == ((p.estadoEncoder >> 2) == 3) &&
      p.nmPorCuentaEncoder >= 10 && p.nmPorCuentaEncoder <= 50 &&
      p.codigoAckObjetivo <= ACK_OBJ_ABRIR_PINZA &&
      (p.signoEncoder == 1 || p.signoEncoder == -1);
  }
  const uint8_t mascaraEncoder = ENC_FLAG_HW_LISTO |
                                 ENC_FLAG_ESCALA_VALIDA |
                                 ENC_FLAG_PULSOS_VISTOS |
                                 ENC_FLAG_EN_MOVIMIENTO |
                                 ENC_FLAG_DIRECCION_POSITIVA |
                                 ENC_FLAG_SATURADO;
  const bool flagsValidos =
    (p.estadoEncoder & static_cast<uint8_t>(~mascaraEncoder)) == 0;
  const bool signoValido = p.signoEncoder == 1 || p.signoEncoder == -1;
  const bool escalaCoherente =
    (p.estadoEncoder & ENC_FLAG_ESCALA_VALIDA) != 0
      ? extraerEscalaEncoderNm(p.nmPorCuentaEncoder) != 0
      : extraerEscalaEncoderNm(p.nmPorCuentaEncoder) == 0;
  const bool ackValido = p.codigoAckObjetivo <= ACK_OBJ_ABRIR_PINZA;
  return flagsValidos && signoValido && escalaCoherente && ackValido;
}void procesarHandshakeObjetivo(
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
}void procesarPinzaAutomaticaV2() {
  const bool enlaceVigente = estadoPortentaValido &&
    millis() - ultimoEstadoPortenta <= TIMEOUT_PORTENTA_MS;
  const bool automaticoV2Activo = enlaceVigente && (
    (estadoPortenta.estadoSistema == SISTEMA_MODO_AUTOMATICO_V2 &&
     (estadoPortenta.opcionMenu == MENU_MODO_AUTOMATICO_V2 ||
      estadoPortenta.opcionMenu == MENU_AJUSTE_CATCH_V2)) ||
    (estadoPortenta.estadoSistema == SISTEMA_MODO_AUTOMATICO &&
     estadoPortenta.opcionMenu == MENU_REGISTRO_ANGULO) ||
    (estadoPortenta.estadoSistema == SISTEMA_ENTRENAMIENTO_ML &&
     (estadoPortenta.opcionMenu == MENU_ENTRENAMIENTO_ML ||
       estadoPortenta.opcionMenu == MENU_ENTRENAMIENTO_ML_V2 ||
       estadoPortenta.opcionMenu == MENU_PRUEBA_SEGUIMIENTO))
  );

  if (!automaticoV2Activo) {
    automaticoV2PinzaAnterior = false;
    return;
  }

  if (!automaticoV2PinzaAnterior) {
    // Toda entrada/reentrada a V2 empieza con la garra abierta. Esto tambien
    // recupera de forma determinista una sesion anterior interrumpida.
    escribirPinzaCalibrada(false);
    automaticoV2PinzaAnterior = true;
    ultimaSecuenciaPinzaV2 = estadoPortenta.ackSecuenciaObjetivo;
    ultimoCodigoPinzaV2 = estadoPortenta.codigoAckObjetivo;
    return;
  }

  const bool ordenNueva =
    estadoPortenta.ackSecuenciaObjetivo != ultimaSecuenciaPinzaV2 ||
    estadoPortenta.codigoAckObjetivo != ultimoCodigoPinzaV2;
  if (!ordenNueva) return;

  ultimaSecuenciaPinzaV2 = estadoPortenta.ackSecuenciaObjetivo;
  ultimoCodigoPinzaV2 = estadoPortenta.codigoAckObjetivo;
  if (ultimoCodigoPinzaV2 == ACK_OBJ_CERRAR_PINZA) {
    escribirPinzaCalibrada(true);
    if (estadoPortenta.opcionMenu == MENU_AJUSTE_CATCH_V2) {
      Serial.print(F("V2LOG|E|mode=CATCH_CAL|event=CAL_GRIP_APPLIED|session="));
      Serial.print(sesionArranque);
      Serial.print(F("|ms=")); Serial.print(millis());
      Serial.print(F("|obj=")); Serial.print(ultimaSecuenciaPinzaV2);
      Serial.print(F("|servo_rot_deg=")); Serial.print(anguloServoRotacion);
      Serial.print(F("|pulse_us=")); Serial.println(PULSO_PINZA_CERRADA_US);
    }
    if (estadoPortenta.estadoSistema == SISTEMA_ENTRENAMIENTO_ML) {
      Serial.print(estadoPortenta.opcionMenu == MENU_PRUEBA_SEGUIMIENTO
        ? F("V2LOG|E|mode=ML_TRACK|event=ML_GRIP_APPLIED|session=")
        : (estadoPortenta.opcionMenu == MENU_ENTRENAMIENTO_ML_V2
            ? F("V2LOG|E|mode=ML_V2|event=ML_GRIP_APPLIED|session=")
            : F("V2LOG|E|mode=ML|event=ML_GRIP_APPLIED|session=")));
      Serial.print(sesionArranque);
      Serial.print(F("|ms=")); Serial.print(millis());
      Serial.print(F("|obj=")); Serial.print(ultimaSecuenciaPinzaV2);
      Serial.print(F("|servo_rot_deg=")); Serial.print(anguloServoRotacion);
      Serial.print(F("|grip_deg=")); Serial.print(anguloServoPinza);
      Serial.print(F("|pulse_us=")); Serial.println(PULSO_PINZA_CERRADA_US);
    }
    Serial.print(F("ML_GRIP_APPLIED|seq="));
    Serial.print(ultimaSecuenciaPinzaV2);
    Serial.print(F("|servo_rot_deg="));
    Serial.print(anguloServoRotacion);
    Serial.print(F("|grip_deg="));
    Serial.print(anguloServoPinza);
    Serial.print(F("|pulse_us="));
    Serial.print(PULSO_PINZA_CERRADA_US);
    Serial.print(F("|esp_ms="));
    Serial.println(millis());
    if (estadoPortenta.estadoSistema == SISTEMA_MODO_AUTOMATICO &&
        estadoPortenta.opcionMenu == MENU_REGISTRO_ANGULO) {
      const EstadoCamaraPublicado camara = copiarEstadoCamara();
      Serial.print(F("V2LOG|E|mode=ANGLE_LABEL|event=ANGLE_FEEDBACK|session="));
      Serial.print(sesionArranque);
      Serial.print(F("|obj=")); Serial.print(ultimaSecuenciaPinzaV2);
      Serial.print(F("|suggested_rot="));
      if (camara.sugerenciaAngulo >= 0) {
        Serial.print(camara.sugerenciaAngulo);
      } else {
        Serial.print(F("NA"));
      }
      Serial.print(F("|servo_rot_deg=")); Serial.print(anguloServoRotacion);
      Serial.print(F("|correction_deg="));
      if (camara.sugerenciaAngulo >= 0) {
        Serial.print(abs(anguloServoRotacion - camara.sugerenciaAngulo));
      } else {
        Serial.print(F("NA"));
      }
      Serial.println();
    }
  } else if (ultimoCodigoPinzaV2 == ACK_OBJ_ABRIR_PINZA ||
             ultimoCodigoPinzaV2 == ACK_OBJ_CANCELADO ||
             ultimoCodigoPinzaV2 == ACK_OBJ_COMPLETADO) {
    // CANCELADO abre de inmediato; COMPLETADO llega cuando Z ya regreso a la
    // posicion segura, dejando la garra lista para la siguiente pieza.
    escribirPinzaCalibrada(false);
  }
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
// Ordenes reales de ESP: entrada abierta, cierre/apertura unicos aunque I2C repita.
estadoPortenta.estadoSistema=SISTEMA_MODO_AUTOMATICO_V2;
estadoPortenta.opcionMenu=MENU_MODO_AUTOMATICO_V2;
estadoPortenta.ackSecuenciaObjetivo=42;
estadoPortenta.codigoAckObjetivo=ACK_OBJ_ACEPTADO;
procesarPinzaAutomaticaV2(); assert(aperturas==1 && !cierres);
estadoPortenta.codigoAckObjetivo=ACK_OBJ_CERRAR_PINZA;
procesarPinzaAutomaticaV2(); procesarPinzaAutomaticaV2(); assert(cierres==1);
estadoPortenta.codigoAckObjetivo=ACK_OBJ_ABRIR_PINZA;
procesarPinzaAutomaticaV2(); procesarPinzaAutomaticaV2(); assert(aperturas==2);
estadoPortenta.opcionMenu=MENU_AJUSTE_CATCH_V2;
estadoPortenta.ackSecuenciaObjetivo=43;
estadoPortenta.codigoAckObjetivo=ACK_OBJ_CERRAR_PINZA;
procesarPinzaAutomaticaV2(); procesarPinzaAutomaticaV2(); assert(cierres==2);
// Resumen: semantica de OLED aceptada, sin convertir el ajuste en encoder.
PaquetePortentaAESP resumen={}; resumen.estadoSistema=SISTEMA_CAMBIOS_CATCH;
resumen.opcionMenu=MENU_CAMBIOS_CATCH; resumen.signoEncoder=1;
resumen.nmPorCuentaEncoder=50; resumen.conteoEncoder=-100;
resumen.velocidadEncoderUmS=-100;
for(int streak=0;streak<4;++streak) {
  resumen.estadoEncoder=(streak<<2)|(streak==3?1:0);
  assert(paquetePortentaSemanticamenteValido(resumen));
  actualizarEncoderDesdePortenta(resumen);
  assert(!estadoEncoder.flags && !estadoEncoder.nmPorCuenta && !estadoEncoder.conteo && estadoEncoder.velocidadMmS==0);
}
resumen.estadoEncoder=1; assert(!paquetePortentaSemanticamenteValido(resumen));
resumen.estadoEncoder=0; resumen.opcionMenu=MENU_DIAGNOSTICO;
assert(!paquetePortentaSemanticamenteValido(resumen));
// Al salir del resumen se vuelven a publicar datos reales.
resumen.estadoSistema=SISTEMA_MENU_PRINCIPAL; resumen.estadoEncoder=ENC_FLAG_HW_LISTO|ENC_FLAG_ESCALA_VALIDA;
resumen.nmPorCuentaEncoder=75000; resumen.conteoEncoder=123; resumen.velocidadEncoderUmS=50000000;
assert(paquetePortentaSemanticamenteValido(resumen)); actualizarEncoderDesdePortenta(resumen);
assert(estadoEncoder.conteo==123 && estadoEncoder.velocidadMmS==50);
PaquetePortentaAESP wire={}; wire.codigoAckObjetivo=ACK_OBJ_ABRIR_PINZA;
prepararPaquete(wire); assert(validarPaquete(wire));
wire.version=VERSION_PROTOCOLO-1;
wire.checksum=calcularCRC8ATM(reinterpret_cast<const uint8_t*>(&wire),31);
assert(!validarPaquete(wire)); // Rechazar version vieja aun con CRC correcto.
}
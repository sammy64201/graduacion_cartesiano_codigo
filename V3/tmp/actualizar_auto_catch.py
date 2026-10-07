from pathlib import Path
root = Path(__file__).resolve().parents[1]
folder = root / 'pruebas de automatico v2'

def edit(path, changes):
    s = path.read_text(encoding='utf-8-sig')
    for old, new in changes:
        assert old in s, (path, old[:100])
        s = s.replace(old, new)
    path.write_text(s, encoding='utf-8')

for board in ['ESP', 'PORTENTA']:
    edit(folder / board / 'ProtocoloI2C.h', [
        ('VERSION_PROTOCOLO = 15', 'VERSION_PROTOCOLO = 16'),
        ('SISTEMA_PRUEBA_ENCODER = 17', 'SISTEMA_PRUEBA_ENCODER = 17,\n    SISTEMA_CAMBIOS_CATCH = 18'),
        ('MENU_AJUSTE_CATCH_V2 = 13', 'MENU_AJUSTE_CATCH_V2 = 13,\n    MENU_CAMBIOS_CATCH = 14')])

p = folder / 'PORTENTA/PORTENTA.ino'
s = p.read_text(encoding='utf-8-sig')
start = s.index('            const bool listoAutomatico =', s.index('        case V2_SIGUIENDO_PIEZA:'))
end = s.index('            automaticoV2.busquedaFinalZActiva = false;', start)
old_trigger = s[start:end]
new_trigger = '''            const int32_t offset = ajusteCatchV2Seleccionado()
                ? ajusteCatchV2.offsetMs : AjusteCatchV2::limitar(V2_AJUSTE_DISPARO_CATCH_MS);
            const unsigned long espera = static_cast<unsigned long>(
                static_cast<int32_t>(V2_SEGUIMIENTO_ESTABLE_MS) + offset);
            if (millis() - automaticoV2.inicioEstable < espera) break;
            if (ajusteCatchV2Seleccionado()) {
                automaticoV2.ajusteProbadoMs = offset;
                automaticoV2.referenciaCatchRegistrada = true;
                automaticoV2.referenciaCatchProyectada =
                    millis() - automaticoV2.inicioEstable < V2_SEGUIMIENTO_ESTABLE_MS;
                automaticoV2.referenciaCatchMs = automaticoV2.inicioEstable + V2_SEGUIMIENTO_ESTABLE_MS;
                registrarAjusteCatchV2("CAL_REFERENCE");
                automaticoV2.disparoCatchMs = millis();
                automaticoV2.errorYDisparo = automaticoV2.ultimoErrorY;
                automaticoV2.velocidadDisparo = velocidadBandaMmS;
                automaticoV2.conteoDisparoCatch = conteoEncoderBanda;
                automaticoV2.brazoYDisparo = posicionYmm();
                automaticoV2.piezaYDisparo = automaticoV2.objetivoBrazoY;
            }
'''
edit(p, [
    ('#include "ProtocoloI2C.h"', '#include "ProtocoloI2C.h"\n#include "AjusteCatchV2.h"'),
    ('    EST_PRUEBA_ENCODER\n};', '    EST_PRUEBA_ENCODER,\n    EST_CAMBIOS_CATCH\n};'),
    ('constexpr unsigned long V2_SEGUIMIENTO_ESTABLE_MS = 300UL;', '''constexpr unsigned long V2_SEGUIMIENTO_ESTABLE_MS = 300UL;
// Positivo retrasa el descenso; negativo reduce la espera estable (minimo 100 ms).
// CAMBIOS CATCH propone esta linea tras evaluar los ensayos del modo separado.
constexpr int32_t V2_AJUSTE_DISPARO_CATCH_MS = 0;'''),
    ('    bool referenciaCatchRegistrada;', '    int32_t ajusteProbadoMs;\n    bool referenciaCatchRegistrada;'),
    ('// Solo para el modo de prueba; el Automatico V2 conserva su disparo autonomo.\nconstexpr bool AJUSTE_CATCH_DISPARO_MANUAL = true;', '''// Persiste entre piezas y al salir/entrar al menu; se reinicia al reiniciar placa.
AjusteCatchV2::Sesion ajusteCatchV2(V2_AJUSTE_DISPARO_CATCH_MS);
uint8_t paginaCambiosCatch = 0;

void imprimirCambiosCatchV2() {
    Serial.print(F("[CAMBIOS CATCH] ensayos=")); Serial.print(ajusteCatchV2.ensayos);
    Serial.print(F("; ultimo probado=")); Serial.print(ajusteCatchV2.ultimoProbadoMs);
    Serial.print(F(" ms; proximo=")); Serial.print(ajusteCatchV2.offsetMs);
    Serial.print(F(" ms; paso=")); Serial.print(ajusteCatchV2.pasoMs);
    Serial.print(F(" ms; agarres consecutivos=")); Serial.println(ajusteCatchV2.aciertosConsecutivos);
    if (ajusteCatchV2.ensayos == 0) {
        Serial.println(F("[CAMBIOS CATCH] Sin ensayos evaluados.")); return;
    }
    Serial.println(ajusteCatchV2.confirmado()
        ? F("[CAMBIOS CATCH] Valor repetido en 3 agarres consecutivos; comprobar otras velocidades.")
        : F("[CAMBIOS CATCH] Valor en prueba; aun no hay 3 agarres consecutivos."));
    Serial.print(F("[CAMBIOS CATCH] PORTENTA/PORTENTA.ino: constexpr int32_t V2_AJUSTE_DISPARO_CATCH_MS = "));
    Serial.print(ajusteCatchV2.ultimoProbadoMs); Serial.println(F("; // ultimo valor PROBADO"));
    if (ajusteCatchV2.limiteAlcanzado)
        Serial.println(F("[CAMBIOS CATCH] Limite de ajuste: revisar escala/distancia fisica; no ampliar automaticamente."));
}

void llenarResumenCatchV2(PaquetePortentaAESP &p) {
    // Semantica exclusiva de SISTEMA_CAMBIOS_CATCH; nunca son datos de encoder.
    p.faseCalibracionBrazo = paginaCambiosCatch;
    p.conteoEncoder = ajusteCatchV2.offsetMs;
    p.velocidadEncoderUmS = ajusteCatchV2.ultimoProbadoMs;
    p.secuenciaEncoder = ajusteCatchV2.ensayos;
    p.nmPorCuentaEncoder = ajusteCatchV2.pasoMs;
    p.estadoEncoder = (ajusteCatchV2.confirmado() ? 1U : 0U) |
        (ajusteCatchV2.limiteAlcanzado ? 2U : 0U) | (ajusteCatchV2.aciertosConsecutivos << 2);
}'''),
    ('        case EST_DIAGNOSTICO: return SISTEMA_DIAGNOSTICO;', '        case EST_DIAGNOSTICO: return SISTEMA_DIAGNOSTICO;\n        case EST_CAMBIOS_CATCH: return SISTEMA_CAMBIOS_CATCH;'),
    ('    prepararPaquete(p);\n}\n\nbool enviarPaquetePortenta()', '    if (estadoGeneral == EST_CAMBIOS_CATCH) llenarResumenCatchV2(p);\n    prepararPaquete(p);\n}\n\nbool enviarPaquetePortenta()'),
    ('        case EST_PRUEBA_SERVOS:\n        case EST_DIAGNOSTICO:', '        case EST_CAMBIOS_CATCH:\n        case EST_PRUEBA_SERVOS:\n        case EST_DIAGNOSTICO:'),
    ('MENU_MODO_AUTOMATICO_V2, MENU_AJUSTE_CATCH_V2, MENU_REGISTRO_ANGULO,', 'MENU_MODO_AUTOMATICO_V2, MENU_AJUSTE_CATCH_V2, MENU_CAMBIOS_CATCH, MENU_REGISTRO_ANGULO,'),
    ('        case MENU_REGISTRO_ANGULO:\n            modoPendiente', '''        case MENU_CAMBIOS_CATCH:
            paginaCambiosCatch = 0;
            cambiarEstadoGeneral(EST_CAMBIOS_CATCH);
            imprimirCambiosCatchV2();
            break;
        case MENU_REGISTRO_ANGULO:
            modoPendiente'''),
    ('    } else if (estadoGeneral == EST_DIAGNOSTICO && eventoBotonX) {', '''    } else if (estadoGeneral == EST_CAMBIOS_CATCH && eventoBotonX) {
        eventoBotonX = false;
        paginaCambiosCatch = 1U - paginaCambiosCatch;
        imprimirCambiosCatchV2();
    } else if (estadoGeneral == EST_DIAGNOSTICO && eventoBotonX) {'''),
    ('    Serial.print(F("|timing_result=")); Serial.print(resultado);', '''    Serial.print(F("|timing_result=")); Serial.print(resultado);
    Serial.print(F("|tested_offset_ms=")); Serial.print(automaticoV2.ajusteProbadoMs);
    Serial.print(F("|next_offset_ms=")); Serial.print(ajusteCatchV2.offsetMs);
    Serial.print(F("|adjust_step_ms=")); Serial.print(ajusteCatchV2.pasoMs);
    Serial.print(F("|adjust_trials=")); Serial.print(ajusteCatchV2.ensayos);
    Serial.print(F("|adjust_success_streak=")); Serial.print(ajusteCatchV2.aciertosConsecutivos);
    Serial.print(F("|adjust_confirmed=")); Serial.print(ajusteCatchV2.confirmado() ? 1 : 0);
    Serial.print(F("|adjust_limit=")); Serial.print(ajusteCatchV2.limiteAlcanzado ? 1 : 0);'''),
    ('            const bool solicitudManual = eventoBotonX;\n', ''),
    ('                if (ajusteCatchV2Seleccionado() && solicitudManual)\n                    registrarAjusteCatchV2("CAL_REJECT");\n', ''),
    (old_trigger, new_trigger),
    ('X=CORRECTO; cuadrado=TEMPRANO; circulo=TARDE;', 'X=LA AGARRO; cuadrado=ANTES; circulo=DESPUES;'),
    ('                registrarAjusteCatchV2("CAL_SAMPLE", resultado);\n                ++exitosV2;', '''                ajusteCatchV2.evaluar(eventoBotonX ? AjusteCatchV2::AGARRO : AjusteCatchV2::SIN_RESULTADO,
                    automaticoV2.ajusteProbadoMs);
                registrarAjusteCatchV2("CAL_SAMPLE", resultado);
                imprimirCambiosCatchV2();
                if (strcmp(resultado, "CORRECTO") == 0) ++exitosV2;''')])
# Evaluar la etiqueta antes de borrar botones; usar resultado estable.
edit(p, [('ajusteCatchV2.evaluar(eventoBotonX ? AjusteCatchV2::AGARRO : AjusteCatchV2::SIN_RESULTADO,',
          'ajusteCatchV2.evaluar(strcmp(resultado, "CORRECTO") == 0 ? AjusteCatchV2::AGARRO :\n                    (strcmp(resultado, "TEMPRANO") == 0 ? AjusteCatchV2::ANTES : AjusteCatchV2::DESPUES),')])

edit(folder / 'ESP/ESP.ino', [
    ('  EstadoEncoderCompartido nuevo = {};\n  nuevo.conteo = paquete.conteoEncoder;', '''  EstadoEncoderCompartido nuevo = {};
  if (paquete.estadoSistema == SISTEMA_CAMBIOS_CATCH) {
    // Datos de resumen: invalidar encoder remoto, sin publicar offsets como velocidad.
    nuevo.recibidoMs = millis();
    portENTER_CRITICAL(&encoderMux); estadoEncoder = nuevo; portEXIT_CRITICAL(&encoderMux);
    return;
  }
  nuevo.conteo = paquete.conteoEncoder;'''),
    ('const char *opciones[12]', 'const char *opciones[13]'),
    ('"AUTOMATICO V2", "AJUSTE CATCH V2",', '"AUTOMATICO V2", "AJUSTE CATCH V2", "CAMBIOS CATCH",'),
    ('const uint8_t valores[12]', 'const uint8_t valores[13]'),
    ('    MENU_AJUSTE_CATCH_V2,\n    MENU_REGISTRO_ANGULO', '    MENU_AJUSTE_CATCH_V2, MENU_CAMBIOS_CATCH,\n    MENU_REGISTRO_ANGULO'),
    ('i < 12; ++i', 'i < 13; ++i'),
    ('F("X: CORRECTO")', 'F("X: LA AGARRO")'),
    ('F("CUAD: TEMPRANO")', 'F("CUAD: ANTES")'),
    ('F("CIRC: TARDE")', 'F("CIRC: DESPUES")'),
    ('F("Y SIGUE  X:CATCH")', 'F("Y SIGUE CATCH AUTO")'),
    ('void mostrarEntrenamientoML(const PaquetePortentaAESP &p) {', '''void mostrarCambiosCatch(const PaquetePortentaAESP &p) {
  dibujarTitulo(F("CAMBIOS CATCH"));
  if (p.secuenciaEncoder == 0) {
    pantalla.setCursor(0, 20); pantalla.print(F("SIN ENSAYOS"));
    pantalla.setCursor(0, 35); pantalla.print(F("USAR AJUSTE CATCH V2"));
  } else if (p.faseCalibracionBrazo == 0) {
    pantalla.setCursor(0, 13); pantalla.print(F("PROBADO:")); pantalla.print(p.velocidadEncoderUmS); pantalla.print(F(" ms"));
    pantalla.setCursor(0, 23); pantalla.print(F("PROXIMO:")); pantalla.print(p.conteoEncoder); pantalla.print(F(" ms"));
    pantalla.setCursor(0, 33); pantalla.print(F("AGARRES:")); pantalla.print(p.estadoEncoder >> 2);
    pantalla.print(F("/3 N:")); pantalla.print(p.secuenciaEncoder);
    pantalla.setCursor(0, 43); pantalla.print((p.estadoEncoder & 2) ? F("LIMITE:REVISAR MEDIDAS") :
      ((p.estadoEncoder & 1) ? F("REPETIDO 3/3") : F("EN PRUEBA")));
  } else {
    pantalla.setCursor(0, 13); pantalla.print(F("PORTENTA.ino"));
    pantalla.setCursor(0, 23); pantalla.print(F("V2_AJUSTE_DISPARO_"));
    pantalla.setCursor(0, 33); pantalla.print(F("CATCH_MS = ")); pantalla.print(p.velocidadEncoderUmS); pantalla.print(F(";"));
    pantalla.setCursor(0, 43); pantalla.print(F("DETALLE EN TERMINAL"));
  }
  pantalla.setCursor(0, 57); pantalla.print(F("X:PAGINA TRI:SALIR"));
}

void mostrarEntrenamientoML(const PaquetePortentaAESP &p) {'''),
    ('    case SISTEMA_DIAGNOSTICO:\n      mostrarDiagnostico(p);', '    case SISTEMA_CAMBIOS_CATCH:\n      mostrarCambiosCatch(p);\n      break;\n\n    case SISTEMA_DIAGNOSTICO:\n      mostrarDiagnostico(p);')])

log = folder / 'registrar_v2.ps1'
fields = ['tested_offset_ms', 'next_offset_ms', 'adjust_step_ms', 'adjust_trials',
          'adjust_success_streak', 'adjust_confirmed', 'adjust_limit']
edit(log, [
    ("    'scale_ratio_camera_encoder', 'scale_camera_suggested_mm_count'", "    'scale_ratio_camera_encoder', 'scale_camera_suggested_mm_count',\n    " + ', '.join(repr(f) for f in fields)),
    ("    'timing_result' = 'timing_result'", "    'timing_result' = 'timing_result'\n" + '\n'.join(f"    '{f}' = '{f}'" for f in fields)),
    ('aqui dispararia el automatico; esperando X', 'referencia nominal de alineacion (300 ms)'),
    ('X inicia descenso | diferencia=', 'descenso automatico | diferencia='),
    ("' ms' }\n        'CAL_REJECT'", "' ms' }\n        'CAL_REJECT'"),
    ("' | diferencia respecto al automatico=' + (Obtener-Dato $Analizada 'delta_reference_ms') + ' ms'", "' | probado=' + (Obtener-Dato $Analizada 'tested_offset_ms') + ' ms | proximo=' + (Obtener-Dato $Analizada 'next_offset_ms') + ' ms | agarres=' + (Obtener-Dato $Analizada 'adjust_success_streak') + '/3'")])
print('Firmware, protocolo y registrador actualizados')

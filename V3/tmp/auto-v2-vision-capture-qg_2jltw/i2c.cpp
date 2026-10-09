#include <cassert>
#include <math.h>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include <sstream>
#define F(x) x
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/ESP/ProtocoloI2C.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/ESP/VisionModelo129.h"
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/pruebas de automatico v2/ESP/CalibracionAnguloMLV2.h"
using namespace ProtocoloI2C;
constexpr bool ensayoRS485=false;
constexpr int8_t CAMERA_SIGNO_Y_LOCAL = -1;
constexpr uint32_t TIMEOUT_MUESTRA_ENCODER_MS = 50;
constexpr int16_t CAMERA_ANCHO_IMAGEN_PX = 640;
constexpr int16_t CAMERA_ALTO_IMAGEN_PX = 480;
constexpr int16_t CAMERA_MARGEN_CAJA_PX = 1;
constexpr uint8_t DETECCIONES_ESTABLES_V2 = 3;
constexpr double DESPLAZAMIENTO_MINIMO_V2_MM = 2.0;
constexpr double TOLERANCIA_TRAYECTORIA_V2_MM = 6.0;
constexpr double DESPLAZAMIENTO_CAMARA_REPETIDA_MM = 8.0;
constexpr double TOLERANCIA_CAMARA_REPETIDA_MM = 2.0;
constexpr double SOLAPE_MINIMO_DUPLICADO = 0.50;
constexpr uint8_t MAX_CANDIDATOS_V2 = 10;
constexpr double RELACION_MINIMA_ORIENTACION = VisionModelo129::RELACION_MINIMA_EJE;
constexpr double BELT_WIDTH_MM = 292.0;
constexpr double TOTAL_WIDTH_MM = 412.0;
constexpr double ALUMINUM_WIDTH_MM =
  (TOTAL_WIDTH_MM - BELT_WIDTH_MM) / 2.0;
constexpr double TAG_X_FROM_CENTER_MM =
  BELT_WIDTH_MM / 2.0 + ALUMINUM_WIDTH_MM / 2.0;
constexpr double TAG_ROWS_DISTANCE_MM = 382.0;
constexpr int ANGULO_GARRA_EJE_X = 90;
constexpr int ANGULO_GARRA_EJE_Y = 0;
constexpr int ANGULO_SERVO_INICIAL = 90;
constexpr bool AUTO_V2_APLICAR_GIRO_POR_CAJA = true;
struct Point2D {
  double x;
  double y;
};
struct CandidatoPiezaV2 {
  uint8_t clase;
  int8_t confianza;
  int16_t centroXpx;
  int16_t centroYpx;
  int16_t anchoPx;
  int16_t altoPx;
  Point2D posicion;
  uint8_t grupoDuplicado;
};
struct FiltroDeteccion {
  uint8_t clase;
  uint8_t consecutivas;
  double sumaX;
  double sumaY;
  double promedioX;
  double promedioY;
  double minimoX;
  double maximoX;
  double minimoY;
  double maximoY;
  uint8_t votosEjeX;
  uint8_t votosEjeY;
  int16_t centroXpx;
  int16_t centroYpx;
  int16_t anchoPx;
  int16_t altoPx;
  int8_t confianza;
};
struct FiltroDeteccionV2 {
  uint8_t clase;
  uint8_t consecutivas;
  int32_t conteoReferencia;
  int32_t ultimoConteo;
  double sumaX;
  double sumaYCompensada;
  double promedioX;
  double promedioYCompensada;
  double minimoX;
  double maximoX;
  double minimoYCompensada;
  double maximoYCompensada;
  int8_t relacionYEncoder;
  int16_t ultimoAnchoPx;
  int16_t ultimoAltoPx;
  double ultimoXRaw;
  double ultimoYRaw;
  uint8_t votosEjeX;
  uint8_t votosEjeY;
  uint32_t primerMs;
  uint32_t ultimoMs;
  double primerYRaw;
  uint8_t muestrasYDistintas;
};
struct ContextoCamara {
  uint8_t estado;
  uint8_t error;
  uint8_t ackComando;
  uint8_t ultimaSecuenciaComando;
  bool conectada;
  bool homografiaValida;
  bool modeloListo;
  bool operacionEstadoIniciada;
  bool recuperarAutomaticamente;
  bool solicitarCalibracion;
  bool solicitarModelo;
  bool portentaEstabaActiva;
  bool autoEstabaActivo;
  uint32_t entradaEstado;
  uint32_t plazoEstado;
  uint32_t proximaConexion;
  uint32_t proximaLectura;
  uint32_t inicioCalibracion;
  uint32_t ultimoEstadoSerial;

  FiltroDeteccion filtro;
  FiltroDeteccionV2 filtroV2;
  uint8_t ausenciasFiltroV2;
  bool rearmada;
  bool esperandoDesaparicion;
  uint32_t inicioAusencia;
  uint8_t claseEsperandoDesaparicion;
  double xEsperandoDesaparicion;
  double yEsperandoDesaparicion;

  bool objetivoValido;
  uint8_t claseObjetivo;
  int16_t objetivoX10;
  int16_t objetivoY10;
  uint16_t secuenciaObjetivo;
  bool objetivoV2;
  int32_t conteoReferenciaObjetivo;
  int16_t sugerenciaAngulo;
  bool pruebaEncoderAnterior;
  uint8_t flagsObjetivoV2;
  uint32_t consultaReferenciaObjetivoMs;
};
struct ControlCamaraCompartido {
  bool portentaActiva;
  bool automaticoActivo;
  bool automaticoV2Activo;
  bool registrarAngulo;
  bool orientarGarraV2;
  bool entrenamientoML;
  bool entrenamientoMLV2;
  bool pruebaSeguimiento;
  bool ajusteCatchV2;
  bool pruebaEncoderActiva;
  bool brazoOcupado;
  uint8_t comando;
  uint8_t secuenciaComando;
  uint16_t ackObjetivo;
  uint8_t codigoAckObjetivo;
};
struct EstadoEncoderCompartido {
  int32_t conteo;
  float velocidadMmS;
  uint32_t nmPorCuenta;
  uint16_t secuencia;
  uint8_t flags;
  int8_t signo;
  uint32_t recibidoMs;
};
enum CausaDiagnosticoV2 : uint8_t {
  V2_DIAG_SIN_BLOQUEO = 0,
  V2_DIAG_INACTIVO,
  V2_DIAG_BRAZO_OCUPADO,
  V2_DIAG_OBJETIVO_ACTIVO,
  V2_DIAG_NO_REARMADO,
  V2_DIAG_ESPERANDO_DESAPARICION,
  V2_DIAG_ENCODER_NO_RECIBIDO,
  V2_DIAG_ENCODER_ANTIGUO,
  V2_DIAG_ENCODER_HW,
  V2_DIAG_ENCODER_ESCALA,
  V2_DIAG_ENCODER_SIN_PULSOS,
  V2_DIAG_ENCODER_SATURADO,
  V2_DIAG_ENCODER_SIGNO,
  V2_DIAG_HUSKY_ERROR,
  V2_DIAG_SIN_RESULTADOS,
  V2_DIAG_HOMOGRAFIA,
  V2_DIAG_FUERA_BANDA,
  V2_DIAG_SIN_PIEZA,
  V2_DIAG_CAMARA_REPETIDA,
  V2_DIAG_TRAYECTORIA,
  V2_DIAG_DETECCIONES,
  V2_DIAG_DESPLAZAMIENTO,
  V2_DIAG_OBJETIVO_LISTO,
  V2_DIAG_PUBLICADO,
  V2_DIAG_CAJA_RECORTADA,
  V2_DIAG_MULTIPLES_PIEZAS,
  V2_DIAG_ORIENTACION_AMBIGUA,
  V2_DIAG_REFERENCIA_CAMARA_INCIERTA
};
struct DiagnosticoDeteccionV2 {
  uint8_t causa;
  uint32_t actualizadoMs;
  uint32_t edadEncoderMs;
  int32_t conteoEncoder;
  float velocidadEncoderMmS;
  uint8_t flagsEncoder;
  int8_t signoEncoder;
  int8_t resultadosHusky;
  uint8_t candidatosValidos;
  uint8_t rechazadosHomografia;
  uint8_t rechazadosFueraBanda;
  uint8_t clase;
  uint8_t deteccionesConsecutivas;
  double xMm;
  double yMm;
  double yCompensadaMm;
  double dispersionXmm;
  double dispersionYCompensadaMm;
  double desplazamientoMm;
  uint16_t secuenciaPublicada;
  uint8_t candidatosUnicos;
  uint8_t duplicadosDescartados;
  uint32_t duracionConsultaMs;
  int32_t conteoAntesConsulta;
  int32_t conteoDespuesConsulta;
  int32_t conteoAsociadoCamara;
  double recorridoDuranteConsultaMm;
  int8_t relacionYEncoder;
  uint8_t rechazadosCajaRecortada;
  uint8_t votosEjeX;
  uint8_t votosEjeY;
  uint8_t flagsObjetivoV2;
  uint32_t edadConsultaReferenciaObjetivoMs;
  bool referenciaEntreMuestras;
};

unsigned long now=1000;
uint32_t millis() { return now; }
struct Terminal {
  std::string log;
  template<class T> void print(const T& v, int=0) { std::ostringstream s; s<<v; log+=s.str(); }
  void print(uint8_t v, int=0) { print(static_cast<int>(v)); }
  template<class T> void println(const T& v, int=0) { print(v); log+='\n'; }
  void println() { log+='\n'; }
  void setCursor(int,int) {}
} Serial;
struct Servo { int writes=0,last=-1; void write(int a) { ++writes;last=a; } } servoRotacion;
int anguloServoRotacion=88;
uint16_t sesionArranque=1;
uint32_t consultasLogV2=0,ultimoReporteCandidatosV2=0;
constexpr uint8_t COMMAND_RETURN_BLOCK=1;
constexpr int PIECE_MODEL=129;
double H[3][3]={{1,0,-320},{0,1,-240},{0,0,1}};
EstadoEncoderCompartido muestra;
EstadoEncoderCompartido copiarEstadoEncoder() { return muestra; }
struct Result { std::string name="pieza6"; int type=1,confidence=-128,xCenter=320,yCenter=240,width=20,height=60; };
struct Husky {
  std::vector<Result> results;
  size_t at=0;
  bool restoreFlags=false;
  int8_t getResult(int) { at=0; now+=5; muestra.recibidoMs=now;
    if(restoreFlags) muestra.flags=ENC_FLAG_HW_LISTO|ENC_FLAG_ESCALA_VALIDA|ENC_FLAG_PULSOS_VISTOS;
    return results.size(); }
  bool available(int) { return at<results.size(); }
  Result* popCachedResult(int) { return &results[at++]; }
} huskylens;
DiagnosticoDeteccionV2 ultimoDiag;
DiagnosticoDeteccionV2 copiarDiagnosticoV2() { return ultimoDiag; }
struct EstadoCamaraMock { bool objetivoValido=false; };
EstadoCamaraMock copiarEstadoCamara() { return {}; }
Terminal pantalla;
std::string titulo;
void dibujarTitulo(const char* t) { titulo=t;pantalla.log.clear(); }
void publicarCausaDiagnosticoV2(DiagnosticoDeteccionV2& d,uint8_t causa) { d.causa=causa;ultimoDiag=d; }
void publicarYRegistrarDiagnosticoV2(DiagnosticoDeteccionV2& d,uint8_t causa,
  const EstadoEncoderCompartido&,uint32_t,uint8_t=UINT8_MAX) { publicarCausaDiagnosticoV2(d,causa); }
void registrarConsultaLogV2(const DiagnosticoDeteccionV2& d,const EstadoEncoderCompartido&,uint32_t,uint8_t) { ultimoDiag=d; }
void registrarCandidatoLogV2(uint32_t,const EstadoEncoderCompartido&,const CandidatoPiezaV2&,uint8_t,uint8_t) {}
int16_t sugerirAnguloPorVotos(uint8_t x,uint8_t y) { return x>=2&&y==0?90:y>=2&&x==0?0:-1; }
bool paquetePortentaSemanticamenteValido(const PaquetePortentaAESP &p) {
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
}
const char *nombreCortoDiagnosticoV2(uint8_t causa) {
  switch (causa) {
    case V2_DIAG_INACTIVO: return "INACTIVO";
    case V2_DIAG_BRAZO_OCUPADO: return "BRAZO OCUP.";
    case V2_DIAG_OBJETIVO_ACTIVO: return "OBJ ACTIVO";
    case V2_DIAG_NO_REARMADO: return "NO REARMADO";
    case V2_DIAG_ESPERANDO_DESAPARICION: return "ESPERA DESAP.";
    case V2_DIAG_ENCODER_NO_RECIBIDO: return "ENC SIN RX";
    case V2_DIAG_ENCODER_ANTIGUO: return "ENC ANTIGUO";
    case V2_DIAG_ENCODER_HW: return "ENC HW";
    case V2_DIAG_ENCODER_ESCALA: return "ENC ESCALA";
    case V2_DIAG_ENCODER_SIN_PULSOS: return "ENC SIN PULSO";
    case V2_DIAG_ENCODER_SATURADO: return "ENC SATURADO";
    case V2_DIAG_ENCODER_SIGNO: return "ENC SIGNO";
    case V2_DIAG_HUSKY_ERROR: return "HUSKY ERROR";
    case V2_DIAG_SIN_RESULTADOS: return "SIN RESULTADO";
    case V2_DIAG_HOMOGRAFIA: return "HOMOGRAFIA";
    case V2_DIAG_FUERA_BANDA: return "FUERA BANDA";
    case V2_DIAG_SIN_PIEZA: return "SIN PIEZA";
    case V2_DIAG_CAMARA_REPETIDA: return "CAM REPETIDA";
    case V2_DIAG_TRAYECTORIA: return "TRAYECTORIA";
    case V2_DIAG_DETECCIONES: return "DETECCIONES";
    case V2_DIAG_DESPLAZAMIENTO: return "DESPLAZAM.";
    case V2_DIAG_OBJETIVO_LISTO: return "OBJ LISTO";
    case V2_DIAG_PUBLICADO: return "PUBLICADO";
    case V2_DIAG_CAJA_RECORTADA: return "CAJA RECORT.";
    case V2_DIAG_MULTIPLES_PIEZAS: return "PIEZAS AMBIG.";
    case V2_DIAG_ORIENTACION_AMBIGUA: return "GIRO AMBIGUO";
    case V2_DIAG_REFERENCIA_CAMARA_INCIERTA: return "REF CAM INCIERTA";
    default: return "---";
  }
}
float escalaEncoderMm(const EstadoEncoderCompartido &encoder) {
  return static_cast<float>(encoder.nmPorCuenta) / 1000000.0f;
}
bool encoderRemotoVigente(const EstadoEncoderCompartido &encoder) {
  const uint8_t requeridos = ENC_FLAG_HW_LISTO |
                             ENC_FLAG_ESCALA_VALIDA |
                             ENC_FLAG_PULSOS_VISTOS;
  return encoder.recibidoMs != 0 &&
         millis() - encoder.recibidoMs <= TIMEOUT_MUESTRA_ENCODER_MS &&
         (encoder.flags & requeridos) == requeridos &&
         (encoder.flags & ENC_FLAG_SATURADO) == 0 &&
         encoder.nmPorCuenta != 0 &&
         (encoder.signo == 1 || encoder.signo == -1);
}
uint8_t diagnosticarEncoderV2(const EstadoEncoderCompartido &encoder) {
  if (encoder.recibidoMs == 0) return V2_DIAG_ENCODER_NO_RECIBIDO;
  if (millis() - encoder.recibidoMs > TIMEOUT_MUESTRA_ENCODER_MS)
    return V2_DIAG_ENCODER_ANTIGUO;
  if ((encoder.flags & ENC_FLAG_HW_LISTO) == 0) return V2_DIAG_ENCODER_HW;
  if ((encoder.flags & ENC_FLAG_ESCALA_VALIDA) == 0 || encoder.nmPorCuenta == 0)
    return V2_DIAG_ENCODER_ESCALA;
  if ((encoder.flags & ENC_FLAG_PULSOS_VISTOS) == 0)
    return V2_DIAG_ENCODER_SIN_PULSOS;
  if ((encoder.flags & ENC_FLAG_SATURADO) != 0)
    return V2_DIAG_ENCODER_SATURADO;
  if (encoder.signo != 1 && encoder.signo != -1)
    return V2_DIAG_ENCODER_SIGNO;
  return V2_DIAG_SIN_BLOQUEO;
}
bool pixelToMillimeters(
  double u,
  double v,
  bool homographyValid,
  Point2D &physicalPoint
) {
  if (!homographyValid) {
    return false;
  }

  const double denominator = H[2][0] * u + H[2][1] * v + H[2][2];
  if (fabs(denominator) < 1e-12 || !isfinite(denominator)) {
    return false;
  }

  physicalPoint.x =
    (H[0][0] * u + H[0][1] * v + H[0][2]) / denominator;
  physicalPoint.y =
    (H[1][0] * u + H[1][1] * v + H[1][2]) / denominator;
  return isfinite(physicalPoint.x) && isfinite(physicalPoint.y);
}
uint8_t estimarEjeCajaV2(const CandidatoPiezaV2 &candidato) {
  if (candidato.anchoPx <= 0 || candidato.altoPx <= 0) return 0;
  const double izquierda = candidato.centroXpx - candidato.anchoPx * 0.5;
  const double derecha = candidato.centroXpx + candidato.anchoPx * 0.5;
  const double arriba = candidato.centroYpx - candidato.altoPx * 0.5;
  const double abajo = candidato.centroYpx + candidato.altoPx * 0.5;
  Point2D esquinas[4];
  if (!pixelToMillimeters(izquierda, arriba, true, esquinas[0]) ||
      !pixelToMillimeters(derecha, arriba, true, esquinas[1]) ||
      !pixelToMillimeters(izquierda, abajo, true, esquinas[2]) ||
      !pixelToMillimeters(derecha, abajo, true, esquinas[3])) return 0;
  double minX = esquinas[0].x, maxX = minX;
  double minY = esquinas[0].y, maxY = minY;
  for (uint8_t i = 1; i < 4; ++i) {
    minX = fmin(minX, esquinas[i].x);
    maxX = fmax(maxX, esquinas[i].x);
    minY = fmin(minY, esquinas[i].y);
    maxY = fmax(maxY, esquinas[i].y);
  }
  const double anchoMm = maxX - minX;
  const double altoMm = maxY - minY;
  return VisionModelo129::ejePorDimensiones(
    anchoMm, altoMm, RELACION_MINIMA_ORIENTACION);
}
bool isInsideCalibrationArea(const Point2D &position) {
  const double halfCalibrationHeight = TAG_ROWS_DISTANCE_MM / 2.0;
  return
    position.x >= -TAG_X_FROM_CENTER_MM &&
    position.x <=  TAG_X_FROM_CENTER_MM &&
    position.y >= -halfCalibrationHeight &&
    position.y <=  halfCalibrationHeight;
}
bool isOverWhiteBelt(const Point2D &position) {
  return
    fabs(position.x) <= BELT_WIDTH_MM / 2.0 &&
    isInsideCalibrationArea(position);
}
bool cajaIntegraAutonomaV2(const CandidatoPiezaV2 &candidato) {
  if (candidato.anchoPx <= 0 || candidato.altoPx <= 0) return false;
  const double izquierda = candidato.centroXpx - candidato.anchoPx * 0.5;
  const double derecha = candidato.centroXpx + candidato.anchoPx * 0.5;
  const double arriba = candidato.centroYpx - candidato.altoPx * 0.5;
  const double abajo = candidato.centroYpx + candidato.altoPx * 0.5;
  if (izquierda <= CAMERA_MARGEN_CAJA_PX || arriba <= CAMERA_MARGEN_CAJA_PX ||
      derecha >= CAMERA_ANCHO_IMAGEN_PX - CAMERA_MARGEN_CAJA_PX ||
      abajo >= CAMERA_ALTO_IMAGEN_PX - CAMERA_MARGEN_CAJA_PX) return false;
  // No estimar eje/centro con una caja que requiere extrapolar la homografia.
  const double u[4] = {izquierda, derecha, izquierda, derecha};
  const double v[4] = {arriba, arriba, abajo, abajo};
  for (uint8_t i = 0; i < 4; ++i) {
    Point2D esquina;
    if (!pixelToMillimeters(u[i], v[i], true, esquina) ||
        !isInsideCalibrationArea(esquina)) return false;
  }
  return true;
}
bool capturaAutonomaV2(const ControlCamaraCompartido &control) {
  // Las correcciones humanas y ensayos mantienen su politica previa.
  return control.automaticoV2Activo && control.orientarGarraV2 &&
         !control.ajusteCatchV2 && !control.entrenamientoML &&
         !control.pruebaEncoderActiva;
}
void orientarGarraAutomatica(uint8_t votosX, uint8_t votosY, bool modoV2,
                            uint8_t clase = 0, uint16_t secuencia = 0) {
  if (modoV2 && !AUTO_V2_APLICAR_GIRO_POR_CAJA) {
    Serial.print(F("[AUTO V2] Giro por caja desactivado; conserva servo_deg="));
    Serial.print(anguloServoRotacion);
    Serial.print(F(" votos_x="));
    Serial.print(votosX);
    Serial.print(F(" votos_y="));
    Serial.println(votosY);
    return;
  }
  if (modoV2) {
    const int16_t sugerencia = CalibracionAnguloMLV2::sugerir(clase, votosX, votosY);
    if (sugerencia >= 0) {
      anguloServoRotacion = sugerencia;
      servoRotacion.write(anguloServoRotacion);
    }
    Serial.print(F("V2LOG|E|event=AUTO_ANGLE_SUGGESTION|session="));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(millis());
    Serial.print(F("|obj=")); Serial.print(secuencia);
    Serial.print(F("|class=")); Serial.print(clase);
    Serial.print(F("|suggested_rot="));
    if (sugerencia >= 0) Serial.print(sugerencia);
    else Serial.print(F("NA"));
    Serial.print(F("|suggestion_source=MLV2_EJES_20261005|votes_x="));
    Serial.print(votosX);
    Serial.print(F("|votes_y=")); Serial.print(votosY);
    Serial.print(F("|servo_rot_deg=")); Serial.println(anguloServoRotacion);
    return;
  }
  const bool ejeX = votosX >= 2 && votosY == 0;
  const bool ejeY = votosY >= 2 && votosX == 0;
  anguloServoRotacion = ejeX ? ANGULO_GARRA_EJE_X :
    (ejeY ? ANGULO_GARRA_EJE_Y : ANGULO_SERVO_INICIAL);
  servoRotacion.write(anguloServoRotacion);
  Serial.print(modoV2 ? F("[AUTO V2]") : F("[AUTO]"));
  Serial.print(F(" Orientacion aprox eje="));
  Serial.print(ejeX ? F("X") : (ejeY ? F("Y") : F("INDETERMINADO")));
  Serial.print(F(" votos_x="));
  Serial.print(votosX);
  Serial.print(F(" votos_y="));
  Serial.print(votosY);
  Serial.print(F(" servo_deg="));
  Serial.println(anguloServoRotacion);
}
void reiniciarFiltroV2(ContextoCamara &ctx) {
  ctx.filtroV2 = {
    0, 0, 0, 0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0.0, 0.0,
    0, 0, 0, 0.0, 0.0, 0, 0
  };
}
double desplazamientoEncoderMm(
  int32_t conteo,
  int32_t referencia,
  const EstadoEncoderCompartido &encoder
) {
  return static_cast<double>(encoder.signo) *
         static_cast<double>(diferenciaConteosConWrap(conteo, referencia)) *
         static_cast<double>(escalaEncoderMm(encoder));
}
double compensarYConEncoder(
  double y,
  int32_t conteo,
  int32_t referencia,
  const EstadoEncoderCompartido &encoder,
  int8_t relacionYEncoder
) {
  const int8_t relacion = relacionYEncoder == 0
    ? static_cast<int8_t>(1 / CAMERA_SIGNO_Y_LOCAL)
    : relacionYEncoder;
  return y - static_cast<double>(relacion) *
             desplazamientoEncoderMm(conteo, referencia, encoder);
}
int32_t conteoMedioConWrap(int32_t antes, int32_t despues) {
  return static_cast<int32_t>(
    static_cast<uint32_t>(antes) +
    static_cast<uint32_t>(diferenciaConteosConWrap(despues, antes) / 2)
  );
}
double areaCajaV2(const CandidatoPiezaV2 &candidato) {
  return static_cast<double>(abs(candidato.anchoPx)) *
         static_cast<double>(abs(candidato.altoPx));
}
bool cajasDuplicadasV2(
  const CandidatoPiezaV2 &a,
  const CandidatoPiezaV2 &b
) {
  if (a.clase != b.clase) return false;
  const double anchoA = fabs(static_cast<double>(a.anchoPx));
  const double altoA = fabs(static_cast<double>(a.altoPx));
  const double anchoB = fabs(static_cast<double>(b.anchoPx));
  const double altoB = fabs(static_cast<double>(b.altoPx));
  if (anchoA < 1.0 || altoA < 1.0 || anchoB < 1.0 || altoB < 1.0) {
    return false;
  }

  const double izquierda = fmax(
    a.centroXpx - anchoA * 0.5, b.centroXpx - anchoB * 0.5
  );
  const double derecha = fmin(
    a.centroXpx + anchoA * 0.5, b.centroXpx + anchoB * 0.5
  );
  const double arriba = fmax(
    a.centroYpx - altoA * 0.5, b.centroYpx - altoB * 0.5
  );
  const double abajo = fmin(
    a.centroYpx + altoA * 0.5, b.centroYpx + altoB * 0.5
  );
  const double interseccion = fmax(0.0, derecha - izquierda) *
                              fmax(0.0, abajo - arriba);
  const double areaMenor = fmin(anchoA * altoA, anchoB * altoB);
  if (areaMenor > 0.0 && interseccion / areaMenor >= SOLAPE_MINIMO_DUPLICADO) {
    return true;
  }

  const double dx = a.centroXpx - b.centroXpx;
  const double dy = a.centroYpx - b.centroYpx;
  const double radio = 0.25 * fmin(
    fmax(anchoA, altoA), fmax(anchoB, altoB)
  );
  return dx * dx + dy * dy <= radio * radio;
}
bool candidatoV2MejorQue(
  const CandidatoPiezaV2 &nuevo,
  const CandidatoPiezaV2 &actual
) {
  if (nuevo.confianza != actual.confianza) {
    return nuevo.confianza > actual.confianza;
  }
  return areaCajaV2(nuevo) > areaCajaV2(actual);
}
DiagnosticoDeteccionV2 crearDiagnosticoBaseV2(
  const EstadoEncoderCompartido &encoder,
  int8_t resultadosHusky
) {
  DiagnosticoDeteccionV2 diag = {};
  diag.causa = V2_DIAG_SIN_BLOQUEO;
  diag.actualizadoMs = millis();
  diag.edadEncoderMs = encoder.recibidoMs == 0
    ? UINT32_MAX : diag.actualizadoMs - encoder.recibidoMs;
  diag.conteoEncoder = encoder.conteo;
  diag.velocidadEncoderMmS = encoder.velocidadMmS;
  diag.flagsEncoder = encoder.flags;
  diag.signoEncoder = encoder.signo;
  diag.resultadosHusky = resultadosHusky;
  return diag;
}
void completarMetricasFiltroV2(
  DiagnosticoDeteccionV2 &diag,
  const FiltroDeteccionV2 &filtro,
  int32_t conteoActual,
  const EstadoEncoderCompartido &encoder
) {
  diag.clase = filtro.clase;
  diag.deteccionesConsecutivas = filtro.consecutivas;
  if (filtro.consecutivas == 0) return;
  diag.dispersionXmm = filtro.maximoX - filtro.minimoX;
  diag.dispersionYCompensadaMm =
    filtro.maximoYCompensada - filtro.minimoYCompensada;
  diag.desplazamientoMm = fabs(
    static_cast<double>(diferenciaConteosConWrap(
      conteoActual, filtro.conteoReferencia
    )) * static_cast<double>(escalaEncoderMm(encoder))
  );
}
int8_t elegirRelacionYEncoderV2(
  const FiltroDeteccionV2 &filtro,
  const Point2D &posicion,
  int32_t conteo,
  const EstadoEncoderCompartido &encoder
) {
  if (filtro.relacionYEncoder == 1 || filtro.relacionYEncoder == -1) {
    return filtro.relacionYEncoder;
  }
  const double yPositiva = compensarYConEncoder(
    posicion.y, conteo, filtro.conteoReferencia, encoder, 1
  );
  const double yNegativa = compensarYConEncoder(
    posicion.y, conteo, filtro.conteoReferencia, encoder, -1
  );
  return fabs(yPositiva - filtro.promedioYCompensada) <=
         fabs(yNegativa - filtro.promedioYCompensada) ? 1 : -1;
}
bool tamanoCompatibleV2(
  const FiltroDeteccionV2 &filtro,
  const CandidatoPiezaV2 &candidato
) {
  if (filtro.ultimoAnchoPx <= 0 || filtro.ultimoAltoPx <= 0 ||
      candidato.anchoPx <= 0 || candidato.altoPx <= 0) {
    return true;
  }
  const double areaAnterior = static_cast<double>(filtro.ultimoAnchoPx) *
                              static_cast<double>(filtro.ultimoAltoPx);
  const double areaActual = areaCajaV2(candidato);
  const double relacionArea = areaActual / areaAnterior;
  return relacionArea >= 0.35 && relacionArea <= 2.85;
}
bool mismaTrayectoriaV2(
  const FiltroDeteccionV2 &filtro,
  const CandidatoPiezaV2 &candidato,
  int32_t conteo,
  const EstadoEncoderCompartido &encoder
) {
  if (filtro.consecutivas == 0 || filtro.clase != candidato.clase ||
      !tamanoCompatibleV2(filtro, candidato)) return false;
  const int8_t relacion = elegirRelacionYEncoderV2(
    filtro, candidato.posicion, conteo, encoder
  );
  const double yCompensada = compensarYConEncoder(
    candidato.posicion.y,
    conteo,
    filtro.conteoReferencia,
    encoder,
    relacion
  );
  const double nuevoMinX = fmin(filtro.minimoX, candidato.posicion.x);
  const double nuevoMaxX = fmax(filtro.maximoX, candidato.posicion.x);
  const double nuevoMinY = fmin(filtro.minimoYCompensada, yCompensada);
  const double nuevoMaxY = fmax(filtro.maximoYCompensada, yCompensada);
  return nuevoMaxX - nuevoMinX <= TOLERANCIA_TRAYECTORIA_V2_MM &&
         nuevoMaxY - nuevoMinY <= TOLERANCIA_TRAYECTORIA_V2_MM;
}
void incorporarDeteccionV2(
  ContextoCamara &ctx,
  const CandidatoPiezaV2 &candidato,
  int32_t conteo,
  const EstadoEncoderCompartido &encoder,
  uint32_t instanteMs
) {
  FiltroDeteccionV2 &filtro = ctx.filtroV2;
  const uint8_t ejeCaja = estimarEjeCajaV2(candidato);
  if (!mismaTrayectoriaV2(filtro, candidato, conteo, encoder)) {
    filtro = {
      candidato.clase, 1, conteo, conteo,
      candidato.posicion.x, candidato.posicion.y,
      candidato.posicion.x, candidato.posicion.y,
      candidato.posicion.x, candidato.posicion.x,
      candidato.posicion.y, candidato.posicion.y,
      0, candidato.anchoPx, candidato.altoPx,
      candidato.posicion.x, candidato.posicion.y,
      static_cast<uint8_t>(ejeCaja == 1),
      static_cast<uint8_t>(ejeCaja == 2),
      instanteMs, instanteMs, candidato.posicion.y, 1U
    };
    ctx.ausenciasFiltroV2 = 0;
    return;
  }

  const int8_t relacion = elegirRelacionYEncoderV2(
    filtro, candidato.posicion, conteo, encoder
  );
  const double yCompensada = compensarYConEncoder(
    candidato.posicion.y,
    conteo,
    filtro.conteoReferencia,
    encoder,
    relacion
  );
  filtro.relacionYEncoder = relacion;
  if (filtro.consecutivas < UINT8_MAX) ++filtro.consecutivas;
  if (ejeCaja == 1 && filtro.votosEjeX < UINT8_MAX) ++filtro.votosEjeX;
  if (ejeCaja == 2 && filtro.votosEjeY < UINT8_MAX) ++filtro.votosEjeY;
  filtro.ultimoConteo = conteo;
  filtro.sumaX += candidato.posicion.x;
  filtro.sumaYCompensada += yCompensada;
  filtro.promedioX = filtro.sumaX / filtro.consecutivas;
  filtro.promedioYCompensada = filtro.sumaYCompensada / filtro.consecutivas;
  filtro.minimoX = fmin(filtro.minimoX, candidato.posicion.x);
  filtro.maximoX = fmax(filtro.maximoX, candidato.posicion.x);
  filtro.minimoYCompensada = fmin(filtro.minimoYCompensada, yCompensada);
  filtro.maximoYCompensada = fmax(filtro.maximoYCompensada, yCompensada);
  filtro.ultimoAnchoPx = candidato.anchoPx;
  filtro.ultimoAltoPx = candidato.altoPx;
  if (fabs(candidato.posicion.y - filtro.ultimoYRaw) >= 0.5 &&
      filtro.muestrasYDistintas < UINT8_MAX) {
    ++filtro.muestrasYDistintas;
  }
  filtro.ultimoXRaw = candidato.posicion.x;
  filtro.ultimoYRaw = candidato.posicion.y;
  filtro.ultimoMs = instanteMs;
  ctx.ausenciasFiltroV2 = 0;
}
bool publicarObjetivoV2(
  ContextoCamara &ctx,
  int32_t conteoActual,
  const EstadoEncoderCompartido &encoder,
  bool orientarGarra,
  bool entrenamientoML,
  bool entrenamientoMLV2,
  bool pruebaSeguimiento,
  bool ajusteCatch,
  bool capturaAutonoma = false
) {
  FiltroDeteccionV2 &filtro = ctx.filtroV2;
  if (filtro.consecutivas < DETECCIONES_ESTABLES_V2) return false;
  if (capturaAutonoma && (!orientarGarra || !AUTO_V2_APLICAR_GIRO_POR_CAJA ||
      CalibracionAnguloMLV2::sugerir(filtro.clase, filtro.votosEjeX,
                                  filtro.votosEjeY) < 0)) return false;
  const double desplazamiento = fabs(
    static_cast<double>(diferenciaConteosConWrap(
      conteoActual,
      filtro.conteoReferencia
    )) * static_cast<double>(escalaEncoderMm(encoder))
  );
  if (desplazamiento < DESPLAZAMIENTO_MINIMO_V2_MM) return false;

  const int8_t relacion = filtro.relacionYEncoder == 0
    ? static_cast<int8_t>(1 / CAMERA_SIGNO_Y_LOCAL)
    : filtro.relacionYEncoder;
  const double yActual = filtro.promedioYCompensada +
    static_cast<double>(relacion) * desplazamientoEncoderMm(
      conteoActual, filtro.conteoReferencia, encoder
    );
  const long x10 = lround(filtro.promedioX * 10.0);
  const long y10 = lround(yActual * 10.0);
  if (x10 < INT16_MIN || x10 > INT16_MAX ||
      y10 < INT16_MIN || y10 > INT16_MAX) {
    reiniciarFiltroV2(ctx);
    return false;
  }

  ++ctx.secuenciaObjetivo;
  if (ctx.secuenciaObjetivo == 0) ++ctx.secuenciaObjetivo;
  ctx.objetivoValido = true;
  ctx.objetivoV2 = true;
  ctx.claseObjetivo = filtro.clase;
  ctx.objetivoX10 = static_cast<int16_t>(x10);
  ctx.objetivoY10 = static_cast<int16_t>(y10);
  ctx.conteoReferenciaObjetivo = conteoActual;
  ctx.flagsObjetivoV2 = capturaAutonoma
    ? OBJ_V2_REFERENCIA_APROXIMADA | OBJ_V2_ORIENTACION_AXIAL | OBJ_V2_GIRO_APLICADO
    : 0;
  ctx.consultaReferenciaObjetivoMs = filtro.ultimoMs;
  ctx.rearmada = false;
  if (entrenamientoMLV2 || ajusteCatch || capturaAutonoma) {
    const uint32_t intervaloMs = filtro.ultimoMs - filtro.primerMs;
    const double deltaCamaraY = filtro.ultimoYRaw - filtro.primerYRaw;
    const double recorridoCamaraMm = fabs(deltaCamaraY);
    const double recorridoEncoderMm = fabs(desplazamientoEncoderMm(
      filtro.ultimoConteo, filtro.conteoReferencia, encoder));
    const bool velocidadCamaraValida = intervaloMs >= 80U &&
      filtro.muestrasYDistintas >= 3U &&
      recorridoCamaraMm >= 2.0 && recorridoEncoderMm >= 2.0;
    const double velocidadCamaraMmS = velocidadCamaraValida
      ? 1000.0 * recorridoCamaraMm / intervaloMs : NAN;
    const double velocidadEncoderVentanaMmS = intervaloMs > 0
      ? 1000.0 * recorridoEncoderMm / intervaloMs : NAN;
    Serial.print(capturaAutonoma
      ? F("V2LOG|E|mode=V2|event=CAMERA_SPEED|session=")
      : (ajusteCatch
      ? F("V2LOG|E|mode=CATCH_CAL|event=CAMERA_SPEED|session=")
      : (pruebaSeguimiento
      ? F("V2LOG|E|mode=ML_TRACK|event=CAMERA_SPEED|session=")
      : F("V2LOG|E|mode=ML_V2|event=CAMERA_SPEED|session="))));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(filtro.ultimoMs);
    Serial.print(F("|obj=")); Serial.print(ctx.secuenciaObjetivo);
    Serial.print(F("|camera_speed_valid="));
    Serial.print(velocidadCamaraValida ? 1 : 0);
    Serial.print(F("|camera_speed_mm_s="));
    Serial.print(velocidadCamaraMmS, 3);
    Serial.print(F("|camera_vy_mm_s="));
    Serial.print(intervaloMs > 0
      ? 1000.0 * deltaCamaraY / intervaloMs : NAN, 3);
    Serial.print(F("|encoder_window_mm_s="));
    Serial.print(velocidadEncoderVentanaMmS, 3);
    Serial.print(F("|encoder_velocity_mm_s="));
    Serial.print(encoder.velocidadMmS, 3);
    Serial.print(F("|speed_error_mm_s="));
    Serial.print(velocidadCamaraValida
      ? velocidadCamaraMmS - velocidadEncoderVentanaMmS : NAN, 3);
    Serial.print(F("|camera_travel_mm="));
    Serial.print(recorridoCamaraMm, 3);
    Serial.print(F("|encoder_travel_mm="));
    Serial.print(recorridoEncoderMm, 3);
    Serial.print(F("|camera_interval_ms="));
    Serial.print(intervaloMs);
    Serial.print(F("|camera_unique_y="));
    Serial.print(filtro.muestrasYDistintas);
    Serial.print(F("|camera_y_first_mm="));
    Serial.print(filtro.primerYRaw, 3);
    Serial.print(F("|camera_y_last_mm="));
    Serial.print(filtro.ultimoYRaw, 3);
    if (ajusteCatch) {
      const double relacionEscala = velocidadCamaraValida
        ? recorridoCamaraMm / recorridoEncoderMm : NAN;
      Serial.print(F("|scale_ratio_camera_encoder=")); Serial.print(relacionEscala, 6);
      Serial.print(F("|scale_camera_suggested_mm_count="));
      Serial.print(velocidadCamaraValida
        ? escalaEncoderMm(encoder) * relacionEscala : NAN, 7);
    }
    Serial.println();
  }
  const bool usarCalibracionEjes = entrenamientoML &&
    (entrenamientoMLV2 || pruebaSeguimiento);
  ctx.sugerenciaAngulo = !entrenamientoML ? -1 :
    (usarCalibracionEjes
      ? CalibracionAnguloMLV2::sugerir(
          filtro.clase, filtro.votosEjeX, filtro.votosEjeY)
      : sugerirAnguloPorVotos(filtro.votosEjeX, filtro.votosEjeY));
  if (entrenamientoML) {
    if (ctx.sugerenciaAngulo >= 0) {
      anguloServoRotacion = ctx.sugerenciaAngulo;
      servoRotacion.write(anguloServoRotacion);
    }
    Serial.print(pruebaSeguimiento
      ? F("V2LOG|E|mode=ML_TRACK|event=ML_ANGLE_SUGGESTION|session=")
      : (entrenamientoMLV2
          ? F("V2LOG|E|mode=ML_V2|event=ML_ANGLE_SUGGESTION|session=")
          : F("V2LOG|E|mode=ML|event=ML_ANGLE_SUGGESTION|session=")));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(millis());
    Serial.print(F("|obj=")); Serial.print(ctx.secuenciaObjetivo);
    Serial.print(F("|class=")); Serial.print(filtro.clase);
    Serial.print(F("|width=")); Serial.print(filtro.ultimoAnchoPx);
    Serial.print(F("|height=")); Serial.print(filtro.ultimoAltoPx);
    Serial.print(F("|suggested_rot="));
    if (ctx.sugerenciaAngulo >= 0) Serial.print(ctx.sugerenciaAngulo);
    else Serial.print(F("NA"));
    Serial.print(usarCalibracionEjes
      ? F("|suggestion_source=MLV2_EJES_20261005")
      : F("|suggestion_source=MODEL129_BOX_AXIS_MM"));
    Serial.print(F("|votes_x=")); Serial.print(filtro.votosEjeX);
    Serial.print(F("|votes_y=")); Serial.print(filtro.votosEjeY);
    Serial.print(F("|servo_rot_deg="));
    Serial.println(anguloServoRotacion);
  }
  if (orientarGarra) {
    ctx.sugerenciaAngulo = AUTO_V2_APLICAR_GIRO_POR_CAJA
      ? CalibracionAnguloMLV2::sugerir(filtro.clase, filtro.votosEjeX, filtro.votosEjeY)
      : -1;
    orientarGarraAutomatica(filtro.votosEjeX, filtro.votosEjeY, true,
                            filtro.clase, ctx.secuenciaObjetivo);
  }
  if (capturaAutonoma) {
    // El eje que devuelve la caja es el lado largo. Los valores aprendidos
    // corresponden al cierre por el lado menor; no sumar 90 grados al servo.
    Serial.print(F("V2LOG|E|event=OBJECTIVE_REFERENCE|mode=V2|session="));
    Serial.print(sesionArranque);
    Serial.print(F("|ms=")); Serial.print(millis());
    Serial.print(F("|obj=")); Serial.print(ctx.secuenciaObjetivo);
    Serial.print(F("|objective_flags=")); Serial.print(ctx.flagsObjetivoV2);
    Serial.print(F("|reference_encoder=")); Serial.print(ctx.conteoReferenciaObjetivo);
    Serial.print(F("|reference_source=QUERY_MIDPOINT_APPROX|capture_timestamp=NA|image_age_known=0"));
    Serial.print(F("|reference_query_age_ms=")); Serial.print(millis() - filtro.ultimoMs);
    Serial.print(F("|closure_axis_minor="));
    Serial.print(filtro.votosEjeX >= 2 && filtro.votosEjeY == 0 ? F("Y") : F("X"));
    Serial.print(F("|suggestion_source=MLV2_EJES_20261005|servo_rot_deg="));
    Serial.print(anguloServoRotacion);
    Serial.println(F("|rotation_command_applied=1|rotation_physically_verified=0"));
  }
  reiniciarFiltroV2(ctx);

  Serial.print(F("[AUTO V2] Objetivo bloqueado seq="));
  Serial.print(ctx.secuenciaObjetivo);
  Serial.print(F(" clase="));
  Serial.print(ctx.claseObjetivo);
  Serial.print(F(" X="));
  Serial.print(ctx.objetivoX10 / 10.0f, 1);
  Serial.print(F(" Y="));
  Serial.print(ctx.objetivoY10 / 10.0f, 1);
  Serial.print(F(" encoder="));
  Serial.println(ctx.conteoReferenciaObjetivo);
  return true;
}
bool leerPiezasV2UnaVez(
  ContextoCamara &ctx,
  const ControlCamaraCompartido &control,
  uint32_t ahora
) {
  uint32_t consulta = ++consultasLogV2;
  const bool capturaAutonoma = capturaAutonomaV2(control);
  if (consulta == 0) consulta = ++consultasLogV2;
  const EstadoEncoderCompartido encoderAntes = copiarEstadoEncoder();
  const uint32_t inicioConsulta = millis();
  const int8_t resultCount = huskylens.getResult(PIECE_MODEL);
  const uint32_t finConsulta = millis();
  const EstadoEncoderCompartido encoderDespues = copiarEstadoEncoder();
  EstadoEncoderCompartido encoder = encoderDespues;
  const bool muestrasCompatibles =
    encoderAntes.recibidoMs != 0 && encoderDespues.recibidoMs != 0 &&
    inicioConsulta - encoderAntes.recibidoMs <= TIMEOUT_MUESTRA_ENCODER_MS &&
    finConsulta - encoderDespues.recibidoMs <= TIMEOUT_MUESTRA_ENCODER_MS &&
    encoderAntes.nmPorCuenta == encoderDespues.nmPorCuenta &&
    encoderAntes.signo == encoderDespues.signo;
  if (muestrasCompatibles) {
    encoder.conteo = conteoMedioConWrap(
      encoderAntes.conteo, encoderDespues.conteo
    );
  }
  const bool encoderValido = encoderRemotoVigente(encoder);
  const uint8_t flagsReferencia = ENC_FLAG_HW_LISTO | ENC_FLAG_ESCALA_VALIDA |
    ENC_FLAG_PULSOS_VISTOS;
  const bool referenciaAutonomaValida = muestrasCompatibles &&
    (encoderAntes.flags & flagsReferencia) == flagsReferencia &&
    (encoderAntes.flags & ENC_FLAG_SATURADO) == 0 && encoderValido;
  DiagnosticoDeteccionV2 diag = crearDiagnosticoBaseV2(
    encoder, resultCount
  );
  diag.duracionConsultaMs = finConsulta - inicioConsulta;
  diag.conteoAntesConsulta = encoderAntes.conteo;
  diag.conteoDespuesConsulta = encoderDespues.conteo;
  diag.conteoAsociadoCamara = encoder.conteo;
  diag.referenciaEntreMuestras = muestrasCompatibles;
  diag.flagsObjetivoV2 = ctx.flagsObjetivoV2;
  diag.edadConsultaReferenciaObjetivoMs = ctx.objetivoValido
    ? finConsulta - ctx.consultaReferenciaObjetivoMs : UINT32_MAX;
  if (muestrasCompatibles) {
    diag.recorridoDuranteConsultaMm = fabs(
      desplazamientoEncoderMm(
        encoderDespues.conteo, encoderAntes.conteo, encoderDespues
      )
    );
  }
  if (resultCount < 0) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_HUSKY_ERROR, encoder, consulta
    );
    return false;
  }

  CandidatoPiezaV2 candidatos[MAX_CANDIDATOS_V2] = {};
  CandidatoPiezaV2 unicos[MAX_CANDIDATOS_V2] = {};
  uint8_t cantidadCandidatos = 0;
  uint8_t cantidadUnicos = 0;
  uint8_t indiceResultado = 0;

  while (huskylens.available(PIECE_MODEL)) {
    Result *result = huskylens.popCachedResult(PIECE_MODEL);
    if (result == nullptr) continue;
    const uint8_t indiceActual = indiceResultado;
    if (indiceResultado < UINT8_MAX) ++indiceResultado;
    CandidatoPiezaV2 candidato = {};
    candidato.grupoDuplicado = UINT8_MAX;
    candidato.clase = VisionModelo129::clasePermitida(result->name.c_str());
    if (candidato.clase == 0 || result->type != COMMAND_RETURN_BLOCK ||
        result->width <= 0 || result->height <= 0) continue;
    candidato.confianza = result->confidence;
    candidato.centroXpx = result->xCenter;
    candidato.centroYpx = result->yCenter;
    candidato.anchoPx = result->width;
    candidato.altoPx = result->height;
    if (!pixelToMillimeters(result->xCenter, result->yCenter,
                            ctx.homografiaValida, candidato.posicion)) {
      if (diag.rechazadosHomografia < UINT8_MAX)
        ++diag.rechazadosHomografia;
      registrarCandidatoLogV2(
        consulta, encoder, candidato, indiceActual, V2_DIAG_HOMOGRAFIA
      );
      continue;
    }
    if (!isOverWhiteBelt(candidato.posicion)) {
      if (diag.rechazadosFueraBanda < UINT8_MAX)
        ++diag.rechazadosFueraBanda;
      registrarCandidatoLogV2(
        consulta, encoder, candidato, indiceActual, V2_DIAG_FUERA_BANDA
      );
      continue;
    }
    if (diag.candidatosValidos < UINT8_MAX) ++diag.candidatosValidos;
    if (capturaAutonoma && !cajaIntegraAutonomaV2(candidato)) {
      if (diag.rechazadosCajaRecortada < UINT8_MAX) ++diag.rechazadosCajaRecortada;
      registrarCandidatoLogV2(consulta, encoder, candidato, indiceActual,
                             V2_DIAG_CAJA_RECORTADA);
      continue;
    }
    if (cantidadCandidatos >= MAX_CANDIDATOS_V2) {
      registrarCandidatoLogV2(
        consulta, encoder, candidato, indiceActual, V2_DIAG_SIN_BLOQUEO
      );
      continue;
    }

    uint8_t grupo = cantidadUnicos;
    bool duplicado = false;
    for (uint8_t i = 0; i < cantidadUnicos; ++i) {
      if (!cajasDuplicadasV2(unicos[i], candidato)) continue;
      grupo = i;
      duplicado = true;
      if (candidatoV2MejorQue(candidato, unicos[i])) {
        unicos[i] = candidato;
      }
      break;
    }
    if (!duplicado && cantidadUnicos < MAX_CANDIDATOS_V2) {
      unicos[cantidadUnicos] = candidato;
      ++cantidadUnicos;
    }
    candidato.grupoDuplicado = grupo;
    candidatos[cantidadCandidatos++] = candidato;
    registrarCandidatoLogV2(
      consulta, encoder, candidato, indiceActual, V2_DIAG_SIN_BLOQUEO
    );
  }
  diag.candidatosUnicos = cantidadUnicos;
  diag.duplicadosDescartados = cantidadCandidatos >= cantidadUnicos
    ? cantidadCandidatos - cantidadUnicos : 0;

  // Observacion continua sin reservar objetivos ni esperar catch/ACK.
  // Se conservan TODOS los candidatos originales y la consulta en V2LOG.
  // Solo una pieza univoca y un encoder vigente se usan como referencia I2C.
  if (control.pruebaEncoderActiva) {
    ctx.objetivoValido = false;
    uint8_t seleccion = UINT8_MAX;
    if (cantidadUnicos == 1 && encoderValido) {
      const CandidatoPiezaV2 &c = unicos[0];
      const long x10 = lround(c.posicion.x * 10.0);
      const long y10 = lround(c.posicion.y * 10.0);
      if (x10 >= INT16_MIN && x10 <= INT16_MAX &&
          y10 >= INT16_MIN && y10 <= INT16_MAX) {
        ++ctx.secuenciaObjetivo;
        if (ctx.secuenciaObjetivo == 0) ++ctx.secuenciaObjetivo;
        ctx.objetivoValido = true;
        ctx.objetivoV2 = true;
        ctx.claseObjetivo = c.clase;
        ctx.objetivoX10 = static_cast<int16_t>(x10);
        ctx.objetivoY10 = static_cast<int16_t>(y10);
        ctx.conteoReferenciaObjetivo = encoder.conteo;
        diag.xMm = c.posicion.x;
        diag.yMm = c.posicion.y;
        diag.clase = c.clase;
        diag.secuenciaPublicada = ctx.secuenciaObjetivo;
        seleccion = 0;
      }
    }
    reiniciarFiltroV2(ctx);
    publicarYRegistrarDiagnosticoV2(diag,
      !encoderValido ? diagnosticarEncoderV2(encoder) :
        (cantidadUnicos == 0 ? V2_DIAG_SIN_PIEZA : V2_DIAG_SIN_BLOQUEO),
      encoder, consulta, seleccion);
    return true;
  }

  if (cantidadCandidatos > 1 &&
      ahora - ultimoReporteCandidatosV2 >= 1000UL) {
    ultimoReporteCandidatosV2 = ahora;
    Serial.print(F("[V2][CAND] total="));
    Serial.print(resultCount);
    Serial.print(F(" valid="));
    Serial.print(cantidadCandidatos);
    Serial.print(F(" unicos="));
    Serial.print(cantidadUnicos);
    Serial.print(F(" duplicados="));
    Serial.print(diag.duplicadosDescartados);
    Serial.print(F(" consulta="));
    Serial.print(diag.duracionConsultaMs);
    Serial.print(F("ms enc="));
    Serial.print(diag.conteoAntesConsulta);
    Serial.print('/');
    Serial.print(diag.conteoAsociadoCamara);
    Serial.print('/');
    Serial.print(diag.conteoDespuesConsulta);
    Serial.print(F(" recorridoConsulta="));
    Serial.println(diag.recorridoDuranteConsultaMm, 2);
    for (uint8_t i = 0; i < cantidadCandidatos; ++i) {
      const CandidatoPiezaV2 &c = candidatos[i];
      Serial.print(F("[V2][CAND] i="));
      Serial.print(i);
      Serial.print(F(" grupo="));
      Serial.print(c.grupoDuplicado);
      Serial.print(F(" id="));
      Serial.print(c.clase);
      Serial.print(F(" conf="));
      Serial.print(static_cast<int>(c.confianza));
      Serial.print(F(" px="));
      Serial.print(c.centroXpx);
      Serial.print(',');
      Serial.print(c.centroYpx);
      Serial.print(F(" caja="));
      Serial.print(c.anchoPx);
      Serial.print('x');
      Serial.print(c.altoPx);
      Serial.print(F(" mm="));
      Serial.print(c.posicion.x, 1);
      Serial.print(',');
      Serial.println(c.posicion.y, 1);
    }
  }

  uint8_t bloqueoPrevio = V2_DIAG_SIN_BLOQUEO;
  if (!control.portentaActiva || !control.automaticoV2Activo) {
    bloqueoPrevio = V2_DIAG_INACTIVO;
  } else if (control.brazoOcupado) {
    bloqueoPrevio = V2_DIAG_BRAZO_OCUPADO;
  } else if (ctx.objetivoValido) {
    bloqueoPrevio = V2_DIAG_OBJETIVO_ACTIVO;
  } else if (ctx.esperandoDesaparicion) {
    bloqueoPrevio = V2_DIAG_ESPERANDO_DESAPARICION;
  } else if (!ctx.rearmada) {
    bloqueoPrevio = V2_DIAG_NO_REARMADO;
  } else if (!encoderValido) {
    bloqueoPrevio = diagnosticarEncoderV2(encoder);
  } else if (capturaAutonoma && !referenciaAutonomaValida) {
    bloqueoPrevio = V2_DIAG_REFERENCIA_CAMARA_INCIERTA;
  } else if (capturaAutonoma && diag.rechazadosCajaRecortada != 0) {
    bloqueoPrevio = V2_DIAG_CAJA_RECORTADA;
  } else if (capturaAutonoma && cantidadUnicos > 1) {
    bloqueoPrevio = V2_DIAG_MULTIPLES_PIEZAS;
  }

  if (bloqueoPrevio != V2_DIAG_SIN_BLOQUEO) {
    publicarYRegistrarDiagnosticoV2(
      diag, bloqueoPrevio, encoder, consulta
    );
    reiniciarFiltroV2(ctx);
    ctx.ausenciasFiltroV2 = 0;
    return true;
  }

  if (cantidadUnicos == 0) {
    if (ctx.ausenciasFiltroV2 < UINT8_MAX) ++ctx.ausenciasFiltroV2;
    if (ctx.ausenciasFiltroV2 > 2) reiniciarFiltroV2(ctx);
    uint8_t causaSinPieza = V2_DIAG_SIN_RESULTADOS;
    if (resultCount == 0) {
      causaSinPieza = V2_DIAG_SIN_PIEZA;
    } else if (diag.rechazadosHomografia > 0 &&
               diag.rechazadosFueraBanda == 0) {
      causaSinPieza = V2_DIAG_HOMOGRAFIA;
    } else if (diag.rechazadosFueraBanda > 0) {
      causaSinPieza = V2_DIAG_FUERA_BANDA;
    }
    publicarYRegistrarDiagnosticoV2(
      diag, causaSinPieza, encoder, consulta
    );
    return true;
  }

  const bool habiaTrayectoria = ctx.filtroV2.consecutivas > 0;
  uint8_t indiceElegido = 0;
  bool hayCoincidente = false;
  double mejorPuntaje = HUGE_VAL;
  if (habiaTrayectoria) {
    for (uint8_t i = 0; i < cantidadUnicos; ++i) {
      const CandidatoPiezaV2 &candidato = unicos[i];
      if (!mismaTrayectoriaV2(
            ctx.filtroV2, candidato, encoder.conteo, encoder
          )) continue;
      const int8_t relacion = elegirRelacionYEncoderV2(
        ctx.filtroV2, candidato.posicion, encoder.conteo, encoder
      );
      const double yCompensada = compensarYConEncoder(
        candidato.posicion.y, encoder.conteo,
        ctx.filtroV2.conteoReferencia, encoder, relacion
      );
      const double dx = candidato.posicion.x - ctx.filtroV2.promedioX;
      const double dy = yCompensada - ctx.filtroV2.promedioYCompensada;
      const double areaAnterior = fmax(
        1.0, static_cast<double>(ctx.filtroV2.ultimoAnchoPx) *
             static_cast<double>(ctx.filtroV2.ultimoAltoPx)
      );
      const double penalizacionTamano = fabs(
        log(fmax(1.0, areaCajaV2(candidato)) / areaAnterior)
      );
      const double puntaje = dx * dx + dy * dy +
                              penalizacionTamano * penalizacionTamano;
      if (puntaje < mejorPuntaje) {
        mejorPuntaje = puntaje;
        indiceElegido = i;
        hayCoincidente = true;
      }
    }
  } else {
    for (uint8_t i = 1; i < cantidadUnicos; ++i) {
      if (candidatoV2MejorQue(unicos[i], unicos[indiceElegido])) {
        indiceElegido = i;
      }
    }
  }

  if (habiaTrayectoria && !hayCoincidente) {
    for (uint8_t i = 0; i < cantidadUnicos; ++i) {
      const CandidatoPiezaV2 &candidato = unicos[i];
      const double recorridoDesdeUltima = fabs(desplazamientoEncoderMm(
        encoder.conteo, ctx.filtroV2.ultimoConteo, encoder
      ));
      const double dxRaw = candidato.posicion.x - ctx.filtroV2.ultimoXRaw;
      const double dyRaw = candidato.posicion.y - ctx.filtroV2.ultimoYRaw;
      if (candidato.clase == ctx.filtroV2.clase &&
          recorridoDesdeUltima >= DESPLAZAMIENTO_CAMARA_REPETIDA_MM &&
          fabs(dxRaw) <= TOLERANCIA_CAMARA_REPETIDA_MM &&
          fabs(dyRaw) <= TOLERANCIA_CAMARA_REPETIDA_MM) {
        diag.clase = candidato.clase;
        diag.xMm = candidato.posicion.x;
        diag.yMm = candidato.posicion.y;
        diag.deteccionesConsecutivas = ctx.filtroV2.consecutivas;
        diag.desplazamientoMm = recorridoDesdeUltima;
        diag.relacionYEncoder = ctx.filtroV2.relacionYEncoder;
        publicarYRegistrarDiagnosticoV2(
          diag, V2_DIAG_CAMARA_REPETIDA, encoder, consulta, i
        );
        return true;
      }
    }
    for (uint8_t i = 1; i < cantidadUnicos; ++i) {
      if (candidatoV2MejorQue(unicos[i], unicos[indiceElegido])) {
        indiceElegido = i;
      }
    }
  }

  const CandidatoPiezaV2 &candidatoElegido = unicos[indiceElegido];
  if (habiaTrayectoria && !hayCoincidente) {
    const double yCompensadaRechazada = compensarYConEncoder(
      candidatoElegido.posicion.y,
      encoder.conteo,
      ctx.filtroV2.conteoReferencia,
      encoder,
      elegirRelacionYEncoderV2(
        ctx.filtroV2, candidatoElegido.posicion, encoder.conteo, encoder
      )
    );
    diag.clase = candidatoElegido.clase;
    diag.deteccionesConsecutivas = ctx.filtroV2.consecutivas;
    diag.xMm = candidatoElegido.posicion.x;
    diag.yMm = candidatoElegido.posicion.y;
    diag.yCompensadaMm = yCompensadaRechazada;
    diag.dispersionXmm =
      fmax(ctx.filtroV2.maximoX, candidatoElegido.posicion.x) -
      fmin(ctx.filtroV2.minimoX, candidatoElegido.posicion.x);
    diag.dispersionYCompensadaMm =
      fmax(ctx.filtroV2.maximoYCompensada, yCompensadaRechazada) -
      fmin(ctx.filtroV2.minimoYCompensada, yCompensadaRechazada);
    diag.desplazamientoMm = fabs(
      static_cast<double>(diferenciaConteosConWrap(
        encoder.conteo, ctx.filtroV2.conteoReferencia
      )) * static_cast<double>(escalaEncoderMm(encoder))
    );
  }

  incorporarDeteccionV2(ctx, candidatoElegido, encoder.conteo, encoder,
                        finConsulta);

  if (habiaTrayectoria && !hayCoincidente) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_TRAYECTORIA, encoder, consulta, indiceElegido
    );
    return true;
  }

  diag.clase = ctx.filtroV2.clase;
  diag.xMm = candidatoElegido.posicion.x;
  diag.yMm = candidatoElegido.posicion.y;
  diag.relacionYEncoder = ctx.filtroV2.relacionYEncoder;
  diag.votosEjeX = ctx.filtroV2.votosEjeX;
  diag.votosEjeY = ctx.filtroV2.votosEjeY;
  diag.yCompensadaMm = compensarYConEncoder(
    candidatoElegido.posicion.y,
    encoder.conteo,
    ctx.filtroV2.conteoReferencia,
    encoder,
    ctx.filtroV2.relacionYEncoder
  );
  completarMetricasFiltroV2(diag, ctx.filtroV2, encoder.conteo, encoder);

  if (ctx.filtroV2.consecutivas < DETECCIONES_ESTABLES_V2) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_DETECCIONES, encoder, consulta, indiceElegido
    );
    return true;
  }
  if (diag.desplazamientoMm < DESPLAZAMIENTO_MINIMO_V2_MM) {
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_DESPLAZAMIENTO, encoder, consulta, indiceElegido
    );
    return true;
  }

  publicarCausaDiagnosticoV2(diag, V2_DIAG_OBJETIVO_LISTO);
  if (capturaAutonoma && (!AUTO_V2_APLICAR_GIRO_POR_CAJA ||
      CalibracionAnguloMLV2::sugerir(ctx.filtroV2.clase,
        ctx.filtroV2.votosEjeX, ctx.filtroV2.votosEjeY) < 0)) {
    // Conservar el giro previo y reunir una nueva ventana; la caja no mide diagonales.
    publicarYRegistrarDiagnosticoV2(diag, V2_DIAG_ORIENTACION_AMBIGUA,
                                  encoder, consulta, indiceElegido);
    reiniciarFiltroV2(ctx);
    return true;
  }
  if (publicarObjetivoV2(
        ctx, encoder.conteo, encoder, control.orientarGarraV2,
        control.entrenamientoML, control.entrenamientoMLV2,
        control.pruebaSeguimiento, control.ajusteCatchV2, capturaAutonoma)) {
    diag.secuenciaPublicada = ctx.secuenciaObjetivo;
    diag.flagsObjetivoV2 = ctx.flagsObjetivoV2;
    diag.edadConsultaReferenciaObjetivoMs = finConsulta - ctx.consultaReferenciaObjetivoMs;
    publicarYRegistrarDiagnosticoV2(
      diag, V2_DIAG_PUBLICADO, encoder, consulta, indiceElegido
    );
  } else {
    registrarConsultaLogV2(diag, encoder, consulta, indiceElegido);
  }
  return true;
}
void mostrarModoAutomaticoV2(const PaquetePortentaAESP &p) {
  const DiagnosticoDeteccionV2 diag = copiarDiagnosticoV2();
  const bool ajusteCatch = p.opcionMenu == MENU_AJUSTE_CATCH_V2;
  dibujarTitulo(p.opcionMenu == MENU_REGISTRO_ANGULO
    ? F("REGISTRO ANGULO") : (ajusteCatch ? F("AJUSTE CATCH V2") : F("AUTO V2 Y FIJO")));
  if (!ajusteCatch && p.opcionMenu == MENU_MODO_AUTOMATICO_V2 &&
      p.errorSistema == SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA) {
    pantalla.setCursor(0, 17); pantalla.print(F("CALIBRAR CAPTURA"));
    pantalla.setCursor(0, 32); pantalla.print(F("PERFIL FISICO"));
    pantalla.setCursor(0, 57); pantalla.print(F("TRI:SALIR"));
    return;
  }
  if (ajusteCatch && p.faseCalibracionBrazo == 17) {
    pantalla.setCursor(0, 13); pantalla.print(F("RESULTADO CATCH?"));
    pantalla.setCursor(0, 25); pantalla.print(F("X: LA AGARRO"));
    pantalla.setCursor(0, 37); pantalla.print(F("CUAD: ANTES"));
    pantalla.setCursor(0, 49); pantalla.print(F("CIRC: DESPUES"));
    pantalla.setCursor(0, 57); pantalla.print(F("TRI: DESCARTAR/SALIR"));
    return;
  }
  pantalla.setCursor(0, 12);
  pantalla.print(F("FASE: "));
  pantalla.print(p.faseCalibracionBrazo);
  if (p.opcionMenu == MENU_MODO_AUTOMATICO_V2 || ajusteCatch) {
    pantalla.print(AUTO_V2_APLICAR_GIRO_POR_CAJA ? F(" ROT:") : F(" FIJA:"));
    pantalla.print(anguloServoRotacion);
  }
  pantalla.println();
  if (p.faseCalibracionBrazo == 0) {
    pantalla.setCursor(0, 23);
    pantalla.print(F("BLOQ: "));
    pantalla.println(nombreCortoDiagnosticoV2(diag.causa));
    pantalla.setCursor(0, 35);
    pantalla.print(F("N:"));
    pantalla.print(diag.deteccionesConsecutivas);
    pantalla.print(F(" D:"));
    pantalla.print(diag.desplazamientoMm, 1);
    pantalla.println(F("mm"));
    pantalla.setCursor(0, 47);
    pantalla.print(F("AGE:"));
    if (diag.edadEncoderMs == UINT32_MAX) pantalla.print(F("---"));
    else pantalla.print(diag.edadEncoderMs);
    pantalla.print(F(" R:"));
    pantalla.print(diag.resultadosHusky);
    pantalla.print(F(" V:"));
    pantalla.print(diag.candidatosValidos);
    pantalla.setCursor(0, 57);
    if (p.opcionMenu == MENU_REGISTRO_ANGULO) {
      pantalla.print(F("ROT:"));
      pantalla.print(anguloServoRotacion);
      pantalla.print(F(" TRI:SALIR"));
    } else {
      pantalla.print(F("PRE:"));
      pantalla.print(Z_MARGEN_PRECAPTURA_PASOS);
      pantalla.print(F("p Z:"));
      pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
      pantalla.print(F("p"));
    }
    return;
  }
  pantalla.setCursor(0, 23);
  pantalla.print(F("ENC: "));
  pantalla.println(p.conteoEncoder);
  pantalla.setCursor(0, 35);
  pantalla.print(F("BANDA: "));
  pantalla.print(p.velocidadEncoderUmS / 1000000.0f, 1);
  pantalla.println(F(" mm/s"));
  pantalla.setCursor(0, 47);
  if (p.opcionMenu == MENU_REGISTRO_ANGULO) {
    pantalla.print(F("ENC:"));
    pantalla.print((copiarEstadoCamara().objetivoValido) ? F("OBJ ") : F("--- "));
    pantalla.print(F("ACK:"));
    pantalla.println(p.ackSecuenciaObjetivo);
  } else {
    pantalla.print(F("Z-DIN04 real: "));
    pantalla.print(extraerZDesdeDin04Pasos(p.nmPorCuentaEncoder));
    pantalla.println(F("p"));
  }
  pantalla.setCursor(0, 57);
  if (p.opcionMenu == MENU_REGISTRO_ANGULO) {
    pantalla.print(F("ROT:"));
    pantalla.print(anguloServoRotacion);
    pantalla.print(F(" STICK DER X"));
  } else if (p.faseCalibracionBrazo == 6) {
    pantalla.print(F("ESPERA CATCH AUTO"));
  } else if (p.faseCalibracionBrazo == 9) {
    pantalla.print(F("PREPARANDO ESPERA"));
  } else if (p.faseCalibracionBrazo == 10) {
    pantalla.print(F("CERRANDO PINZA"));
  } else if (p.faseCalibracionBrazo == 11) {
    pantalla.print(F("Z FINAL + CATCH"));
  } else if (p.faseCalibracionBrazo == 7) {
    pantalla.print(F("CICLO ENTREGADO"));
  } else if (p.faseCalibracionBrazo == 12) {
    pantalla.print(F("ENTREGA DERECHA"));
  } else if (p.faseCalibracionBrazo == 13) {
    pantalla.print(F("Z ENTREGA"));
  } else if (p.faseCalibracionBrazo == 14) {
    pantalla.print(F("SOLTANDO PIEZA"));
  } else if (p.faseCalibracionBrazo == 15) {
    pantalla.print(F("Z REGRESANDO"));
  } else if (p.faseCalibracionBrazo == 16) {
    pantalla.print(ajusteCatch ? F("Y SIGUE CATCH AUTO") : F("ESPERA Y FIJO"));
  } else {
    pantalla.print(F("TRI:CANCELAR"));
  }
}

ControlCamaraCompartido control;
ContextoCamara ctx;
void reset(bool teaching=false,bool ajuste=false) {
  ctx={};ctx.homografiaValida=true;ctx.rearmada=true;
  control={};control.portentaActiva=true;control.automaticoActivo=true;
  control.automaticoV2Activo=true;control.orientarGarraV2=!teaching;
  control.entrenamientoML=control.entrenamientoMLV2=teaching;control.ajusteCatchV2=ajuste;
  muestra={};muestra.nmPorCuenta=100000;muestra.signo=1;
  muestra.flags=ENC_FLAG_HW_LISTO|ENC_FLAG_ESCALA_VALIDA|ENC_FLAG_PULSOS_VISTOS;
  muestra.velocidadMmS=33.33f;
  servoRotacion={};anguloServoRotacion=88;Serial.log.clear();ultimoDiag={};
  huskylens.restoreFlags=false;now=1000;
  H[0][0]=H[1][1]=H[2][2]=1;H[0][2]=-320;H[1][2]=-240;
}
void frame(int n,int px=320,int py=240,int w=20,int h=60,bool duplicate=false,bool second=false) {
  now+=60;muestra.conteo=n*20;muestra.recibidoMs=now;muestra.secuencia++;
  Result r;r.xCenter=px;r.yCenter=py-2*n;r.width=w;r.height=h;
  huskylens.results={r};
  if(duplicate) { r.xCenter++;huskylens.results.push_back(r); }
  if(second) { r.xCenter+=90;huskylens.results.push_back(r); }
  assert(leerPiezasV2UnaVez(ctx,control,now));
}
int main() {
  // Toda la region interior calibrada: ninguna linea fija de deteccion.
  for(int px: {200,320,440}) for(int py: {100,240,380}) {
    reset();frame(0,px,py);frame(1,px,py);assert(!ctx.objetivoValido);
    frame(2,px,py);assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==7);
    assert(ctx.conteoReferenciaObjetivo==40&&ctx.objetivoX10==(px-320)*10);
    assert(ctx.objetivoY10==(py-244)*10);
    assert(servoRotacion.writes==1&&anguloServoRotacion==155);
    assert(Serial.log.find("capture_timestamp=NA")!=std::string::npos);
    assert(Serial.log.find("closure_axis_minor=X")!=std::string::npos);
    assert(Serial.log.find("mode=V2|event=CAMERA_SPEED")!=std::string::npos);
    const auto ref=ctx.conteoReferenciaObjetivo;const auto x=ctx.objetivoX10;
    frame(3,px+10,py);assert(ctx.conteoReferenciaObjetivo==ref&&ctx.objetivoX10==x);
    assert(ultimoDiag.causa==V2_DIAG_OBJETIVO_ACTIVO&&ultimoDiag.edadConsultaReferenciaObjetivoMs>0);
  }
  // Las cajas duplicadas de la misma pieza no representan dos objetivos.
  reset();for(int i=0;i<3;++i) frame(i,320,240,60,20,true);
  assert(ctx.objetivoValido&&anguloServoRotacion==59);
  assert(Serial.log.find("closure_axis_minor=Y")!=std::string::npos);
  // Multiples piezas, cajas cortadas por la calibracion y borde de imagen.
  reset();frame(0,320,240,20,60,false,true);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_MULTIPLES_PIEZAS);
  reset();frame(0,320,70);assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_CAJA_RECORTADA);
  reset();H[0][0]=.4;H[0][2]=-128;frame(0,2,240);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_CAJA_RECORTADA);
  // Sin consenso, el giro previo se conserva y no se publica autonomo.
  reset();for(int i=0;i<3;++i) frame(i,320,240,30,30);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_ORIENTACION_AMBIGUA);
  assert(anguloServoRotacion==88&&servoRotacion.writes==0);
  reset();frame(0,320,240,60,20);frame(1);frame(2);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_ORIENTACION_AMBIGUA);
  // Un extremo anterior invalido no puede hacerse valido por recibir otro nuevo.
  reset();muestra.flags=0;huskylens.restoreFlags=true;frame(0);
  assert(!ctx.objetivoValido&&ultimoDiag.causa==V2_DIAG_REFERENCIA_CAMARA_INCIERTA);
  // Ensenanza y ajuste no heredan los nuevos bloqueos ni flags autonomos.
  reset(true);for(int i=0;i<3;++i) frame(i,320,70,30,30,false,true);
  assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==0&&anguloServoRotacion==88);
  reset(false,true);for(int i=0;i<3;++i) frame(i,320,70,30,30,false,true);
  assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==0&&anguloServoRotacion==88);
  // Modo encoder observa candidatos sin reserva/consenso.
  reset();control.pruebaEncoderActiva=true;frame(0,320,70,30,30);
  assert(ctx.objetivoValido&&ctx.flagsObjetivoV2==0&&!capturaAutonomaV2(control));
  // El aviso viaja como telemetria valida. RS485 permite continuar el ensayo
  // y conserva la fase visible; el antecedente I2C conserva su aviso historico.
  PaquetePortentaAESP p={};p.signoEncoder=1;
  p.estadoSistema=SISTEMA_MODO_AUTOMATICO_V2;p.opcionMenu=MENU_MODO_AUTOMATICO_V2;
  p.errorSistema=SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA;
  assert(paquetePortentaSemanticamenteValido(p));mostrarModoAutomaticoV2(p);
  assert(titulo=="AUTO V2 Y FIJO");
  if(ensayoRS485) {
    assert(pantalla.log.find("PRUEBA CATCH")!=std::string::npos);
    assert(pantalla.log.find("VALORES NOMINALES")!=std::string::npos);
    assert(pantalla.log.find("CALIBRAR CAPTURA")==std::string::npos);
    for(int fase : {6,10,11,12}) {
      p.faseCalibracionBrazo=fase;mostrarModoAutomaticoV2(p);
      assert(pantalla.log.find("FASE: "+std::to_string(fase))!=std::string::npos);
      assert(pantalla.log.find("VALORES NOMINALES")!=std::string::npos);
    }
  } else {
    assert(pantalla.log.find("CALIBRAR CAPTURA")!=std::string::npos);
    assert(pantalla.log.find("PERFIL FISICO")!=std::string::npos);
  }
  p.opcionMenu=MENU_AJUSTE_CATCH_V2;
  assert(!paquetePortentaSemanticamenteValido(p));mostrarModoAutomaticoV2(p);
  assert(titulo=="AJUSTE CATCH V2"&&pantalla.log.find("CALIBRAR CAPTURA")==std::string::npos);
  assert(pantalla.log.find("PRUEBA CATCH")==std::string::npos);
  p.errorSistema=SISTEMA_ERROR_NINGUNO;p.faseCalibracionBrazo=16;
  mostrarModoAutomaticoV2(p);assert(pantalla.log.find("Y SIGUE CATCH AUTO")!=std::string::npos);
  p.opcionMenu=MENU_MODO_AUTOMATICO_V2;mostrarModoAutomaticoV2(p);
  assert(pantalla.log.find("ESPERA Y FIJO")!=std::string::npos);
  p.errorSistema=SISTEMA_ERROR_CALIBRACION_CAPTURA_FIJA+1;
  assert(!paquetePortentaSemanticamenteValido(p));
  puts("PASS: ESP vision captura fija; region, cuentas, ejes, reserva, cajas, ambiguedad y compatibilidad de ensayos.");
}

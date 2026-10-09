// Coordinador general para Arduino Portenta H7 + Machine Control.
// Mantiene el nucleo mecanico del sketch funcional MAster/MAster.ino e integra
// arranque autonomo, camara, checklist, menu de cinco opciones y modos automaticos.

#include <Arduino_MachineControl.h>
#include <Wire.h>
#include "hal/trng_api.h"
#include "mbed.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ProtocoloRS485.h"
#include "EnlaceRS485.h"
#include "AjusteCatchV2.h"
#include "RecuperacionEnlace.h"

using namespace machinecontrol;
using namespace ProtocoloRS485;

//-------------------------------------------------------------------------------------------------
// CONFIGURACION MECANICA CONSERVADA
//-------------------------------------------------------------------------------------------------
// Canales de salida de Machine Control (no son GPIO Arduino convencionales).
const int pP_Z = 0;
const int pP_X = 4;
const int pP_Y = 2;

const int pD_Z = 1;
const int pD_X = 5;
const int pD_Y = 3;

// VELOCIDAD DE LOS TRES MOTORES: el ticker conmuta STEP cada 100 us y un paso
// completo requiere dos llamadas. Con 0.0001 s y divisor 1 son ~5000 pasos/s.
// Para aumentar velocidad reduzca este periodo con pruebas graduales (por
// ejemplo 0.00008 s ~= 6250 pasos/s). DIV_POSICION=1 ya es el divisor maximo.
const float velocidadMotores = 0.0001f;
mbed::Ticker motorTicker;

const uint16_t DIV_MANUAL = 1;
const uint16_t DIV_CAL_RAPIDA = 2;
const uint16_t DIV_CAL_LENTA = 8;
const uint16_t DIV_HOME = 2;
const uint16_t DIV_POSICION = 1;

const long PASOS_SEPARACION = 300;
const long RANGO_MINIMO_VALIDO = 100;
const unsigned long TIMEOUT_FASE_MS = 180000UL;
const unsigned long TIMEOUT_MOVIMIENTO_MS = 180000UL;

const float RANGO_FISICO_X_MM = 446.0f;
const float RANGO_FISICO_Y_MM = 336.0f;
const float MARGEN_SEGURIDAD_MM = 2.0f;

// Ajuste fisico camara -> brazo. Se aplica swap, luego signo y finalmente offset.
constexpr bool CAMERA_SWAP_XY = false;
constexpr int8_t CAMERA_SIGN_X = -1;
constexpr int8_t CAMERA_SIGN_Y = -1;
// AJUSTES DEL DESFASE DE CAMARA, en mm del sistema del brazo, despues de signos.
// Positivo desplaza la estimacion hacia +X/+Y; negativo hacia -X/-Y.
// Se aplican a Automatico, Automatico V2 y Ensenanza ML.
constexpr float DESFASE_CAMARA_X_MM = -5.0f;
constexpr float DESFASE_CAMARA_Y_MM = 0.0f;
constexpr float CAMERA_OFFSET_X_MM = DESFASE_CAMARA_X_MM;
constexpr float CAMERA_OFFSET_Y_MM = DESFASE_CAMARA_Y_MM;
constexpr float CAMARA_A_HOME_Y_BASE_MM = 510.0f;
// Ajuste robusto obtenido con las confirmaciones manuales: tres marcas se
// agruparon entre Y=116.7 y 133.2 mm. Se agregan 125 mm a la distancia anterior
// para que ese punto fisico corresponda nuevamente a Y=0.
constexpr float V2_AJUSTE_DISTANCIA_CATCH_MM = 335.0f;
constexpr float CAMARA_A_HOME_Y_MM =
    CAMARA_A_HOME_Y_BASE_MM + V2_AJUSTE_DISTANCIA_CATCH_MM;

// E6B2-CWZ6C de 1024 P/R, lectura X2 y rueda de 49 mm en contacto 1:1.
// La geometria fija la distancia por cuenta; la tercera calibracion de arranque
// mide automaticamente el sentido y la velocidad real con la banda al 50 %.
constexpr float ENCODER_DIAMETRO_RUEDA_MM = 49.0f;
constexpr float ENCODER_RELACION_ENCODER_RUEDA = 1.0f;
constexpr int ENCODER_CUENTAS_X2_POR_VUELTA = 2048;
constexpr float ENCODER_PI = 3.14159265358979323846f;
constexpr float ENCODER_MM_POR_CUENTA =
    ENCODER_PI * ENCODER_DIAMETRO_RUEDA_MM /
    (static_cast<float>(ENCODER_CUENTAS_X2_POR_VUELTA) *
     ENCODER_RELACION_ENCODER_RUEDA);
static_assert(ENCODER_MM_POR_CUENTA * 1000000.0f <= MASCARA_ESCALA_ENCODER_NM,
    "La escala del encoder no cabe en los 18 bits del protocolo RS485");

constexpr unsigned long ENC_TIEMPO_ESTABILIZACION_MS = 2000UL;
constexpr unsigned long ENC_TIEMPO_MEDICION_MS = 5000UL;
constexpr unsigned long ENC_PERIODO_VENTANA_MS = 200UL;
constexpr unsigned long ENC_TIMEOUT_PARO_MS = 30000UL;
constexpr int32_t ENC_CUENTAS_MINIMAS_MEDICION = 100;
constexpr uint8_t ENC_VENTANAS_MINIMAS = 20;

static_assert(CAMERA_SIGN_X == 1 || CAMERA_SIGN_X == -1, "CAMERA_SIGN_X debe ser +/-1");
static_assert(CAMERA_SIGN_Y == 1 || CAMERA_SIGN_Y == -1, "CAMERA_SIGN_Y debe ser +/-1");
static_assert(ENCODER_RELACION_ENCODER_RUEDA > 0.0f,
              "La relacion encoder/rueda debe ser positiva");

//-------------------------------------------------------------------------------------------------
// PERIODOS Y TIMEOUTS DEL COORDINADOR
//-------------------------------------------------------------------------------------------------
const unsigned long RETARDO_ARRANQUE_ESP32_MS = 3000UL;
// Un intercambio nominal cada 10 ms. Solo hay una solicitud pendiente;
// la recepcion y el timeout se atienden sin esperar en el loop de motores.
const unsigned long PERIODO_CONTROL_MS = 10UL;
const unsigned long PERIODO_ESTADO_ESP_MS = 10UL; // Solicitud+respuesta cada 10 ms si el bus esta libre.
const unsigned long PERIODO_ENCODER_MS = 10UL;
const unsigned long TIMEOUT_MOVIMIENTO_ENCODER_MS = 500UL;
const unsigned long TIMEOUT_RS485_MS = RecuperacionEnlace::SIN_RESPUESTA_MS;
// La secuencia y los errores siguen disponibles para diagnostico. La perdida
// general depende solo del tiempo desde una respuesta aceptada.
const unsigned long TIMEOUT_SECUENCIA_ESP_MS = RecuperacionEnlace::SIN_RESPUESTA_MS;
const unsigned long TIEMPO_ESTABILIZACION_RS485_MS = 5000UL;
const unsigned long TIMEOUT_CAMARA_MS = 240000UL;
const uint8_t MAX_PAQUETES_INVALIDOS_CONSECUTIVOS = 10;

//-------------------------------------------------------------------------------------------------
// ESTADOS LOCALES
//-------------------------------------------------------------------------------------------------
enum EstadoGeneral : uint8_t {
    EST_BOOT_SAFE = 0,
    EST_WAIT_RS485,
    EST_RS485_SETTLE,
    EST_CAMERA_CALIBRATION,
    EST_ARM_CALIBRATION,
    EST_WAIT_CONTROLLER,
    EST_FINAL_CHECKLIST,
    EST_MAIN_MENU,
    EST_MANUAL,
    EST_AUTOMATICO,
    EST_USER_ARM_CALIBRATION,
    EST_USER_CAMERA_CALIBRATION,
    EST_SYSTEM_ERROR,
    EST_AUTOMATICO_V2,
    EST_ENCODER_CALIBRATION,
    EST_CALIBRACIONES_MENU,
    EST_PRUEBA_SERVOS,
    EST_DIAGNOSTICO,
    EST_ENTRENAMIENTO_ML,
    EST_PRUEBA_ENCODER,
    EST_CAMBIOS_CATCH
};

enum FaseCalibracion : uint8_t {
    CAL_ESPERA = 0,
    CAL_X_MIN_1,
    CAL_X_MIN_LIBERAR,
    CAL_X_MIN_SEPARAR,
    CAL_X_MIN_2,
    CAL_X_MAX_1,
    CAL_X_MAX_LIBERAR,
    CAL_X_MAX_SEPARAR,
    CAL_X_MAX_2,
    CAL_Y_MIN_1,
    CAL_Y_MIN_LIBERAR,
    CAL_Y_MIN_SEPARAR,
    CAL_Y_MIN_2,
    CAL_Y_MAX_1,
    CAL_Y_MAX_LIBERAR,
    CAL_Y_MAX_SEPARAR,
    CAL_Y_MAX_2,
    CAL_Z_MIN_1,
    CAL_Z_MIN_LIBERAR,
    CAL_Z_MIN_SEPARAR,
    CAL_Z_MIN_2,
    CAL_Z_MAX_1,
    CAL_Z_MAX_LIBERAR,
    CAL_Z_MAX_SEPARAR,
    CAL_Z_MAX_2,
    CAL_HOME,
    CAL_COMPLETA,
    CAL_ERROR
};

enum PropietarioMovimiento : uint8_t {
    MOV_SIN_PROPIETARIO = 0,
    MOV_TERMINAL,
    MOV_AUTOMATICO,
    MOV_AUTOMATICO_V2
};

enum FaseAutomaticoV2 : uint8_t {
    V2_ESPERANDO_PIEZA = 0,
    V2_PREPOSICIONANDO = 1,
    V2_ESPERANDO_LLEGADA = 2,
    V2_DISPARANDO_CATCH = 3,
    V2_BAJANDO_Z = 4,
    V2_SUBIENDO_Z = 5,
    // Se conserva el numero 6 para no romper la interpretacion de CSV previos.
    V2_ESPERANDO_CATCH_AUTOMATICO = 6,
    V2_COMPLETADO = 7,
    V2_CANCELANDO = 8,
    V2_PREPARANDO_ESPERA = 9,
    V2_CERRANDO_PINZA = 10,
    V2_BAJANDO_CATCH = 11,
    V2_MOVIENDO_ENTREGA = 12,
    V2_BAJANDO_ENTREGA = 13,
    V2_ABRIENDO_PINZA = 14,
    V2_SUBIENDO_FINAL = 15,
    V2_SIGUIENDO_PIEZA = 16,
    V2_EVALUANDO_CATCH = 17
};

enum FaseEntrenamientoML : uint8_t {
    ML_ESPERANDO_PIEZA = 0,
    ML_PREPOSICIONANDO = 1,
    ML_BAJANDO_CAPTURA = 2,
    ML_ALINEACION_MANUAL = 3,
    ML_CERRANDO_PINZA = 4,
    ML_SUBIENDO_CON_PIEZA = 5,
    ML_MOVIENDO_ENTREGA = 6,
    ML_BAJANDO_ENTREGA = 7,
    ML_ABRIENDO_PINZA = 8,
    ML_SUBIENDO_FINAL = 9,
    ML_LISTO = 10,
    ML_CANCELANDO = 11,
    ML_PREPARANDO_ESPERA = 12,
    ML_BAJANDO_CATCH = 13,
    ML_ESPERANDO_CONFIRMACION = 14,
    ML_SEGUIMIENTO = 15
};

enum FaseCalibracionEncoder : uint8_t {
    ENC_CAL_ESPERA_50 = 0,
    ENC_CAL_ESTABILIZANDO = 1,
    ENC_CAL_MIDIENDO = 2,
    ENC_CAL_ESPERA_PARO = 3,
    ENC_CAL_COMPLETA = 4,
    ENC_CAL_ERROR = 5
};

enum CodigoErrorLocal : uint8_t {
    ERROR_NINGUNO = 0,
    ERROR_TIMEOUT_RS485 = 1,
    ERROR_PROTOCOLO_INVALIDO = 2,
    ERROR_TIMEOUT_CAMARA = 3,
    ERROR_CAMARA = 4,
    ERROR_CALIBRACION_BRAZO = 5,
    ERROR_FINALES_INCOHERENTES = 6,
    ERROR_TIMEOUT_MOVIMIENTO = 7,
    ERROR_FINAL_INESPERADO = 8,
    ERROR_CHECKLIST = 9,
    ERROR_OBJETIVO_INVALIDO = 10,
    ERROR_REINICIO_ESP32 = 11,
    ERROR_CANCELADO = 12
};

enum ResultadoChecklist : uint8_t {
    CHECK_OK = 0,
    CHECK_RS485,
    CHECK_PROTOCOLO,
    CHECK_CAMARA,
    CHECK_HOMOGRAFIA,
    CHECK_MODELO,
    CHECK_XY,
    CHECK_Z,
    CHECK_HOME,
    CHECK_MOTORES,
    CHECK_BT,
    CHECK_ENCODER,
    CHECK_CAL_ERROR,
    CHECK_FINALES
};

//-------------------------------------------------------------------------------------------------
// VARIABLES COMPARTIDAS CON EL TICKER
//-------------------------------------------------------------------------------------------------
volatile int8_t movX = 0;
volatile int8_t movY = 0;
volatile int8_t movZ = 0;

volatile bool pulsoX = false;
volatile bool pulsoY = false;
volatile bool pulsoZ = false;

volatile uint16_t cuentaX = 0;
volatile uint16_t cuentaY = 0;
volatile uint16_t cuentaZ = 0;

volatile uint16_t divisorX = DIV_MANUAL;
volatile uint16_t divisorY = DIV_MANUAL;
volatile uint16_t divisorZ = DIV_MANUAL;

volatile long pasosX = 0;
volatile long pasosY = 0;
volatile long pasosZ = 0;

volatile bool objetivoXActivo = false;
volatile bool objetivoYActivo = false;
volatile bool objetivoZActivo = false;
volatile long objetivoX = 0;
volatile long objetivoY = 0;
volatile long objetivoZ = 0;

//-------------------------------------------------------------------------------------------------
// ESTADO MECANICO Y DE CALIBRACION
//-------------------------------------------------------------------------------------------------
bool limiteXmas = false;
bool limiteXmenos = false;
bool limiteYmenos = false;
bool limiteYmas = false;
bool limiteZarriba = false;
bool limiteZabajo = false;

FaseCalibracion faseCal = CAL_ESPERA;
unsigned long inicioFase = 0;
bool homeIniciado = false;
bool calibracionXYValida = false;
bool calibracionZValida = false;
const char *mensajeErrorCalibracion = "";

long rangoXPasos = 0;
long rangoYPasos = 0;
long rangoZPasos = 0;
float pasosPorMmX = 0.0f;
float pasosPorMmY = 0.0f;

bool movimientoPosicionadoActivo = false;
PropietarioMovimiento propietarioMovimiento = MOV_SIN_PROPIETARIO;
enum FaseHomeManual : uint8_t {
    HOME_MANUAL_INACTIVO,
    HOME_MANUAL_Z,
    HOME_MANUAL_XY,
    HOME_MANUAL_ESPERANDO_NEUTRO
};
FaseHomeManual faseHomeManual = HOME_MANUAL_INACTIVO;
unsigned long inicioHomeManual = 0;
unsigned long inicioMovimientoPosicionado = 0;
float objetivoXmm = 0.0f;
float objetivoYmm = 0.0f;
float objetivoZmm = 0.0f;
long objetivoCartesianoXPasos = 0;
long objetivoCartesianoYPasos = 0;

//-------------------------------------------------------------------------------------------------
// ESTADO DEL COORDINADOR Y DEL ENLACE
//-------------------------------------------------------------------------------------------------
EstadoGeneral estadoGeneral = EST_BOOT_SAFE;
unsigned long inicioEstadoGeneral = 0;
CodigoErrorLocal errorSistema = ERROR_NINGUNO;
const char *mensajeErrorSistema = "";
ResultadoChecklist ultimoResultadoChecklist = CHECK_OK;

uint8_t opcionMenu = 0;
uint8_t indiceMenu = 0;
uint8_t opcionCalibracion = 0;
uint8_t paginaDiagnostico = 0;
EstadoGeneral modoPendiente = EST_MAIN_MENU;
EstadoGeneral retornoCalibracion = EST_MAIN_MENU;
int8_t entradaMenuYAnterior = 0;

PaqueteESPAPortenta paqueteESP = {};
bool existePaqueteValido = false;
bool protocoloValido = false;
unsigned long ultimoPaqueteValidoMs = 0;
uint8_t fallosPaqueteConsecutivos = 0;
uint16_t sesionArranqueESP = 0;
bool sesionESPConocida = false;
bool cambioSesionESPPendiente = false;
uint8_t ultimaSecuenciaPaqueteESP = 0;
bool secuenciaPaqueteESPConocida = false;
unsigned long ultimoCambioSecuenciaESPMs = 0;

int8_t joystickX = 0;
int8_t joystickY = 0;
int8_t joystickZ = 0;
bool btConectado = false;
bool botonX = false;
bool botonCirculo = false;
bool botonTriangulo = false;
bool botonCuadrado = false;
bool botonXAnterior = false;
bool botonCirculoAnterior = false;
bool botonTrianguloAnterior = false;
bool botonCuadradoAnterior = false;
bool eventoBotonX = false;
bool eventoBotonCirculo = false;
bool eventoBotonTriangulo = false;
bool eventoBotonCuadrado = false;
uint8_t posServoRot = 90;
uint8_t posServoPin = 90;

uint8_t estadoCamara = 0;
uint8_t errorCamara = 0;
uint8_t flagsCamara = 0;
uint8_t muestrasTag[4] = {0, 0, 0, 0};
uint8_t claseObjetivo = 0;
int16_t objetivoCamaraX10 = 0;
int16_t objetivoCamaraY10 = 0;
uint16_t secuenciaObjetivoRecibida = 0;
int32_t conteoReferenciaObjetivoRecibido = 0;

uint8_t comandoCamaraActual = 0;
uint8_t secuenciaComandoCamara = 0;
bool inicioCalibracionCamaraObservado = false;

uint16_t ackSecuenciaObjetivo = 0;
uint8_t codigoAckObjetivo = 0;
uint16_t secuenciaObjetivoEnMovimiento = 0;

enum FaseRegistroAngulo : uint8_t {
    REG_PREPARANDO_Z, REG_ESPERANDO, REG_MOVIENDO_XY, REG_AJUSTANDO,
    REG_BAJANDO_Z, REG_CERRANDO, REG_SUBIENDO, REG_CANCELANDO
};
struct EstadoRegistroAngulo {
    FaseRegistroAngulo fase;
    unsigned long inicioFase;
    unsigned long ultimoCambioRot;
    uint16_t secuencia;
    uint8_t clase;
    uint8_t anguloAnterior;
    uint8_t anguloCatch;
    float camX;
    float camY;
    bool busquedaFinalZActiva;
    bool salirPorDesconexion;
};
EstadoRegistroAngulo registroAngulo = {};

uint8_t estadoEncoderBanda = 0;
int32_t conteoEncoderBanda = 0;
int32_t ultimoConteoEncoderVelocidad = 0;
int revolucionesIndiceEncoder = 0;
int ultimoEstadoABEncoder = 0;
int ultimoIndiceReportado = 0;
bool referenciaIndiceDiagnosticoValida = false;
int indiceReferenciaDiagnostico = 0;
int32_t conteoReferenciaIndiceDiagnostico = 0;
uint32_t intervalosIndiceCompletos = 0;
uint64_t cuentasIndiceAcumuladas = 0;
float ultimaMedicionCuentasPorVuelta = 0.0f;
uint16_t secuenciaEncoder = 0;
unsigned long ultimaMuestraEncoder = 0;
unsigned long ultimoPulsoEncoder = 0;
float velocidadBandaMmS = 0.0f;
float frecuenciaEncoderCuentasS = 0.0f;
float escalaEncoderMmPorCuenta = ENCODER_MM_POR_CUENTA;
int32_t conteoCeroUsuario = 0;
int8_t signoEncoderAvance = 1;
bool calibracionEncoderValida = false;
float velocidadReferencia50MmS = 0.0f;
float velocidadMaximaEstimadaMmS = 0.0f;

struct ContextoCalibracionEncoder {
    FaseCalibracionEncoder fase;
    unsigned long inicioFase;
    unsigned long inicioVentana;
    int32_t conteoInicioFase;
    int32_t conteoInicioVentana;
    float minimoCuentasS;
    float maximoCuentasS;
    uint8_t ventanasValidas;
    uint8_t ventanasInvalidas;
    int8_t signoObservado;
    const char *mensajeError;
};

ContextoCalibracionEncoder calibracionEncoder = {};

// Parametros de puesta en marcha de Automatico V2. Z se expresa en pasos
// absolutos porque el firmware actual no dispone de una escala Z en mm.
constexpr float V2_POSICION_CATCH_Y_MM = 0.0f;
constexpr float V2_AJUSTE_ANTICIPACION_Z_MM = 0.0f;
// Z debe estar abajo antes de que la pieza llegue a la estacion. Esta reserva
// minima evita aceptar una prebajada tardia. Al expresarse en segundos se
// adapta a la velocidad de la banda.
constexpr float V2_RESERVA_MANUAL_CATCH_S = 0.50f;
// Tiempo medido/ajustable desde que la ESP recibe la orden hasta que la garra
// llega a cerrada. La orden se adelanta esta cantidad usando la velocidad real
// del encoder, mas el margen de transporte RS485.
constexpr unsigned long V2_TIEMPO_CIERRE_PINZA_MS = 450UL;
// Solicitud pendiente + respuesta + nueva orden, con margen de loop.
constexpr unsigned long V2_LATENCIA_ORDEN_PINZA_MS = 158UL;
static_assert(V2_LATENCIA_ORDEN_PINZA_MS >=
              3UL * EnlaceRS485::TIEMPO_TRAMA_MS + 20UL,
              "La anticipacion de pinza debe cubrir el turno del transporte");
static_assert(V2_LATENCIA_ORDEN_PINZA_MS >= EnlaceRS485::TIMEOUT_RESPUESTA_MS +
              EnlaceRS485::GIRO_SOLICITUD_MS + EnlaceRS485::TIEMPO_TRAMA_MS + 20UL,
              "La anticipacion debe cubrir una solicitud previa sin respuesta");
// Aumentar si el catch sigue llegando tarde al acelerar: es tiempo, no mm.
// El adelanto extra no alarga el recorrido mecanico ni el tiempo de retirada.
constexpr float CATCH_ADELANTO_EXTRA_MS = 0.0f;
constexpr float ENCODER_FILTRO_VELOCIDAD_ALPHA = 0.5f;
constexpr long V2_Z_SEGURO_PASOS = 0;
constexpr float V2_ERROR_ESTABLE_MM = 5.0f;
constexpr unsigned long V2_TIEMPO_COMPLETADO_MS = 500UL;
constexpr unsigned long V2_TIMEOUT_FASE_MS = 30000UL;
constexpr long V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS = 2L * PASOS_SEPARACION;
constexpr uint16_t V2_DIV_BUSQUEDA_FINAL_Z = DIV_CAL_LENTA;
constexpr unsigned long V2_PERIODO_LOG_TELEMETRIA_MS = 50UL;
constexpr float ML_MARGEN_FINAL_DERECHO_MM = 10.0f;
constexpr unsigned long ML_TIEMPO_SERVO_MS = 450UL;
constexpr unsigned long ML_TIEMPO_LISTO_MS = 700UL;
constexpr float ML_SEGUIMIENTO_MARGEN_Y_MM = 4.0f;
constexpr float ML_SEGUIMIENTO_RESERVA_S = 0.20f;
constexpr unsigned long ML_SEGUIMIENTO_LOG_MS = 200UL;
// Automatico V2 concentra las mejoras comprobables en los modos de prueba.
// false conserva el catch en estacion fija para comparar ambos recorridos.
constexpr bool AUTO_V2_SEGUIMIENTO_Y = true;
constexpr unsigned long V2_SEGUIMIENTO_ESTABLE_MS = 300UL;
// Positivo retrasa el descenso; negativo reduce la espera estable (minimo 100 ms).
// CAMBIOS CATCH propone esta linea tras evaluar los ensayos del modo separado.
constexpr int32_t V2_AJUSTE_DISPARO_CATCH_MS = 0;

struct ContextoAutomaticoV2 {
    FaseAutomaticoV2 fase;
    unsigned long inicioFase;
    unsigned long inicioEstable;
    bool salidaAEsperaControl;
    bool salidaAlMenu;
    uint16_t secuencia;
    uint8_t clase;
    float camXReferencia;
    float camYReferencia;
    int32_t conteoReferencia;
    float objetivoBrazoX;
    float objetivoBrazoY;
    float objetivoBrazoYAnterior;
    int32_t ultimoConteoProcesado;
    unsigned long instanteObjetivoAnterior;
    float velocidadObjetivoY;
    float velocidadInicialY;
    bool busquedaFinalZActiva;
    float umbralDisparoY;
    unsigned long instanteListoCatch;
    int32_t conteoListoCatch;
    bool catchAutomaticoDisparado;
    uint8_t anguloCatch;
    float umbralCierrePinzaY;
    float ultimoErrorY;
    float ultimoErrorX;
    unsigned long ultimoLogSeguimiento;
    int32_t ajusteProbadoMs;
    bool referenciaCatchRegistrada;
    bool referenciaCatchProyectada;
    unsigned long referenciaCatchMs;
    unsigned long disparoCatchMs;
    unsigned long din04CatchMs;
    unsigned long cierreCatchMs;
    float errorYDisparo;
    float velocidadDisparo;
    int32_t conteoDisparoCatch;
    float brazoYDisparo;
    float piezaYDisparo;
};

ContextoAutomaticoV2 automaticoV2 = {};
uint32_t intentosV2 = 0;
uint32_t exitosV2 = 0;
unsigned long ultimoLogTelemetriaV2 = 0;

bool ajusteCatchV2Seleccionado() {
    return opcionMenu == MENU_AJUSTE_CATCH_V2;
}

// Persiste entre piezas y al salir/entrar al menu; se reinicia al reiniciar placa.
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
}

struct ContextoEntrenamientoML {
    FaseEntrenamientoML fase;
    unsigned long inicioFase;
    uint16_t secuencia;
    uint8_t clase;
    float camX;
    float camY;
    float xInicial;
    float yInicial;
    float xCorregida;
    float yCorregida;
    float piezaYEstimada;
    float catchYConfirmada;
    float umbralCierreY;
    int32_t conteoReferencia;
    uint8_t rotacionCorregida;
    bool busquedaFinalZActiva;
    bool rebaseManualRegistrado;
    bool salidaAlMenu;
    bool salidaAEsperaControl;
    float velocidadDisparo;
    float piezaYDisparo;
    float diferenciaDisparoMm;
    unsigned long instanteOrdenCierre;
    unsigned long instanteDeteccion;
    unsigned long instanteBotonCatch;
    unsigned long instanteDin04Catch;
    int32_t conteoBotonCatch;
    int32_t conteoOrdenCierre;
    float piezaYBotonCatch;
    const char *disparadorCatch;
    float errorSeguimientoBoton;
    float errorSeguimientoDin04;
    float errorSeguimientoCierre;
    float brazoYBoton;
    float brazoYDin04;
    float brazoYCierre;
    unsigned long inicioSeguimiento;
    unsigned long ultimoLogSeguimiento;
};

ContextoEntrenamientoML entrenamientoML = {};
uint32_t muestrasEntrenamientoML = 0;

bool ensenanzaMLV2Seleccionada() {
    return opcionMenu == MENU_ENTRENAMIENTO_ML_V2;
}

bool pruebaSeguimientoSeleccionada() {
    return opcionMenu == MENU_PRUEBA_SEGUIMIENTO;
}

bool entrenamientoConResultadoSeleccionado() {
    return ensenanzaMLV2Seleccionada() || pruebaSeguimientoSeleccionada();
}

// Solo observacion: no reinicia el contador hardware ni acciona motores/servos.
struct ContextoPruebaEncoder {
    uint32_t ensayo;
    uint32_t marcas;
    uint32_t inicioMs;
    uint32_t finMs;
    uint32_t ultimoLogMs;
    int32_t conteoInicio;
    bool iniciado;
    bool congelado;
    int32_t conteoFinal;
};
ContextoPruebaEncoder pruebaEncoder = {};

unsigned long tiempoEncendidoSistema = 0;
unsigned long tAnteriorRS485 = 0;
unsigned long tAnteriorEstadoESP = 0;
EnlaceRS485::Receptor receptorRS485;
EnlaceRS485::Cliente clienteRS485;
bool envioRS485Urgente = false;
uint32_t respuestasRS485Ajenas = 0, timeoutsRS485 = 0;
uint32_t ultimaLatenciaRS485Ms = 0, maximaLatenciaRS485Ms = 0;

bool comunicacionRS485Habilitada = false;
uint32_t lecturasRS485Ok = 0;
uint32_t lecturasRS485Error = 0;
uint32_t erroresRS485Longitud = 0;
uint32_t erroresRS485CRC = 0;
uint32_t erroresRS485Semantica = 0;
uint32_t enviosRS485Ok = 0;
uint32_t enviosRS485Error = 0;
uint8_t ultimoCodigoErrorEnvioRS485 = 0;
uint32_t reiniciosBusRS485Maestro = 0;
unsigned long ultimoReinicioBusRS485Ms = 0;
unsigned long ultimaEntradaLoopMs = 0;
unsigned long maximaPausaLoopMs = 0;
unsigned long ultimoReporteRS485 = 0;

String lineaTerminal = "";

// Declaraciones de funciones que cruzan secciones.
void entrarErrorSistema(CodigoErrorLocal codigo, const char *mensaje);
void cambiarEstadoGeneral(EstadoGeneral nuevoEstado);
void cancelarMovimientoPosicionado(const char *motivo, bool posicionPerdida);
void registrarEventoPortentaV2(const char *evento, const char *mensaje);
void reiniciarAutomaticoV2();
long posicionCapturaZV2();
long posicionPrecapturaZV2();
void avanzarEntradaModo();
void terminarCalibracionSolicitada();
void iniciarCancelacionAutomaticoV2(
    const char *motivo,
    bool esperarControl,
    bool salirDelModo = false
);
long posicionCapturaZV2();
void iniciarCalibracionEncoder();
void procesarCalibracionEncoder();
void reiniciarEntrenamientoML();
void procesarEntrenamientoML();
void reiniciarMedicionPruebaEncoder();
uint32_t pulsosPruebaEncoder();
uint32_t milisegundosPruebaEncoder();
void registrarPruebaEncoder(const char *evento);
void procesarPruebaEncoder();

//-------------------------------------------------------------------------------------------------
// ACCESO ATOMICO A CONTADORES Y OBJETIVOS
//-------------------------------------------------------------------------------------------------
long leerPasosX() {
    noInterrupts();
    long valor = pasosX;
    interrupts();
    return valor;
}

long leerPasosY() {
    noInterrupts();
    long valor = pasosY;
    interrupts();
    return valor;
}

long leerPasosZ() {
    noInterrupts();
    long valor = pasosZ;
    interrupts();
    return valor;
}

void fijarPasosX(long valor) {
    noInterrupts();
    pasosX = valor;
    interrupts();
}

void fijarPasosY(long valor) {
    noInterrupts();
    pasosY = valor;
    interrupts();
}

void fijarPasosZ(long valor) {
    noInterrupts();
    pasosZ = valor;
    interrupts();
}

bool objetivoXEnCurso() {
    noInterrupts();
    bool valor = objetivoXActivo;
    interrupts();
    return valor;
}

bool objetivoYEnCurso() {
    noInterrupts();
    bool valor = objetivoYActivo;
    interrupts();
    return valor;
}

bool objetivoZEnCurso() {
    noInterrupts();
    bool valor = objetivoZActivo;
    interrupts();
    return valor;
}

bool motoresEnMovimiento() {
    noInterrupts();
    bool valor = (movX != 0 || movY != 0 || movZ != 0 ||
                  objetivoXActivo || objetivoYActivo || objetivoZActivo);
    interrupts();
    return valor;
}

//-------------------------------------------------------------------------------------------------
// FINALES DE CARRERA: LOGICA NC CONSERVADA
//-------------------------------------------------------------------------------------------------
void leerFinalesCarrera() {
    // true = presionado o circuito NC abierto/desconectado.
    limiteXmas = !digital_inputs.read(DIN_READ_CH_PIN_01);
    limiteXmenos = !digital_inputs.read(DIN_READ_CH_PIN_00);
    limiteYmenos = !digital_inputs.read(DIN_READ_CH_PIN_02);
    limiteYmas = !digital_inputs.read(DIN_READ_CH_PIN_03);
    // Cableado fisico de esta maquina: DIN04 esta abajo y DIN05 arriba.
    limiteZabajo = !digital_inputs.read(DIN_READ_CH_PIN_04);
    limiteZarriba = !digital_inputs.read(DIN_READ_CH_PIN_05);
}

bool finalesCoherentes() {
    return !(limiteXmas && limiteXmenos) &&
           !(limiteYmas && limiteYmenos) &&
           !(limiteZarriba && limiteZabajo);
}

bool algunFinalActivo() {
    return limiteXmas || limiteXmenos || limiteYmas ||
           limiteYmenos || limiteZarriba || limiteZabajo;
}

//-------------------------------------------------------------------------------------------------
// TICKER DE GENERACION DE PULSOS
//-------------------------------------------------------------------------------------------------
void generarPulsoMotor() {
    if (movX != 0) {
        cuentaX++;
        if (cuentaX >= divisorX) {
            cuentaX = 0;
            pulsoX = !pulsoX;
            digital_outputs.set(pP_X, pulsoX ? HIGH : LOW);
            if (pulsoX) {
                pasosX += movX;
                if (objetivoXActivo &&
                    ((movX > 0 && pasosX >= objetivoX) ||
                     (movX < 0 && pasosX <= objetivoX))) {
                    pasosX = objetivoX;
                    movX = 0;
                    objetivoXActivo = false;
                    digital_outputs.set(pP_X, LOW);
                }
            }
        }
    } else {
        cuentaX = 0;
        pulsoX = false;
        digital_outputs.set(pP_X, LOW);
    }

    if (movY != 0) {
        cuentaY++;
        if (cuentaY >= divisorY) {
            cuentaY = 0;
            pulsoY = !pulsoY;
            digital_outputs.set(pP_Y, pulsoY ? HIGH : LOW);
            if (pulsoY) {
                pasosY += movY;
                if (objetivoYActivo &&
                    ((movY > 0 && pasosY >= objetivoY) ||
                     (movY < 0 && pasosY <= objetivoY))) {
                    pasosY = objetivoY;
                    movY = 0;
                    objetivoYActivo = false;
                    digital_outputs.set(pP_Y, LOW);
                }
            }
        }
    } else {
        cuentaY = 0;
        pulsoY = false;
        digital_outputs.set(pP_Y, LOW);
    }

    if (movZ != 0) {
        cuentaZ++;
        if (cuentaZ >= divisorZ) {
            cuentaZ = 0;
            pulsoZ = !pulsoZ;
            digital_outputs.set(pP_Z, pulsoZ ? HIGH : LOW);
            if (pulsoZ) {
                pasosZ += movZ;
                if (objetivoZActivo &&
                    ((movZ > 0 && pasosZ >= objetivoZ) ||
                     (movZ < 0 && pasosZ <= objetivoZ))) {
                    pasosZ = objetivoZ;
                    movZ = 0;
                    objetivoZActivo = false;
                    digital_outputs.set(pP_Z, LOW);
                }
            }
        }
    } else {
        cuentaZ = 0;
        pulsoZ = false;
        digital_outputs.set(pP_Z, LOW);
    }
}

//-------------------------------------------------------------------------------------------------
// PRIMITIVAS DE MOVIMIENTO CONSERVADAS
//-------------------------------------------------------------------------------------------------
void detenerX() {
    noInterrupts();
    movX = 0;
    objetivoXActivo = false;
    cuentaX = 0;
    pulsoX = false;
    interrupts();
    digital_outputs.set(pP_X, LOW);
}

void detenerY() {
    noInterrupts();
    movY = 0;
    objetivoYActivo = false;
    cuentaY = 0;
    pulsoY = false;
    interrupts();
    digital_outputs.set(pP_Y, LOW);
}

void detenerZ() {
    noInterrupts();
    movZ = 0;
    objetivoZActivo = false;
    cuentaZ = 0;
    pulsoZ = false;
    interrupts();
    digital_outputs.set(pP_Z, LOW);
}

void detenerTodos() {
    detenerX();
    detenerY();
    detenerZ();
}

void moverXContinuo(int8_t direccion, uint16_t divisor) {
    if (direccion == 0) {
        detenerX();
        return;
    }
    noInterrupts();
    bool igual = movX == direccion && !objetivoXActivo && divisorX == divisor;
    interrupts();
    if (igual) return;
    detenerX();
    delayMicroseconds(150);
    digital_outputs.set(pD_X, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    divisorX = divisor;
    cuentaX = 0;
    pulsoX = false;
    objetivoXActivo = false;
    movX = direccion;
    interrupts();
}

void moverYContinuo(int8_t direccion, uint16_t divisor) {
    if (direccion == 0) {
        detenerY();
        return;
    }
    noInterrupts();
    bool igual = movY == direccion && !objetivoYActivo && divisorY == divisor;
    interrupts();
    if (igual) return;
    detenerY();
    delayMicroseconds(150);
    digital_outputs.set(pD_Y, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    divisorY = divisor;
    cuentaY = 0;
    pulsoY = false;
    objetivoYActivo = false;
    movY = direccion;
    interrupts();
}

void moverZContinuo(int8_t direccion, uint16_t divisor) {
    if (direccion == 0) {
        detenerZ();
        return;
    }
    noInterrupts();
    bool igual = movZ == direccion && !objetivoZActivo && divisorZ == divisor;
    interrupts();
    if (igual) return;
    detenerZ();
    delayMicroseconds(150);
    // El driver Z esta invertido respecto a X/Y: signo negativo baja.
    digital_outputs.set(pD_Z, direccion > 0 ? LOW : HIGH);
    delayMicroseconds(10);
    noInterrupts();
    divisorZ = divisor;
    cuentaZ = 0;
    pulsoZ = false;
    objetivoZActivo = false;
    movZ = direccion;
    interrupts();
}

void moverXHasta(long destino, uint16_t divisor) {
    long actual = leerPasosX();
    if (actual == destino) {
        detenerX();
        return;
    }
    int8_t direccion = destino > actual ? 1 : -1;
    detenerX();
    delayMicroseconds(150);
    digital_outputs.set(pD_X, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    objetivoX = destino;
    objetivoXActivo = true;
    divisorX = divisor;
    cuentaX = 0;
    pulsoX = false;
    movX = direccion;
    interrupts();
}

void moverYHasta(long destino, uint16_t divisor) {
    long actual = leerPasosY();
    if (actual == destino) {
        detenerY();
        return;
    }
    int8_t direccion = destino > actual ? 1 : -1;
    detenerY();
    delayMicroseconds(150);
    digital_outputs.set(pD_Y, direccion > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    noInterrupts();
    objetivoY = destino;
    objetivoYActivo = true;
    divisorY = divisor;
    cuentaY = 0;
    pulsoY = false;
    movY = direccion;
    interrupts();
}

void moverZHasta(long destino, uint16_t divisor) {
    long actual = leerPasosZ();
    if (actual == destino) {
        detenerZ();
        return;
    }
    int8_t direccion = destino > actual ? 1 : -1;
    detenerZ();
    delayMicroseconds(150);
    // Mantiene la misma convencion logica: negativo=abajo, positivo=arriba.
    digital_outputs.set(pD_Z, direccion > 0 ? LOW : HIGH);
    delayMicroseconds(10);
    noInterrupts();
    objetivoZ = destino;
    objetivoZActivo = true;
    divisorZ = divisor;
    cuentaZ = 0;
    pulsoZ = false;
    movZ = direccion;
    interrupts();
}

void aplicarBloqueoPorFinales() {
    if (movX > 0 && limiteXmas) detenerX();
    if (movX < 0 && limiteXmenos) detenerX();
    if (movY > 0 && limiteYmas) detenerY();
    if (movY < 0 && limiteYmenos) detenerY();
    if (movZ > 0 && limiteZarriba) detenerZ();
    if (movZ < 0 && limiteZabajo) detenerZ();
}

//-------------------------------------------------------------------------------------------------
// ESCALA, POSICION Y CINEMATICA CARTESIANA
//-------------------------------------------------------------------------------------------------
bool escalaConfigurada() {
    return pasosPorMmX > 0.0f && pasosPorMmY > 0.0f;
}

float posicionXmm() {
    return pasosPorMmX > 0.0f ? (float)leerPasosX() / pasosPorMmX : 0.0f;
}

float posicionYmm() {
    return pasosPorMmY > 0.0f ? (float)leerPasosY() / pasosPorMmY : 0.0f;
}

float rangoXmm() {
    return pasosPorMmX > 0.0f ? (float)rangoXPasos / pasosPorMmX : 0.0f;
}

float rangoYmm() {
    return pasosPorMmY > 0.0f ? (float)rangoYPasos / pasosPorMmY : 0.0f;
}

long limiteMinimoXPasos() { return -(rangoXPasos / 2); }
long limiteMaximoXPasos() { return rangoXPasos - rangoXPasos / 2; }
long limiteMinimoYPasos() { return -(rangoYPasos / 2); }
long limiteMaximoYPasos() { return rangoYPasos - rangoYPasos / 2; }
long limiteMinimoZPasos() { return -(rangoZPasos / 2); }
long limiteMaximoZPasos() { return rangoZPasos - rangoZPasos / 2; }

bool posicionZSeguraV2(long destino) {
    if (!calibracionZValida || rangoZPasos <= 0) {
        return false;
    }
    return destino >= limiteMinimoZPasos() &&
           destino <= limiteMaximoZPasos();
}

bool calcularEscalaAutomatica() {
    if (rangoXPasos <= 0 || rangoYPasos <= 0) {
        pasosPorMmX = 0.0f;
        pasosPorMmY = 0.0f;
        Serial.println(F("[CAL][ERROR] Rangos invalidos para calcular escala"));
        return false;
    }
    pasosPorMmX = (float)rangoXPasos / RANGO_FISICO_X_MM;
    pasosPorMmY = (float)rangoYPasos / RANGO_FISICO_Y_MM;
    Serial.print(F("[CAL] Escala X="));
    Serial.print(pasosPorMmX, 6);
    Serial.print(F(" pasos/mm, Y="));
    Serial.print(pasosPorMmY, 6);
    Serial.println(F(" pasos/mm"));
    return true;
}

bool cinematicaInversaCartesiana(float xMm, float yMm, float zMm,
                                 long &xPasosDestino, long &yPasosDestino) {
    if (!isfinite(xMm) || !isfinite(yMm) || !isfinite(zMm)) {
        Serial.println(F("[ERROR] Coordenada no finita rechazada"));
        return false;
    }
    if (!calibracionXYValida || !escalaConfigurada()) {
        Serial.println(F("[ERROR] X/Y no estan calibrados"));
        return false;
    }

    xPasosDestino = lroundf(xMm * pasosPorMmX);
    yPasosDestino = lroundf(yMm * pasosPorMmY);
    const long margenX = lroundf(MARGEN_SEGURIDAD_MM * pasosPorMmX);
    const long margenY = lroundf(MARGEN_SEGURIDAD_MM * pasosPorMmY);
    const long xMin = limiteMinimoXPasos() + margenX;
    const long xMax = limiteMaximoXPasos() - margenX;
    const long yMin = limiteMinimoYPasos() + margenY;
    const long yMax = limiteMaximoYPasos() - margenY;

    if (xPasosDestino < xMin || xPasosDestino > xMax ||
        yPasosDestino < yMin || yPasosDestino > yMax) {
        Serial.print(F("[WARN] Objetivo fuera de rango seguro X="));
        Serial.print(xMm, 2);
        Serial.print(F(" Y="));
        Serial.println(yMm, 2);
        return false;
    }
    return true;
}

bool transformarCamaraABrazo(float camXmm, float camYmm,
                             float &brazoXmm, float &brazoYmm) {
    if (!isfinite(camXmm) || !isfinite(camYmm)) return false;
    const float baseX = CAMERA_SWAP_XY ? camYmm : camXmm;
    const float baseY = CAMERA_SWAP_XY ? camXmm : camYmm;
    brazoXmm = (float)CAMERA_SIGN_X * baseX + CAMERA_OFFSET_X_MM;
    brazoYmm = (float)CAMERA_SIGN_Y * baseY + CAMERA_OFFSET_Y_MM;
    return isfinite(brazoXmm) && isfinite(brazoYmm);
}

bool brazoEnHome() {
    return calibracionXYValida && calibracionZValida &&
           leerPasosX() == 0 && leerPasosY() == 0 && leerPasosZ() == 0 &&
           !motoresEnMovimiento() && !movimientoPosicionadoActivo;
}

//-------------------------------------------------------------------------------------------------
// MOVIMIENTO POSICIONADO XY (TERMINAL Y AUTOMATICO)
//-------------------------------------------------------------------------------------------------
bool iniciarMovimientoXY(float xMm, float yMm, float zMm,
                         PropietarioMovimiento propietario) {
    long destinoX = 0;
    long destinoY = 0;
    if (!cinematicaInversaCartesiana(xMm, yMm, zMm, destinoX, destinoY)) {
        return false;
    }

    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;

    objetivoXmm = xMm;
    objetivoYmm = yMm;
    objetivoZmm = zMm;
    objetivoCartesianoXPasos = destinoX;
    objetivoCartesianoYPasos = destinoY;

    Serial.print(propietario == MOV_AUTOMATICO ? F("[AUTO] Objetivo X=") : F("[XYZ] Objetivo X="));
    Serial.print(xMm, 2);
    Serial.print(F(" mm, Y="));
    Serial.print(yMm, 2);
    Serial.print(F(" mm; Z="));
    Serial.print(zMm, 2);
    Serial.println(F(" mm; ordenando movimiento XY"));

    moverXHasta(destinoX, DIV_POSICION);
    moverYHasta(destinoY, DIV_POSICION);

    movimientoPosicionadoActivo = objetivoXEnCurso() || objetivoYEnCurso();
    propietarioMovimiento = movimientoPosicionadoActivo ? propietario : MOV_SIN_PROPIETARIO;
    inicioMovimientoPosicionado = millis();

    if (!movimientoPosicionadoActivo) {
        Serial.println(propietario == MOV_AUTOMATICO
                           ? F("[AUTO] Brazo ya estaba en el objetivo")
                           : F("[XYZ] Brazo ya estaba en el objetivo"));
    }
    return true;
}

void cancelarMovimientoPosicionado(const char *motivo, bool posicionPerdida) {
    PropietarioMovimiento propietarioAnterior = propietarioMovimiento;
    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;

    if (posicionPerdida) calibracionXYValida = false;
    if (propietarioAnterior == MOV_AUTOMATICO && secuenciaObjetivoEnMovimiento != 0) {
        ackSecuenciaObjetivo = secuenciaObjetivoEnMovimiento;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
    }

    Serial.print(F("[XYZ] Movimiento detenido: "));
    Serial.println(motivo);
}

void actualizarMovimientoPosicionado() {
    if (!movimientoPosicionadoActivo) return;

    const bool finalInesperado =
        (movX > 0 && limiteXmas) || (movX < 0 && limiteXmenos) ||
        (movY > 0 && limiteYmas) || (movY < 0 && limiteYmenos);

    if (finalInesperado) {
        cancelarMovimientoPosicionado("final de carrera inesperado", true);
        entrarErrorSistema(ERROR_FINAL_INESPERADO,
                           "Final de carrera durante movimiento XY");
        return;
    }

    if (millis() - inicioMovimientoPosicionado > TIMEOUT_MOVIMIENTO_MS) {
        cancelarMovimientoPosicionado("timeout", true);
        entrarErrorSistema(ERROR_TIMEOUT_MOVIMIENTO,
                           "Timeout de movimiento XY");
        return;
    }

    if (!objetivoXEnCurso() && !objetivoYEnCurso() && movX == 0 && movY == 0) {
        PropietarioMovimiento propietarioFinal = propietarioMovimiento;
        movimientoPosicionadoActivo = false;
        propietarioMovimiento = MOV_SIN_PROPIETARIO;
        Serial.println(propietarioFinal == MOV_AUTOMATICO
                           ? F("[AUTO] Pulsos XY ordenados completados (sin realimentacion del brazo)")
                           : F("[XYZ] Pulsos XY ordenados completados (sin realimentacion del brazo)"));
    }
}

//-------------------------------------------------------------------------------------------------
// CALIBRACION X/Y/Z CONSERVADA, AHORA PROCESADA DIRECTAMENTE DESDE loop()
//-------------------------------------------------------------------------------------------------
const char *nombreFaseCalibracion() {
    switch (faseCal) {
        case CAL_ESPERA: return "ESPERA";
        case CAL_X_MIN_1: return "BUSCANDO X-";
        case CAL_X_MIN_LIBERAR: return "LIBERANDO X-";
        case CAL_X_MIN_SEPARAR: return "SEPARANDO X-";
        case CAL_X_MIN_2: return "VERIFICANDO X-";
        case CAL_X_MAX_1: return "BUSCANDO X+";
        case CAL_X_MAX_LIBERAR: return "LIBERANDO X+";
        case CAL_X_MAX_SEPARAR: return "SEPARANDO X+";
        case CAL_X_MAX_2: return "VERIFICANDO X+";
        case CAL_Y_MIN_1: return "BUSCANDO Y-";
        case CAL_Y_MIN_LIBERAR: return "LIBERANDO Y-";
        case CAL_Y_MIN_SEPARAR: return "SEPARANDO Y-";
        case CAL_Y_MIN_2: return "VERIFICANDO Y-";
        case CAL_Y_MAX_1: return "BUSCANDO Y+";
        case CAL_Y_MAX_LIBERAR: return "LIBERANDO Y+";
        case CAL_Y_MAX_SEPARAR: return "SEPARANDO Y+";
        case CAL_Y_MAX_2: return "VERIFICANDO Y+";
        case CAL_Z_MIN_1: return "BUSCANDO Z ABAJO";
        case CAL_Z_MIN_LIBERAR: return "LIBERANDO Z ABAJO";
        case CAL_Z_MIN_SEPARAR: return "SEPARANDO Z ABAJO";
        case CAL_Z_MIN_2: return "VERIFICANDO Z ABAJO";
        case CAL_Z_MAX_1: return "BUSCANDO Z ARRIBA";
        case CAL_Z_MAX_LIBERAR: return "LIBERANDO Z ARRIBA";
        case CAL_Z_MAX_SEPARAR: return "SEPARANDO Z ARRIBA";
        case CAL_Z_MAX_2: return "VERIFICANDO Z ARRIBA";
        case CAL_HOME: return "YENDO A HOME";
        case CAL_COMPLETA: return "CALIBRACION OK";
        case CAL_ERROR: return "ERROR";
        default: return "DESCONOCIDA";
    }
}

void cambiarFaseCalibracion(FaseCalibracion nuevaFase) {
    faseCal = nuevaFase;
    inicioFase = millis();
    Serial.print(F("[CAL] "));
    Serial.println(nombreFaseCalibracion());
}

uint8_t codigoErrorCalibracion() {
    if (faseCal != CAL_ERROR) return 0;
    if (strcmp(mensajeErrorCalibracion, "Tiempo maximo excedido") == 0) return 1;
    if (strcmp(mensajeErrorCalibracion, "Rango X invalido") == 0) return 2;
    if (strcmp(mensajeErrorCalibracion, "Rango Y invalido") == 0) return 3;
    if (strcmp(mensajeErrorCalibracion, "Rango Z invalido") == 0) return 4;
    if (strcmp(mensajeErrorCalibracion, "No se pudo calcular la escala") == 0) return 5;
    if (strcmp(mensajeErrorCalibracion, "HOME interrumpido") == 0) return 6;
    return 255;
}

void detenerPorErrorCalibracion(const char *texto) {
    detenerTodos();
    calibracionXYValida = false;
    calibracionZValida = false;
    mensajeErrorCalibracion = texto;
    cambiarFaseCalibracion(CAL_ERROR);
    Serial.print(F("[CAL][ERROR] "));
    Serial.println(texto);
    entrarErrorSistema(ERROR_CALIBRACION_BRAZO, texto);
}

void iniciarCalibracionBrazo() {
    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    calibracionXYValida = false;
    calibracionZValida = false;
    pasosPorMmX = 0.0f;
    pasosPorMmY = 0.0f;
    rangoXPasos = 0;
    rangoYPasos = 0;
    rangoZPasos = 0;
    homeIniciado = false;
    mensajeErrorCalibracion = "";
    Serial.println(F("[CAL] Inicio autonomo de calibracion X/Y/Z"));
    cambiarFaseCalibracion(CAL_X_MIN_1);
}

void procesarCalibracionBrazo() {
    if (faseCal == CAL_ESPERA || faseCal == CAL_COMPLETA || faseCal == CAL_ERROR) {
        return;
    }

    if (millis() - inicioFase > TIMEOUT_FASE_MS) {
        detenerPorErrorCalibracion("Tiempo maximo excedido");
        return;
    }

    switch (faseCal) {
        case CAL_X_MIN_1:
            if (limiteXmenos) {
                detenerX();
                cambiarFaseCalibracion(CAL_X_MIN_LIBERAR);
            } else moverXContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MIN_LIBERAR:
            if (!limiteXmenos) {
                detenerX();
                moverXHasta(leerPasosX() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_X_MIN_SEPARAR);
            } else moverXContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MIN_SEPARAR:
            if (!objetivoXEnCurso()) cambiarFaseCalibracion(CAL_X_MIN_2);
            break;

        case CAL_X_MIN_2:
            if (limiteXmenos) {
                detenerX();
                fijarPasosX(0);
                cambiarFaseCalibracion(CAL_X_MAX_1);
            } else moverXContinuo(-1, DIV_CAL_LENTA);
            break;

        case CAL_X_MAX_1:
            if (limiteXmas) {
                detenerX();
                cambiarFaseCalibracion(CAL_X_MAX_LIBERAR);
            } else moverXContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MAX_LIBERAR:
            if (!limiteXmas) {
                detenerX();
                moverXHasta(leerPasosX() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_X_MAX_SEPARAR);
            } else moverXContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_X_MAX_SEPARAR:
            if (!objetivoXEnCurso()) cambiarFaseCalibracion(CAL_X_MAX_2);
            break;

        case CAL_X_MAX_2:
            if (limiteXmas) {
                detenerX();
                rangoXPasos = leerPasosX();
                if (rangoXPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorErrorCalibracion("Rango X invalido");
                    return;
                }
                Serial.print(F("[CAL][X] Rango="));
                Serial.println(rangoXPasos);
                cambiarFaseCalibracion(CAL_Y_MIN_1);
            } else moverXContinuo(1, DIV_CAL_LENTA);
            break;

        case CAL_Y_MIN_1:
            if (limiteYmenos) {
                detenerY();
                cambiarFaseCalibracion(CAL_Y_MIN_LIBERAR);
            } else moverYContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MIN_LIBERAR:
            if (!limiteYmenos) {
                detenerY();
                moverYHasta(leerPasosY() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Y_MIN_SEPARAR);
            } else moverYContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MIN_SEPARAR:
            if (!objetivoYEnCurso()) cambiarFaseCalibracion(CAL_Y_MIN_2);
            break;

        case CAL_Y_MIN_2:
            if (limiteYmenos) {
                detenerY();
                fijarPasosY(0);
                cambiarFaseCalibracion(CAL_Y_MAX_1);
            } else moverYContinuo(-1, DIV_CAL_LENTA);
            break;

        case CAL_Y_MAX_1:
            if (limiteYmas) {
                detenerY();
                cambiarFaseCalibracion(CAL_Y_MAX_LIBERAR);
            } else moverYContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MAX_LIBERAR:
            if (!limiteYmas) {
                detenerY();
                moverYHasta(leerPasosY() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Y_MAX_SEPARAR);
            } else moverYContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Y_MAX_SEPARAR:
            if (!objetivoYEnCurso()) cambiarFaseCalibracion(CAL_Y_MAX_2);
            break;

        case CAL_Y_MAX_2:
            if (limiteYmas) {
                detenerY();
                rangoYPasos = leerPasosY();
                if (rangoYPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorErrorCalibracion("Rango Y invalido");
                    return;
                }
                Serial.print(F("[CAL][Y] Rango="));
                Serial.println(rangoYPasos);
                cambiarFaseCalibracion(CAL_Z_MIN_1);
            } else moverYContinuo(1, DIV_CAL_LENTA);
            break;

        case CAL_Z_MIN_1:
            if (limiteZabajo) {
                detenerZ();
                cambiarFaseCalibracion(CAL_Z_MIN_LIBERAR);
            } else moverZContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MIN_LIBERAR:
            if (!limiteZabajo) {
                detenerZ();
                moverZHasta(leerPasosZ() + PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Z_MIN_SEPARAR);
            } else moverZContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MIN_SEPARAR:
            if (!objetivoZEnCurso()) cambiarFaseCalibracion(CAL_Z_MIN_2);
            break;

        case CAL_Z_MIN_2:
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(0);
                cambiarFaseCalibracion(CAL_Z_MAX_1);
            } else moverZContinuo(-1, DIV_CAL_LENTA);
            break;

        case CAL_Z_MAX_1:
            if (limiteZarriba) {
                detenerZ();
                cambiarFaseCalibracion(CAL_Z_MAX_LIBERAR);
            } else moverZContinuo(1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MAX_LIBERAR:
            if (!limiteZarriba) {
                detenerZ();
                moverZHasta(leerPasosZ() - PASOS_SEPARACION, DIV_CAL_RAPIDA);
                cambiarFaseCalibracion(CAL_Z_MAX_SEPARAR);
            } else moverZContinuo(-1, DIV_CAL_RAPIDA);
            break;

        case CAL_Z_MAX_SEPARAR:
            if (!objetivoZEnCurso()) cambiarFaseCalibracion(CAL_Z_MAX_2);
            break;

        case CAL_Z_MAX_2:
            if (limiteZarriba) {
                detenerZ();
                rangoZPasos = leerPasosZ();
                if (rangoZPasos < RANGO_MINIMO_VALIDO) {
                    detenerPorErrorCalibracion("Rango Z invalido");
                    return;
                }
                Serial.print(F("[CAL][Z] Rango="));
                Serial.println(rangoZPasos);
                homeIniciado = false;
                cambiarFaseCalibracion(CAL_HOME);
            } else moverZContinuo(1, DIV_CAL_LENTA);
            break;

        case CAL_HOME: {
            const long centroX = rangoXPasos / 2;
            const long centroY = rangoYPasos / 2;
            const long centroZ = rangoZPasos / 2;
            if (!homeIniciado) {
                homeIniciado = true;
                moverXHasta(centroX, DIV_HOME);
                moverYHasta(centroY, DIV_HOME);
                moverZHasta(centroZ, DIV_HOME);
            }

            if (!objetivoXEnCurso() && !objetivoYEnCurso() && !objetivoZEnCurso() &&
                movX == 0 && movY == 0 && movZ == 0) {
                if (leerPasosX() != centroX || leerPasosY() != centroY ||
                    leerPasosZ() != centroZ) {
                    detenerPorErrorCalibracion("HOME interrumpido");
                    return;
                }
                fijarPasosX(0);
                fijarPasosY(0);
                fijarPasosZ(0);
                if (!calcularEscalaAutomatica()) {
                    detenerPorErrorCalibracion("No se pudo calcular la escala");
                    return;
                }
                calibracionXYValida = true;
                calibracionZValida = true;
                cambiarFaseCalibracion(CAL_COMPLETA);
                Serial.println(F("[CAL] Calibracion completa; HOME X=0 Y=0 Z=0"));
            }
            break;
        }

        case CAL_ESPERA:
        case CAL_COMPLETA:
        case CAL_ERROR:
            break;
    }
}

//-------------------------------------------------------------------------------------------------
// ADAPTACION DEL ESTADO LOCAL AL PROTOCOLO COMPARTIDO
//-------------------------------------------------------------------------------------------------
uint8_t estadoGeneralWire() {
    switch (estadoGeneral) {
        case EST_BOOT_SAFE: return SISTEMA_ARRANQUE_SEGURO;
        case EST_WAIT_RS485: return SISTEMA_ESPERANDO_RS485;
        case EST_RS485_SETTLE: return SISTEMA_ESPERA_5S;
        case EST_CAMERA_CALIBRATION:
        case EST_USER_CAMERA_CALIBRATION:
            return SISTEMA_CALIBRANDO_CAMARA;
        case EST_ARM_CALIBRATION:
        case EST_USER_ARM_CALIBRATION:
            return SISTEMA_CALIBRANDO_BRAZO;
        case EST_WAIT_CONTROLLER: return SISTEMA_ESPERANDO_CONTROL;
        case EST_FINAL_CHECKLIST: return SISTEMA_CHECKLIST;
        case EST_MAIN_MENU: return SISTEMA_MENU_PRINCIPAL;
        case EST_MANUAL: return SISTEMA_MODO_MANUAL;
        case EST_AUTOMATICO: return SISTEMA_MODO_AUTOMATICO;
        case EST_AUTOMATICO_V2: return SISTEMA_MODO_AUTOMATICO_V2;
        case EST_ENCODER_CALIBRATION: return SISTEMA_CALIBRANDO_ENCODER;
        case EST_CALIBRACIONES_MENU: return SISTEMA_MENU_CALIBRACIONES;
        case EST_PRUEBA_SERVOS: return SISTEMA_PRUEBA_SERVOS;
        case EST_DIAGNOSTICO: return SISTEMA_DIAGNOSTICO;
        case EST_CAMBIOS_CATCH: return SISTEMA_CAMBIOS_CATCH;
        case EST_ENTRENAMIENTO_ML: return SISTEMA_ENTRENAMIENTO_ML;
        case EST_PRUEBA_ENCODER: return SISTEMA_PRUEBA_ENCODER;
        case EST_SYSTEM_ERROR: return SISTEMA_ERROR;
        default: return SISTEMA_ERROR;
    }
}

uint8_t errorSistemaWire() {
    switch (errorSistema) {
        case ERROR_NINGUNO: return SISTEMA_ERROR_NINGUNO;
        case ERROR_TIMEOUT_RS485:
        case ERROR_PROTOCOLO_INVALIDO:
        case ERROR_REINICIO_ESP32:
            return SISTEMA_ERROR_RS485;
        case ERROR_TIMEOUT_CAMARA:
        case ERROR_CAMARA:
            return SISTEMA_ERROR_CAMARA;
        case ERROR_CALIBRACION_BRAZO:
            return SISTEMA_ERROR_CALIBRACION_BRAZO;
        case ERROR_CHECKLIST:
            return SISTEMA_ERROR_CHECKLIST;
        case ERROR_OBJETIVO_INVALIDO:
            return SISTEMA_ERROR_OBJETIVO_FUERA_RANGO;
        case ERROR_TIMEOUT_MOVIMIENTO:
            return SISTEMA_ERROR_TIMEOUT_MOVIMIENTO;
        case ERROR_FINALES_INCOHERENTES:
        case ERROR_FINAL_INESPERADO:
            return SISTEMA_ERROR_FINALES_INCOHERENTES;
        case ERROR_CANCELADO:
            return SISTEMA_ERROR_CANCELADO;
        default:
            return SISTEMA_ERROR_CANCELADO;
    }
}

bool enlaceRS485Vigente() {
    return RecuperacionEnlace::vigente(existePaqueteValido,
                                      millis(), ultimoPaqueteValidoMs);
}

bool baseESPLista() {
    return (paqueteESP.flags & ESP_FLAG_BASE_LISTA) != 0;
}

bool camaraConectada() {
    return (flagsCamara & ESP_FLAG_CAMARA_CONECTADA) != 0;
}

bool homografiaValida() {
    return (flagsCamara & ESP_FLAG_HOMOGRAFIA_VALIDA) != 0;
}

bool modeloListo() {
    return (flagsCamara & ESP_FLAG_MODELO_LISTO) != 0;
}

bool objetivoCamaraValido() {
    return (flagsCamara & ESP_FLAG_OBJETIVO_VALIDO) != 0;
}

bool camaraOcupada() {
    return (flagsCamara & ESP_FLAG_CAMARA_OCUPADA) != 0;
}

bool camaraListaCompleta() {
    return camaraConectada() && homografiaValida() && modeloListo() &&
           estadoCamara == CAMARA_LISTA && !camaraOcupada() && errorCamara == 0;
}

void actualizarEncoderBanda() {
    const unsigned long ahora = millis();
    if (ahora - ultimaMuestraEncoder < PERIODO_ENCODER_MS) return;
    const unsigned long dt = ultimaMuestraEncoder == 0
        ? PERIODO_ENCODER_MS : ahora - ultimaMuestraEncoder;
    ultimaMuestraEncoder = ahora;

    const int32_t conteo = static_cast<int32_t>(encoders[0].getPulses());
    const int revoluciones = encoders[0].getRevolutions();
    ultimoEstadoABEncoder = encoders[0].getCurrentState();
    const int32_t delta = diferenciaConteosConWrap(
        conteo, ultimoConteoEncoderVelocidad
    );

    estadoEncoderBanda = ENC_FLAG_HW_LISTO;
    if (escalaEncoderMmPorCuenta > 0.0f && isfinite(escalaEncoderMmPorCuenta)) {
        estadoEncoderBanda |= ENC_FLAG_ESCALA_VALIDA;
    }
    if (ultimoPulsoEncoder != 0) estadoEncoderBanda |= ENC_FLAG_PULSOS_VISTOS;

    if (delta != 0) {
        const float cuentasInstantaneas = static_cast<float>(delta) *
            1000.0f / static_cast<float>(dt);
        const float instantanea = static_cast<float>(delta) *
            escalaEncoderMmPorCuenta * static_cast<float>(signoEncoderAvance) *
            1000.0f / static_cast<float>(dt);
        frecuenciaEncoderCuentasS = ultimoPulsoEncoder == 0
            ? cuentasInstantaneas
            : 0.8f * frecuenciaEncoderCuentasS +
              0.2f * cuentasInstantaneas;
        velocidadBandaMmS = ultimoPulsoEncoder == 0
            ? instantanea
            : (1.0f - ENCODER_FILTRO_VELOCIDAD_ALPHA) * velocidadBandaMmS +
              ENCODER_FILTRO_VELOCIDAD_ALPHA * instantanea;
        ultimoPulsoEncoder = ahora;
        estadoEncoderBanda |= ENC_FLAG_PULSOS_VISTOS | ENC_FLAG_EN_MOVIMIENTO;
        if (instantanea > 0.0f) estadoEncoderBanda |= ENC_FLAG_DIRECCION_POSITIVA;
    } else if (ultimoPulsoEncoder != 0 &&
               ahora - ultimoPulsoEncoder <= TIMEOUT_MOVIMIENTO_ENCODER_MS) {
        estadoEncoderBanda |= ENC_FLAG_PULSOS_VISTOS | ENC_FLAG_EN_MOVIMIENTO;
        if (velocidadBandaMmS > 0.0f)
            estadoEncoderBanda |= ENC_FLAG_DIRECCION_POSITIVA;
    } else {
        velocidadBandaMmS = 0.0f;
        frecuenciaEncoderCuentasS = 0.0f;
    }

    conteoEncoderBanda = conteo;
    revolucionesIndiceEncoder = revoluciones;
    ultimoConteoEncoderVelocidad = conteo;
    ++secuenciaEncoder;

    if (revoluciones != ultimoIndiceReportado) {
        if (referenciaIndiceDiagnosticoValida) {
            const int deltaIndices = revoluciones - indiceReferenciaDiagnostico;
            if (deltaIndices != 0) {
                const int32_t deltaCuentas = diferenciaConteosConWrap(
                    conteo, conteoReferenciaIndiceDiagnostico
                );
                const uint32_t vueltasCompletas = static_cast<uint32_t>(
                    deltaIndices < 0 ? -deltaIndices : deltaIndices
                );
                const uint64_t cuentasCompletas = static_cast<uint64_t>(
                    deltaCuentas < 0
                        ? -static_cast<int64_t>(deltaCuentas)
                        : static_cast<int64_t>(deltaCuentas)
                );
                ultimaMedicionCuentasPorVuelta =
                    static_cast<float>(cuentasCompletas) /
                    static_cast<float>(vueltasCompletas);
                intervalosIndiceCompletos += vueltasCompletas;
                cuentasIndiceAcumuladas += cuentasCompletas;
            }
        }
        referenciaIndiceDiagnosticoValida = true;
        indiceReferenciaDiagnostico = revoluciones;
        conteoReferenciaIndiceDiagnostico = conteo;
        Serial.print(F("[ENC][INDEX] vueltas="));
        Serial.print(revoluciones);
        Serial.print(F(" conteo="));
        Serial.print(conteo);
        if (ultimaMedicionCuentasPorVuelta > 0.0f) {
            Serial.print(F(" ultima="));
            Serial.print(ultimaMedicionCuentasPorVuelta, 2);
            Serial.print(F(" cuentas/vuelta"));
        }
        Serial.println();
        ultimoIndiceReportado = revoluciones;
    }
}

bool encoderListoAutomaticoV2() {
    const uint8_t requeridos = ENC_FLAG_HW_LISTO |
                               ENC_FLAG_ESCALA_VALIDA;
    return calibracionEncoderValida && velocidadReferencia50MmS > 0.0f &&
           escalaEncoderMmPorCuenta > 0.0f &&
           (estadoEncoderBanda & requeridos) == requeridos &&
           (estadoEncoderBanda & ENC_FLAG_SATURADO) == 0;
}

bool calibracionEncoderListaParaEntrarV2() {
    // La entrada al modo depende de la calibracion persistente de esta sesion,
    // no de que la banda este moviendose ni de una bandera instantanea. Las
    // condiciones dinamicas del encoder se vigilan dentro de V2 antes de
    // aceptar una pieza y durante toda la prediccion.
    return calibracionEncoderValida && velocidadReferencia50MmS > 0.0f &&
           isfinite(velocidadReferencia50MmS) &&
           escalaEncoderMmPorCuenta > 0.0f &&
           isfinite(escalaEncoderMmPorCuenta) &&
           (signoEncoderAvance == 1 || signoEncoderAvance == -1);
}

void cambiarFaseCalibracionEncoder(FaseCalibracionEncoder nueva) {
    calibracionEncoder.fase = nueva;
    calibracionEncoder.inicioFase = millis();
    calibracionEncoder.inicioVentana = millis();
    calibracionEncoder.conteoInicioFase = conteoEncoderBanda;
    calibracionEncoder.conteoInicioVentana = conteoEncoderBanda;
    Serial.print(F("[ENC][CAL] Fase -> "));
    Serial.println(static_cast<uint8_t>(nueva));
}

void fallarCalibracionEncoder(const char *mensaje) {
    calibracionEncoderValida = false;
    velocidadReferencia50MmS = 0.0f;
    velocidadMaximaEstimadaMmS = 0.0f;
    calibracionEncoder.mensajeError = mensaje;
    cambiarFaseCalibracionEncoder(ENC_CAL_ERROR);
    Serial.print(F("[ENC][CAL][ERROR] "));
    Serial.println(mensaje);
    Serial.println(F("[ENC][CAL] Corrija la banda y presione X para reintentar"));
}

void iniciarCalibracionEncoder() {
    calibracionEncoder = {};
    calibracionEncoder.fase = ENC_CAL_ESPERA_50;
    calibracionEncoder.inicioFase = millis();
    calibracionEncoder.inicioVentana = millis();
    calibracionEncoder.conteoInicioFase = conteoEncoderBanda;
    calibracionEncoder.conteoInicioVentana = conteoEncoderBanda;
    calibracionEncoder.minimoCuentasS = INFINITY;
    calibracionEncoder.mensajeError = "";
    calibracionEncoderValida = false;
    velocidadReferencia50MmS = 0.0f;
    velocidadMaximaEstimadaMmS = 0.0f;
    signoEncoderAvance = 1;
    Serial.println(F("[ENC][CAL] Coloque manualmente la banda al 50 %"));
    Serial.println(F("[ENC][CAL] Cuando la banda avance estable camara->brazo, presione X"));
    Serial.print(F("[ENC][CAL] Geometria: rueda="));
    Serial.print(ENCODER_DIAMETRO_RUEDA_MM, 1);
    Serial.print(F(" mm, 1:1, escala="));
    Serial.print(escalaEncoderMmPorCuenta, 9);
    Serial.println(F(" mm/cuenta"));
}

void procesarCalibracionEncoder() {
    detenerTodos();
    if (eventoBotonTriangulo && btConectado) {
        eventoBotonTriangulo = false;
        const EstadoGeneral regreso = modoPendiente == EST_MAIN_MENU
            ? retornoCalibracion : EST_MAIN_MENU;
        modoPendiente = EST_MAIN_MENU;
        cambiarEstadoGeneral(regreso);
        return;
    }
    if (!btConectado) {
        Serial.println(F("[ENC][CAL] Control desconectado; calibracion reiniciada"));
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }

    const unsigned long ahora = millis();
    switch (calibracionEncoder.fase) {
        case ENC_CAL_ESPERA_50:
            if (eventoBotonX) {
                eventoBotonX = false;
                if ((estadoEncoderBanda & ENC_FLAG_EN_MOVIMIENTO) == 0) {
                    Serial.println(F("[ENC][CAL] La banda no se mueve; pongala al 50 % antes de X"));
                    return;
                }
                calibracionEncoder.mensajeError = "";
                cambiarFaseCalibracionEncoder(ENC_CAL_ESTABILIZANDO);
                Serial.println(F("[ENC][CAL] Estabilizando durante 2 s..."));
            }
            break;

        case ENC_CAL_ESTABILIZANDO:
            if (ahora - calibracionEncoder.inicioFase >=
                ENC_TIEMPO_ESTABILIZACION_MS) {
                const int32_t delta = diferenciaConteosConWrap(
                    conteoEncoderBanda, calibracionEncoder.conteoInicioFase);
                if (labs(static_cast<long>(delta)) < 20L) {
                    fallarCalibracionEncoder("No se detecto movimiento continuo al 50 %");
                    return;
                }
                calibracionEncoder.minimoCuentasS = INFINITY;
                calibracionEncoder.maximoCuentasS = 0.0f;
                calibracionEncoder.ventanasValidas = 0;
                calibracionEncoder.ventanasInvalidas = 0;
                calibracionEncoder.signoObservado = delta > 0 ? 1 : -1;
                cambiarFaseCalibracionEncoder(ENC_CAL_MIDIENDO);
                Serial.println(F("[ENC][CAL] Midiendo velocidad durante 5 s..."));
            }
            break;

        case ENC_CAL_MIDIENDO: {
            if (ahora - calibracionEncoder.inicioVentana >=
                ENC_PERIODO_VENTANA_MS) {
                const unsigned long dt = ahora - calibracionEncoder.inicioVentana;
                const int32_t deltaVentana = diferenciaConteosConWrap(
                    conteoEncoderBanda, calibracionEncoder.conteoInicioVentana);
                const int8_t signoVentana = deltaVentana > 0 ? 1 :
                                            (deltaVentana < 0 ? -1 : 0);
                if (signoVentana == 0 ||
                    signoVentana != calibracionEncoder.signoObservado) {
                    if (calibracionEncoder.ventanasInvalidas < 255)
                        ++calibracionEncoder.ventanasInvalidas;
                } else {
                    const float cuentasS = fabsf(static_cast<float>(deltaVentana)) *
                        1000.0f / static_cast<float>(dt);
                    if (cuentasS < calibracionEncoder.minimoCuentasS)
                        calibracionEncoder.minimoCuentasS = cuentasS;
                    if (cuentasS > calibracionEncoder.maximoCuentasS)
                        calibracionEncoder.maximoCuentasS = cuentasS;
                    if (calibracionEncoder.ventanasValidas < 255)
                        ++calibracionEncoder.ventanasValidas;
                }
                calibracionEncoder.inicioVentana = ahora;
                calibracionEncoder.conteoInicioVentana = conteoEncoderBanda;
            }

            if (ahora - calibracionEncoder.inicioFase >= ENC_TIEMPO_MEDICION_MS) {
                const unsigned long dtTotal = ahora - calibracionEncoder.inicioFase;
                const int32_t deltaTotal = diferenciaConteosConWrap(
                    conteoEncoderBanda, calibracionEncoder.conteoInicioFase);
                if (labs(static_cast<long>(deltaTotal)) <
                        static_cast<long>(ENC_CUENTAS_MINIMAS_MEDICION) ||
                    calibracionEncoder.ventanasValidas < ENC_VENTANAS_MINIMAS ||
                    calibracionEncoder.ventanasInvalidas > 1) {
                    fallarCalibracionEncoder("Pulsos insuficientes, paro o inversion durante la medicion");
                    return;
                }

                // El promedio de los 5 s absorbe las perturbaciones breves de
                // velocidad. Las ventanas siguen detectando paro o inversion,
                // pero sus extremos no invalidan una medicion continua.
                const float mediaCuentasS = fabsf(static_cast<float>(deltaTotal)) *
                    1000.0f / static_cast<float>(dtTotal);
                if (!isfinite(mediaCuentasS) || mediaCuentasS <= 0.0f) {
                    fallarCalibracionEncoder("Promedio de velocidad invalido");
                    return;
                }

                signoEncoderAvance = deltaTotal > 0 ? 1 : -1;
                velocidadReferencia50MmS = mediaCuentasS *
                    escalaEncoderMmPorCuenta;
                velocidadMaximaEstimadaMmS = 2.0f * velocidadReferencia50MmS;
                velocidadBandaMmS = 0.0f;
                frecuenciaEncoderCuentasS = 0.0f;
                ultimoConteoEncoderVelocidad = conteoEncoderBanda;

                Serial.print(F("[ENC][CAL] 50 %="));
                Serial.print(velocidadReferencia50MmS, 2);
                Serial.print(F(" mm/s; maxima estimada="));
                Serial.print(velocidadMaximaEstimadaMmS, 2);
                Serial.print(F(" mm/s; signo="));
                Serial.println(signoEncoderAvance);
                Serial.print(F("[ENC][CAL] ventanas min/promedio5s/max="));
                Serial.print(calibracionEncoder.minimoCuentasS, 1);
                Serial.print('/');
                Serial.print(mediaCuentasS, 1);
                Serial.print('/');
                Serial.print(calibracionEncoder.maximoCuentasS, 1);
                Serial.println(F(" cuentas/s"));
                Serial.print(F("[ENC][CAL] ventanas validas="));
                Serial.print(calibracionEncoder.ventanasValidas);
                Serial.print(F(" invalidas="));
                Serial.println(calibracionEncoder.ventanasInvalidas);
                cambiarFaseCalibracionEncoder(ENC_CAL_ESPERA_PARO);
                Serial.println(F("[ENC][CAL] Detenga ahora la banda"));
            }
            break;
        }

        case ENC_CAL_ESPERA_PARO:
            if ((estadoEncoderBanda & ENC_FLAG_EN_MOVIMIENTO) == 0) {
                calibracionEncoderValida = true;
                cambiarFaseCalibracionEncoder(ENC_CAL_COMPLETA);
                Serial.println(F("[ENC][CAL] Calibracion automatica completa"));
                terminarCalibracionSolicitada();
            } else if (ahora - calibracionEncoder.inicioFase > ENC_TIMEOUT_PARO_MS) {
                fallarCalibracionEncoder("Timeout esperando que el tecnico detenga la banda");
            }
            break;

        case ENC_CAL_ERROR:
            if (eventoBotonX) {
                eventoBotonX = false;
                iniciarCalibracionEncoder();
            }
            break;

        case ENC_CAL_COMPLETA:
            break;
    }
}

bool bandaEnMovimientoAutomaticoV2() {
    return (estadoEncoderBanda &
            (ENC_FLAG_EN_MOVIMIENTO | ENC_FLAG_DIRECCION_POSITIVA)) ==
           (ENC_FLAG_EN_MOVIMIENTO | ENC_FLAG_DIRECCION_POSITIVA);
}

bool paqueteSemanticamenteValido(const PaqueteESPAPortenta &p) {
    const bool joystickValido =
        p.joystickX >= -1 && p.joystickX <= 1 &&
        p.joystickY >= -1 && p.joystickY <= 1 &&
        p.joystickZ >= -1 && p.joystickZ <= 1;
    const bool botonesValidos =
        (p.botones & static_cast<uint8_t>(~(
            BOTON_X | BOTON_TRIANGULO | BOTON_CIRCULO | BOTON_CUADRADO
        ))) == 0;
    const bool servosValidos = p.servoRotacion <= 180 && p.servoPinza <= 180;
    const bool camaraValida = p.estadoCamara <= CAMARA_ERROR;
    const bool reservadoValido = p.reservadoV2 == 0;
    return joystickValido && botonesValidos && servosValidos &&
           camaraValida && reservadoValido;
}

void registrarPaqueteValido(const PaqueteESPAPortenta &nuevo) {
    if (!sesionESPConocida) {
        sesionArranqueESP = nuevo.sesionArranque;
        sesionESPConocida = true;
        secuenciaPaqueteESPConocida = false;
        ackSecuenciaObjetivo = 0;
        codigoAckObjetivo = ACK_OBJ_NINGUNO;
        secuenciaObjetivoEnMovimiento = 0;
        Serial.print(F("[RS485] Sesion ESP32 inicial="));
        Serial.println(sesionArranqueESP);
    } else if (nuevo.sesionArranque != sesionArranqueESP) {
        Serial.print(F("[RS485] Reinicio ESP32: sesion "));
        Serial.print(sesionArranqueESP);
        Serial.print(F(" -> "));
        Serial.println(nuevo.sesionArranque);
        sesionArranqueESP = nuevo.sesionArranque;
        secuenciaPaqueteESPConocida = false;
        cambioSesionESPPendiente = true;
        ackSecuenciaObjetivo = 0;
        codigoAckObjetivo = ACK_OBJ_NINGUNO;
        secuenciaObjetivoEnMovimiento = 0;
        if (movimientoPosicionadoActivo) {
            cancelarMovimientoPosicionado("reinicio de ESP32", false);
        }
    }

    if (!secuenciaPaqueteESPConocida ||
        nuevo.secuenciaPaquete != ultimaSecuenciaPaqueteESP) {
        ultimaSecuenciaPaqueteESP = nuevo.secuenciaPaquete;
        secuenciaPaqueteESPConocida = true;
        ultimoCambioSecuenciaESPMs = millis();
    }

    paqueteESP = nuevo;
    joystickX = nuevo.joystickX;
    joystickY = nuevo.joystickY;
    joystickZ = nuevo.joystickZ;
    btConectado = (nuevo.flags & ESP_FLAG_BT_CONECTADO) != 0;
    posServoRot = nuevo.servoRotacion;
    posServoPin = nuevo.servoPinza;

    botonX = (nuevo.botones & BOTON_X) != 0;
    botonCirculo = (nuevo.botones & BOTON_CIRCULO) != 0;
    botonTriangulo = (nuevo.botones & BOTON_TRIANGULO) != 0;
    botonCuadrado = (nuevo.botones & BOTON_CUADRADO) != 0;
    if (botonX && !botonXAnterior) {
        eventoBotonX = true;
        if (estadoGeneral == EST_AUTOMATICO_V2) {
            registrarEventoPortentaV2("BUTTON_X", "flanco X recibido");
        }
    }
    if (botonCirculo && !botonCirculoAnterior) eventoBotonCirculo = true;
    if (botonTriangulo && !botonTrianguloAnterior) eventoBotonTriangulo = true;
    if ((estadoGeneral == EST_MANUAL ||
         (estadoGeneral == EST_AUTOMATICO_V2 && ajusteCatchV2Seleccionado() &&
          automaticoV2.fase == V2_EVALUANDO_CATCH) ||
         (estadoGeneral == EST_ENTRENAMIENTO_ML &&
           entrenamientoConResultadoSeleccionado() &&
          entrenamientoML.fase == ML_ESPERANDO_CONFIRMACION)) &&
        botonCuadrado &&
        !botonCuadradoAnterior) eventoBotonCuadrado = true;
    botonXAnterior = botonX;
    botonCirculoAnterior = botonCirculo;
    botonTrianguloAnterior = botonTriangulo;
    botonCuadradoAnterior = botonCuadrado;

    estadoCamara = nuevo.estadoCamara;
    errorCamara = nuevo.errorCamara;
    flagsCamara = nuevo.flags;
    desempacarMuestrasTags(nuevo.muestrasTagEmpacadas, muestrasTag);
    claseObjetivo = nuevo.claseObjetivo;
    objetivoCamaraX10 = nuevo.objetivoX10;
    objetivoCamaraY10 = nuevo.objetivoY10;
    secuenciaObjetivoRecibida = nuevo.secuenciaObjetivo;
    conteoReferenciaObjetivoRecibido = nuevo.conteoReferenciaObjetivo;

    existePaqueteValido = true;
    protocoloValido = true;
    ultimoPaqueteValidoMs = millis();
    fallosPaqueteConsecutivos = 0;
}

bool leerPaqueteESP32() {
    EnlaceRS485::Mensaje mensaje;
    bool aceptado = false;
    while (comm_protocols.rs485.available()) {
        const uint8_t dato = static_cast<uint8_t>(comm_protocols.rs485.read());
        const uint32_t recibidoMs = millis();
        clienteRS485.registrarRecepcion(recibidoMs);
        if (!receptorRS485.agregar(dato, recibidoMs, mensaje))
            continue;
        if (!clienteRS485.coincide(mensaje, millis())) { ++respuestasRS485Ajenas; continue; }
        PaqueteESPAPortenta recibido;
        memcpy(&recibido, mensaje.datos, sizeof(recibido));
        if (!validarPaquete(recibido) || !paqueteSemanticamenteValido(recibido)) {
            ++lecturasRS485Error;
            ++erroresRS485Semantica;
            if (fallosPaqueteConsecutivos < 255) ++fallosPaqueteConsecutivos;
            continue;
        }
        ultimaLatenciaRS485Ms = millis() - clienteRS485.enviadaMs;
        if (ultimaLatenciaRS485Ms > maximaLatenciaRS485Ms)
            maximaLatenciaRS485Ms = ultimaLatenciaRS485Ms;
        clienteRS485.confirmar();
        registrarPaqueteValido(recibido);
        ++lecturasRS485Ok;
        aceptado = true;
    }
    receptorRS485.vencerFragmento(millis());
    if (clienteRS485.vencer(millis())) {
        ++timeoutsRS485;
        ++lecturasRS485Error;
        if (fallosPaqueteConsecutivos < 255) ++fallosPaqueteConsecutivos;
    }
    return aceptado;
}

uint8_t construirFlagsSistema() {
    uint8_t flags = 0;
    if (calibracionXYValida) flags |= SIS_FLAG_XY_CALIBRADO;
    if (calibracionZValida) flags |= SIS_FLAG_Z_CALIBRADO;
    if (brazoEnHome()) flags |= SIS_FLAG_EN_HOME;
    if (estadoGeneral == EST_AUTOMATICO ||
        estadoGeneral == EST_AUTOMATICO_V2 ||
        estadoGeneral == EST_ENTRENAMIENTO_ML) {
        flags |= SIS_FLAG_AUTO_ACTIVO;
    }
    if (motoresEnMovimiento() || movimientoPosicionadoActivo ||
        (faseCal > CAL_ESPERA && faseCal < CAL_COMPLETA) ||
        (estadoGeneral == EST_AUTOMATICO &&
         opcionMenu == MENU_REGISTRO_ANGULO &&
         registroAngulo.fase != REG_ESPERANDO) ||
        (estadoGeneral == EST_AUTOMATICO_V2 &&
         automaticoV2.fase != V2_ESPERANDO_PIEZA &&
         automaticoV2.fase != V2_COMPLETADO)) {
        flags |= SIS_FLAG_BRAZO_OCUPADO;
    }
    if (estadoGeneral == EST_ENTRENAMIENTO_ML &&
        entrenamientoML.fase != ML_ESPERANDO_PIEZA &&
        entrenamientoML.fase != ML_LISTO) {
        flags |= SIS_FLAG_BRAZO_OCUPADO;
    }
    if (estadoGeneral == EST_SYSTEM_ERROR) flags |= SIS_FLAG_ERROR_CRITICO;
    if (calibracionEncoderValida) flags |= SIS_FLAG_ENCODER_CALIBRADO;
    if (enlaceRS485Vigente() && estadoGeneral != EST_SYSTEM_ERROR &&
        (estadoGeneral == EST_MANUAL || estadoGeneral == EST_AUTOMATICO ||
         estadoGeneral == EST_AUTOMATICO_V2 ||
         estadoGeneral == EST_ENTRENAMIENTO_ML ||
         estadoGeneral == EST_ARM_CALIBRATION ||
         estadoGeneral == EST_USER_ARM_CALIBRATION ||
         movimientoPosicionadoActivo)) {
        flags |= SIS_FLAG_MOTORES_HABILITADOS;
    }
    return flags;
}

uint8_t construirFlagsLimites() {
    uint8_t flags = 0;
    if (limiteXmas) flags |= LIM_FLAG_X_MAS;
    if (limiteXmenos) flags |= LIM_FLAG_X_MENOS;
    if (limiteYmas) flags |= LIM_FLAG_Y_MAS;
    if (limiteYmenos) flags |= LIM_FLAG_Y_MENOS;
    if (limiteZarriba) flags |= LIM_FLAG_Z_ARRIBA;
    if (limiteZabajo) flags |= LIM_FLAG_Z_ABAJO;
    if (finalesCoherentes()) flags |= LIM_FLAG_COHERENTES;
    return flags;
}

void construirPaquetePortenta(PaquetePortentaAESP &p) {
    memset(&p, 0, sizeof(p));
    p.estadoSistema = estadoGeneralWire();
    p.opcionMenu = estadoGeneral == EST_CALIBRACIONES_MENU
        ? opcionCalibracion
        : (estadoGeneral == EST_DIAGNOSTICO ? paginaDiagnostico : opcionMenu);
    p.faseCalibracionBrazo =
        (estadoGeneral == EST_AUTOMATICO &&
         opcionMenu == MENU_REGISTRO_ANGULO)
        ? static_cast<uint8_t>(registroAngulo.fase)
        : (estadoGeneral == EST_AUTOMATICO_V2
        ? static_cast<uint8_t>(automaticoV2.fase)
        : (estadoGeneral == EST_ENTRENAMIENTO_ML
            ? static_cast<uint8_t>(entrenamientoML.fase)
        : (estadoGeneral == EST_ENCODER_CALIBRATION
            ? static_cast<uint8_t>(calibracionEncoder.fase)
            : static_cast<uint8_t>(faseCal))));
    p.flagsSistema = construirFlagsSistema();
    p.flagsLimites = construirFlagsLimites();
    noInterrupts();
    p.movimientos = codificarMovimientos(movX, movY, movZ);
    interrupts();
    p.errorSistema = errorSistemaWire();
    p.comandoCamara = comandoCamaraActual;
    p.secuenciaComandoCamara = secuenciaComandoCamara;
    p.ackSecuenciaObjetivo = ackSecuenciaObjetivo;
    p.codigoAckObjetivo = codigoAckObjetivo;
    p.conteoEncoder = conteoEncoderBanda;
    p.velocidadEncoderUmS = static_cast<int32_t>(constrain(
        static_cast<double>(velocidadBandaMmS) * 1000000.0,
        static_cast<double>(INT32_MIN), static_cast<double>(INT32_MAX)
    ));
    const uint32_t nmPorCuentaEncoder = escalaEncoderMmPorCuenta > 0.0f
        ? static_cast<uint32_t>(lroundf(escalaEncoderMmPorCuenta * 1000000.0f))
        : 0U;
    const long zDesdeDin04 = calibracionZValida
        ? constrain(leerPasosZ() - limiteMinimoZPasos(),
                    0L, static_cast<long>(MAX_Z_DESDE_DIN04_PASOS))
        : 0L;
    p.nmPorCuentaEncoder = empacarEscalaEncoderYZ(
        nmPorCuentaEncoder, static_cast<uint16_t>(zDesdeDin04));
    p.secuenciaEncoder = secuenciaEncoder;
    p.estadoEncoder = estadoEncoderBanda;
    p.signoEncoder = signoEncoderAvance;
    if (estadoGeneral == EST_PRUEBA_ENCODER) {
        // En este modo, estos campos transportan solo datos de la prueba a
        // la OLED. Flags y escala nulos impiden tratarlos como velocidad real.
        p.faseCalibracionBrazo = pruebaEncoder.congelado ? 2 :
            (pruebaEncoder.iniciado ? 1 : 0);
        const uint32_t pulsos = pulsosPruebaEncoder();
        const uint32_t tiempoMs = milisegundosPruebaEncoder();
        p.conteoEncoder = pulsos > INT32_MAX ? INT32_MAX :
            static_cast<int32_t>(pulsos);
        p.velocidadEncoderUmS = tiempoMs > INT32_MAX ? INT32_MAX :
            static_cast<int32_t>(tiempoMs);
        p.nmPorCuentaEncoder = 0;
        p.estadoEncoder = 0;
    }
    if (estadoGeneral == EST_CAMBIOS_CATCH) llenarResumenCatchV2(p);
    prepararPaquete(p);
}

bool enviarPaquetePortenta() {
    if (!clienteRS485.sesion) return false;
    if (!clienteRS485.iniciar(millis())) {
        // Las ordenes inmediatas de cierre/apertura no se pierden: el siguiente
        // envio construye el estado actual al quedar libre el bus. Validar el
        // silencio tambien aqui evita saltarlo desde una orden urgente.
        envioRS485Urgente = true;
        return false;
    }
    PaquetePortentaAESP salida = {};
    construirPaquetePortenta(salida);
    EnlaceRS485::Mensaje mensaje = {};
    mensaje.sesion = clienteRS485.sesion;
    mensaje.solicitud = clienteRS485.solicitud;
    memcpy(mensaje.datos, &salida, sizeof(salida));
    uint8_t trama[EnlaceRS485::TRAMA];
    const size_t n = EnlaceRS485::codificar(mensaje, trama);
    comm_protocols.rs485.noReceive();
    comm_protocols.rs485.beginTransmission();
    const size_t escritos = comm_protocols.rs485.write(trama, n);
    comm_protocols.rs485.endTransmission();
    comm_protocols.rs485.receive();
    clienteRS485.enviadaMs = millis();
    envioRS485Urgente = false;
    if (escritos == n) {
        ++enviosRS485Ok;
        ultimoCodigoErrorEnvioRS485 = 0;
        return true; // TX local completo; la respuesta se valida por separado.
    }
    clienteRS485.confirmar();
    ++enviosRS485Error;
    ultimoCodigoErrorEnvioRS485 = 1;
    return false;
}

bool iniciarRS485Maestro() {
    if (!clienteRS485.sesion) {
        trng_t rng;
        trng_init(&rng);
        size_t cantidad = 0;
        uint32_t sesion = 0;
        const int error = trng_get_bytes(&rng, reinterpret_cast<uint8_t *>(&sesion), sizeof(sesion), &cantidad);
        trng_free(&rng);
        if (error != 0 || cantidad != sizeof(sesion) || !sesion) {
            Serial.println(F("[RS485][ERROR] No se obtuvo sesion aleatoria; enlace detenido"));
            return false;
        }
        clienteRS485.sesion = sesion;
    }
    comm_protocols.init();
    comm_protocols.rs485ModeRS232(false);
    comm_protocols.rs485FullDuplex(false);
    comm_protocols.rs485ABTerm(EnlaceRS485::PORTENTA_TERMINACION);
    comm_protocols.rs485Enable(true);
    // Sobrecarga (baud, pre, post), 8N1 implicito: igual al test a 115200.
    comm_protocols.rs485.begin(EnlaceRS485::BAUD,
                              EnlaceRS485::PORTENTA_PRE_TX_US,
                              EnlaceRS485::PORTENTA_POST_TX_US);
    comm_protocols.rs485.receive();
    clienteRS485.confirmar();
    receptorRS485.reiniciar();
    Serial.print(F("[RS485] Half duplex TX P/TX N; "));
    Serial.print(EnlaceRS485::BAUD);
    Serial.print(F(" 8N1; protocolo 17; sesion="));
    Serial.println(clienteRS485.sesion);
    return true;
}

void mantenerBusRS485MaestroRecuperable() {
    if (!comunicacionRS485Habilitada || enlaceRS485Vigente()) return;
    const bool estadoSeguro = estadoGeneral == EST_BOOT_SAFE ||
        estadoGeneral == EST_WAIT_RS485 || estadoGeneral == EST_RS485_SETTLE ||
        estadoGeneral == EST_SYSTEM_ERROR;
    if (!estadoSeguro || motoresEnMovimiento() || movimientoPosicionadoActivo) return;
    const unsigned long ahora = millis();
    if (ahora - ultimoReinicioBusRS485Ms < 1000UL || clienteRS485.pendiente) return;
    ultimoReinicioBusRS485Ms = ahora;
    comm_protocols.rs485.end();
    iniciarRS485Maestro();
    ++reiniciosBusRS485Maestro;
    Serial.print(F("[RS485][RECUPERACION] Maestro reiniciado, intento="));
    Serial.println(reiniciosBusRS485Maestro);
}

//-------------------------------------------------------------------------------------------------
// CAMBIOS DE ESTADO, COMANDOS DE CAMARA Y ERRORES
//-------------------------------------------------------------------------------------------------
void registrarEventoPortentaV2(const char *evento, const char *mensaje) {
    const bool esML = opcionMenu == MENU_ENTRENAMIENTO_ML ||
                       entrenamientoConResultadoSeleccionado();
    Serial.print(F("V2LOG|P|ms="));
    Serial.print(millis());
    if (opcionMenu == MENU_REGISTRO_ANGULO) {
        Serial.print(F("|mode=ANGLE_LABEL"));
    } else if (ajusteCatchV2Seleccionado()) {
        Serial.print(F("|mode=CATCH_CAL"));
    } else if (esML) {
        Serial.print(pruebaSeguimientoSeleccionada()
            ? F("|mode=ML_TRACK")
            : (ensenanzaMLV2Seleccionada() ? F("|mode=ML_V2") : F("|mode=ML")));
    }
    Serial.print(F("|session="));
    Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
    Serial.print(F("|event="));
    Serial.print(evento);
    Serial.print(F("|obj="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? registroAngulo.secuencia
        : (esML
            ? entrenamientoML.secuencia
        : (automaticoV2.secuencia != 0
            ? automaticoV2.secuencia : secuenciaObjetivoRecibida)));
    Serial.print(F("|encseq="));
    Serial.print(secuenciaEncoder);
    Serial.print(F("|enc="));
    Serial.print(conteoEncoderBanda);
    Serial.print(F("|ref_enc="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? 0
        : (esML
            ? entrenamientoML.conteoReferencia
        : (automaticoV2.secuencia != 0
            ? automaticoV2.conteoReferencia : conteoReferenciaObjetivoRecibido)));
    Serial.print(F("|phase="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? static_cast<uint8_t>(registroAngulo.fase)
        : (esML
            ? static_cast<uint8_t>(entrenamientoML.fase)
            : static_cast<uint8_t>(automaticoV2.fase)));
    Serial.print(F("|class="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? registroAngulo.clase
        : (esML
            ? entrenamientoML.clase
        : (automaticoV2.clase != 0
            ? automaticoV2.clase : claseObjetivo)));
    Serial.print(F("|arm_x="));
    Serial.print(posicionXmm(), 3);
    Serial.print(F("|arm_y="));
    Serial.print(posicionYmm(), 3);
    Serial.print(F("|z_steps="));
    Serial.print(leerPasosZ());
    Serial.print(F("|target_x="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? objetivoXmm : (esML
            ? entrenamientoML.xInicial : automaticoV2.objetivoBrazoX), 3);
    Serial.print(F("|target_y="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? objetivoYmm : (esML
            ? (pruebaSeguimientoSeleccionada()
                ? entrenamientoML.piezaYEstimada
                : entrenamientoML.yInicial)
            : automaticoV2.objetivoBrazoY), 3);
    Serial.print(F("|error_x="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? objetivoXmm - posicionXmm() : (esML
            ? entrenamientoML.xInicial - posicionXmm()
            : automaticoV2.ultimoErrorX), 3);
    Serial.print(F("|error_y="));
    Serial.print(opcionMenu == MENU_REGISTRO_ANGULO
        ? objetivoYmm - posicionYmm() : (esML
            ? (pruebaSeguimientoSeleccionada()
                ? entrenamientoML.piezaYEstimada
                : entrenamientoML.yInicial) - posicionYmm()
            : automaticoV2.ultimoErrorY), 3);
    Serial.print(F("|vel="));
    Serial.print(velocidadBandaMmS, 3);
    Serial.print(F("|mov_x="));
    Serial.print(movX);
    Serial.print(F("|mov_y="));
    Serial.print(movY);
    Serial.print(F("|mov_z="));
    Serial.print(movZ);
    Serial.print(F("|ack="));
    Serial.print(ackSecuenciaObjetivo);
    Serial.print(F("|ack_code="));
    Serial.print(codigoAckObjetivo);
    Serial.print(F("|error="));
    Serial.print(static_cast<uint8_t>(errorSistema));
    if (mensaje != nullptr && mensaje[0] != '\0') {
        Serial.print(F("|message="));
        Serial.print(mensaje);
    }
    Serial.println();
}

void registrarAckPortentaV2(const char *mensaje) {
    registrarEventoPortentaV2("ACK", mensaje);
}

void registrarTelemetriaPortentaV2() {
    if (estadoGeneral != EST_AUTOMATICO_V2) return;
    const unsigned long ahora = millis();
    if (ahora - ultimoLogTelemetriaV2 < V2_PERIODO_LOG_TELEMETRIA_MS) return;
    ultimoLogTelemetriaV2 = ahora;
    registrarEventoPortentaV2("TELEMETRY", "");
}

void solicitarComandoCamara(uint8_t comando) {
    comandoCamaraActual = comando;
    secuenciaComandoCamara++;
    if (secuenciaComandoCamara == 0) secuenciaComandoCamara = 1;
    inicioCalibracionCamaraObservado = false;
    Serial.print(F("[CAM] Comando="));
    Serial.print(comandoCamaraActual);
    Serial.print(F(" secuencia="));
    Serial.println(secuenciaComandoCamara);
}

void cambiarEstadoGeneral(EstadoGeneral nuevoEstado) {
    if (estadoGeneral == nuevoEstado) return;
    const EstadoGeneral estadoAnterior = estadoGeneral;
    if (estadoAnterior == EST_MANUAL && nuevoEstado != EST_MANUAL) {
        faseHomeManual = HOME_MANUAL_INACTIVO;
        eventoBotonCuadrado = false;
    }
    if (estadoAnterior == EST_PRUEBA_ENCODER) {
        registrarPruebaEncoder("ENCODER_TEST_END");
    }
    if (estadoAnterior == EST_AUTOMATICO_V2 &&
        nuevoEstado != EST_AUTOMATICO_V2) {
        registrarEventoPortentaV2("SESSION_END", "salida de Automatico V2");
    }
    if (estadoAnterior == EST_AUTOMATICO &&
        opcionMenu == MENU_REGISTRO_ANGULO &&
        nuevoEstado != EST_AUTOMATICO) {
        registrarEventoPortentaV2("SESSION_END", "salida de Registro Angulo");
    }
    if (estadoAnterior == EST_ENTRENAMIENTO_ML &&
        nuevoEstado != EST_ENTRENAMIENTO_ML) {
        registrarEventoPortentaV2("SESSION_END", "salida de Ensenanza ML");
    }
    estadoGeneral = nuevoEstado;
    inicioEstadoGeneral = millis();
    entradaMenuYAnterior = joystickY;

    Serial.print(F("[BOOT] Estado general -> "));
    Serial.println(estadoGeneralWire());

    switch (nuevoEstado) {
        case EST_CAMERA_CALIBRATION:
            detenerTodos();
            solicitarComandoCamara(CAM_CMD_CALIBRAR);
            break;
        case EST_USER_CAMERA_CALIBRATION:
            detenerTodos();
            solicitarComandoCamara(CAM_CMD_CALIBRAR);
            break;
        case EST_ARM_CALIBRATION:
        case EST_USER_ARM_CALIBRATION:
            detenerTodos();
            iniciarCalibracionBrazo();
            break;
        case EST_ENCODER_CALIBRATION:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            iniciarCalibracionEncoder();
            break;
        case EST_MAIN_MENU:
            eventoBotonX = false;
            eventoBotonCirculo = false;
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            break;
        case EST_CALIBRACIONES_MENU:
            eventoBotonX = false;
            eventoBotonCirculo = false;
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            break;
        case EST_CAMBIOS_CATCH:
        case EST_PRUEBA_SERVOS:
        case EST_DIAGNOSTICO:
        case EST_WAIT_CONTROLLER:
        case EST_FINAL_CHECKLIST:
        case EST_BOOT_SAFE:
        case EST_WAIT_RS485:
        case EST_RS485_SETTLE:
        case EST_SYSTEM_ERROR:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            break;
        case EST_MANUAL:
        case EST_AUTOMATICO:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            if (nuevoEstado == EST_MANUAL) {
                faseHomeManual = HOME_MANUAL_INACTIVO;
                eventoBotonCuadrado = false;
            }
            if (nuevoEstado == EST_AUTOMATICO &&
                opcionMenu == MENU_REGISTRO_ANGULO) {
                registroAngulo = {};
                registroAngulo.fase = REG_PREPARANDO_Z;
                registroAngulo.inicioFase = millis();
                registroAngulo.ultimoCambioRot = millis();
                registroAngulo.anguloAnterior = posServoRot;
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
                }
            }
            break;
        case EST_AUTOMATICO_V2:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            reiniciarAutomaticoV2();
            break;
        case EST_ENTRENAMIENTO_ML:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            reiniciarEntrenamientoML();
            break;
        case EST_PRUEBA_ENCODER:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            eventoBotonX = false;
            eventoBotonCirculo = false;
            eventoBotonTriangulo = false;
            ackSecuenciaObjetivo = 0;
            codigoAckObjetivo = ACK_OBJ_NINGUNO;
            pruebaEncoder = {};
            reiniciarMedicionPruebaEncoder();
            registrarPruebaEncoder("ENCODER_TEST_START");
            Serial.println(F("[PRUEBA ENCODER] X: iniciar/detener; circulo: poner a cero; triangulo: salir. No detiene la banda"));
            break;
    }
    if (estadoAnterior != EST_AUTOMATICO_V2 &&
        nuevoEstado == EST_AUTOMATICO_V2) {
        ultimoLogTelemetriaV2 = 0;
        registrarEventoPortentaV2("SESSION_START", "entrada a Automatico V2");
    }
    if (estadoAnterior != EST_AUTOMATICO &&
        nuevoEstado == EST_AUTOMATICO &&
        opcionMenu == MENU_REGISTRO_ANGULO) {
        registrarEventoPortentaV2("SESSION_START", "banda detenida; sin encoder");
    }
    if (estadoAnterior != EST_ENTRENAMIENTO_ML &&
        nuevoEstado == EST_ENTRENAMIENTO_ML) {
        registrarEventoPortentaV2("SESSION_START",
            pruebaSeguimientoSeleccionada()
                ? "Prueba seguimiento Y; X inicia catch, X/cuadrado confirma"
                : (ensenanzaMLV2Seleccionada()
                    ? "Ensenanza ML V2; catch solo X, resultado X/cuadrado"
                    : "Ensenanza ML; catch X o encoder"));
    }
}

void entrarErrorSistema(CodigoErrorLocal codigo, const char *mensaje) {
    detenerTodos();
    if ((propietarioMovimiento == MOV_AUTOMATICO ||
         propietarioMovimiento == MOV_AUTOMATICO_V2 ||
         estadoGeneral == EST_AUTOMATICO_V2) &&
        secuenciaObjetivoEnMovimiento != 0) {
        ackSecuenciaObjetivo = secuenciaObjetivoEnMovimiento;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
    }
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;

    if (estadoGeneral == EST_ARM_CALIBRATION ||
        estadoGeneral == EST_USER_ARM_CALIBRATION) {
        calibracionXYValida = false;
        calibracionZValida = false;
        if (faseCal != CAL_ERROR) {
            mensajeErrorCalibracion = mensaje;
            faseCal = CAL_ERROR;
        }
    }

    errorSistema = codigo;
    mensajeErrorSistema = mensaje;
    if (estadoGeneral == EST_AUTOMATICO_V2) {
        registrarEventoPortentaV2("ERROR", mensaje);
    }
    eventoBotonX = false;
    eventoBotonCirculo = false;
    eventoBotonTriangulo = false;
    eventoBotonCuadrado = false;
    cambiarEstadoGeneral(EST_SYSTEM_ERROR);
    Serial.print(F("[ERROR] codigo="));
    Serial.print(static_cast<uint8_t>(codigo));
    Serial.print(F(" "));
    Serial.println(mensaje);
}

//-------------------------------------------------------------------------------------------------
// CHECKLIST FINAL
//-------------------------------------------------------------------------------------------------
ResultadoChecklist evaluarChecklistFinal() {
    if (!enlaceRS485Vigente()) return CHECK_RS485;
    if (!protocoloValido || !baseESPLista()) return CHECK_PROTOCOLO;
    // En el arranque solo se comprueban los dos enlaces solicitados. Las
    // calibraciones se validan y ejecutan al entrar al modo correspondiente.
    if (!camaraConectada() || estadoCamara == CAMARA_OFFLINE ||
        estadoCamara == CAMARA_CONECTANDO || estadoCamara == CAMARA_ERROR ||
        errorCamara != CAM_ERROR_NINGUNO) return CHECK_CAMARA;
    return CHECK_OK;
}

const char *textoChecklist(ResultadoChecklist resultado) {
    switch (resultado) {
        case CHECK_OK: return "Portenta y camara conectadas";
        case CHECK_RS485: return "Comunicacion RS485 inactiva";
        case CHECK_PROTOCOLO: return "Protocolo/base ESP32 invalido";
        case CHECK_CAMARA: return "Camara no conectada o con error";
        case CHECK_HOMOGRAFIA: return "Homografia invalida";
        case CHECK_MODELO: return "Modelo de piezas no listo";
        case CHECK_XY: return "Calibracion X/Y invalida";
        case CHECK_Z: return "Calibracion Z invalida";
        case CHECK_HOME: return "Brazo fuera de HOME";
        case CHECK_MOTORES: return "Motores no detenidos";
        case CHECK_BT: return "Control Bluetooth desconectado";
        case CHECK_ENCODER: return "Calibracion del encoder invalida";
        case CHECK_CAL_ERROR: return "Error de calibracion pendiente";
        case CHECK_FINALES: return "Finales de carrera incoherentes";
        default: return "Fallo de checklist desconocido";
    }
}

//-------------------------------------------------------------------------------------------------
// MODOS DE USUARIO
//-------------------------------------------------------------------------------------------------
void avanzarEntradaModo() {
    if (modoPendiente != EST_MANUAL && modoPendiente != EST_AUTOMATICO &&
        modoPendiente != EST_AUTOMATICO_V2 &&
        modoPendiente != EST_ENTRENAMIENTO_ML) return;
    if (!btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (!calibracionXYValida || !calibracionZValida || !escalaConfigurada()) {
        Serial.println(F("[MENU] Falta calibracion del brazo"));
        cambiarEstadoGeneral(EST_USER_ARM_CALIBRATION);
        return;
    }
    if (modoPendiente != EST_MANUAL && !camaraListaCompleta()) {
        Serial.println(F("[MENU] Falta calibracion completa de camara"));
        cambiarEstadoGeneral(EST_USER_CAMERA_CALIBRATION);
        return;
    }
    if (modoPendiente == EST_AUTOMATICO_V2 ||
        modoPendiente == EST_ENTRENAMIENTO_ML) {
        if (!calibracionEncoderListaParaEntrarV2()) {
            Serial.println(F("[MENU] Falta calibracion del encoder"));
            cambiarEstadoGeneral(EST_ENCODER_CALIBRATION);
            return;
        }
        if (!posicionZSeguraV2(V2_Z_SEGURO_PASOS) ||
            !posicionZSeguraV2(posicionCapturaZV2()) ||
            !posicionZSeguraV2(posicionPrecapturaZV2())) {
            Serial.println(F("[MENU] Geometria Z de V2 fuera del rango calibrado"));
            cambiarEstadoGeneral(EST_USER_ARM_CALIBRATION);
            return;
        }
    }
    if (modoPendiente == EST_AUTOMATICO &&
        opcionMenu == MENU_REGISTRO_ANGULO &&
        (!posicionZSeguraV2(V2_Z_SEGURO_PASOS) ||
         !posicionZSeguraV2(posicionCapturaZV2()))) {
        Serial.println(F("[MENU] Geometria Z de registro fuera de rango"));
        cambiarEstadoGeneral(EST_USER_ARM_CALIBRATION);
        return;
    }
    if (modoPendiente == EST_ENTRENAMIENTO_ML &&
        (!posicionZSeguraV2(V2_Z_SEGURO_PASOS) ||
         !posicionZSeguraV2(posicionCapturaZV2()))) {
        Serial.println(F("[MENU] Geometria Z de entrenamiento fuera de rango"));
        cambiarEstadoGeneral(EST_USER_ARM_CALIBRATION);
        return;
    }
    const EstadoGeneral destino = modoPendiente;
    modoPendiente = EST_MAIN_MENU;
    cambiarEstadoGeneral(destino);
}

void terminarCalibracionSolicitada() {
    if (modoPendiente != EST_MAIN_MENU) {
        avanzarEntradaModo();
        return;
    }
    cambiarEstadoGeneral(btConectado ? retornoCalibracion : EST_WAIT_CONTROLLER);
}

void procesarMenuPrincipal() {
    if (!btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (movimientoPosicionadoActivo || motoresEnMovimiento()) return;

    const uint8_t opciones[] = {
        MENU_MODO_MANUAL, MENU_MODO_AUTOMATICO,
        MENU_MODO_AUTOMATICO_V2, MENU_AJUSTE_CATCH_V2, MENU_CAMBIOS_CATCH, MENU_REGISTRO_ANGULO,
        MENU_CALIBRACIONES,
        MENU_PRUEBA_SERVOS, MENU_ENTRENAMIENTO_ML,
        MENU_ENTRENAMIENTO_ML_V2, MENU_PRUEBA_SEGUIMIENTO,
        MENU_PRUEBA_ENCODER,
        MENU_DIAGNOSTICO
    };
    if (joystickY != 0 && entradaMenuYAnterior == 0) {
        int nueva = static_cast<int>(indiceMenu) - joystickY;
        const int ultima = sizeof(opciones) / sizeof(opciones[0]) - 1;
        if (nueva < 0) nueva = ultima;
        if (nueva > ultima) nueva = 0;
        indiceMenu = static_cast<uint8_t>(nueva);
        opcionMenu = opciones[indiceMenu];
        Serial.print(F("[MENU] Opcion="));
        Serial.println(opcionMenu);
    }
    entradaMenuYAnterior = joystickY;

    if (!eventoBotonX) return;
    eventoBotonX = false;

    switch (opcionMenu) {
        case MENU_MODO_MANUAL:
            modoPendiente = EST_MANUAL;
            avanzarEntradaModo();
            break;
        case MENU_MODO_AUTOMATICO:
            modoPendiente = EST_AUTOMATICO;
            avanzarEntradaModo();
            break;
        case MENU_MODO_AUTOMATICO_V2:
        case MENU_AJUSTE_CATCH_V2:
            modoPendiente = EST_AUTOMATICO_V2;
            avanzarEntradaModo();
            break;
        case MENU_CAMBIOS_CATCH:
            paginaCambiosCatch = 0;
            cambiarEstadoGeneral(EST_CAMBIOS_CATCH);
            imprimirCambiosCatchV2();
            break;
        case MENU_REGISTRO_ANGULO:
            modoPendiente = EST_AUTOMATICO;
            avanzarEntradaModo();
            break;
        case MENU_CALIBRACIONES:
            cambiarEstadoGeneral(EST_CALIBRACIONES_MENU);
            break;
        case MENU_PRUEBA_SERVOS:
            cambiarEstadoGeneral(EST_PRUEBA_SERVOS);
            break;
        case MENU_ENTRENAMIENTO_ML:
        case MENU_ENTRENAMIENTO_ML_V2:
        case MENU_PRUEBA_SEGUIMIENTO:
            modoPendiente = EST_ENTRENAMIENTO_ML;
            avanzarEntradaModo();
            break;
        case MENU_DIAGNOSTICO:
            paginaDiagnostico = 0;
            cambiarEstadoGeneral(EST_DIAGNOSTICO);
            break;
        case MENU_PRUEBA_ENCODER:
            cambiarEstadoGeneral(EST_PRUEBA_ENCODER);
            break;
        default:
            opcionMenu = MENU_MODO_MANUAL;
            break;
    }
}

void procesarMenuCalibraciones() {
    if (!btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        cambiarEstadoGeneral(EST_MAIN_MENU);
        return;
    }
    if (joystickY != 0 && entradaMenuYAnterior == 0) {
        int nueva = static_cast<int>(opcionCalibracion) - joystickY;
        if (nueva < 0) nueva = 2;
        if (nueva > 2) nueva = 0;
        opcionCalibracion = static_cast<uint8_t>(nueva);
    }
    entradaMenuYAnterior = joystickY;
    if (!eventoBotonX) return;
    eventoBotonX = false;
    modoPendiente = EST_MAIN_MENU;
    retornoCalibracion = EST_CALIBRACIONES_MENU;
    if (opcionCalibracion == 0) {
        cambiarEstadoGeneral(EST_USER_ARM_CALIBRATION);
    } else if (opcionCalibracion == 1) {
        cambiarEstadoGeneral(EST_USER_CAMERA_CALIBRATION);
    } else {
        cambiarEstadoGeneral(EST_ENCODER_CALIBRATION);
    }
}

void procesarPantallaSinMotores() {
    detenerTodos();
    if (!btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
    } else if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        cambiarEstadoGeneral(EST_MAIN_MENU);
    } else if (estadoGeneral == EST_CAMBIOS_CATCH && eventoBotonX) {
        eventoBotonX = false;
        paginaCambiosCatch = 1U - paginaCambiosCatch;
        imprimirCambiosCatchV2();
    } else if (estadoGeneral == EST_DIAGNOSTICO && eventoBotonX) {
        eventoBotonX = false;
        paginaDiagnostico = 1U - paginaDiagnostico;
    }
}

void reiniciarMedicionPruebaEncoder() {
    pruebaEncoder.marcas = 0;
    pruebaEncoder.inicioMs = 0;
    pruebaEncoder.finMs = 0;
    pruebaEncoder.ultimoLogMs = 0;
    pruebaEncoder.conteoInicio = static_cast<int32_t>(encoders[0].getPulses());
    pruebaEncoder.iniciado = false;
    pruebaEncoder.congelado = false;
    pruebaEncoder.conteoFinal = pruebaEncoder.conteoInicio;
}

uint32_t pulsosPruebaEncoder() {
    if (!pruebaEncoder.iniciado) return 0;
    const int32_t conteo = pruebaEncoder.congelado ? pruebaEncoder.conteoFinal
        : static_cast<int32_t>(encoders[0].getPulses());
    const int32_t delta = diferenciaConteosConWrap(conteo, pruebaEncoder.conteoInicio);
    return delta < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(delta))
        : static_cast<uint32_t>(delta);
}

uint32_t milisegundosPruebaEncoder() {
    if (!pruebaEncoder.iniciado) return 0;
    return (pruebaEncoder.congelado ? pruebaEncoder.finMs : millis()) -
        pruebaEncoder.inicioMs;
}

void registrarPruebaEncoder(const char *evento) {
    const int32_t conteo = pruebaEncoder.congelado ? pruebaEncoder.conteoFinal
        : static_cast<int32_t>(encoders[0].getPulses());
    // Magnitud del desplazamiento neto en cuentas; no convertir a mm.
    const uint32_t pulsos = pulsosPruebaEncoder();
    Serial.print(F("V2LOG|P|mode=ENCODER_TEST|event=")); Serial.print(evento);
    Serial.print(F("|ms=")); Serial.print(millis());
    Serial.print(F("|trial=")); Serial.print(pruebaEncoder.ensayo);
    Serial.print(F("|mark=")); Serial.print(pruebaEncoder.marcas);
    Serial.print(F("|enc=")); Serial.print(conteo);
    Serial.print(F("|ref_enc=")); Serial.print(pruebaEncoder.conteoInicio);
    Serial.print(F("|pulses=")); Serial.print(pulsos);
    Serial.print(F("|elapsed_ms=")); Serial.print(milisegundosPruebaEncoder());
    Serial.print(F("|running=")); Serial.print(pruebaEncoder.iniciado && !pruebaEncoder.congelado ? 1 : 0);
    Serial.print(F("|frozen=")); Serial.print(pruebaEncoder.congelado ? 1 : 0);
    Serial.println(F("|message=SOLO_PULSOS_SIN_CATCH"));
}

void procesarPruebaEncoder() {
    detenerTodos();
    if (!btConectado || eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        cambiarEstadoGeneral(btConectado ? EST_MAIN_MENU : EST_WAIT_CONTROLLER);
        return;
    }
    if (eventoBotonCirculo) {
        eventoBotonCirculo = false;
        eventoBotonX = false;
        reiniciarMedicionPruebaEncoder();
        registrarPruebaEncoder("ENCODER_TEST_RESET");
    }
    if (eventoBotonX) {
        eventoBotonX = false;
        if (!pruebaEncoder.iniciado) {
            ++pruebaEncoder.ensayo;
            pruebaEncoder.conteoInicio = static_cast<int32_t>(encoders[0].getPulses());
            pruebaEncoder.conteoFinal = pruebaEncoder.conteoInicio;
            pruebaEncoder.inicioMs = millis();
            pruebaEncoder.ultimoLogMs = pruebaEncoder.inicioMs;
            pruebaEncoder.iniciado = true;
            registrarPruebaEncoder("ENCODER_TEST_RUN_START");
        } else if (!pruebaEncoder.congelado) {
            pruebaEncoder.conteoFinal = static_cast<int32_t>(encoders[0].getPulses());
            pruebaEncoder.finMs = millis();
            pruebaEncoder.congelado = true;
            ++pruebaEncoder.marcas;
            registrarPruebaEncoder("ENCODER_TEST_MARK");
        }
    }
    if (!pruebaEncoder.iniciado || pruebaEncoder.congelado) return;
    const uint32_t ahora = millis();
    if (ahora - pruebaEncoder.ultimoLogMs >= 100UL) {
        pruebaEncoder.ultimoLogMs = ahora;
        registrarPruebaEncoder("ENCODER_TEST_SAMPLE");
    }
}

void cancelarHomeManual(const char *motivo) {
    if (movimientoPosicionadoActivo) {
        cancelarMovimientoPosicionado(motivo, false);
    } else {
        detenerTodos();
    }
    faseHomeManual = HOME_MANUAL_INACTIVO;
    Serial.print(F("[MANUAL][HOME] Cancelado: "));
    Serial.println(motivo);
}

void procesarModoManual() {
    if (!btConectado) {
        if (faseHomeManual != HOME_MANUAL_INACTIVO) {
            cancelarHomeManual("control desconectado");
        } else detenerTodos();
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        if (faseHomeManual != HOME_MANUAL_INACTIVO) {
            cancelarHomeManual("triangulo");
        } else detenerTodos();
        cambiarEstadoGeneral(EST_MAIN_MENU);
        return;
    }
    // La ESP procesa el flanco de circulo y conmuta la pinza. Se consume aqui
    // el evento duplicado del paquete para que no quede pendiente al salir.
    eventoBotonCirculo = false;

    if (eventoBotonCuadrado) {
        eventoBotonCuadrado = false;
        if (faseHomeManual == HOME_MANUAL_INACTIVO) {
            if (!calibracionXYValida || !calibracionZValida ||
                !escalaConfigurada()) {
                Serial.println(F("[MANUAL][HOME] Rechazado: calibre X/Y/Z primero"));
            } else if (brazoEnHome()) {
                Serial.println(F("[MANUAL][HOME] Ya esta en X=0 Y=0 Z=0"));
                faseHomeManual = HOME_MANUAL_ESPERANDO_NEUTRO;
            } else {
                detenerTodos();
                movimientoPosicionadoActivo = false;
                propietarioMovimiento = MOV_SIN_PROPIETARIO;
                faseHomeManual = HOME_MANUAL_Z;
                inicioHomeManual = millis();
                moverZHasta(0, DIV_HOME);
                Serial.println(F("[MANUAL][HOME] Cuadrado: deteniendo mando; Z -> 0, luego X/Y -> 0"));
            }
        }
    }

    if (faseHomeManual == HOME_MANUAL_Z) {
        if (millis() - inicioHomeManual > TIMEOUT_MOVIMIENTO_MS) {
            calibracionZValida = false;
            entrarErrorSistema(ERROR_TIMEOUT_MOVIMIENTO, "Timeout Z volviendo a HOME manual");
            return;
        }
        if (objetivoZEnCurso() || movZ != 0) return;
        if (leerPasosZ() != 0) {
            calibracionZValida = false;
            entrarErrorSistema(ERROR_FINAL_INESPERADO, "Z se detuvo antes de HOME manual");
            return;
        }
        if (!iniciarMovimientoXY(0.0f, 0.0f, 0.0f, MOV_TERMINAL)) {
            entrarErrorSistema(ERROR_OBJETIVO_INVALIDO, "No se pudo iniciar XY hacia HOME manual");
            return;
        }
        faseHomeManual = HOME_MANUAL_XY;
        Serial.println(F("[MANUAL][HOME] Z listo; moviendo X/Y a 0"));
        return;
    }
    if (faseHomeManual == HOME_MANUAL_XY) {
        if (movimientoPosicionadoActivo) return;
        if (!brazoEnHome()) {
            entrarErrorSistema(ERROR_FINAL_INESPERADO, "X/Y no llegaron a HOME manual");
            return;
        }
        faseHomeManual = HOME_MANUAL_ESPERANDO_NEUTRO;
        Serial.println(F("[MANUAL][HOME] Completo X=0 Y=0 Z=0; suelte joysticks"));
        return;
    }
    if (faseHomeManual == HOME_MANUAL_ESPERANDO_NEUTRO) {
        if (joystickX != 0 || joystickY != 0 || joystickZ != 0) return;
        faseHomeManual = HOME_MANUAL_INACTIVO;
        return;
    }

    int8_t x = joystickX;
    int8_t y = joystickY;
    int8_t z = joystickZ;
    if ((x > 0 && limiteXmas) || (x < 0 && limiteXmenos)) x = 0;
    if ((y > 0 && limiteYmas) || (y < 0 && limiteYmenos)) y = 0;
    if ((z > 0 && limiteZarriba) || (z < 0 && limiteZabajo)) z = 0;
    moverXContinuo(x, DIV_MANUAL);
    moverYContinuo(y, DIV_MANUAL);
    moverZContinuo(z, DIV_MANUAL);
}

void rechazarObjetivoFueraDeRango(
    uint16_t secuencia,
    const char *motivo = "objetivo fuera de rango"
) {
    ackSecuenciaObjetivo = secuencia;
    codigoAckObjetivo = ACK_OBJ_RECHAZADO_RANGO;
    secuenciaObjetivoEnMovimiento = 0;
    Serial.print(F("[AUTO][WARN] Objetivo descartado sin movimiento; secuencia="));
    Serial.print(secuencia);
    Serial.print(F(" motivo="));
    Serial.println(motivo);
    if (estadoGeneral == EST_AUTOMATICO_V2) {
        registrarEventoPortentaV2("REJECT", motivo);
        registrarAckPortentaV2("objetivo rechazado");
    }
}

void cancelarRegistroAngulo(const char *motivo, bool desconexion) {
    Serial.print(F("[ANGULO] Cancelando: "));
    Serial.println(motivo);
    registrarEventoPortentaV2("CANCEL", motivo);
    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    if (registroAngulo.secuencia != 0) {
        ackSecuenciaObjetivo = registroAngulo.secuencia;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
    }
    secuenciaObjetivoEnMovimiento = 0;
    registroAngulo.salirPorDesconexion = desconexion;
    registroAngulo.fase = REG_CANCELANDO;
    registroAngulo.inicioFase = millis();
    if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
        moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
    }
}

void registrarMuestraAnguloEstatico() {
    Serial.print(F("V2LOG|P|ms=")); Serial.print(millis());
    Serial.print(F("|mode=ANGLE_LABEL|event=ANGLE_SAMPLE|session="));
    Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
    Serial.print(F("|obj=")); Serial.print(registroAngulo.secuencia);
    Serial.print(F("|class=")); Serial.print(registroAngulo.clase);
    Serial.print(F("|cam_x=")); Serial.print(registroAngulo.camX, 3);
    Serial.print(F("|cam_y=")); Serial.print(registroAngulo.camY, 3);
    Serial.print(F("|servo_rot_deg=")); Serial.print(registroAngulo.anguloCatch);
    Serial.print(F("|label_rot=")); Serial.print(registroAngulo.anguloCatch);
    Serial.print(F("|arm_x=")); Serial.print(posicionXmm(), 3);
    Serial.print(F("|arm_y=")); Serial.print(posicionYmm(), 3);
    Serial.println(F("|catch_type=STATIC|trigger=ANGLE_SETTLED|physical_result=NO_VERIFICADO|message=CIERRE_CON_BANDA_DETENIDA"));
}

void procesarRegistroAngulo() {
    if (registroAngulo.fase == REG_CANCELANDO) {
        if (movZ > 0 && limiteZarriba) {
            detenerTodos();
            entrarErrorSistema(ERROR_FINAL_INESPERADO,
                               "Final de Z durante retirada de registro");
            return;
        }
        if (!objetivoZEnCurso() && movZ == 0) {
            cambiarEstadoGeneral(registroAngulo.salirPorDesconexion
                ? EST_WAIT_CONTROLLER : EST_MAIN_MENU);
        } else if (millis() - registroAngulo.inicioFase > V2_TIMEOUT_FASE_MS) {
            detenerTodos();
            entrarErrorSistema(ERROR_TIMEOUT_MOVIMIENTO,
                               "Timeout retirando Z en registro");
        }
        return;
    }
    if (!btConectado || eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        cancelarRegistroAngulo("salida del modo", !btConectado);
        return;
    }
    if (!camaraListaCompleta()) {
        cancelarRegistroAngulo("camara no disponible", false);
        return;
    }
    if (registroAngulo.fase != REG_ESPERANDO &&
        registroAngulo.fase != REG_AJUSTANDO &&
        millis() - registroAngulo.inicioFase > V2_TIMEOUT_FASE_MS) {
        cancelarRegistroAngulo("timeout de fase", false);
        return;
    }

    switch (registroAngulo.fase) {
        case REG_PREPARANDO_Z:
            if (!objetivoZEnCurso() && movZ == 0) {
                registroAngulo.fase = REG_ESPERANDO;
                registroAngulo.inicioFase = millis();
                Serial.println(F("[ANGULO] Esperando pieza con banda detenida"));
            }
            break;

        case REG_ESPERANDO: {
            eventoBotonX = false;
            if (!objetivoCamaraValido() || secuenciaObjetivoRecibida == 0 ||
                secuenciaObjetivoRecibida == ackSecuenciaObjetivo) break;
            const float camX = objetivoCamaraX10 / 10.0f;
            const float camY = objetivoCamaraY10 / 10.0f;
            float brazoX = 0.0f;
            float brazoY = 0.0f;
            long pruebaX = 0;
            long pruebaY = 0;
            if (!transformarCamaraABrazo(camX, camY, brazoX, brazoY) ||
                !cinematicaInversaCartesiana(brazoX, brazoY, 0.0f,
                                              pruebaX, pruebaY) ||
                !iniciarMovimientoXY(brazoX, brazoY, 0.0f,
                                     MOV_AUTOMATICO)) {
                rechazarObjetivoFueraDeRango(secuenciaObjetivoRecibida);
                break;
            }
            registroAngulo.secuencia = secuenciaObjetivoRecibida;
            registroAngulo.clase = claseObjetivo;
            registroAngulo.camX = camX;
            registroAngulo.camY = camY;
            secuenciaObjetivoEnMovimiento = registroAngulo.secuencia;
            ackSecuenciaObjetivo = registroAngulo.secuencia;
            codigoAckObjetivo = ACK_OBJ_ACEPTADO;
            registroAngulo.fase = REG_MOVIENDO_XY;
            registroAngulo.inicioFase = millis();
            registrarEventoPortentaV2("ACCEPT", "objetivo estatico aceptado");
            break;
        }

        case REG_MOVIENDO_XY:
            eventoBotonX = false;
            if (!movimientoPosicionadoActivo && !motoresEnMovimiento()) {
                registroAngulo.fase = REG_AJUSTANDO;
                registroAngulo.inicioFase = millis();
                registroAngulo.ultimoCambioRot = millis();
                registroAngulo.anguloAnterior = posServoRot;
                Serial.println(F("[ANGULO] Ajustar ROT con joystick derecho; catch automatico tras estabilizar"));
                registrarEventoPortentaV2("READY_ANGLE", "ajustar servo de giro");
            }
            break;

        case REG_AJUSTANDO: {
            eventoBotonX = false;
            if (posServoRot != registroAngulo.anguloAnterior) {
                registroAngulo.anguloAnterior = posServoRot;
                registroAngulo.ultimoCambioRot = millis();
            }
            const unsigned long ahora = millis();
            if (ahora - registroAngulo.inicioFase < 2500UL ||
                ahora - registroAngulo.ultimoCambioRot < 1000UL) break;
            registroAngulo.busquedaFinalZActiva = false;
            moverZHasta(posicionCapturaZV2(), DIV_POSICION);
            registroAngulo.fase = REG_BAJANDO_Z;
            registroAngulo.inicioFase = ahora;
            registrarEventoPortentaV2("TRIGGER", "descenso estatico sin encoder");
            break;
        }

        case REG_BAJANDO_Z:
            eventoBotonX = false;
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                registroAngulo.busquedaFinalZActiva = false;
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!registroAngulo.busquedaFinalZActiva) {
                    registroAngulo.busquedaFinalZActiva = true;
                    moverZHasta(limiteMinimoZPasos() -
                        V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                        V2_DIV_BUSQUEDA_FINAL_Z);
                } else {
                    cancelarRegistroAngulo("DIN04 no aparecio durante descenso", false);
                }
                break;
            }
            registroAngulo.anguloCatch = posServoRot;
            ackSecuenciaObjetivo = registroAngulo.secuencia;
            registroAngulo.fase = REG_CERRANDO;
            registroAngulo.inicioFase = millis();
            enviarOrdenCierreCatchAhora();
            registrarEventoPortentaV2("GRIP_COMMAND", "cierre estatico sin encoder");
            break;

        case REG_CERRANDO:
            eventoBotonX = false;
            detenerTodos();
            if (!limiteZabajo) {
                cancelarRegistroAngulo("DIN04 se perdio durante cierre", false);
                break;
            }
            if (millis() - registroAngulo.inicioFase <
                V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) break;
            registrarMuestraAnguloEstatico();
            moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
            registroAngulo.fase = REG_SUBIENDO;
            registroAngulo.inicioFase = millis();
            break;

        case REG_SUBIENDO:
            eventoBotonX = false;
            if (objetivoZEnCurso() || movZ != 0) break;
            ackSecuenciaObjetivo = registroAngulo.secuencia;
            codigoAckObjetivo = ACK_OBJ_COMPLETADO;
            secuenciaObjetivoEnMovimiento = 0;
            registrarEventoPortentaV2("RESULT", "catch estatico terminado");
            registroAngulo.secuencia = 0;
            registroAngulo.fase = REG_ESPERANDO;
            registroAngulo.inicioFase = millis();
            break;

        case REG_CANCELANDO:
            break;
    }
}

void procesarModoAutomatico() {
    if (opcionMenu == MENU_REGISTRO_ANGULO) {
        procesarRegistroAngulo();
        return;
    }
    // Z nunca se mueve en automatico.
    moverZContinuo(0, DIV_MANUAL);

    if (!btConectado) {
        if (movimientoPosicionadoActivo)
            cancelarMovimientoPosicionado("control Bluetooth desconectado", false);
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }

    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        if (movimientoPosicionadoActivo)
            cancelarMovimientoPosicionado("cancelado por usuario", false);
        cambiarEstadoGeneral(EST_MAIN_MENU);
        return;
    }

    if (!camaraListaCompleta()) {
        if (movimientoPosicionadoActivo)
            cancelarMovimientoPosicionado("camara dejo de estar lista", false);
        entrarErrorSistema(ERROR_CAMARA, "Camara no disponible en automatico");
        return;
    }

    if (movimientoPosicionadoActivo || motoresEnMovimiento()) return;
    if (!objetivoCamaraValido() || secuenciaObjetivoRecibida == 0) return;
    if (secuenciaObjetivoRecibida == ackSecuenciaObjetivo) return;

    const float camX = static_cast<float>(objetivoCamaraX10) / 10.0f;
    const float camY = static_cast<float>(objetivoCamaraY10) / 10.0f;
    float brazoX = 0.0f;
    float brazoY = 0.0f;
    if (!transformarCamaraABrazo(camX, camY, brazoX, brazoY)) {
        rechazarObjetivoFueraDeRango(secuenciaObjetivoRecibida);
        return;
    }

    long pruebaX = 0;
    long pruebaY = 0;
    if (!cinematicaInversaCartesiana(brazoX, brazoY, 0.0f, pruebaX, pruebaY)) {
        rechazarObjetivoFueraDeRango(secuenciaObjetivoRecibida);
        return;
    }

    secuenciaObjetivoEnMovimiento = secuenciaObjetivoRecibida;
    ackSecuenciaObjetivo = secuenciaObjetivoRecibida;
    codigoAckObjetivo = ACK_OBJ_ACEPTADO;

    Serial.print(F("[AUTO] Pieza clase="));
    Serial.print(claseObjetivo);
    Serial.print(F(" camara=("));
    Serial.print(camX, 1);
    Serial.print(F(","));
    Serial.print(camY, 1);
    Serial.print(F(") brazo=("));
    Serial.print(brazoX, 1);
    Serial.print(F(","));
    Serial.print(brazoY, 1);
    Serial.println(F(")"));

    if (!iniciarMovimientoXY(brazoX, brazoY, 0.0f, MOV_AUTOMATICO)) {
        rechazarObjetivoFueraDeRango(secuenciaObjetivoRecibida);
    }
}

const char *nombreFaseAutomaticoV2(FaseAutomaticoV2 fase) {
    switch (fase) {
        case V2_ESPERANDO_PIEZA: return "ESPERANDO PIEZA";
        case V2_PREPOSICIONANDO: return "PREPOSICIONANDO";
        case V2_ESPERANDO_LLEGADA: return "PREPARANDO DESCENSO";
        case V2_DISPARANDO_CATCH: return "DISPARANDO CATCH";
        case V2_BAJANDO_Z: return "BAJANDO PRECAPTURA";
        case V2_BAJANDO_CATCH: return "BAJANDO CATCH";
        case V2_SUBIENDO_Z: return "SUBIENDO Z";
        case V2_ESPERANDO_CATCH_AUTOMATICO: return "ESPERANDO CATCH AUTO";
        case V2_COMPLETADO: return "COMPLETADO";
        case V2_CANCELANDO: return "CANCELANDO";
        case V2_PREPARANDO_ESPERA: return "PREPARANDO ESPERA";
        case V2_CERRANDO_PINZA: return "CERRANDO PINZA";
        case V2_MOVIENDO_ENTREGA: return "YENDO A DERECHA";
        case V2_BAJANDO_ENTREGA: return "BAJANDO ENTREGA";
        case V2_ABRIENDO_PINZA: return "SOLTANDO PIEZA";
        case V2_SUBIENDO_FINAL: return "SUBIENDO FINAL";
        case V2_SIGUIENDO_PIEZA: return "SIGUIENDO PIEZA Y";
        case V2_EVALUANDO_CATCH: return "EVALUAR CATCH";
        default: return "DESCONOCIDA";
    }
}

void cambiarFaseAutomaticoV2(FaseAutomaticoV2 nueva) {
    automaticoV2.fase = nueva;
    automaticoV2.inicioFase = millis();
    automaticoV2.inicioEstable = 0;
    Serial.print(F("[AUTO V2] Fase -> "));
    Serial.println(nombreFaseAutomaticoV2(nueva));
    registrarEventoPortentaV2("PHASE", nombreFaseAutomaticoV2(nueva));
}

void reiniciarAutomaticoV2() {
    automaticoV2 = {};
    automaticoV2.fase = leerPasosZ() == V2_Z_SEGURO_PASOS
        ? V2_ESPERANDO_PIEZA : V2_PREPARANDO_ESPERA;
    automaticoV2.inicioFase = millis();
    automaticoV2.objetivoBrazoYAnterior = posicionYmm();
    automaticoV2.ultimoConteoProcesado = conteoEncoderBanda;
    automaticoV2.instanteObjetivoAnterior = millis();
    if (automaticoV2.fase == V2_PREPARANDO_ESPERA) {
        Serial.println(F(
            "[AUTO V2] Moviendo Z automaticamente a posicion segura de espera"
        ));
        moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
    }
}

void finalizarCancelacionAutomaticoV2() {
    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    secuenciaObjetivoEnMovimiento = 0;
    const bool esperarControl = automaticoV2.salidaAEsperaControl;
    const bool salirAlMenu = automaticoV2.salidaAlMenu;
    reiniciarAutomaticoV2();

    if (esperarControl || !btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
    } else if (salirAlMenu) {
        cambiarEstadoGeneral(EST_MAIN_MENU);
    } else {
        // Una condicion transitoria cancela solamente la pieza actual. El modo
        // V2 sigue activo y espera el rearme del detector para la siguiente.
        Serial.println(F("[AUTO V2] Intento cancelado; modo V2 sigue activo"));
        registrarEventoPortentaV2("REARM", "V2 permanece activo");
    }
}

void iniciarCancelacionAutomaticoV2(
    const char *motivo,
    bool esperarControl,
    bool salirDelModo
) {
    Serial.print(F("[AUTO V2] Cancelando: "));
    Serial.println(motivo);
    if (automaticoV2.secuencia != 0) {
        ackSecuenciaObjetivo = automaticoV2.secuencia;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
        registrarAckPortentaV2("objetivo cancelado");
    }
    registrarEventoPortentaV2("CANCEL", motivo);
    detenerX();
    detenerY();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    automaticoV2.salidaAEsperaControl = esperarControl;
    automaticoV2.salidaAlMenu = salirDelModo;
    cambiarFaseAutomaticoV2(V2_CANCELANDO);
    if (leerPasosZ() == V2_Z_SEGURO_PASOS) {
        finalizarCancelacionAutomaticoV2();
    } else {
        moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
    }
}

float posicionCatchYV2() {
    return V2_POSICION_CATCH_Y_MM;
}

long posicionCapturaZV2() {
    // La calibracion fija Z abajo como limiteMinimoZPasos(). El catch debe
    // recorrer todo el eje y confirmar fisicamente DIN04 antes de retirarse.
    return limiteMinimoZPasos();
}

long posicionPrecapturaZV2() {
    return posicionCapturaZV2() + Z_MARGEN_PRECAPTURA_PASOS;
}

bool actualizarObjetivoMovilV2() {
    // La posicion usa el contador vivo; la velocidad conserva su ventana de 10 ms.
    conteoEncoderBanda = static_cast<int32_t>(encoders[0].getPulses());
    const int32_t delta = diferenciaConteosConWrap(
        conteoEncoderBanda,
        automaticoV2.conteoReferencia
    );
    const float baseX = CAMERA_SWAP_XY
        ? automaticoV2.camYReferencia : automaticoV2.camXReferencia;
    const float baseY = CAMERA_SWAP_XY
        ? automaticoV2.camXReferencia : automaticoV2.camYReferencia;
    const float brazoX = static_cast<float>(CAMERA_SIGN_X) * baseX +
                         CAMERA_OFFSET_X_MM;
    const float yLocalCamara = static_cast<float>(CAMERA_SIGN_Y) * baseY +
                               CAMERA_OFFSET_Y_MM;
    const float brazoY = -CAMARA_A_HOME_Y_MM + yLocalCamara +
        static_cast<float>(signoEncoderAvance) *
        static_cast<float>(delta) * escalaEncoderMmPorCuenta;
    if (!isfinite(brazoX) || !isfinite(brazoY)) {
        return false;
    }

    const unsigned long ahora = millis();
    if (conteoEncoderBanda != automaticoV2.ultimoConteoProcesado &&
        automaticoV2.instanteObjetivoAnterior != 0 &&
        ahora > automaticoV2.instanteObjetivoAnterior) {
        const float dt = static_cast<float>(
            ahora - automaticoV2.instanteObjetivoAnterior
        ) / 1000.0f;
        const float instantanea =
            (brazoY - automaticoV2.objetivoBrazoYAnterior) / dt;
        automaticoV2.velocidadObjetivoY =
            0.8f * automaticoV2.velocidadObjetivoY + 0.2f * instantanea;
        automaticoV2.objetivoBrazoYAnterior = brazoY;
        automaticoV2.ultimoConteoProcesado = conteoEncoderBanda;
        automaticoV2.instanteObjetivoAnterior = ahora;
    }
    automaticoV2.objetivoBrazoX = brazoX;
    automaticoV2.objetivoBrazoY = brazoY;
    automaticoV2.velocidadObjetivoY = velocidadBandaMmS;
    return true;
}

float tiempoDescensoZV2Segundos() {
    const float pasosZPorSegundo = 1.0f / (2.0f * velocidadMotores);
    if (pasosZPorSegundo <= 0.0f) return 0.0f;
    return fabsf(static_cast<float>(
        posicionCapturaZV2() - V2_Z_SEGURO_PASOS
    )) / pasosZPorSegundo;
}

float tiempoDescensoFinalZV2Segundos() {
    const float pasosZPorSegundo = 1.0f / (2.0f * velocidadMotores);
    if (pasosZPorSegundo <= 0.0f) return 0.0f;
    return static_cast<float>(Z_MARGEN_PRECAPTURA_PASOS) /
        pasosZPorSegundo;
}

float calcularUmbralDisparoYV2() {
    // El brazo permanece quieto en Y. La velocidad solo convierte el tiempo
    // mecanico de descenso y la reserva manual en distancia; no gobierna Y.
    const float anticipacion = fmaxf(0.0f, velocidadBandaMmS) *
        (tiempoDescensoZV2Segundos() + V2_RESERVA_MANUAL_CATCH_S) +
        V2_AJUSTE_ANTICIPACION_Z_MM;
    return posicionCatchYV2() - anticipacion;
}

float anticipacionCierreCatchSegundos() {
    return fmaxf(0.0f, static_cast<float>(
        V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS
    ) + CATCH_ADELANTO_EXTRA_MS) / 1000.0f;
}

float calcularUmbralCierrePinzaYV2() {
    return posicionCatchYV2() - fmaxf(0.0f, velocidadBandaMmS) *
        (tiempoDescensoFinalZV2Segundos() +
         anticipacionCierreCatchSegundos());
}

void enviarOrdenCierreCatchAhora() {
    // No esperar al siguiente periodo de telemetria ni a imprimir por Serial.
    codigoAckObjetivo = ACK_OBJ_CERRAR_PINZA;
    if (comunicacionRS485Habilitada && enviarPaquetePortenta()) {
        tAnteriorEstadoESP = millis();
    }
    // Si falla, el envio periodico vuelve a intentar la misma orden/secuencia.
}

void aceptarObjetivoAutomaticoV2() {
    const float camX = static_cast<float>(objetivoCamaraX10) / 10.0f;
    const float camY = static_cast<float>(objetivoCamaraY10) / 10.0f;
    const float baseX = CAMERA_SWAP_XY ? camY : camX;
    const float brazoX = static_cast<float>(CAMERA_SIGN_X) * baseX +
                         CAMERA_OFFSET_X_MM;
    const float yCatch = posicionCatchYV2();

    long pruebaX = 0;
    long pruebaY = 0;
    if (!cinematicaInversaCartesiana(
            brazoX, yCatch, 0.0f,
            pruebaX, pruebaY)) {
        rechazarObjetivoFueraDeRango(
            secuenciaObjetivoRecibida,
            "coordenada inicial fuera del espacio de trabajo"
        );
        return;
    }
    automaticoV2.secuencia = secuenciaObjetivoRecibida;
    automaticoV2.clase = claseObjetivo;
    automaticoV2.camXReferencia = camX;
    automaticoV2.camYReferencia = camY;
    automaticoV2.conteoReferencia = conteoReferenciaObjetivoRecibido;
    // Esta instantanea queda inmutable durante el catch. Los paquetes RS485
    // posteriores no sustituyen camX/camY/secuencia; solo el delta del encoder
    // actualiza objetivoBrazoY en actualizarObjetivoMovilV2().
    automaticoV2.objetivoBrazoX = brazoX;
    automaticoV2.objetivoBrazoY = yCatch;
    automaticoV2.objetivoBrazoYAnterior = yCatch;
    automaticoV2.ultimoConteoProcesado = conteoEncoderBanda;
    automaticoV2.instanteObjetivoAnterior = millis();
    automaticoV2.velocidadInicialY = velocidadBandaMmS;
    automaticoV2.busquedaFinalZActiva = false;
    automaticoV2.umbralDisparoY = 0.0f;
    if (!actualizarObjetivoMovilV2()) {
        rechazarObjetivoFueraDeRango(
            secuenciaObjetivoRecibida,
            "prediccion inicial V2 no valida"
        );
        reiniciarAutomaticoV2();
        return;
    }
    automaticoV2.umbralDisparoY = calcularUmbralDisparoYV2();
    if (automaticoV2.objetivoBrazoY > automaticoV2.umbralDisparoY) {
        rechazarObjetivoFueraDeRango(
            secuenciaObjetivoRecibida,
            "pieza demasiado cerca para posicionar XYZ"
        );
        reiniciarAutomaticoV2();
        return;
    }
    const long precapturaZ = posicionPrecapturaZV2();
    if (!posicionZSeguraV2(precapturaZ)) {
        rechazarObjetivoFueraDeRango(
            secuenciaObjetivoRecibida,
            "altura Z de captura fuera de rango"
        );
        reiniciarAutomaticoV2();
        return;
    }

    Serial.print(F("[AUTO V2] Objetivo aceptado seq="));
    Serial.print(automaticoV2.secuencia);
    Serial.print(F(" encoder="));
    Serial.print(automaticoV2.conteoReferencia);
    Serial.print(F(" yCatch="));
    Serial.print(yCatch, 2);
    Serial.println(F(" (posicion de espera atras)"));

    if (!iniciarMovimientoXY(
            brazoX, yCatch, 0.0f,
            MOV_AUTOMATICO_V2)) {
        rechazarObjetivoFueraDeRango(
            secuenciaObjetivoRecibida,
            "no se pudo iniciar la preposicion XY"
        );
        reiniciarAutomaticoV2();
        return;
    }
    // XY y Z arrancan juntos, pero Z espera sobre DIN04 hasta el catch.
    moverZHasta(precapturaZ, DIV_POSICION);
    secuenciaObjetivoEnMovimiento = secuenciaObjetivoRecibida;
    ackSecuenciaObjetivo = secuenciaObjetivoRecibida;
    codigoAckObjetivo = ACK_OBJ_ACEPTADO;
    registrarEventoPortentaV2("ACCEPT", "objetivo aceptado; Z a precaptura");
    registrarAckPortentaV2("objetivo aceptado");
    cambiarFaseAutomaticoV2(V2_PREPOSICIONANDO);
}

void imprimirContadoresV2() {
    Serial.print(F("[AUTO V2] intentos="));
    Serial.print(intentosV2);
    Serial.print(F(" ciclos_entregados_no_verificados="));
    Serial.println(exitosV2);
}

void completarResultadoV2(const char *resultado) {
    detenerTodos();
    ackSecuenciaObjetivo = automaticoV2.secuencia;
    codigoAckObjetivo = ACK_OBJ_COMPLETADO;
    secuenciaObjetivoEnMovimiento = 0;
    Serial.print(F("[AUTO V2] "));
    Serial.println(resultado);
    registrarEventoPortentaV2("RESULT", resultado);
    registrarAckPortentaV2("objetivo completado");
    imprimirContadoresV2();
    cambiarFaseAutomaticoV2(V2_COMPLETADO);
}

// Compartido por Seguimiento Y y Automatico V2: actualizar el destino sin
// reiniciar el tren de pulsos cuando se mantiene direccion y divisor.
bool seguirPiezaY(float piezaY) {
    const float yMin = -RANGO_FISICO_Y_MM * 0.5f +
        MARGEN_SEGURIDAD_MM + ML_SEGUIMIENTO_MARGEN_Y_MM;
    const float yMax = RANGO_FISICO_Y_MM * 0.5f -
        MARGEN_SEGURIDAD_MM - ML_SEGUIMIENTO_MARGEN_Y_MM;
    if (!isfinite(piezaY) || piezaY > yMax || limiteYmas || limiteYmenos) {
        detenerY();
        return false;
    }
    const float destinoMm = fmaxf(yMin, fminf(yMax, piezaY));
    const long destino = lroundf(destinoMm * pasosPorMmY);
    const long actual = leerPasosY();
    const long tolerancia = lroundf(1.5f * pasosPorMmY);
    const int8_t direccion = destino > actual ? 1 : -1;
    if (labs(destino - actual) <= tolerancia) {
        detenerY();
    } else {
        noInterrupts();
        const bool continuar = objetivoYActivo && movY == direccion &&
            divisorY == DIV_POSICION;
        if (continuar) objetivoY = destino;
        interrupts();
        if (!continuar) moverYHasta(destino, DIV_POSICION);
    }
    return true;
}

bool seguirPiezaYAutomaticoV2() {
    if (!seguirPiezaY(automaticoV2.objetivoBrazoY)) return false;
    automaticoV2.ultimoErrorY = automaticoV2.objetivoBrazoY - posicionYmm();
    if (millis() - automaticoV2.ultimoLogSeguimiento >= ML_SEGUIMIENTO_LOG_MS) {
        automaticoV2.ultimoLogSeguimiento = millis();
        registrarEventoPortentaV2("AUTO_TRACK", "Y sigue pieza por encoder");
    }
    return true;
}

bool iniciarTrasladoEntrega() {
    const float xEntrega = rangoXmm() * 0.5f - ML_MARGEN_FINAL_DERECHO_MM;
    if (!iniciarMovimientoXY(xEntrega, posicionCatchYV2(), 0.0f,
                             MOV_AUTOMATICO_V2)) return false;
    moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
    return true;
}

void registrarAjusteCatchV2(const char *evento, const char *resultado = "PENDIENTE") {
    Serial.print(F("V2LOG|P|mode=CATCH_CAL|event=")); Serial.print(evento);
    Serial.print(F("|session=")); Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
    Serial.print(F("|ms=")); Serial.print(millis());
    Serial.print(F("|obj=")); Serial.print(automaticoV2.secuencia);
    Serial.print(F("|class=")); Serial.print(automaticoV2.clase);
    Serial.print(F("|phase=")); Serial.print(static_cast<uint8_t>(automaticoV2.fase));
    Serial.print(F("|timing_result=")); Serial.print(resultado);
    Serial.print(F("|tested_offset_ms=")); Serial.print(automaticoV2.ajusteProbadoMs);
    Serial.print(F("|next_offset_ms=")); Serial.print(ajusteCatchV2.offsetMs);
    Serial.print(F("|adjust_step_ms=")); Serial.print(ajusteCatchV2.pasoMs);
    Serial.print(F("|adjust_trials=")); Serial.print(ajusteCatchV2.ensayos);
    Serial.print(F("|adjust_success_streak=")); Serial.print(ajusteCatchV2.aciertosConsecutivos);
    Serial.print(F("|adjust_confirmed=")); Serial.print(ajusteCatchV2.confirmado() ? 1 : 0);
    Serial.print(F("|adjust_limit=")); Serial.print(ajusteCatchV2.limiteAlcanzado ? 1 : 0);
    Serial.print(F("|auto_reference_ms="));
    if (automaticoV2.referenciaCatchRegistrada) Serial.print(automaticoV2.referenciaCatchMs);
    else Serial.print(F("NA"));
    Serial.print(F("|reference_status="));
    Serial.print(!automaticoV2.referenciaCatchRegistrada ? F("AUSENTE") :
        (automaticoV2.referenciaCatchProyectada ? F("PROYECTADA") : F("OBSERVADA")));
    Serial.print(F("|catch_start_ms=")); Serial.print(automaticoV2.disparoCatchMs);
    Serial.print(F("|delta_reference_ms="));
    if (automaticoV2.disparoCatchMs != 0 && automaticoV2.referenciaCatchRegistrada)
        Serial.print(static_cast<int32_t>(automaticoV2.disparoCatchMs - automaticoV2.referenciaCatchMs));
    else Serial.print(F("NA"));
    Serial.print(F("|z_bottom_ms=")); Serial.print(automaticoV2.din04CatchMs);
    Serial.print(F("|catch_command_ms=")); Serial.print(automaticoV2.cierreCatchMs);
    Serial.print(F("|scale_mm_count=")); Serial.print(escalaEncoderMmPorCuenta, 7);
    Serial.print(F("|encoder_sign=")); Serial.print(signoEncoderAvance);
    Serial.print(F("|camera_distance_mm=")); Serial.print(CAMARA_A_HOME_Y_MM, 3);
    Serial.print(F("|camera_x=")); Serial.print(automaticoV2.camXReferencia, 3);
    Serial.print(F("|camera_y=")); Serial.print(automaticoV2.camYReferencia, 3);
    Serial.print(F("|ref_enc=")); Serial.print(automaticoV2.conteoReferencia);
    Serial.print(F("|enc=")); Serial.print(conteoEncoderBanda);
    Serial.print(F("|piece_y=")); Serial.print(automaticoV2.objetivoBrazoY, 3);
    Serial.print(F("|arm_y=")); Serial.print(posicionYmm(), 3);
    Serial.print(F("|error_y=")); Serial.print(automaticoV2.objetivoBrazoY - posicionYmm(), 3);
    Serial.print(F("|trigger_error_y=")); Serial.print(automaticoV2.errorYDisparo, 3);
    Serial.print(F("|trigger_speed_mm_s=")); Serial.print(automaticoV2.velocidadDisparo, 3);
    Serial.print(F("|trigger_encoder=")); Serial.print(automaticoV2.conteoDisparoCatch);
    Serial.print(F("|trigger_arm_y=")); Serial.print(automaticoV2.brazoYDisparo, 3);
    Serial.print(F("|trigger_piece_y=")); Serial.print(automaticoV2.piezaYDisparo, 3);
    Serial.print(F("|servo_rot_deg=")); Serial.println(posServoRot);
}

void procesarModoAutomaticoV2() {
    if (automaticoV2.fase == V2_CANCELANDO) {
        if ((movZ > 0 && limiteZarriba) || (movZ < 0 && limiteZabajo)) {
            detenerTodos();
            entrarErrorSistema(ERROR_FINAL_INESPERADO,
                               "Final de Z durante retirada V2");
        } else if (!objetivoZEnCurso() && movZ == 0) {
            finalizarCancelacionAutomaticoV2();
        } else if (millis() - automaticoV2.inicioFase > V2_TIMEOUT_FASE_MS) {
            detenerTodos();
            entrarErrorSistema(ERROR_TIMEOUT_MOVIMIENTO,
                               "Timeout retirando Z en Automatico V2");
        }
        return;
    }

    if (!btConectado) {
        iniciarCancelacionAutomaticoV2("control Bluetooth desconectado", true);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        iniciarCancelacionAutomaticoV2("cancelado por usuario", false, true);
        return;
    }
    const bool finalZAbajoEsperado =
        (automaticoV2.fase == V2_BAJANDO_CATCH ||
         automaticoV2.fase == V2_BAJANDO_ENTREGA) &&
        movZ < 0 && limiteZabajo;
    const bool finalInesperado =
        (movX > 0 && limiteXmas) || (movX < 0 && limiteXmenos) ||
        (movY > 0 && limiteYmas) || (movY < 0 && limiteYmenos) ||
        (movZ > 0 && limiteZarriba) ||
        (movZ < 0 && limiteZabajo && !finalZAbajoEsperado);
    if (finalInesperado) {
        detenerTodos();
        entrarErrorSistema(ERROR_FINAL_INESPERADO,
                           "Final de carrera durante Automatico V2");
        return;
    }
    const bool prediccionLlegadaActivaV2 =
        (automaticoV2.fase >= V2_PREPOSICIONANDO &&
         automaticoV2.fase <= V2_DISPARANDO_CATCH) ||
        automaticoV2.fase == V2_BAJANDO_Z ||
        automaticoV2.fase == V2_BAJANDO_CATCH ||
        automaticoV2.fase == V2_ESPERANDO_CATCH_AUTOMATICO ||
        automaticoV2.fase == V2_SIGUIENDO_PIEZA ||
        automaticoV2.fase == V2_CERRANDO_PINZA;
    if (!camaraListaCompleta()) {
        if (automaticoV2.fase == V2_ESPERANDO_PIEZA) {
            detenerTodos();
            return;
        }
        // Despues de publicar, las coordenadas de camara estan congeladas.
        // La llegada depende solo del conteo del encoder.
    }
    if (!encoderListoAutomaticoV2()) {
        if (automaticoV2.fase == V2_ESPERANDO_PIEZA) {
            detenerTodos();
            return;
        }
        if (prediccionLlegadaActivaV2) {
            iniciarCancelacionAutomaticoV2("encoder no valido", false);
            return;
        }
    }
    if (prediccionLlegadaActivaV2 && !bandaEnMovimientoAutomaticoV2()) {
        iniciarCancelacionAutomaticoV2("la banda se detuvo", false);
        return;
    }
    if (automaticoV2.fase != V2_ESPERANDO_PIEZA &&
        automaticoV2.fase != V2_COMPLETADO &&
        automaticoV2.fase != V2_EVALUANDO_CATCH &&
        millis() - automaticoV2.inicioFase > V2_TIMEOUT_FASE_MS) {
        iniciarCancelacionAutomaticoV2("timeout de fase", false);
        return;
    }

    switch (automaticoV2.fase) {
        case V2_PREPARANDO_ESPERA:
            detenerX();
            detenerY();
            if (!objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    entrarErrorSistema(
                        ERROR_TIMEOUT_MOVIMIENTO,
                        "Z no alcanzo la posicion segura al entrar a V2"
                    );
                    return;
                }
                cambiarFaseAutomaticoV2(V2_ESPERANDO_PIEZA);
            }
            break;

        case V2_ESPERANDO_PIEZA:
            detenerTodos();
            // La igualdad solo bloquea la repeticion del mismo mensaje RS485.
            // Una deteccion posterior, incluso de la misma pieza o clase,
            // genera otra secuencia y se acepta normalmente.
            if (!objetivoCamaraValido() || secuenciaObjetivoRecibida == 0 ||
                secuenciaObjetivoRecibida == ackSecuenciaObjetivo) {
                return;
            }
            aceptarObjetivoAutomaticoV2();
            break;

        case V2_PREPOSICIONANDO:
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion de pieza no valida", false);
                return;
            }
            if (automaticoV2.objetivoBrazoY >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2(
                    "pieza rebaso el catch antes de terminar la preposicion", false);
                return;
            }
            if (!movimientoPosicionadoActivo && !objetivoXEnCurso() &&
                !objetivoYEnCurso() && movX == 0 && movY == 0) {
                // Z se inicio junto con XY y termina en la altura de espera.
                cambiarFaseAutomaticoV2(V2_BAJANDO_Z);
            }
            break;

        case V2_ESPERANDO_LLEGADA:
            detenerY();
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion de pieza no valida preparando descenso", false);
                return;
            }
            automaticoV2.ultimoErrorY =
                automaticoV2.objetivoBrazoY - posicionCatchYV2();
            automaticoV2.umbralDisparoY = calcularUmbralDisparoYV2();
            if (automaticoV2.objetivoBrazoY >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2(
                    "pieza rebaso la estacion de catch", false);
                return;
            }
            // Si esta ruta de preparacion se usa, Z baja solo hasta
            // precaptura y espera elevado hasta el disparo final.
            if (automaticoV2.objetivoBrazoY >
                automaticoV2.umbralDisparoY) {
                iniciarCancelacionAutomaticoV2(
                    "pieza demasiado cerca para prebajar Z con seguridad", false);
                return;
            }
            Serial.print(F("[AUTO V2] Precaptura Z inmediata piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 2);
            Serial.print(F(" limite_seguro="));
            Serial.print(automaticoV2.umbralDisparoY, 2);
            Serial.print(F(" vel="));
            Serial.print(velocidadBandaMmS, 2);
            Serial.print(F(" tZ="));
            Serial.print(tiempoDescensoZV2Segundos(), 3);
            Serial.print(F(" reservaX="));
            Serial.println(V2_RESERVA_MANUAL_CATCH_S, 3);
            registrarEventoPortentaV2(
                "TRIGGER", "preposicion lista; Z espera sobre DIN04"
            );
            cambiarFaseAutomaticoV2(V2_DISPARANDO_CATCH);
            break;

        case V2_DISPARANDO_CATCH: {
            detenerY();
            const long precapturaZ = posicionPrecapturaZV2();
            if (!posicionZSeguraV2(precapturaZ)) {
                iniciarCancelacionAutomaticoV2(
                    "altura Z de precaptura fuera de rango", false);
                return;
            }
            moverZHasta(precapturaZ, DIV_POSICION);
            cambiarFaseAutomaticoV2(V2_BAJANDO_Z);
            break;
        }

        case V2_BAJANDO_Z:
            detenerY();
            if (limiteZabajo) {
                iniciarCancelacionAutomaticoV2(
                    "DIN04 activo antes del catch", false);
                return;
            }
            if (!objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != posicionPrecapturaZV2()) {
                    iniciarCancelacionAutomaticoV2(
                        "Z no alcanzo la precaptura", false);
                    return;
                }
                automaticoV2.ultimoErrorX =
                    automaticoV2.objetivoBrazoX - posicionXmm();
                // El encoder seguira la pieza mientras Z espera sobre DIN04.
                if (encoderListoAutomaticoV2()) {
                    actualizarObjetivoMovilV2();
                }
                automaticoV2.ultimoErrorY =
                    automaticoV2.objetivoBrazoY - posicionCatchYV2();
                automaticoV2.instanteListoCatch = millis();
                automaticoV2.conteoListoCatch = conteoEncoderBanda;
                automaticoV2.catchAutomaticoDisparado = false;
                // X ya no gobierna el catch. Se limpia cualquier evento viejo
                // para que no afecte otro modo al terminar este ciclo.
                eventoBotonX = false;
                detenerY();
                detenerZ();
                Serial.print(F("[AUTO V2] LISTO PARA CATCH AUTOMATICO X="));
                Serial.print(posicionXmm(), 2);
                Serial.print(F(" Y="));
                Serial.print(posicionYmm(), 2);
                Serial.print(F(" errorX="));
                Serial.print(automaticoV2.ultimoErrorX, 2);
                Serial.print(F(" errorY="));
                Serial.print(automaticoV2.ultimoErrorY, 2);
                Serial.print(F(" velocidad="));
                Serial.print(velocidadBandaMmS, 2);
                Serial.print(F(" encoder="));
                Serial.print(conteoEncoderBanda);
                Serial.print(F(" z_espera="));
                Serial.print(leerPasosZ());
                Serial.print(F(" margen_DIN04="));
                Serial.println(Z_MARGEN_PRECAPTURA_PASOS);
                registrarEventoPortentaV2(
                    "READY_CATCH",
                    "Z en precaptura; esperando descenso final por encoder"
                );
                cambiarFaseAutomaticoV2(AUTO_V2_SEGUIMIENTO_Y
                    ? V2_SIGUIENDO_PIEZA : V2_ESPERANDO_CATCH_AUTOMATICO);
            }
            break;

        case V2_SUBIENDO_Z:
            detenerY();
            if (!objetivoZEnCurso() && movZ == 0) {
                if (!automaticoV2.catchAutomaticoDisparado) {
                    iniciarCancelacionAutomaticoV2(
                        "Z subio sin disparo de catch automatico", false);
                    return;
                }
                if (!iniciarTrasladoEntrega()) {
                    iniciarCancelacionAutomaticoV2("entrega fuera de rango", false);
                    return;
                }
                cambiarFaseAutomaticoV2(V2_MOVIENDO_ENTREGA);
            }
            break;

        case V2_SIGUIENDO_PIEZA: {
            detenerX();
            detenerZ();
            eventoBotonX = false;
            if (limiteZabajo || leerPasosZ() != posicionPrecapturaZV2()) {
                iniciarCancelacionAutomaticoV2("Z salio de precaptura siguiendo", false);
                return;
            }
            if (!actualizarObjetivoMovilV2() || !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("seguimiento Y fuera de recorrido", false);
                return;
            }
            const float yMax = RANGO_FISICO_Y_MM * 0.5f -
                MARGEN_SEGURIDAD_MM - ML_SEGUIMIENTO_MARGEN_Y_MM;
            const float finCatchY = automaticoV2.objetivoBrazoY +
                fmaxf(0.0f, velocidadBandaMmS) *
                (tiempoDescensoFinalZV2Segundos() + anticipacionCierreCatchSegundos() +
                 ML_SEGUIMIENTO_RESERVA_S);
            if (!isfinite(finCatchY) || finCatchY > yMax) {
                iniciarCancelacionAutomaticoV2("sin recorrido Y para completar catch", false);
                return;
            }
            const float yMin = -RANGO_FISICO_Y_MM * 0.5f +
                MARGEN_SEGURIDAD_MM + ML_SEGUIMIENTO_MARGEN_Y_MM;
            if (automaticoV2.objetivoBrazoY < yMin ||
                fabsf(automaticoV2.ultimoErrorY) > V2_ERROR_ESTABLE_MM ||
                fabsf(automaticoV2.objetivoBrazoX - posicionXmm()) > V2_ERROR_ESTABLE_MM) {
                automaticoV2.inicioEstable = 0;
                break;
            }
            if (automaticoV2.inicioEstable == 0) automaticoV2.inicioEstable = millis();
            const int32_t offset = ajusteCatchV2Seleccionado()
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
            automaticoV2.busquedaFinalZActiva = false;
            moverZHasta(posicionCapturaZV2(), DIV_POSICION);
            registrarEventoPortentaV2("TRIGGER", "seguimiento estable; descenso automatico");
            cambiarFaseAutomaticoV2(V2_BAJANDO_CATCH);
            if (ajusteCatchV2Seleccionado()) registrarAjusteCatchV2("CAL_TRIGGER");
            break;
        }

        case V2_ESPERANDO_CATCH_AUTOMATICO: {
            // Z espera sobre DIN04; el disparo incluye el tiempo del ultimo
            // descenso y del cierre para alcanzar la pieza en Y=0.
            detenerX();
            detenerY();
            detenerZ();
            // X queda ignorada: ya no es confirmacion ni cambia la trayectoria.
            eventoBotonX = false;
            if (limiteZabajo || leerPasosZ() != posicionPrecapturaZV2()) {
                iniciarCancelacionAutomaticoV2(
                    "Z salio de la altura de precaptura", false);
                return;
            }
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion invalida esperando catch automatico", false);
                return;
            }
            automaticoV2.ultimoErrorY =
                automaticoV2.objetivoBrazoY - posicionCatchYV2();
            automaticoV2.umbralCierrePinzaY = calcularUmbralCierrePinzaYV2();
            if (automaticoV2.objetivoBrazoY <
                automaticoV2.umbralCierrePinzaY) break;

            automaticoV2.busquedaFinalZActiva = false;
            moverZHasta(posicionCapturaZV2(), DIV_POSICION);
            Serial.print(F("[AUTO V2] Descenso final Z por encoder piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 3);
            Serial.print(F(" umbral_cierre="));
            Serial.print(automaticoV2.umbralCierrePinzaY, 3);
            Serial.print(F(" tZ="));
            Serial.println(tiempoDescensoFinalZV2Segundos(), 3);
            cambiarFaseAutomaticoV2(V2_BAJANDO_CATCH);
            break;
        }

        case V2_BAJANDO_CATCH: {
            detenerX();
            if (!AUTO_V2_SEGUIMIENTO_Y) detenerY();
            eventoBotonX = false;
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion invalida durante descenso final", false);
                return;
            }
            if (AUTO_V2_SEGUIMIENTO_Y && !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("fin recorrido Y bajando catch", false);
                return;
            }
            if (!AUTO_V2_SEGUIMIENTO_Y && automaticoV2.objetivoBrazoY >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2(
                    "pieza rebaso el catch antes de DIN04", false);
                return;
            }
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                automaticoV2.busquedaFinalZActiva = false;
                if (ajusteCatchV2Seleccionado()) automaticoV2.din04CatchMs = millis();
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!automaticoV2.busquedaFinalZActiva) {
                    automaticoV2.busquedaFinalZActiva = true;
                    moverZHasta(limiteMinimoZPasos() -
                        V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                        V2_DIV_BUSQUEDA_FINAL_Z);
                    registrarEventoPortentaV2(
                        "Z_SEARCH", "busqueda DIN04 durante catch");
                    break;
                }
                iniciarCancelacionAutomaticoV2(
                    "DIN04 no aparecio durante catch", false);
                return;
            }
            if (AUTO_V2_SEGUIMIENTO_Y &&
                fabsf(automaticoV2.ultimoErrorY) > V2_ERROR_ESTABLE_MM) {
                iniciarCancelacionAutomaticoV2("Y no alineada al confirmar DIN04", false);
                return;
            }
            automaticoV2.catchAutomaticoDisparado = true;
            automaticoV2.anguloCatch = posServoRot;
            ++intentosV2;
            automaticoV2.ultimoErrorY =
                automaticoV2.objetivoBrazoY -
                (AUTO_V2_SEGUIMIENTO_Y ? posicionYmm() : posicionCatchYV2());
            automaticoV2.fase = V2_CERRANDO_PINZA;
            automaticoV2.inicioFase = millis();
            enviarOrdenCierreCatchAhora();
            if (ajusteCatchV2Seleccionado()) {
                automaticoV2.cierreCatchMs = automaticoV2.inicioFase;
                registrarAjusteCatchV2("CAL_GRIP");
            }
            Serial.print(F("[AUTO V2] DIN04; ORDEN CERRAR piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 3);
            Serial.print(F(" espera_ms="));
            Serial.println(millis() - automaticoV2.instanteListoCatch);
            registrarEventoPortentaV2(
                "GRIP_COMMAND", "DIN04 confirmado; cerrar pinza");
            registrarAckPortentaV2("orden cerrar pinza");
            break;
        }

        case V2_CERRANDO_PINZA:
            detenerX();
            if (!AUTO_V2_SEGUIMIENTO_Y) detenerY();
            detenerZ();
            eventoBotonX = false;
            if (!limiteZabajo) {
                iniciarCancelacionAutomaticoV2(
                    "se perdio DIN04 mientras cerraba la pinza", false);
                return;
            }
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion invalida mientras cerraba la pinza", false);
                return;
            }
            if (AUTO_V2_SEGUIMIENTO_Y && !seguirPiezaYAutomaticoV2()) {
                iniciarCancelacionAutomaticoV2("fin recorrido Y cerrando pinza", false);
                return;
            }
            automaticoV2.ultimoErrorY = automaticoV2.objetivoBrazoY -
                (AUTO_V2_SEGUIMIENTO_Y ? posicionYmm() : posicionCatchYV2());
            if (millis() - automaticoV2.inicioFase <
                V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) break;

            Serial.print(F("[AUTO V2] PINZA CERRADA piezaY="));
            Serial.print(automaticoV2.objetivoBrazoY, 3);
            Serial.print(F(" errorY="));
            Serial.println(automaticoV2.ultimoErrorY, 3);
            registrarEventoPortentaV2(
                "CAPTURE", "PINZA CERRADA POR PREDICCION DE ENCODER");
            if (ajusteCatchV2Seleccionado()) registrarAjusteCatchV2("CAL_CLOSE");
            // La garra ya termino su recorrido de cierre. Desde aqui Z puede
            // retirarse verticalmente sin intervencion del operador.
            detenerY();
            if (!iniciarTrasladoEntrega()) {
                iniciarCancelacionAutomaticoV2("destino de entrega fuera de rango", false);
                return;
            }
            cambiarFaseAutomaticoV2(V2_MOVIENDO_ENTREGA);
            break;

        case V2_MOVIENDO_ENTREGA:
            if (!movimientoPosicionadoActivo && !objetivoXEnCurso() &&
                !objetivoYEnCurso() && movX == 0 && movY == 0 &&
                !objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    iniciarCancelacionAutomaticoV2("Z no alcanzo altura de entrega", false);
                    return;
                }
                automaticoV2.busquedaFinalZActiva = false;
                moverZHasta(posicionCapturaZV2(), DIV_POSICION);
                cambiarFaseAutomaticoV2(V2_BAJANDO_ENTREGA);
            }
            break;

        case V2_BAJANDO_ENTREGA:
            detenerX();
            detenerY();
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!automaticoV2.busquedaFinalZActiva) {
                    automaticoV2.busquedaFinalZActiva = true;
                    moverZHasta(limiteMinimoZPasos() - V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                                V2_DIV_BUSQUEDA_FINAL_Z);
                    break;
                }
                iniciarCancelacionAutomaticoV2("DIN04 ausente durante entrega", false);
                return;
            }
            codigoAckObjetivo = ACK_OBJ_ABRIR_PINZA;
            cambiarFaseAutomaticoV2(V2_ABRIENDO_PINZA);
            if (comunicacionRS485Habilitada && enviarPaquetePortenta())
                tAnteriorEstadoESP = millis();
            registrarEventoPortentaV2("RELEASE", "abrir pinza en entrega derecha");
            break;

        case V2_ABRIENDO_PINZA:
            detenerTodos();
            if (millis() - automaticoV2.inicioFase <
                ML_TIEMPO_SERVO_MS + V2_LATENCIA_ORDEN_PINZA_MS) break;
            moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
            cambiarFaseAutomaticoV2(V2_SUBIENDO_FINAL);
            break;

        case V2_SUBIENDO_FINAL:
            if (!objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    iniciarCancelacionAutomaticoV2("Z no regreso a HOME tras entrega", false);
                    return;
                }
                if (ajusteCatchV2Seleccionado()) {
                    eventoBotonX = eventoBotonCuadrado = eventoBotonCirculo = false;
                    cambiarFaseAutomaticoV2(V2_EVALUANDO_CATCH);
                    registrarAjusteCatchV2("CAL_AWAIT_FEEDBACK");
                    Serial.println(F("[AJUSTE CATCH] X=LA AGARRO; cuadrado=ANTES; circulo=DESPUES; triangulo=descartar/salir"));
                } else {
                    ++exitosV2;
                    completarResultadoV2("CICLO_ENTREGADO_NO_VERIFICADO");
                }
            }
            break;

        case V2_EVALUANDO_CATCH:
            detenerTodos();
            if (!eventoBotonX && !eventoBotonCuadrado && !eventoBotonCirculo) break;
            {
                const char *resultado = eventoBotonX ? "CORRECTO" :
                    (eventoBotonCuadrado ? "TEMPRANO" : "TARDE");
                eventoBotonX = eventoBotonCuadrado = eventoBotonCirculo = false;
                ajusteCatchV2.evaluar(strcmp(resultado, "CORRECTO") == 0 ? AjusteCatchV2::AGARRO :
                    (strcmp(resultado, "TEMPRANO") == 0 ? AjusteCatchV2::ANTES : AjusteCatchV2::DESPUES),
                    automaticoV2.ajusteProbadoMs);
                registrarAjusteCatchV2("CAL_SAMPLE", resultado);
                imprimirCambiosCatchV2();
                if (strcmp(resultado, "CORRECTO") == 0) ++exitosV2;
                completarResultadoV2("CICLO_EVALUADO_POR_OPERADOR");
            }
            break;

        case V2_COMPLETADO:
            detenerTodos();
            if (millis() - automaticoV2.inicioFase >= V2_TIEMPO_COMPLETADO_MS) {
                reiniciarAutomaticoV2();
            }
            break;

        case V2_CANCELANDO:
            break;
    }
}

const char *nombreFaseEntrenamientoML(FaseEntrenamientoML fase) {
    switch (fase) {
        case ML_ESPERANDO_PIEZA: return "ESPERANDO PIEZA";
        case ML_PREPOSICIONANDO: return "ALINEANDO CAMARA";
        case ML_BAJANDO_CAPTURA: return "BAJANDO PRECAPTURA";
        case ML_BAJANDO_CATCH: return "BAJANDO CATCH";
        case ML_ALINEACION_MANUAL: return "CORRECCION MANUAL";
        case ML_CERRANDO_PINZA: return "CERRANDO PINZA";
        case ML_SUBIENDO_CON_PIEZA: return "SUBIENDO PIEZA";
        case ML_MOVIENDO_ENTREGA: return "YENDO A DERECHA";
        case ML_BAJANDO_ENTREGA: return "BAJANDO ENTREGA";
        case ML_ABRIENDO_PINZA: return "SOLTANDO PIEZA";
        case ML_SUBIENDO_FINAL: return "SUBIENDO FINAL";
        case ML_LISTO: return "LISTO";
        case ML_CANCELANDO: return "CANCELANDO";
        case ML_PREPARANDO_ESPERA: return "PREPARANDO ESPERA";
        case ML_ESPERANDO_CONFIRMACION: return "ESPERANDO CONFIRMACION";
        case ML_SEGUIMIENTO: return "SIGUIENDO PIEZA EN Y";
        default: return "DESCONOCIDA";
    }
}

void cambiarFaseEntrenamientoML(FaseEntrenamientoML nueva) {
    entrenamientoML.fase = nueva;
    entrenamientoML.inicioFase = millis();
    Serial.print(F("[ML] Fase -> "));
    Serial.println(nombreFaseEntrenamientoML(nueva));
}

void reiniciarEntrenamientoML() {
    entrenamientoML = {};
    entrenamientoML.fase = leerPasosZ() == V2_Z_SEGURO_PASOS
        ? ML_ESPERANDO_PIEZA : ML_PREPARANDO_ESPERA;
    entrenamientoML.inicioFase = millis();
    if (entrenamientoML.fase == ML_PREPARANDO_ESPERA) {
        moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
        Serial.println(F("[ML] Llevando Z a altura segura antes de detectar"));
    }
}

void cancelarEntrenamientoML(
    const char *motivo,
    bool esperarControl,
    bool salirAlMenu
) {
    Serial.print(F("[ML] Cancelando: "));
    Serial.println(motivo);
    detenerX();
    detenerY();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    if (entrenamientoML.secuencia != 0) {
        ackSecuenciaObjetivo = entrenamientoML.secuencia;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
    }
    entrenamientoML.salidaAEsperaControl = esperarControl;
    entrenamientoML.salidaAlMenu = salirAlMenu;
    cambiarFaseEntrenamientoML(ML_CANCELANDO);
    if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
        moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
    }
}

void terminarCancelacionEntrenamientoML() {
    detenerTodos();
    const bool esperarControl = entrenamientoML.salidaAEsperaControl;
    const bool salirAlMenu = entrenamientoML.salidaAlMenu;
    reiniciarEntrenamientoML();
    if (esperarControl || !btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
    } else if (salirAlMenu) {
        cambiarEstadoGeneral(EST_MAIN_MENU);
    }
}

void registrarMuestraEntrenamientoML(
    const char *disparador, const char *resultadoFisico = nullptr) {
    const bool catchManual = strcmp(disparador, "X") == 0;
    const bool esV2 = entrenamientoConResultadoSeleccionado();
    ++muestrasEntrenamientoML;
    if (!esV2) {
        entrenamientoML.xCorregida = posicionXmm();
        entrenamientoML.yCorregida = posicionYmm();
        entrenamientoML.rotacionCorregida = posServoRot;
    }
    Serial.print(pruebaSeguimientoSeleccionada()
        ? F("ML_SAMPLE|mode=ML_TRACK|ms=")
        : (esV2 ? F("ML_SAMPLE|mode=ML_V2|ms=")
                : F("ML_SAMPLE|mode=ML|ms=")));
    Serial.print(millis());
    Serial.print(F("|session="));
    Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
    Serial.print(F("|n="));
    Serial.print(muestrasEntrenamientoML);
    Serial.print(F("|seq="));
    Serial.print(entrenamientoML.secuencia);
    Serial.print(F("|class="));
    Serial.print(entrenamientoML.clase);
    Serial.print(F("|cam_x="));
    Serial.print(entrenamientoML.camX, 3);
    Serial.print(F("|cam_y="));
    Serial.print(entrenamientoML.camY, 3);
    Serial.print(F("|auto_x="));
    Serial.print(entrenamientoML.xInicial, 3);
    Serial.print(F("|auto_y="));
    Serial.print(entrenamientoML.yInicial, 3);
    Serial.print(F("|label_x="));
    Serial.print(entrenamientoML.xCorregida, 3);
    Serial.print(F("|label_y="));
    Serial.print(entrenamientoML.yCorregida, 3);
    Serial.print(F("|delta_x="));
    Serial.print(entrenamientoML.xCorregida - entrenamientoML.xInicial, 3);
    Serial.print(F("|delta_y="));
    Serial.print(entrenamientoML.yCorregida - entrenamientoML.yInicial, 3);
    Serial.print(F("|label_rot="));
    Serial.print(entrenamientoML.rotacionCorregida);
    Serial.print(F("|piece_y="));
    Serial.print(esV2 ? entrenamientoML.piezaYDisparo
                      : entrenamientoML.piezaYEstimada, 3);
    Serial.print(F("|catch_y="));
    Serial.print(entrenamientoML.catchYConfirmada, 3);
    Serial.print(F("|close_threshold_y="));
    Serial.print(entrenamientoML.umbralCierreY, 3);
    Serial.print(F("|belt_mm_s="));
    Serial.print(esV2 ? entrenamientoML.velocidadDisparo
                      : velocidadBandaMmS, 3);
    Serial.print(F("|encoder_ref="));
    Serial.print(entrenamientoML.conteoReferencia);
    Serial.print(F("|encoder="));
    Serial.print(esV2 ? entrenamientoML.conteoOrdenCierre
                      : conteoEncoderBanda);
    Serial.print(F("|trigger="));
    Serial.print(disparador);
    Serial.print(F("|catch_type="));
    Serial.print(catchManual ? F("MANUAL") : F("AUTOMATICO"));
    Serial.print(F("|servo_rot_deg="));
    Serial.print(entrenamientoML.rotacionCorregida);
    Serial.print(F("|catch_command_ms="));
    Serial.print(entrenamientoML.instanteOrdenCierre);
    Serial.print(F("|detection_to_command_ms="));
    Serial.print(entrenamientoML.instanteOrdenCierre -
                 entrenamientoML.instanteDeteccion);
    Serial.print(F("|error_disparo_mm="));
    Serial.print(entrenamientoML.diferenciaDisparoMm, 3);
    const float velocidad = entrenamientoML.velocidadDisparo;
    Serial.print(F("|error_disparo_ms="));
    Serial.print(velocidad > 0.0f
        ? 1000.0f * entrenamientoML.diferenciaDisparoMm / velocidad : NAN, 3);
    Serial.print(F("|anticipacion_ms="));
    Serial.print(1000.0f * anticipacionCierreCatchSegundos(), 3);
    Serial.print(F("|cierre_y_predicha="));
    Serial.print(entrenamientoML.piezaYDisparo + velocidad *
        static_cast<float>(V2_TIEMPO_CIERRE_PINZA_MS +
                           V2_LATENCIA_ORDEN_PINZA_MS) / 1000.0f, 3);
    Serial.print(F("|adelanto_extra_sugerido_ms="));
    // Solo X es una etiqueta humana. Un disparo automatico no demuestra exito.
    Serial.print(strcmp(disparador, "X") == 0 && velocidad > 0.0f
        ? CATCH_ADELANTO_EXTRA_MS -
          1000.0f * entrenamientoML.diferenciaDisparoMm / velocidad : NAN, 3);
    Serial.print(F("|desfase_x_mm="));
    Serial.print(DESFASE_CAMARA_X_MM, 3);
    Serial.print(F("|desfase_y_mm="));
    Serial.print(DESFASE_CAMARA_Y_MM, 3);
    if (esV2) {
        Serial.print(F("|catch_button_ms="));
        Serial.print(entrenamientoML.instanteBotonCatch);
        Serial.print(F("|catch_button_encoder="));
        Serial.print(entrenamientoML.conteoBotonCatch);
        Serial.print(F("|catch_button_piece_y="));
        Serial.print(entrenamientoML.piezaYBotonCatch, 3);
        Serial.print(F("|z_bottom_ms="));
        Serial.print(entrenamientoML.instanteDin04Catch);
        Serial.print(F("|button_to_grip_ms="));
        Serial.print(entrenamientoML.instanteOrdenCierre -
                     entrenamientoML.instanteBotonCatch);
        Serial.print(F("|physical_result="));
        if (resultadoFisico == nullptr) Serial.print(F("SIN_CONFIRMAR"));
        else Serial.print(resultadoFisico);
        if (pruebaSeguimientoSeleccionada()) {
            Serial.print(F("|track_arm_y_button="));
            Serial.print(entrenamientoML.brazoYBoton, 3);
            Serial.print(F("|track_error_button="));
            Serial.print(entrenamientoML.errorSeguimientoBoton, 3);
            Serial.print(F("|track_arm_y_bottom="));
            Serial.print(entrenamientoML.brazoYDin04, 3);
            Serial.print(F("|track_error_bottom="));
            Serial.print(entrenamientoML.errorSeguimientoDin04, 3);
            Serial.print(F("|track_arm_y_close="));
            Serial.print(entrenamientoML.brazoYCierre, 3);
            Serial.print(F("|track_error_close="));
            Serial.print(entrenamientoML.errorSeguimientoCierre, 3);
            Serial.print(F("|track_before_button_ms="));
            Serial.print(entrenamientoML.instanteBotonCatch -
                         entrenamientoML.inicioSeguimiento);
        }
    }
    Serial.println(esV2 ? F("|error_fisico_medido=SI")
                        : F("|error_fisico_medido=NO"));
    Serial.print(F("[ML][MEDICION] servo_rot="));
    Serial.print(entrenamientoML.rotacionCorregida);
    Serial.print(F(" grados; catch="));
    Serial.print(catchManual ? F("MANUAL (X)") : F("AUTOMATICO (ENCODER)"));
    Serial.print(F("; diferencia respecto al disparo encoder="));
    Serial.print(entrenamientoML.diferenciaDisparoMm, 3);
    Serial.print(F(" mm / "));
    Serial.print(velocidad > 0.0f
        ? 1000.0f * entrenamientoML.diferenciaDisparoMm / velocidad : NAN, 3);
    Serial.println(F(" ms (negativo=antes, positivo=despues; estimacion)"));
}

bool actualizarPiezaEntrenamientoML() {
    conteoEncoderBanda = static_cast<int32_t>(encoders[0].getPulses());
    const int32_t delta = diferenciaConteosConWrap(
        conteoEncoderBanda,
        entrenamientoML.conteoReferencia
    );
    const float baseY = CAMERA_SWAP_XY
        ? entrenamientoML.camX : entrenamientoML.camY;
    const float yLocalCamara = static_cast<float>(CAMERA_SIGN_Y) * baseY +
                               CAMERA_OFFSET_Y_MM;
    const float piezaY = -CAMARA_A_HOME_Y_MM + yLocalCamara +
        static_cast<float>(signoEncoderAvance) *
        static_cast<float>(delta) * escalaEncoderMmPorCuenta;
    if (!isfinite(piezaY)) return false;
    entrenamientoML.piezaYEstimada = piezaY;
    return true;
}

bool seguirPiezaYEntrenamientoML() {
    // La camara fija la posicion inicial; el encoder mueve esa referencia
    // durante este intento. Y se mantiene sobre la pieza incluso en Z final.
    const float piezaY = entrenamientoML.piezaYEstimada;
    if (!seguirPiezaY(piezaY)) return false;
    const unsigned long ahora = millis();
    if (ahora - entrenamientoML.ultimoLogSeguimiento >=
        ML_SEGUIMIENTO_LOG_MS) {
        entrenamientoML.ultimoLogSeguimiento = ahora;
        Serial.print(F("V2LOG|P|mode=ML_TRACK|event=ML_TRACK|session="));
        Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
        Serial.print(F("|ms=")); Serial.print(ahora);
        Serial.print(F("|obj=")); Serial.print(entrenamientoML.secuencia);
        Serial.print(F("|piece_y=")); Serial.print(piezaY, 3);
        Serial.print(F("|arm_y=")); Serial.print(posicionYmm(), 3);
        Serial.print(F("|target_y=")); Serial.print(objetivoY / pasosPorMmY, 3);
        Serial.print(F("|error_y="));
        Serial.print(piezaY - posicionYmm(), 3);
        Serial.print(F("|vel=")); Serial.println(velocidadBandaMmS, 3);
    }
    return true;
}

bool iniciarTrasladoEntregaML() {
    // La retirada vertical y el traslado X/Y ocurren al mismo tiempo. La
    // bajada de entrega solo comienza cuando los tres ejes terminaron.
    return iniciarTrasladoEntrega();
}

void procesarEntrenamientoML() {
    if (entrenamientoML.fase == ML_CANCELANDO) {
        if (!objetivoZEnCurso() && movZ == 0) {
            terminarCancelacionEntrenamientoML();
        } else if (millis() - entrenamientoML.inicioFase > V2_TIMEOUT_FASE_MS) {
            entrarErrorSistema(
                ERROR_TIMEOUT_MOVIMIENTO,
                "Timeout retirando Z en entrenamiento ML"
            );
        }
        return;
    }

    if (!btConectado) {
        cancelarEntrenamientoML("control Bluetooth desconectado", true, false);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        cancelarEntrenamientoML("cancelado por usuario", false, true);
        return;
    }
    const bool seguimientoMovilActivo =
        entrenamientoML.fase == ML_PREPOSICIONANDO ||
        entrenamientoML.fase == ML_BAJANDO_CAPTURA ||
        entrenamientoML.fase == ML_ALINEACION_MANUAL ||
        entrenamientoML.fase == ML_SEGUIMIENTO ||
        entrenamientoML.fase == ML_BAJANDO_CATCH ||
        entrenamientoML.fase == ML_CERRANDO_PINZA;
    if (!encoderListoAutomaticoV2()) {
        if (entrenamientoML.fase == ML_ESPERANDO_PIEZA) {
            detenerTodos();
            return;
        }
        if (seguimientoMovilActivo) {
            cancelarEntrenamientoML("encoder no valido", false, false);
            return;
        }
    }
    if (seguimientoMovilActivo && !bandaEnMovimientoAutomaticoV2()) {
        cancelarEntrenamientoML("la banda se detuvo", false, false);
        return;
    }
    if (entrenamientoML.fase != ML_ESPERANDO_PIEZA &&
        entrenamientoML.fase != ML_ALINEACION_MANUAL &&
        entrenamientoML.fase != ML_SEGUIMIENTO &&
        entrenamientoML.fase != ML_ESPERANDO_CONFIRMACION &&
        entrenamientoML.fase != ML_LISTO &&
        millis() - entrenamientoML.inicioFase > V2_TIMEOUT_FASE_MS) {
        cancelarEntrenamientoML("timeout de fase", false, false);
        return;
    }

    switch (entrenamientoML.fase) {
        case ML_PREPARANDO_ESPERA:
            detenerX();
            detenerY();
            if (!objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    entrarErrorSistema(
                        ERROR_TIMEOUT_MOVIMIENTO,
                        "Z no alcanzo altura segura en entrenamiento"
                    );
                    return;
                }
                cambiarFaseEntrenamientoML(ML_ESPERANDO_PIEZA);
            }
            break;

        case ML_ESPERANDO_PIEZA: {
            detenerTodos();
            if (!camaraListaCompleta() || !objetivoCamaraValido() ||
                !encoderListoAutomaticoV2() ||
                !bandaEnMovimientoAutomaticoV2() ||
                secuenciaObjetivoRecibida == 0 ||
                secuenciaObjetivoRecibida == ackSecuenciaObjetivo) return;

            const float camX = static_cast<float>(objetivoCamaraX10) / 10.0f;
            const float camY = static_cast<float>(objetivoCamaraY10) / 10.0f;
            const float baseX = CAMERA_SWAP_XY ? camY : camX;
            const float brazoX = static_cast<float>(CAMERA_SIGN_X) * baseX +
                                 CAMERA_OFFSET_X_MM;
            const float brazoY = posicionCatchYV2();
            long pruebaX = 0;
            long pruebaY = 0;
            if (!cinematicaInversaCartesiana(
                    brazoX, brazoY, 0.0f, pruebaX, pruebaY)) {
                rechazarObjetivoFueraDeRango(
                    secuenciaObjetivoRecibida,
                    "objetivo ML fuera del espacio de trabajo"
                );
                return;
            }

            entrenamientoML.secuencia = secuenciaObjetivoRecibida;
            entrenamientoML.instanteDeteccion = millis();
            entrenamientoML.clase = claseObjetivo;
            entrenamientoML.camX = camX;
            entrenamientoML.camY = camY;
            entrenamientoML.xInicial = brazoX;
            entrenamientoML.yInicial = brazoY;
            entrenamientoML.conteoReferencia = conteoReferenciaObjetivoRecibido;
            if (!actualizarPiezaEntrenamientoML()) {
                rechazarObjetivoFueraDeRango(
                    secuenciaObjetivoRecibida,
                    "prediccion inicial ML no valida"
                );
                reiniciarEntrenamientoML();
                return;
            }
            const float umbralDescenso = calcularUmbralDisparoYV2();
            if (!pruebaSeguimientoSeleccionada() &&
                entrenamientoML.piezaYEstimada > umbralDescenso) {
                rechazarObjetivoFueraDeRango(
                    secuenciaObjetivoRecibida,
                    "pieza demasiado cerca para posicionar XYZ en ML"
                );
                reiniciarEntrenamientoML();
                return;
            }
            const long precapturaZ = posicionPrecapturaZV2();
            if (!posicionZSeguraV2(precapturaZ)) {
                rechazarObjetivoFueraDeRango(
                    secuenciaObjetivoRecibida,
                    "altura Z de captura ML fuera de rango"
                );
                reiniciarEntrenamientoML();
                return;
            }
            if (!iniciarMovimientoXY(
                    brazoX, brazoY, 0.0f, MOV_AUTOMATICO_V2)) {
                rechazarObjetivoFueraDeRango(
                    secuenciaObjetivoRecibida,
                    "no se pudo iniciar alineacion ML"
                );
                reiniciarEntrenamientoML();
                return;
            }
            moverZHasta(precapturaZ, DIV_POSICION);
            ackSecuenciaObjetivo = secuenciaObjetivoRecibida;
            codigoAckObjetivo = ACK_OBJ_ACEPTADO;
            secuenciaObjetivoEnMovimiento = secuenciaObjetivoRecibida;
            Serial.print(F("[ML] Deteccion aceptada; X="));
            Serial.print(brazoX, 2);
            Serial.print(F(" Y=0 atras piezaY="));
            Serial.print(entrenamientoML.piezaYEstimada, 2);
            Serial.print(F(" encoder_ref="));
            Serial.print(entrenamientoML.conteoReferencia);
            Serial.print(F(" | Z precaptura +"));
            Serial.println(Z_MARGEN_PRECAPTURA_PASOS);
            cambiarFaseEntrenamientoML(ML_PREPOSICIONANDO);
            registrarEventoPortentaV2("ML_ACCEPT", "pieza aceptada para catch ML");
            break;
        }

        case ML_PREPOSICIONANDO:
            if (!actualizarPiezaEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "prediccion invalida durante preposicion", false, false);
                return;
            }
            if (!pruebaSeguimientoSeleccionada() &&
                entrenamientoML.piezaYEstimada >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                cancelarEntrenamientoML(
                    "pieza rebaso el catch durante preposicion", false, false);
                return;
            }
            if (limiteZabajo) {
                cancelarEntrenamientoML(
                    "DIN04 activo antes del catch ML", false, false);
                return;
            }
            if (!movimientoPosicionadoActivo && !objetivoXEnCurso() &&
                !objetivoYEnCurso() && movX == 0 && movY == 0) {
                Serial.print(F("[ML] XY listo; Z ya se movia en paralelo piezaY="));
                Serial.print(entrenamientoML.piezaYEstimada, 2);
                Serial.println();
                cambiarFaseEntrenamientoML(ML_BAJANDO_CAPTURA);
            }
            break;

        case ML_BAJANDO_CAPTURA:
            detenerX();
            detenerY();
            if (!actualizarPiezaEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "prediccion invalida durante precaptura", false, false);
                return;
            }
            if (!pruebaSeguimientoSeleccionada() &&
                entrenamientoML.piezaYEstimada >
                posicionCatchYV2() + V2_ERROR_ESTABLE_MM) {
                cancelarEntrenamientoML(
                    "pieza rebaso el catch durante precaptura", false, false);
                return;
            }
            if (limiteZabajo) {
                cancelarEntrenamientoML(
                    "DIN04 activo en precaptura ML", false, false);
                return;
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (leerPasosZ() != posicionPrecapturaZV2()) {
                cancelarEntrenamientoML(
                    "Z no alcanzo la precaptura ML", false, false);
                return;
            }
            if (entrenamientoConResultadoSeleccionado()) {
                eventoBotonX = false; // exige una pulsacion nueva despues de quedar listo
                Serial.println(pruebaSeguimientoSeleccionada()
                    ? F("[ML TRACK] Z en precaptura; Y sigue pieza; X inicia catch")
                    : F("[ML V2] Z en precaptura; X inicia catch manual"));
            } else {
                Serial.println(F(
                    "[ML] Ajuste X/Y y rotacion con Z sobre DIN04; catch por encoder o X"
                ));
            }
            if (pruebaSeguimientoSeleccionada()) {
                entrenamientoML.inicioSeguimiento = millis();
                entrenamientoML.ultimoLogSeguimiento = 0;
                cambiarFaseEntrenamientoML(ML_SEGUIMIENTO);
            } else {
                cambiarFaseEntrenamientoML(ML_ALINEACION_MANUAL);
            }
            break;

        case ML_BAJANDO_ENTREGA: {
            detenerX();
            detenerY();
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                entrenamientoML.busquedaFinalZActiva = false;
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!entrenamientoML.busquedaFinalZActiva) {
                    entrenamientoML.busquedaFinalZActiva = true;
                    moverZHasta(
                        limiteMinimoZPasos() - V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                        V2_DIV_BUSQUEDA_FINAL_Z
                    );
                    break;
                }
                cancelarEntrenamientoML(
                    "DIN04 no aparecio durante descenso", false, false);
                return;
            }
            codigoAckObjetivo = ACK_OBJ_ABRIR_PINZA;
            cambiarFaseEntrenamientoML(ML_ABRIENDO_PINZA);
            break;
        }

        case ML_ALINEACION_MANUAL:
        case ML_SEGUIMIENTO: {
            if (!actualizarPiezaEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "prediccion invalida durante alineacion", false, false);
                return;
            }
            if (pruebaSeguimientoSeleccionada()) {
                detenerX();
                if (!seguirPiezaYEntrenamientoML()) {
                    cancelarEntrenamientoML(
                        "pieza salio del recorrido Y de seguimiento",
                        false, false);
                    return;
                }
            } else {
                int8_t x = joystickX;
                int8_t y = joystickY;
                if ((x > 0 && limiteXmas) || (x < 0 && limiteXmenos)) x = 0;
                if ((y > 0 && limiteYmas) || (y < 0 && limiteYmenos)) y = 0;
                moverXContinuo(x, DIV_MANUAL);
                moverYContinuo(y, DIV_MANUAL);
            }
            detenerZ();
            if (limiteZabajo || leerPasosZ() != posicionPrecapturaZV2()) {
                cancelarEntrenamientoML(
                    "Z salio de la precaptura ML", false, false);
                return;
            }
            entrenamientoML.catchYConfirmada = posicionYmm();
            const float anticipacionCierreS =
                tiempoDescensoFinalZV2Segundos() +
                anticipacionCierreCatchSegundos();
            entrenamientoML.umbralCierreY =
                entrenamientoML.catchYConfirmada -
                fmaxf(0.0f, velocidadBandaMmS) * anticipacionCierreS;
            if (!entrenamientoConResultadoSeleccionado() &&
                entrenamientoML.piezaYEstimada >
                entrenamientoML.catchYConfirmada + V2_ERROR_ESTABLE_MM) {
                cancelarEntrenamientoML(
                    "pieza rebaso la posicion corregida", false, false);
                return;
            }
            const bool disparoPorX = eventoBotonX;
            const bool disparoPorEncoder =
                !entrenamientoConResultadoSeleccionado() &&
                entrenamientoML.piezaYEstimada >=
                entrenamientoML.umbralCierreY;
            if (!disparoPorX && !disparoPorEncoder) break;

            eventoBotonX = false;
            if (pruebaSeguimientoSeleccionada()) {
                const float limiteY = RANGO_FISICO_Y_MM * 0.5f -
                    MARGEN_SEGURIDAD_MM - ML_SEGUIMIENTO_MARGEN_Y_MM;
                const float recorridoRestante = fmaxf(0.0f, velocidadBandaMmS) *
                    (tiempoDescensoFinalZV2Segundos() +
                     anticipacionCierreCatchSegundos() +
                     ML_SEGUIMIENTO_RESERVA_S);
                if (posicionYmm() + recorridoRestante > limiteY) {
                    registrarEventoPortentaV2(
                        "ML_TRACK_REJECT", "X sin recorrido Y para bajar y cerrar");
                    break;
                }
            }
            if (entrenamientoConResultadoSeleccionado()) {
                entrenamientoML.instanteBotonCatch = millis();
                entrenamientoML.conteoBotonCatch = conteoEncoderBanda;
                entrenamientoML.piezaYBotonCatch = entrenamientoML.piezaYEstimada;
                entrenamientoML.brazoYBoton = posicionYmm();
                entrenamientoML.errorSeguimientoBoton =
                    entrenamientoML.piezaYBotonCatch -
                    entrenamientoML.brazoYBoton;
                entrenamientoML.xCorregida = posicionXmm();
                entrenamientoML.yCorregida = posicionYmm();
                entrenamientoML.rotacionCorregida = posServoRot;
                Serial.print(pruebaSeguimientoSeleccionada()
                    ? F("V2LOG|P|mode=ML_TRACK|event=ML_BUTTON_CATCH|session=")
                    : F("V2LOG|P|mode=ML_V2|event=ML_BUTTON_CATCH|session="));
                Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
                Serial.print(F("|ms=")); Serial.print(entrenamientoML.instanteBotonCatch);
                Serial.print(F("|obj=")); Serial.print(entrenamientoML.secuencia);
                Serial.print(F("|enc=")); Serial.print(entrenamientoML.conteoBotonCatch);
                Serial.print(F("|piece_y="));
                Serial.print(entrenamientoML.piezaYBotonCatch, 3);
                if (pruebaSeguimientoSeleccionada()) {
                    Serial.print(F("|arm_y="));
                    Serial.print(entrenamientoML.brazoYBoton, 3);
                    Serial.print(F("|error_y="));
                    Serial.print(entrenamientoML.errorSeguimientoBoton, 3);
                }
                Serial.print(F("|belt_mm_s=")); Serial.println(velocidadBandaMmS, 3);
            }
            detenerX();
            if (!pruebaSeguimientoSeleccionada()) detenerY();
            entrenamientoML.catchYConfirmada = posicionYmm();
            entrenamientoML.umbralCierreY =
                entrenamientoML.catchYConfirmada -
                fmaxf(0.0f, velocidadBandaMmS) * anticipacionCierreS;
            const char *disparador = disparoPorX ? "X" : "ENCODER";
            entrenamientoML.disparadorCatch = disparador;
            entrenamientoML.busquedaFinalZActiva = false;
            entrenamientoML.rebaseManualRegistrado = false;
            moverZHasta(posicionCapturaZV2(), DIV_POSICION);
            cambiarFaseEntrenamientoML(ML_BAJANDO_CATCH);
            registrarEventoPortentaV2("ML_Z_FINAL", "descenso final para catch");
            break;
        }

        case ML_BAJANDO_CATCH: {
            detenerX();
            if (!pruebaSeguimientoSeleccionada()) detenerY();
            eventoBotonX = false;
            if (!actualizarPiezaEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "prediccion invalida durante descenso final ML", false, false);
                return;
            }
            if (pruebaSeguimientoSeleccionada() &&
                !seguirPiezaYEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "fin del recorrido Y durante descenso final", false, false);
                return;
            }
            const bool catchManualMLV2 = ensenanzaMLV2Seleccionada() &&
                entrenamientoML.disparadorCatch != nullptr &&
                strcmp(entrenamientoML.disparadorCatch, "X") == 0;
            if (!pruebaSeguimientoSeleccionada() &&
                entrenamientoML.piezaYEstimada >
                entrenamientoML.catchYConfirmada + V2_ERROR_ESTABLE_MM) {
                if (!catchManualMLV2) {
                    cancelarEntrenamientoML(
                        "pieza rebaso el catch antes de DIN04 ML", false, false);
                    return;
                }
                // ML V2 aprende del catch que el operador confirma con X.
                // Tras aceptarlo, la prediccion Y se registra como error,
                // sin impedir alcanzar DIN04 y ordenar el cierre manual.
                // Se mantienen timeout, finales, encoder, control y STOP.
                if (!entrenamientoML.rebaseManualRegistrado) {
                    entrenamientoML.rebaseManualRegistrado = true;
                    registrarEventoPortentaV2("ML_MANUAL_OVERRUN",
                        "estimacion Y rebaso catch; X confirmado, continuar a DIN04");
                }
            }
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                entrenamientoML.busquedaFinalZActiva = false;
                entrenamientoML.instanteDin04Catch = millis();
                if (pruebaSeguimientoSeleccionada()) {
                    entrenamientoML.brazoYDin04 = posicionYmm();
                    entrenamientoML.errorSeguimientoDin04 =
                        entrenamientoML.piezaYEstimada -
                        entrenamientoML.brazoYDin04;
                }
            }
            if (objetivoZEnCurso() || movZ != 0) break;
            if (!limiteZabajo) {
                if (!entrenamientoML.busquedaFinalZActiva) {
                    entrenamientoML.busquedaFinalZActiva = true;
                    moverZHasta(limiteMinimoZPasos() -
                        V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                        V2_DIV_BUSQUEDA_FINAL_Z);
                    registrarEventoPortentaV2(
                        "ML_Z_SEARCH", "busqueda DIN04 durante catch ML");
                    break;
                }
                cancelarEntrenamientoML(
                    "DIN04 no aparecio durante catch ML", false, false);
                return;
            }
            const char *disparador = entrenamientoML.disparadorCatch;
            const bool disparoPorX = strcmp(disparador, "X") == 0;
            if (pruebaSeguimientoSeleccionada()) {
                entrenamientoML.catchYConfirmada = posicionYmm();
            }
            entrenamientoML.umbralCierreY =
                entrenamientoML.catchYConfirmada -
                fmaxf(0.0f, velocidadBandaMmS) *
                anticipacionCierreCatchSegundos();
            entrenamientoML.velocidadDisparo = velocidadBandaMmS;
            entrenamientoML.piezaYDisparo = entrenamientoML.piezaYEstimada;
            entrenamientoML.diferenciaDisparoMm =
                entrenamientoML.piezaYEstimada -
                (pruebaSeguimientoSeleccionada()
                    ? entrenamientoML.catchYConfirmada
                    : entrenamientoML.umbralCierreY);
            entrenamientoML.instanteOrdenCierre = millis();
            entrenamientoML.conteoOrdenCierre = conteoEncoderBanda;
            entrenamientoML.fase = ML_CERRANDO_PINZA;
            entrenamientoML.inicioFase = entrenamientoML.instanteOrdenCierre;
            enviarOrdenCierreCatchAhora();
            Serial.print(pruebaSeguimientoSeleccionada()
                ? F("V2LOG|P|mode=ML_TRACK|event=ML_CATCH_TRIGGER|session=")
                : (ensenanzaMLV2Seleccionada()
                    ? F("V2LOG|P|mode=ML_V2|event=ML_CATCH_TRIGGER|session=")
                    : F("V2LOG|P|mode=ML|event=ML_CATCH_TRIGGER|session=")));
            Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
            Serial.print(F("|ms=")); Serial.print(entrenamientoML.instanteOrdenCierre);
            Serial.print(F("|obj=")); Serial.print(entrenamientoML.secuencia);
            Serial.print(F("|trigger=")); Serial.print(disparador);
            Serial.print(F("|catch_type="));
            Serial.print(disparoPorX ? F("MANUAL") : F("AUTOMATICO"));
            Serial.print(F("|enc=")); Serial.print(conteoEncoderBanda);
            if (entrenamientoConResultadoSeleccionado()) {
                Serial.print(F("|catch_button_ms="));
                Serial.print(entrenamientoML.instanteBotonCatch);
                Serial.print(F("|z_bottom_ms="));
                Serial.print(entrenamientoML.instanteDin04Catch);
                if (pruebaSeguimientoSeleccionada()) {
                    Serial.print(F("|arm_y="));
                    Serial.print(posicionYmm(), 3);
                    Serial.print(F("|error_y="));
                    Serial.print(entrenamientoML.piezaYDisparo - posicionYmm(), 3);
                }
            }
            Serial.print(F("|piece_y=")); Serial.print(entrenamientoML.piezaYDisparo, 3);
            Serial.print(F("|catch_y=")); Serial.print(entrenamientoML.catchYConfirmada, 3);
            Serial.print(F("|close_threshold_y="));
            Serial.print(entrenamientoML.umbralCierreY, 3);
            Serial.print(F("|servo_rot_deg=")); Serial.print(posServoRot);
            Serial.print(F("|error_disparo_mm="));
            Serial.print(entrenamientoML.diferenciaDisparoMm, 3);
            Serial.print(F("|error_disparo_ms="));
            Serial.println(entrenamientoML.velocidadDisparo > 0.0f
                ? 1000.0f * entrenamientoML.diferenciaDisparoMm /
                  entrenamientoML.velocidadDisparo : NAN, 3);
            if (!entrenamientoConResultadoSeleccionado()) {
                registrarMuestraEntrenamientoML(disparador);
            }
            Serial.print(F("[ML] ORDEN CERRAR disparador="));
            Serial.print(disparador);
            Serial.print(F(" piezaY="));
            Serial.print(entrenamientoML.piezaYEstimada, 2);
            Serial.print(F(" umbral_cierre="));
            Serial.print(entrenamientoML.umbralCierreY, 2);
            Serial.print(F(" catchY="));
            Serial.println(entrenamientoML.catchYConfirmada, 2);
            Serial.println(F("[ML] Fase -> CERRANDO PINZA"));
            break;
        }

        case ML_CERRANDO_PINZA:
            if (pruebaSeguimientoSeleccionada()) {
                detenerX();
                detenerZ();
            } else {
                detenerTodos();
            }
            eventoBotonX = false;
            if (!limiteZabajo) {
                cancelarEntrenamientoML(
                    "se perdio DIN04 cerrando la pinza", false, false);
                return;
            }
            if (!actualizarPiezaEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "prediccion invalida durante cierre", false, false);
                return;
            }
            if (pruebaSeguimientoSeleccionada() &&
                !seguirPiezaYEntrenamientoML()) {
                cancelarEntrenamientoML(
                    "fin del recorrido Y cerrando pinza", false, false);
                return;
            }
            if (millis() - entrenamientoML.inicioFase <
                V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) break;
            if (pruebaSeguimientoSeleccionada()) {
                entrenamientoML.brazoYCierre = posicionYmm();
                entrenamientoML.errorSeguimientoCierre =
                    entrenamientoML.piezaYEstimada -
                    entrenamientoML.brazoYCierre;
                entrenamientoML.yCorregida = entrenamientoML.brazoYCierre;
            }
            Serial.print(F("[ML] PINZA CERRADA piezaY="));
            Serial.print(entrenamientoML.piezaYEstimada, 3);
            Serial.print(F(" errorY="));
            Serial.println(
                entrenamientoML.piezaYEstimada -
                entrenamientoML.catchYConfirmada,
                3
            );
            Serial.print(pruebaSeguimientoSeleccionada()
                ? F("V2LOG|P|mode=ML_TRACK|event=ML_CLOSE|session=")
                : (ensenanzaMLV2Seleccionada()
                    ? F("V2LOG|P|mode=ML_V2|event=ML_CLOSE|session=")
                    : F("V2LOG|P|mode=ML|event=ML_CLOSE|session=")));
            Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
            Serial.print(F("|ms=")); Serial.print(millis());
            Serial.print(F("|obj="));
            Serial.print(entrenamientoML.secuencia);
            Serial.print(F("|catch_type="));
            Serial.print(strcmp(entrenamientoML.disparadorCatch, "X") == 0
                ? F("MANUAL") : F("AUTOMATICO"));
            Serial.print(F("|trigger="));
            Serial.print(entrenamientoML.disparadorCatch);
            Serial.print(F("|servo_rot_deg=")); Serial.print(posServoRot);
            Serial.print(F("|piece_y="));
            Serial.print(entrenamientoML.piezaYEstimada, 3);
            Serial.print(F("|close_elapsed_ms="));
            Serial.print(millis() - entrenamientoML.instanteOrdenCierre);
            Serial.print(F("|advance_mm="));
            Serial.print(entrenamientoML.piezaYEstimada -
                         entrenamientoML.piezaYDisparo, 3);
            Serial.print(F("|error_y_estimado_mm="));
            Serial.print(pruebaSeguimientoSeleccionada()
                ? entrenamientoML.errorSeguimientoCierre
                : entrenamientoML.piezaYEstimada -
                  entrenamientoML.catchYConfirmada, 3);
            if (pruebaSeguimientoSeleccionada()) {
                Serial.print(F("|arm_y="));
                Serial.print(entrenamientoML.brazoYCierre, 3);
                Serial.print(F("|error_y="));
                Serial.print(entrenamientoML.errorSeguimientoCierre, 3);
            }
            Serial.print(F("|belt_mm_s="));
            Serial.print(velocidadBandaMmS, 3);
            Serial.println(F("|error_fisico_medido=NO"));
            if (!iniciarTrasladoEntregaML()) {
                cancelarEntrenamientoML(
                    "destino de entrega derecho fuera de rango", false, false);
                return;
            }
            cambiarFaseEntrenamientoML(ML_MOVIENDO_ENTREGA);
            break;

        case ML_SUBIENDO_CON_PIEZA:
            if (!objetivoZEnCurso() && movZ == 0) {
                if (!iniciarTrasladoEntregaML()) {
                    cancelarEntrenamientoML(
                        "destino de entrega derecho fuera de rango", false, false);
                    return;
                }
                cambiarFaseEntrenamientoML(ML_MOVIENDO_ENTREGA);
            }
            break;

        case ML_MOVIENDO_ENTREGA:
            if (!movimientoPosicionadoActivo && !objetivoXEnCurso() &&
                !objetivoYEnCurso() && movX == 0 && movY == 0 &&
                !objetivoZEnCurso() && movZ == 0) {
                if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                    cancelarEntrenamientoML(
                        "Z no alcanzo altura segura durante entrega", false, false);
                    return;
                }
                entrenamientoML.busquedaFinalZActiva = false;
                moverZHasta(posicionCapturaZV2(), DIV_POSICION);
                cambiarFaseEntrenamientoML(ML_BAJANDO_ENTREGA);
            }
            break;

        case ML_ABRIENDO_PINZA:
            detenerTodos();
            if (millis() - entrenamientoML.inicioFase < ML_TIEMPO_SERVO_MS) break;
            moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
            cambiarFaseEntrenamientoML(ML_SUBIENDO_FINAL);
            break;

        case ML_SUBIENDO_FINAL:
            if (!objetivoZEnCurso() && movZ == 0) {
                codigoAckObjetivo = ACK_OBJ_COMPLETADO;
                secuenciaObjetivoEnMovimiento = 0;
                Serial.print(F("[ML] Pieza entregada; muestras="));
                Serial.println(muestrasEntrenamientoML);
                if (entrenamientoConResultadoSeleccionado()) {
                    eventoBotonX = false;
                    eventoBotonCuadrado = false;
                    cambiarFaseEntrenamientoML(ML_ESPERANDO_CONFIRMACION);
                    registrarEventoPortentaV2("ML_AWAIT_FEEDBACK",
                        "X=agarro; cuadrado=no agarro");
                } else {
                    cambiarFaseEntrenamientoML(ML_LISTO);
                }
            }
            break;

        case ML_ESPERANDO_CONFIRMACION:
            detenerTodos();
            if (!eventoBotonX && !eventoBotonCuadrado) break;
            {
                const bool exitoFisico = eventoBotonX;
                eventoBotonX = false;
                eventoBotonCuadrado = false;
                registrarMuestraEntrenamientoML(
                    "X",
                    exitoFisico ? "EXITO" : "FALLO");
                Serial.print(pruebaSeguimientoSeleccionada()
                    ? F("V2LOG|P|mode=ML_TRACK|event=ML_RESULT|session=")
                    : F("V2LOG|P|mode=ML_V2|event=ML_RESULT|session="));
                Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
                Serial.print(F("|ms=")); Serial.print(millis());
                Serial.print(F("|obj=")); Serial.print(entrenamientoML.secuencia);
                Serial.print(F("|physical_result="));
                Serial.print(exitoFisico ? F("EXITO") : F("FALLO"));
                Serial.print(F("|catch_button_ms="));
                Serial.print(entrenamientoML.instanteBotonCatch);
                Serial.print(F("|catch_command_ms="));
                Serial.println(entrenamientoML.instanteOrdenCierre);
                cambiarFaseEntrenamientoML(ML_LISTO);
            }
            break;

        case ML_LISTO:
            detenerTodos();
            if (millis() - entrenamientoML.inicioFase >= ML_TIEMPO_LISTO_MS) {
                reiniciarEntrenamientoML();
            }
            break;

        case ML_CANCELANDO:
            break;
    }
}

void procesarCalibracionCamaraEnCurso(bool iniciadaDesdeArranque) {
    const bool comandoAplicado =
        paqueteESP.ackSecuenciaComandoCamara == secuenciaComandoCamara;

    if (comandoAplicado &&
        (estadoCamara != CAMARA_LISTA || camaraOcupada() ||
         !homografiaValida() || !modeloListo())) {
        inicioCalibracionCamaraObservado = true;
    }

    // Antes del ACK todavia puede estar visible el error del intento anterior.
    // El ESP limpia ese error al aceptar el CALIBRAR nuevo; solo entonces debe
    // evaluarse como resultado de esta solicitud.
    if (comandoAplicado &&
        (estadoCamara == CAMARA_ERROR || errorCamara != CAM_ERROR_NINGUNO)) {
        entrarErrorSistema(ERROR_CAMARA, "La camara reporto un error");
        return;
    }
    if (millis() - inicioEstadoGeneral > TIMEOUT_CAMARA_MS) {
        entrarErrorSistema(ERROR_TIMEOUT_CAMARA, "Timeout calibrando camara");
        return;
    }

    if (comandoAplicado && inicioCalibracionCamaraObservado &&
        camaraListaCompleta()) {
        Serial.println(F("[CAM] Homografia valida y modelo listo"));
        terminarCalibracionSolicitada();
        return;
    }

    if (!iniciadaDesdeArranque && eventoBotonTriangulo && btConectado) {
        eventoBotonTriangulo = false;
        const EstadoGeneral regreso = modoPendiente == EST_MAIN_MENU
            ? retornoCalibracion : EST_MAIN_MENU;
        modoPendiente = EST_MAIN_MENU;
        solicitarComandoCamara(CAM_CMD_STANDBY);
        cambiarEstadoGeneral(regreso);
    }
}

void procesarCalibracionBrazoEnCurso(bool iniciadaDesdeArranque) {
    procesarCalibracionBrazo();
    if (estadoGeneral == EST_SYSTEM_ERROR) return;

    if (faseCal == CAL_COMPLETA) {
        terminarCalibracionSolicitada();
        return;
    }

    if (!iniciadaDesdeArranque && eventoBotonTriangulo && btConectado) {
        eventoBotonTriangulo = false;
        const EstadoGeneral regreso = modoPendiente == EST_MAIN_MENU
            ? retornoCalibracion : EST_MAIN_MENU;
        detenerTodos();
        calibracionXYValida = false;
        calibracionZValida = false;
        faseCal = CAL_ESPERA;
        mensajeErrorCalibracion = "";
        modoPendiente = EST_MAIN_MENU;
        cambiarEstadoGeneral(regreso);
    }
}

//-------------------------------------------------------------------------------------------------
// SEGURIDAD DE ENLACE Y SESION
//-------------------------------------------------------------------------------------------------
void volverAEsperaRS485(const char *motivo) {
    const bool movimientoInterrumpido =
        motoresEnMovimiento() || movimientoPosicionadoActivo;
    detenerTodos();
    if (movimientoInterrumpido) {
        // Politica solicitada: conservar las referencias de la sesion aunque
        // la posicion fisica pueda dejar de coincidir con el conteo de pasos.
        Serial.println(F("[RS485] Movimiento cancelado; calibracion y escalas conservadas"));
    }
    if (secuenciaObjetivoEnMovimiento != 0) {
        ackSecuenciaObjetivo = secuenciaObjetivoEnMovimiento;
        codigoAckObjetivo = ACK_OBJ_CANCELADO;
        secuenciaObjetivoEnMovimiento = 0;
    }
    if (estadoGeneral == EST_MANUAL || estadoGeneral == EST_AUTOMATICO ||
        estadoGeneral == EST_AUTOMATICO_V2 ||
        estadoGeneral == EST_ENTRENAMIENTO_ML) {
        modoPendiente = EST_MAIN_MENU;
    }
    Serial.println(motivo);
    cambiarEstadoGeneral(EST_WAIT_RS485);
}

void vigilarSeguridadComunicacion() {
    if (cambioSesionESPPendiente) {
        cambioSesionESPPendiente = false;
        if (estadoGeneral == EST_SYSTEM_ERROR) return;
        volverAEsperaRS485("[RS485] Nueva sesion ESP32; esperando enlace estable");
        return;
    }

    if (estadoGeneral == EST_BOOT_SAFE || estadoGeneral == EST_WAIT_RS485 ||
        estadoGeneral == EST_SYSTEM_ERROR) {
        return;
    }

    if (!enlaceRS485Vigente()) {
        volverAEsperaRS485("[RS485] Enlace perdido; reintentando sin reinicio fisico");
    }
}

bool resultadoChecklistRelacionadoConCamara() {
    return ultimoResultadoChecklist == CHECK_CAMARA ||
           ultimoResultadoChecklist == CHECK_HOMOGRAFIA ||
           ultimoResultadoChecklist == CHECK_MODELO;
}

bool errorActualRelacionadoConCamara() {
    return errorSistema == ERROR_TIMEOUT_CAMARA ||
           errorSistema == ERROR_CAMARA ||
           (errorSistema == ERROR_CHECKLIST &&
            resultadoChecklistRelacionadoConCamara());
}

void reiniciarSecuenciaCompleta() {
    detenerTodos();
    calibracionXYValida = false;
    calibracionZValida = false;
    pasosPorMmX = 0.0f;
    pasosPorMmY = 0.0f;
    faseCal = CAL_ESPERA;
    mensajeErrorCalibracion = "";
    errorSistema = ERROR_NINGUNO;
    mensajeErrorSistema = "";
    ultimoResultadoChecklist = CHECK_OK;
    calibracionEncoderValida = false;
    velocidadReferencia50MmS = 0.0f;
    velocidadMaximaEstimadaMmS = 0.0f;
    signoEncoderAvance = 1;
    calibracionEncoder = {};
    ackSecuenciaObjetivo = 0;
    codigoAckObjetivo = ACK_OBJ_NINGUNO;
    secuenciaObjetivoEnMovimiento = 0;
    modoPendiente = EST_MAIN_MENU;
    retornoCalibracion = EST_MAIN_MENU;
    if (enlaceRS485Vigente()) solicitarComandoCamara(CAM_CMD_REINICIAR_ERROR);
    cambiarEstadoGeneral(EST_BOOT_SAFE);
}

void reintentarDesdeEstadoError() {
    const bool errorDeCamara = errorActualRelacionadoConCamara();
    if (!enlaceRS485Vigente()) {
        cambiarEstadoGeneral(EST_WAIT_RS485);
        return;
    }
    if (errorSistema != ERROR_CAMARA && errorSistema != ERROR_TIMEOUT_CAMARA &&
        errorSistema != ERROR_CALIBRACION_BRAZO) {
        reiniciarSecuenciaCompleta();
        return;
    }
    errorSistema = ERROR_NINGUNO;
    mensajeErrorSistema = "";
    eventoBotonX = false;
    if (modoPendiente != EST_MAIN_MENU) {
        avanzarEntradaModo();
    } else {
        cambiarEstadoGeneral(errorDeCamara
            ? EST_USER_CAMERA_CALIBRATION : EST_USER_ARM_CALIBRATION);
    }
}

//-------------------------------------------------------------------------------------------------
// MAQUINA GENERAL DEL SISTEMA
//-------------------------------------------------------------------------------------------------
void procesarMaquinaGeneral() {
    switch (estadoGeneral) {
        case EST_BOOT_SAFE:
            detenerTodos();
            if (millis() - inicioEstadoGeneral >= RETARDO_ARRANQUE_ESP32_MS) {
                cambiarEstadoGeneral(EST_WAIT_RS485);
            }
            break;

        case EST_WAIT_RS485:
            detenerTodos();
            if (enlaceRS485Vigente() && protocoloValido && baseESPLista()) {
                Serial.println(F("[RS485] Primer paquete completo y valido"));
                cambiarEstadoGeneral(EST_RS485_SETTLE);
            }
            break;

        case EST_RS485_SETTLE:
            detenerTodos();
            if (millis() - inicioEstadoGeneral >= TIEMPO_ESTABILIZACION_RS485_MS) {
                // El siguiente paquete de estado informa SISTEMA_CHECKLIST y
                // confirma a la ESP32 que el enlace termino de estabilizarse.
                Serial.println(F("[RS485] Enlace estable; enviando confirmacion de checklist"));
                modoPendiente = EST_MAIN_MENU;
                cambiarEstadoGeneral(EST_FINAL_CHECKLIST);
            }
            break;

        case EST_CAMERA_CALIBRATION:
            procesarCalibracionCamaraEnCurso(true);
            break;

        case EST_ARM_CALIBRATION:
            procesarCalibracionBrazoEnCurso(true);
            break;

        case EST_WAIT_CONTROLLER:
            detenerTodos();
            if (btConectado) {
                if (modoPendiente == EST_MAIN_MENU)
                    cambiarEstadoGeneral(EST_MAIN_MENU);
                else
                    avanzarEntradaModo();
            }
            break;

        case EST_ENCODER_CALIBRATION:
            procesarCalibracionEncoder();
            break;

        case EST_FINAL_CHECKLIST:
            detenerTodos();
            ultimoResultadoChecklist = evaluarChecklistFinal();
            if (eventoBotonX && ultimoResultadoChecklist == CHECK_OK &&
                btConectado) {
                eventoBotonX = false;
                Serial.println(F("[BOOT] Checklist de conexiones confirmada con X"));
                cambiarEstadoGeneral(EST_MAIN_MENU);
            } else if (eventoBotonX) {
                eventoBotonX = false;
                Serial.print(F("[BOOT][CHECKLIST] Aun no listo: "));
                Serial.println(textoChecklist(ultimoResultadoChecklist));
            }
            break;

        case EST_MAIN_MENU:
            procesarMenuPrincipal();
            break;

        case EST_CALIBRACIONES_MENU:
            procesarMenuCalibraciones();
            break;

        case EST_CAMBIOS_CATCH:
        case EST_PRUEBA_SERVOS:
        case EST_DIAGNOSTICO:
            procesarPantallaSinMotores();
            break;

        case EST_MANUAL:
            procesarModoManual();
            break;

        case EST_AUTOMATICO:
            procesarModoAutomatico();
            break;

        case EST_AUTOMATICO_V2:
            procesarModoAutomaticoV2();
            break;

        case EST_ENTRENAMIENTO_ML:
            procesarEntrenamientoML();
            break;
        case EST_PRUEBA_ENCODER:
            procesarPruebaEncoder();
            break;

        case EST_USER_ARM_CALIBRATION:
            procesarCalibracionBrazoEnCurso(false);
            break;

        case EST_USER_CAMERA_CALIBRATION:
            procesarCalibracionCamaraEnCurso(false);
            break;

        case EST_SYSTEM_ERROR:
            detenerTodos();
            if (btConectado && eventoBotonX) {
                eventoBotonX = false;
                Serial.println(F("[ERROR] Reintento seguro solicitado"));
                reintentarDesdeEstadoError();
            }
            break;
    }
}

//-------------------------------------------------------------------------------------------------
// DIAGNOSTICO Y TERMINAL (COMANDOS CONSERVADOS)
//-------------------------------------------------------------------------------------------------
void imprimirPosicionActual() {
    Serial.println(F("----- POSICION ACTUAL -----"));
    Serial.print(F("X="));
    if (escalaConfigurada()) {
        Serial.print(posicionXmm(), 2);
        Serial.print(F(" mm | "));
    }
    Serial.print(leerPasosX());
    Serial.println(F(" pasos"));

    Serial.print(F("Y="));
    if (escalaConfigurada()) {
        Serial.print(posicionYmm(), 2);
        Serial.print(F(" mm | "));
    }
    Serial.print(leerPasosY());
    Serial.println(F(" pasos"));

    Serial.print(F("Z="));
    Serial.print(leerPasosZ());
    Serial.print(F(" pasos | rango="));
    Serial.println(rangoZPasos);
}

void imprimirRangoTrabajo() {
    Serial.println(F("===== ESPACIO DE TRABAJO ====="));
    Serial.print(F("Rango X="));
    Serial.print(rangoXPasos);
    Serial.print(F(" pasos"));
    if (escalaConfigurada()) {
        Serial.print(F(" = "));
        Serial.print(rangoXmm(), 2);
        Serial.print(F(" mm"));
    }
    Serial.println();

    Serial.print(F("Rango Y="));
    Serial.print(rangoYPasos);
    Serial.print(F(" pasos"));
    if (escalaConfigurada()) {
        Serial.print(F(" = "));
        Serial.print(rangoYmm(), 2);
        Serial.print(F(" mm"));
    }
    Serial.println();
    Serial.print(F("Rango Z="));
    Serial.print(rangoZPasos);
    Serial.println(F(" pasos"));
    Serial.print(F("X pasos permitidos: "));
    Serial.print(limiteMinimoXPasos());
    Serial.print(F(" a "));
    Serial.println(limiteMaximoXPasos());
    Serial.print(F("Y pasos permitidos: "));
    Serial.print(limiteMinimoYPasos());
    Serial.print(F(" a "));
    Serial.println(limiteMaximoYPasos());
    Serial.println(F("Origen: centro fisico X=0, Y=0, Z=0"));
}

void mostrarAyudaTerminal() {
    Serial.println(F("===== COMANDOS DISPONIBLES ====="));
    Serial.println(F("GOTO X Y Z  -> posicion absoluta en mm (solo mueve X/Y)"));
    Serial.println(F("X Y Z       -> forma abreviada de GOTO"));
    Serial.println(F("HOME        -> X=0, Y=0; Z no se mueve"));
    Serial.println(F("POS         -> posicion actual"));
    Serial.println(F("RANGO       -> espacio de trabajo"));
    Serial.println(F("ENC ESTADO  -> A/B/Z, conteo, distancia y velocidad"));
    Serial.println(F("La calibracion del encoder se realiza en el arranque con la banda al 50 %"));
    Serial.println(F("ENC VUELTA  -> diagnostico de indice y 2048 cuentas/vuelta"));
    Serial.println(F("ENC CERO    -> reinicia conteo e indice manualmente"));
    Serial.println(F("STOP        -> parada inmediata"));
    Serial.println(F("REINTENTAR  -> reinicio seguro desde estado de error"));
    Serial.println(F("AYUDA       -> esta ayuda"));
    Serial.println(F("Escala automatica: X=446 mm, Y=336 mm"));
}

bool movimientoTerminalPermitido() {
    return estadoGeneral == EST_MAIN_MENU && enlaceRS485Vigente() &&
           calibracionXYValida && escalaConfigurada() &&
           (propietarioMovimiento == MOV_SIN_PROPIETARIO ||
            propietarioMovimiento == MOV_TERMINAL);
}

void ejecutarStopTerminal() {
    const bool estabaCalibrando =
        estadoGeneral == EST_ARM_CALIBRATION ||
        estadoGeneral == EST_USER_ARM_CALIBRATION;
    if (movimientoPosicionadoActivo) {
        cancelarMovimientoPosicionado("orden STOP", false);
    } else {
        detenerTodos();
    }

    if (estabaCalibrando) {
        calibracionXYValida = false;
        calibracionZValida = false;
        mensajeErrorCalibracion = "STOP por terminal";
        faseCal = CAL_ERROR;
        entrarErrorSistema(ERROR_CANCELADO, "Calibracion cancelada por STOP");
    } else if (estadoGeneral == EST_MANUAL ||
               estadoGeneral == EST_AUTOMATICO ||
               estadoGeneral == EST_AUTOMATICO_V2 ||
               estadoGeneral == EST_ENTRENAMIENTO_ML ||
               estadoGeneral == EST_PRUEBA_ENCODER) {
        cambiarEstadoGeneral(btConectado ? EST_MAIN_MENU : EST_WAIT_CONTROLLER);
    }
    Serial.println(F("[SEGURIDAD] STOP ejecutado"));
}

void procesarComandoTerminal(String comando) {
    comando.trim();
    if (comando.length() == 0) return;

    String mayuscula = comando;
    mayuscula.toUpperCase();
    if (mayuscula == "AYUDA") {
        mostrarAyudaTerminal();
        return;
    }
    if (mayuscula == "POS") {
        imprimirPosicionActual();
        return;
    }
    if (mayuscula == "RANGO") {
        imprimirRangoTrabajo();
        return;
    }
    if (mayuscula == "ENC ESTADO" || mayuscula == "ENCODER") {
        Serial.print(F("[ENC] conteo="));
        Serial.print(conteoEncoderBanda);
        Serial.print(F(" relativo="));
        Serial.print(diferenciaConteosConWrap(
            conteoEncoderBanda, conteoCeroUsuario
        ));
        Serial.print(F(" AB=0b"));
        Serial.print(ultimoEstadoABEncoder, BIN);
        Serial.print(F(" indice="));
        Serial.print(revolucionesIndiceEncoder);
        Serial.print(F(" flags=0x"));
        Serial.print(estadoEncoderBanda, HEX);
        Serial.print(F(" cuentas/s="));
        Serial.print(frecuenciaEncoderCuentasS, 1);
        Serial.print(F(" velocidad="));
        Serial.print(velocidadBandaMmS, 3);
        Serial.print(F(" mm/s distancia="));
        Serial.print(static_cast<float>(diferenciaConteosConWrap(
            conteoEncoderBanda, conteoCeroUsuario
        )) * escalaEncoderMmPorCuenta * signoEncoderAvance, 3);
        Serial.print(F(" mm escala="));
        Serial.print(escalaEncoderMmPorCuenta, 9);
        Serial.print(F(" mm/cuenta signo="));
        Serial.print(signoEncoderAvance);
        Serial.print(F(" V50="));
        Serial.print(velocidadReferencia50MmS, 2);
        Serial.print(F(" mm/s Vmax_est="));
        Serial.print(velocidadMaximaEstimadaMmS, 2);
        Serial.print(F(" mm/s cal="));
        Serial.println(calibracionEncoderValida ? F("OK") : F("PENDIENTE"));
        return;
    }
    if (mayuscula == "ENC VUELTA") {
        Serial.print(F("[ENC][VUELTA] indices="));
        Serial.print(revolucionesIndiceEncoder);
        Serial.print(F(" intervalos_completos="));
        Serial.print(intervalosIndiceCompletos);
        if (intervalosIndiceCompletos != 0) {
            const float porVuelta =
                static_cast<float>(cuentasIndiceAcumuladas) /
                static_cast<float>(intervalosIndiceCompletos);
            Serial.print(F(" ultima="));
            Serial.print(ultimaMedicionCuentasPorVuelta, 2);
            Serial.print(F(" cuentas/vuelta="));
            Serial.print(porVuelta, 2);
            Serial.print(F(" esperado_X2="));
            Serial.print(ENCODER_CUENTAS_X2_POR_VUELTA);
            Serial.print(F(" error="));
            Serial.print(100.0f * fabsf(
                porVuelta - ENCODER_CUENTAS_X2_POR_VUELTA
            ) / ENCODER_CUENTAS_X2_POR_VUELTA, 2);
            Serial.print('%');
        }
        Serial.println();
        return;
    }
    if (mayuscula == "ENC CERO") {
        if (estadoGeneral == EST_AUTOMATICO_V2 ||
            estadoGeneral == EST_ENCODER_CALIBRATION || motoresEnMovimiento() ||
            movimientoPosicionadoActivo) {
            Serial.println(F("[ENC][ERROR] STOP antes de reiniciar el conteo"));
            return;
        }
        encoders[0].reset();
        conteoEncoderBanda = 0;
        ultimoConteoEncoderVelocidad = 0;
        conteoCeroUsuario = 0;
        revolucionesIndiceEncoder = 0;
        ultimoIndiceReportado = 0;
        referenciaIndiceDiagnosticoValida = false;
        indiceReferenciaDiagnostico = 0;
        conteoReferenciaIndiceDiagnostico = 0;
        intervalosIndiceCompletos = 0;
        cuentasIndiceAcumuladas = 0;
        ultimaMedicionCuentasPorVuelta = 0.0f;
        ultimoPulsoEncoder = 0;
        velocidadBandaMmS = 0.0f;
        frecuenciaEncoderCuentasS = 0.0f;
        Serial.println(F("[ENC] Conteo e indice reiniciados por orden explicita"));
        return;
    }
    if (mayuscula == "STOP") {
        ejecutarStopTerminal();
        return;
    }
    if (mayuscula == "REINTENTAR") {
        if (estadoGeneral != EST_SYSTEM_ERROR) {
            Serial.println(F("[ERROR] REINTENTAR solo aplica en estado de error"));
            return;
        }
        Serial.println(F("[ERROR] Reintento seguro solicitado por terminal"));
        reintentarDesdeEstadoError();
        return;
    }
    if (mayuscula == "HOME") {
        if (!movimientoTerminalPermitido()) {
            Serial.println(F("[ERROR] HOME requiere menu, enlace y X/Y calibrados"));
            return;
        }
        iniciarMovimientoXY(0.0f, 0.0f, 0.0f, MOV_TERMINAL);
        return;
    }

    if (!movimientoTerminalPermitido()) {
        Serial.println(F("[ERROR] GOTO requiere menu, enlace y X/Y calibrados"));
        return;
    }

    String datos = comando;
    if (mayuscula.startsWith("GOTO ")) datos = comando.substring(5);
    datos.replace(',', ' ');
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    char sobrante = '\0';
    const int leidos = sscanf(datos.c_str(), "%f %f %f %c", &x, &y, &z, &sobrante);
    if (leidos == 3 && isfinite(x) && isfinite(y) && isfinite(z)) {
        iniciarMovimientoXY(x, y, z, MOV_TERMINAL);
        return;
    }

    Serial.print(F("[ERROR] Comando no reconocido: "));
    Serial.println(comando);
    Serial.println(F("Usa: GOTO X Y Z"));
}

void leerTerminal() {
    while (Serial.available()) {
        const char caracter = static_cast<char>(Serial.read());
        if (caracter == '\n' || caracter == '\r') {
            if (lineaTerminal.length() > 0) {
                procesarComandoTerminal(lineaTerminal);
                lineaTerminal = "";
            }
        } else if (lineaTerminal.length() < 80) {
            lineaTerminal += caracter;
        } else {
            lineaTerminal = "";
            Serial.println(F("[ERROR] Comando demasiado largo"));
        }
    }
}

//-------------------------------------------------------------------------------------------------
// SETUP Y LOOP
//-------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // I2C interno de Machine Control: el expansor de entradas/finales usa Wire.
    // Migrar el enlace externo a RS485 no elimina este bus de la Portenta.
    // Sin begin(), el core mbed accede a un master nulo al leer digital_inputs.
    Wire.begin();
    Wire.setClock(100000);

    iniciarRS485Maestro();

    digital_inputs.init();
    encoders[0].reset();
    digital_outputs.set(pP_X, LOW);
    digital_outputs.set(pP_Y, LOW);
    digital_outputs.set(pP_Z, LOW);
    digital_outputs.set(pD_X, LOW);
    digital_outputs.set(pD_Y, LOW);
    digital_outputs.set(pD_Z, LOW);
    detenerTodos();
    leerFinalesCarrera();

    motorTicker.attach(&generarPulsoMotor, velocidadMotores);

    tiempoEncendidoSistema = millis();
    inicioEstadoGeneral = tiempoEncendidoSistema;
    tAnteriorRS485 = tiempoEncendidoSistema;
    tAnteriorEstadoESP = tiempoEncendidoSistema;
    comunicacionRS485Habilitada = false;
    comandoCamaraActual = CAM_CMD_NINGUNO;
    secuenciaComandoCamara = 0;

    Serial.println(F("[BOOT] Portenta coordinadora iniciada en estado seguro"));
    Serial.print(F("[BOOT] RS485 integrado, "));
    Serial.print(EnlaceRS485::BAUD);
    Serial.println(F(" 8N1; retencion inicial 3000 ms"));
    Serial.println(F("[BOOT] RS485: intercambio nominal 10 ms, una solicitud pendiente; OLED conserva I2C"));
    Serial.println(F("[BOOT] Finales Z: DIN04=abajo, DIN05=arriba"));
    Serial.println(F("[BOOT] Encoder 0: OUTA=A0 OUTB=B0 OUTC=Z0, decodificacion X2"));
    Serial.print(F("[BOOT] Escala encoder compilada="));
    Serial.print(escalaEncoderMmPorCuenta, 9);
    Serial.println(F(" mm/cuenta"));
    Serial.print(F("[BOOT] Distancia efectiva camara-catch V2="));
    Serial.print(CAMARA_A_HOME_Y_MM, 1);
    Serial.print(F(" mm (ajuste="));
    Serial.print(V2_AJUSTE_DISTANCIA_CATCH_MM, 1);
    Serial.println(F(" mm)"));
    Serial.println(F("[BOOT] Monitor serial: 115200 baudios"));
    mostrarAyudaTerminal();
}

void loop() {
    const unsigned long entradaLoop = millis();
    if (ultimaEntradaLoopMs != 0) {
        const unsigned long pausa = entradaLoop - ultimaEntradaLoopMs;
        if (pausa > maximaPausaLoopMs) maximaPausaLoopMs = pausa;
    }
    ultimaEntradaLoopMs = entradaLoop;

    actualizarEncoderBanda();
    leerTerminal();
    leerFinalesCarrera();

    const unsigned long ahora = millis();
    if (!comunicacionRS485Habilitada &&
        ahora - tiempoEncendidoSistema >= RETARDO_ARRANQUE_ESP32_MS) {
        comunicacionRS485Habilitada = true;
        tAnteriorRS485 = ahora;
        tAnteriorEstadoESP = ahora;
        Serial.println(F("[RS485] Inicio de sondeo versionado a ESP32"));
    }

    if (comunicacionRS485Habilitada) {
        // RX/timeout sin esperar. Solo una solicitud pendiente permite que
        // los dos extremos alternen DE sin colisiones en el par compartido.
        leerPaqueteESP32();
        if (!clienteRS485.pendiente &&
            (envioRS485Urgente || ahora - tAnteriorEstadoESP >= PERIODO_ESTADO_ESP_MS)) {
            if (enviarPaquetePortenta()) tAnteriorEstadoESP = ahora;
        }
    }

    if (!finalesCoherentes() && estadoGeneral != EST_SYSTEM_ERROR) {
        entrarErrorSistema(ERROR_FINALES_INCOHERENTES,
                           "Ambos finales de un eje estan activos");
    }

    actualizarMovimientoPosicionado();
    vigilarSeguridadComunicacion();
    procesarMaquinaGeneral();
    registrarTelemetriaPortentaV2();
    aplicarBloqueoPorFinales();
    mantenerBusRS485MaestroRecuperable();

    // Los clicks son eventos de una sola iteracion, nunca quedan latched al cambiar de estado.
    eventoBotonX = false;
    eventoBotonCirculo = false;
    eventoBotonTriangulo = false;
    eventoBotonCuadrado = false;

    if (ahora - ultimoReporteRS485 >= 1000UL) {
        ultimoReporteRS485 = ahora;
        Serial.print(F("[RS485] rxOK="));
        Serial.print(lecturasRS485Ok);
        Serial.print(F(" rxError="));
        Serial.print(lecturasRS485Error);
        Serial.print(F("(len="));
        Serial.print(receptorRS485.longitudIncorrecta);
        Serial.print(F(" crc="));
        Serial.print(receptorRS485.crcIncorrecto);
        Serial.print(F(" sem="));
        Serial.print(erroresRS485Semantica);
        Serial.print(')');
        Serial.print(F(" txOK="));
        Serial.print(enviosRS485Ok);
        Serial.print(F(" txError="));
        Serial.print(enviosRS485Error);
        Serial.print(F(" ultimoTx="));
        Serial.print(ultimoCodigoErrorEnvioRS485);
        Serial.print(F(" reinicios="));
        Serial.print(reiniciosBusRS485Maestro);
        Serial.print(F(" pausaLoopMax="));
        Serial.print(maximaPausaLoopMs);
        Serial.print(F("ms encVel="));
        Serial.print(velocidadBandaMmS, 1);
        Serial.print(F(" encCps="));
        Serial.print(frecuenciaEncoderCuentasS, 0);
        Serial.print(F(" estado="));
        Serial.print(estadoGeneralWire());
        Serial.print(F(" timeout=")); Serial.print(timeoutsRS485);
        Serial.print(F(" ajena=")); Serial.print(respuestasRS485Ajenas);
        Serial.print(F(" fragmentos=")); Serial.print(receptorRS485.fragmentos);
        Serial.print(F(" rtt=")); Serial.print(ultimaLatenciaRS485Ms);
        Serial.print(F(" rttMax=")); Serial.print(maximaLatenciaRS485Ms);
        Serial.print(F(" rxBytes=")); Serial.print(receptorRS485.bytes);
        Serial.print(F(" ceros=")); Serial.println(receptorRS485.delimitadores);
    }
}

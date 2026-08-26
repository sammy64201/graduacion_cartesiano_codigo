#include <Arduino.h>
#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
// Coordinador general para Arduino Portenta H7 + Machine Control.
// Mantiene el nucleo mecanico del sketch funcional MAster/MAster.ino e integra
// arranque autonomo, camara, checklist, menu de cinco opciones y modos automaticos.

#include <Arduino_MachineControl.h>
#include <Wire.h>
#include "mbed.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ProtocoloI2C.h"

using namespace machinecontrol;
using namespace ProtocoloI2C;

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

// El ticker conmuta STEP cada 100 us. Un paso se cuenta en el flanco ascendente.
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
constexpr float CAMERA_OFFSET_X_MM = 0.0f;
constexpr float CAMERA_OFFSET_Y_MM = 0.0f;
constexpr float CAMARA_A_HOME_Y_BASE_MM = 510.0f;
// Ajuste iterativo del timing. La prueba con 0 mm quedo adelantada y la de
// 420 mm quedo atrasada; 210 mm es el punto medio para la siguiente pasada.
// Se conserva separado para afinarlo sin perder la medida geometrica base.
constexpr float V2_AJUSTE_DISTANCIA_CATCH_MM = 210.0f;
constexpr float CAMARA_A_HOME_Y_MM =
    CAMARA_A_HOME_Y_BASE_MM + V2_AJUSTE_DISTANCIA_CATCH_MM;

// E6B2-CWZ6C de 1024 P/R, lectura X2 y rueda de 65 mm acoplada 1:1.
// La geometria fija la distancia por cuenta; la tercera calibracion de arranque
// mide automaticamente el sentido y la velocidad real con la banda al 50 %.
constexpr float ENCODER_DIAMETRO_RUEDA_MM = 15.0f;
constexpr float ENCODER_RELACION_ENCODER_RUEDA = 1.0f;
constexpr int ENCODER_CUENTAS_X2_POR_VUELTA = 2048;
constexpr float ENCODER_PI = 3.14159265358979323846f;
constexpr float ENCODER_MM_POR_CUENTA =
    ENCODER_PI * ENCODER_DIAMETRO_RUEDA_MM /
    (static_cast<float>(ENCODER_CUENTAS_X2_POR_VUELTA) *
     ENCODER_RELACION_ENCODER_RUEDA);

constexpr unsigned long ENC_TIEMPO_ESTABILIZACION_MS = 2000UL;
constexpr unsigned long ENC_TIEMPO_MEDICION_MS = 5000UL;
constexpr unsigned long ENC_PERIODO_VENTANA_MS = 200UL;
constexpr unsigned long ENC_TIMEOUT_PARO_MS = 30000UL;
constexpr int32_t ENC_CUENTAS_MINIMAS_MEDICION = 100;
constexpr uint8_t ENC_VENTANAS_MINIMAS = 20;
constexpr float ENC_VARIACION_MAXIMA = 0.10f;

static_assert(CAMERA_SIGN_X == 1 || CAMERA_SIGN_X == -1, "CAMERA_SIGN_X debe ser +/-1");
static_assert(CAMERA_SIGN_Y == 1 || CAMERA_SIGN_Y == -1, "CAMERA_SIGN_Y debe ser +/-1");
static_assert(ENCODER_RELACION_ENCODER_RUEDA > 0.0f,
              "La relacion encoder/rueda debe ser positiva");

//-------------------------------------------------------------------------------------------------
// PERIODOS Y TIMEOUTS DEL COORDINADOR
//-------------------------------------------------------------------------------------------------
const unsigned long RETARDO_ARRANQUE_ESP32_MS = 3000UL;
// Cada paquete ocupa aproximadamente 3 ms a 100 kHz. La combinacion anterior
// 5 ms/10 ms consumia cerca del 90 % del bus y las interrupciones X2 del
// encoder agotaban el margen restante. 10 ms/20 ms reduce la carga a ~45 %.
const unsigned long PERIODO_CONTROL_MS = 10UL;
const unsigned long PERIODO_ESTADO_ESP_MS = 20UL;
const unsigned long PERIODO_ENCODER_MS = 10UL;
const unsigned long TIMEOUT_MOVIMIENTO_ENCODER_MS = 500UL;
const unsigned long TIMEOUT_I2C_MS = 150UL;
// Un callback I2C todavia podria responder aunque el loop de la ESP32 se
// hubiera congelado. La secuencia de su snapshot debe avanzar periodicamente.
const unsigned long TIMEOUT_SECUENCIA_ESP_MS = 150UL;
const unsigned long TIEMPO_ESTABILIZACION_I2C_MS = 5000UL;
const unsigned long TIMEOUT_CAMARA_MS = 240000UL;
const uint8_t MAX_PAQUETES_INVALIDOS_CONSECUTIVOS = 10;

//-------------------------------------------------------------------------------------------------
// ESTADOS LOCALES
//-------------------------------------------------------------------------------------------------
enum EstadoGeneral : uint8_t {
    EST_BOOT_SAFE = 0,
    EST_WAIT_I2C,
    EST_I2C_SETTLE,
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
    EST_ENCODER_CALIBRATION
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
    // Valor 6 reservado para leer registros historicos sin renumerar fases.
    V2_ESPERANDO_CONFIRMACION = 6,
    V2_COMPLETADO = 7,
    V2_CANCELANDO = 8
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
    ERROR_TIMEOUT_I2C = 1,
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
    CHECK_I2C,
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
bool checklistInicialCompletado = false;

uint8_t opcionMenu = 0;
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
bool botonXAnterior = false;
bool botonCirculoAnterior = false;
bool botonTrianguloAnterior = false;
bool eventoBotonX = false;
bool eventoBotonCirculo = false;
bool eventoBotonTriangulo = false;
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
bool volverChecklistTrasCalibracionCamara = false;

uint16_t ackSecuenciaObjetivo = 0;
uint8_t codigoAckObjetivo = 0;
uint16_t secuenciaObjetivoEnMovimiento = 0;

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
    float sumaCuentasS;
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
constexpr long V2_Z_SEGURO_PASOS = 0;
constexpr float V2_ERROR_ESTABLE_MM = 5.0f;
constexpr unsigned long V2_TIEMPO_COMPLETADO_MS = 500UL;
constexpr unsigned long V2_TIMEOUT_FASE_MS = 30000UL;
constexpr long V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS = 2L * PASOS_SEPARACION;
constexpr uint16_t V2_DIV_BUSQUEDA_FINAL_Z = DIV_CAL_LENTA;
constexpr unsigned long V2_PERIODO_LOG_TELEMETRIA_MS = 50UL;

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
    float ultimoErrorY;
    float ultimoErrorX;
};

ContextoAutomaticoV2 automaticoV2 = {};
uint32_t intentosV2 = 0;
uint32_t exitosV2 = 0;
unsigned long ultimoLogTelemetriaV2 = 0;

unsigned long tiempoEncendidoSistema = 0;
unsigned long tAnteriorI2C = 0;
unsigned long tAnteriorEstadoESP = 0;
bool comunicacionI2CHabilitada = false;
uint32_t lecturasI2COk = 0;
uint32_t lecturasI2CError = 0;
uint32_t erroresI2CLongitud = 0;
uint32_t erroresI2CCRC = 0;
uint32_t erroresI2CSemantica = 0;
uint32_t enviosI2COk = 0;
uint32_t enviosI2CError = 0;
uint8_t ultimoCodigoErrorEnvioI2C = 0;
uint32_t reiniciosBusI2CMaestro = 0;
unsigned long ultimoReinicioBusI2CMs = 0;
unsigned long ultimaEntradaLoopMs = 0;
unsigned long maximaPausaLoopMs = 0;
unsigned long ultimoReporteI2C = 0;

String lineaTerminal = "";

// Declaraciones de funciones que cruzan secciones.
void entrarErrorSistema(CodigoErrorLocal codigo, const char *mensaje);
void cambiarEstadoGeneral(EstadoGeneral nuevoEstado);
void cancelarMovimientoPosicionado(const char *motivo, bool posicionPerdida);
void reiniciarAutomaticoV2();
void iniciarCancelacionAutomaticoV2(
    const char *motivo,
    bool esperarControl,
    bool salirDelModo = false
);
long posicionCapturaZV2();
void iniciarCalibracionEncoder();
void procesarCalibracionEncoder();

//-------------------------------------------------------------------------------------------------
// ACCESO ATOMICO A CONTADORES Y OBJETIVOS
//-------------------------------------------------------------------------------------------------
#line 460 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long leerPasosX();
#line 467 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long leerPasosY();
#line 474 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long leerPasosZ();
#line 481 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void fijarPasosX(long valor);
#line 487 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void fijarPasosY(long valor);
#line 493 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void fijarPasosZ(long valor);
#line 499 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool objetivoXEnCurso();
#line 506 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool objetivoYEnCurso();
#line 513 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool objetivoZEnCurso();
#line 520 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool motoresEnMovimiento();
#line 531 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void leerFinalesCarrera();
#line 542 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool finalesCoherentes();
#line 548 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool algunFinalActivo();
#line 556 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void generarPulsoMotor();
#line 633 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void detenerX();
#line 643 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void detenerY();
#line 653 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void detenerZ();
#line 663 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void detenerTodos();
#line 669 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void moverXContinuo(int8_t direccion, uint16_t divisor);
#line 691 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void moverYContinuo(int8_t direccion, uint16_t divisor);
#line 713 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void moverZContinuo(int8_t direccion, uint16_t divisor);
#line 736 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void moverXHasta(long destino, uint16_t divisor);
#line 757 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void moverYHasta(long destino, uint16_t divisor);
#line 778 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void moverZHasta(long destino, uint16_t divisor);
#line 800 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void aplicarBloqueoPorFinales();
#line 812 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool escalaConfigurada();
#line 816 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
float posicionXmm();
#line 820 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
float posicionYmm();
#line 824 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
float rangoXmm();
#line 828 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
float rangoYmm();
#line 832 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long limiteMinimoXPasos();
#line 833 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long limiteMaximoXPasos();
#line 834 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long limiteMinimoYPasos();
#line 835 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long limiteMaximoYPasos();
#line 836 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long limiteMinimoZPasos();
#line 837 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
long limiteMaximoZPasos();
#line 839 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool posicionZSeguraV2(long destino);
#line 847 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool calcularEscalaAutomatica();
#line 864 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool cinematicaInversaCartesiana(float xMm, float yMm, float zMm, long &xPasosDestino, long &yPasosDestino);
#line 895 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool transformarCamaraABrazo(float camXmm, float camYmm, float &brazoXmm, float &brazoYmm);
#line 905 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool brazoEnHome();
#line 914 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool iniciarMovimientoXY(float xMm, float yMm, float zMm, PropietarioMovimiento propietario);
#line 971 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void actualizarMovimientoPosicionado();
#line 1005 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
const char * nombreFaseCalibracion();
#line 1039 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void cambiarFaseCalibracion(FaseCalibracion nuevaFase);
#line 1046 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
uint8_t codigoErrorCalibracion();
#line 1057 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void detenerPorErrorCalibracion(const char *texto);
#line 1068 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void iniciarCalibracionBrazo();
#line 1085 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarCalibracionBrazo();
#line 1320 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
uint8_t estadoGeneralWire();
#line 1343 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
uint8_t errorSistemaWire();
#line 1371 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool enlaceI2CVigente();
#line 1378 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool baseESPLista();
#line 1382 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool camaraConectada();
#line 1386 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool homografiaValida();
#line 1390 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool modeloListo();
#line 1394 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool objetivoCamaraValido();
#line 1398 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool camaraOcupada();
#line 1402 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool camaraListaCompleta();
#line 1407 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void actualizarEncoderBanda();
#line 1497 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool encoderListoAutomaticoV2();
#line 1506 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void cambiarFaseCalibracionEncoder(FaseCalibracionEncoder nueva);
#line 1516 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void fallarCalibracionEncoder(const char *mensaje);
#line 1694 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool bandaEnMovimientoAutomaticoV2();
#line 1700 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool paqueteSemanticamenteValido(const PaqueteESPAPortenta &p);
#line 1716 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void registrarPaqueteValido(const PaqueteESPAPortenta &nuevo);
#line 1783 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool leerPaqueteESP32();
#line 1823 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
uint8_t construirFlagsSistema();
#line 1854 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
uint8_t construirFlagsLimites();
#line 1866 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void construirPaquetePortenta(PaquetePortentaAESP &p);
#line 1899 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool enviarPaquetePortenta();
#line 1917 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void mantenerBusI2CMaestroRecuperable();
#line 1947 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void registrarEventoPortentaV2(const char *evento, const char *mensaje);
#line 2004 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void registrarAckPortentaV2(const char *mensaje);
#line 2008 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void registrarTelemetriaPortentaV2();
#line 2016 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void solicitarComandoCamara(uint8_t comando);
#line 2133 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
ResultadoChecklist evaluarChecklistFinal();
#line 2152 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
const char * textoChecklist(ResultadoChecklist resultado);
#line 2175 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarMenuPrincipal();
#line 2248 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarModoManual();
#line 2289 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarModoAutomatico();
#line 2356 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
const char * nombreFaseAutomaticoV2(FaseAutomaticoV2 fase);
#line 2371 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void cambiarFaseAutomaticoV2(FaseAutomaticoV2 nueva);
#line 2389 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void finalizarCancelacionAutomaticoV2();
#line 2410 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void iniciarCancelacionAutomaticoV2( const char *motivo, bool esperarControl, bool salirDelModo );
#line 2437 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
float posicionCatchYV2();
#line 2447 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool actualizarObjetivoMovilV2();
#line 2488 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
float tiempoDescensoZV2Segundos();
#line 2496 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
float calcularUmbralDisparoYV2();
#line 2504 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void aceptarObjetivoAutomaticoV2();
#line 2565 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void imprimirContadoresV2();
#line 2572 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void completarResultadoV2(const char *resultado);
#line 2585 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarModoAutomaticoV2();
#line 2812 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarCalibracionCamaraEnCurso(bool iniciadaDesdeArranque);
#line 2859 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarCalibracionBrazoEnCurso(bool iniciadaDesdeArranque);
#line 2886 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void vigilarSeguridadComunicacion();
#line 2923 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool resultadoChecklistRelacionadoConCamara();
#line 2929 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool errorActualRelacionadoConCamara();
#line 2936 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void reiniciarSecuenciaCompleta();
#line 2961 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void reintentarDesdeEstadoError();
#line 3006 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarMaquinaGeneral();
#line 3108 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void imprimirPosicionActual();
#line 3132 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void imprimirRangoTrabajo();
#line 3167 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void mostrarAyudaTerminal();
#line 3184 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
bool movimientoTerminalPermitido();
#line 3191 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void ejecutarStopTerminal();
#line 3215 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void procesarComandoTerminal(String comando);
#line 3360 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void leerTerminal();
#line 3380 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void setup();
#line 3424 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
void loop();
#line 460 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\pruebas de automatico v2\\PORTENTA\\PORTENTA.ino"
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
    Serial.println(F(" mm recibido pero no se movera"));

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
                           ? F("[AUTO] Objetivo alcanzado")
                           : F("[XYZ] Objetivo alcanzado"));
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
        case EST_WAIT_I2C: return SISTEMA_ESPERANDO_I2C;
        case EST_I2C_SETTLE: return SISTEMA_ESPERA_5S;
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
        case EST_SYSTEM_ERROR: return SISTEMA_ERROR;
        default: return SISTEMA_ERROR;
    }
}

uint8_t errorSistemaWire() {
    switch (errorSistema) {
        case ERROR_NINGUNO: return SISTEMA_ERROR_NINGUNO;
        case ERROR_TIMEOUT_I2C:
        case ERROR_PROTOCOLO_INVALIDO:
        case ERROR_REINICIO_ESP32:
            return SISTEMA_ERROR_I2C;
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

bool enlaceI2CVigente() {
    return existePaqueteValido &&
           secuenciaPaqueteESPConocida &&
           millis() - ultimoPaqueteValidoMs <= TIMEOUT_I2C_MS &&
           millis() - ultimoCambioSecuenciaESPMs <= TIMEOUT_SECUENCIA_ESP_MS;
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
        velocidadBandaMmS = ultimaMuestraEncoder == ahora && ultimoPulsoEncoder == 0
            ? instantanea
            : 0.8f * velocidadBandaMmS + 0.2f * instantanea;
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
    Serial.println(F("[ENC][CAL] ETAPA 3/3: coloque manualmente la banda al 50 %"));
    Serial.println(F("[ENC][CAL] Cuando la banda avance estable camara->brazo, presione X"));
    Serial.print(F("[ENC][CAL] Geometria: rueda="));
    Serial.print(ENCODER_DIAMETRO_RUEDA_MM, 1);
    Serial.print(F(" mm, 1:1, escala="));
    Serial.print(escalaEncoderMmPorCuenta, 9);
    Serial.println(F(" mm/cuenta"));
}

void procesarCalibracionEncoder() {
    detenerTodos();
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
                calibracionEncoder.sumaCuentasS = 0.0f;
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
                    calibracionEncoder.sumaCuentasS += cuentasS;
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

                const float mediaCuentasS = fabsf(static_cast<float>(deltaTotal)) *
                    1000.0f / static_cast<float>(dtTotal);
                const float limiteInferior = mediaCuentasS *
                    (1.0f - ENC_VARIACION_MAXIMA);
                const float limiteSuperior = mediaCuentasS *
                    (1.0f + ENC_VARIACION_MAXIMA);
                if (!isfinite(mediaCuentasS) || mediaCuentasS <= 0.0f ||
                    calibracionEncoder.minimoCuentasS < limiteInferior ||
                    calibracionEncoder.maximoCuentasS > limiteSuperior) {
                    fallarCalibracionEncoder("Velocidad inestable: variacion mayor al 10 %");
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
                Serial.print(F("[ENC][CAL] ventanas min/media/max="));
                Serial.print(calibracionEncoder.minimoCuentasS, 1);
                Serial.print('/');
                Serial.print(mediaCuentasS, 1);
                Serial.print('/');
                Serial.print(calibracionEncoder.maximoCuentasS, 1);
                Serial.println(F(" cuentas/s"));
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
                cambiarEstadoGeneral(EST_FINAL_CHECKLIST);
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
            BOTON_X | BOTON_TRIANGULO | BOTON_CIRCULO
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
        Serial.print(F("[I2C] Sesion ESP32 inicial="));
        Serial.println(sesionArranqueESP);
    } else if (nuevo.sesionArranque != sesionArranqueESP) {
        Serial.print(F("[I2C] Reinicio ESP32: sesion "));
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
            cancelarMovimientoPosicionado("reinicio de ESP32", true);
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
    if (botonX && !botonXAnterior) eventoBotonX = true;
    if (botonCirculo && !botonCirculoAnterior) eventoBotonCirculo = true;
    if (botonTriangulo && !botonTrianguloAnterior) eventoBotonTriangulo = true;
    botonXAnterior = botonX;
    botonCirculoAnterior = botonCirculo;
    botonTrianguloAnterior = botonTriangulo;

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
    const int recibidos = Wire.requestFrom(
        static_cast<uint8_t>(DIRECCION_ESP32),
        static_cast<uint8_t>(sizeof(PaqueteESPAPortenta))
    );

    if (recibidos != static_cast<int>(sizeof(PaqueteESPAPortenta)) ||
        Wire.available() < static_cast<int>(sizeof(PaqueteESPAPortenta))) {
        while (Wire.available()) Wire.read();
        lecturasI2CError++;
        erroresI2CLongitud++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }

    PaqueteESPAPortenta recibido = {};
    uint8_t *destino = reinterpret_cast<uint8_t *>(&recibido);
    for (size_t i = 0; i < sizeof(recibido); ++i) {
        destino[i] = static_cast<uint8_t>(Wire.read());
    }
    while (Wire.available()) Wire.read();

    if (!validarPaquete(recibido)) {
        lecturasI2CError++;
        erroresI2CCRC++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }
    if (!paqueteSemanticamenteValido(recibido)) {
        lecturasI2CError++;
        erroresI2CSemantica++;
        if (fallosPaqueteConsecutivos < 255) fallosPaqueteConsecutivos++;
        return false;
    }

    registrarPaqueteValido(recibido);
    lecturasI2COk++;
    return true;
}

uint8_t construirFlagsSistema() {
    uint8_t flags = 0;
    if (calibracionXYValida) flags |= SIS_FLAG_XY_CALIBRADO;
    if (calibracionZValida) flags |= SIS_FLAG_Z_CALIBRADO;
    if (brazoEnHome()) flags |= SIS_FLAG_EN_HOME;
    if (estadoGeneral == EST_AUTOMATICO ||
        estadoGeneral == EST_AUTOMATICO_V2) {
        flags |= SIS_FLAG_AUTO_ACTIVO;
    }
    if (motoresEnMovimiento() || movimientoPosicionadoActivo ||
        (faseCal > CAL_ESPERA && faseCal < CAL_COMPLETA) ||
        (estadoGeneral == EST_AUTOMATICO_V2 &&
         automaticoV2.fase != V2_ESPERANDO_PIEZA &&
         automaticoV2.fase != V2_COMPLETADO)) {
        flags |= SIS_FLAG_BRAZO_OCUPADO;
    }
    if (estadoGeneral == EST_SYSTEM_ERROR) flags |= SIS_FLAG_ERROR_CRITICO;
    if (checklistInicialCompletado && estadoGeneral != EST_SYSTEM_ERROR) {
        flags |= SIS_FLAG_CHECKLIST_OK;
    }
    if (enlaceI2CVigente() && estadoGeneral != EST_SYSTEM_ERROR &&
        (estadoGeneral == EST_MANUAL || estadoGeneral == EST_AUTOMATICO ||
         estadoGeneral == EST_AUTOMATICO_V2 ||
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
    p.opcionMenu = opcionMenu;
    p.faseCalibracionBrazo = estadoGeneral == EST_AUTOMATICO_V2
        ? static_cast<uint8_t>(automaticoV2.fase)
        : (estadoGeneral == EST_ENCODER_CALIBRATION
            ? static_cast<uint8_t>(calibracionEncoder.fase)
            : static_cast<uint8_t>(faseCal));
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
    p.nmPorCuentaEncoder = escalaEncoderMmPorCuenta > 0.0f
        ? static_cast<uint32_t>(lroundf(escalaEncoderMmPorCuenta * 1000000.0f))
        : 0U;
    p.secuenciaEncoder = secuenciaEncoder;
    p.estadoEncoder = estadoEncoderBanda;
    p.signoEncoder = signoEncoderAvance;
    prepararPaquete(p);
}

bool enviarPaquetePortenta() {
    PaquetePortentaAESP salida = {};
    construirPaquetePortenta(salida);
    Wire.beginTransmission(static_cast<uint8_t>(DIRECCION_ESP32));
    const size_t escritos = Wire.write(
        reinterpret_cast<const uint8_t *>(&salida), sizeof(salida)
    );
    const uint8_t error = Wire.endTransmission(true);
    if (escritos == sizeof(salida) && error == 0) {
        enviosI2COk++;
        ultimoCodigoErrorEnvioI2C = 0;
        return true;
    }
    enviosI2CError++;
    ultimoCodigoErrorEnvioI2C = error;
    return false;
}

void mantenerBusI2CMaestroRecuperable() {
    if (!comunicacionI2CHabilitada || enlaceI2CVigente()) return;

    // Nunca se reinicia el periferico durante movimiento. Ante una perdida en
    // automatico, la maquina de seguridad detiene primero todos los ejes y
    // entra en error; la recuperacion queda disponible en la siguiente vuelta.
    const bool estadoSeguro =
        estadoGeneral == EST_BOOT_SAFE ||
        estadoGeneral == EST_WAIT_I2C ||
        estadoGeneral == EST_I2C_SETTLE ||
        estadoGeneral == EST_SYSTEM_ERROR;
    if (!estadoSeguro || motoresEnMovimiento() || movimientoPosicionadoActivo)
        return;

    const unsigned long ahora = millis();
    if (ahora - ultimoReinicioBusI2CMs < 1000UL) return;
    ultimoReinicioBusI2CMs = ahora;

    Wire.end();
    Wire.begin();
    Wire.setClock(100000);
    fallosPaqueteConsecutivos = 0;
    ++reiniciosBusI2CMaestro;
    Serial.print(F("[I2C][RECUPERACION] Maestro reiniciado, intento="));
    Serial.println(reiniciosBusI2CMaestro);
}

//-------------------------------------------------------------------------------------------------
// CAMBIOS DE ESTADO, COMANDOS DE CAMARA Y ERRORES
//-------------------------------------------------------------------------------------------------
void registrarEventoPortentaV2(const char *evento, const char *mensaje) {
    Serial.print(F("V2LOG|P|ms="));
    Serial.print(millis());
    Serial.print(F("|session="));
    Serial.print(sesionESPConocida ? sesionArranqueESP : 0);
    Serial.print(F("|event="));
    Serial.print(evento);
    Serial.print(F("|obj="));
    Serial.print(automaticoV2.secuencia != 0
        ? automaticoV2.secuencia : secuenciaObjetivoRecibida);
    Serial.print(F("|encseq="));
    Serial.print(secuenciaEncoder);
    Serial.print(F("|enc="));
    Serial.print(conteoEncoderBanda);
    Serial.print(F("|ref_enc="));
    Serial.print(automaticoV2.secuencia != 0
        ? automaticoV2.conteoReferencia : conteoReferenciaObjetivoRecibido);
    Serial.print(F("|phase="));
    Serial.print(static_cast<uint8_t>(automaticoV2.fase));
    Serial.print(F("|class="));
    Serial.print(automaticoV2.clase != 0
        ? automaticoV2.clase : claseObjetivo);
    Serial.print(F("|arm_x="));
    Serial.print(posicionXmm(), 3);
    Serial.print(F("|arm_y="));
    Serial.print(posicionYmm(), 3);
    Serial.print(F("|z_steps="));
    Serial.print(leerPasosZ());
    Serial.print(F("|target_x="));
    Serial.print(automaticoV2.objetivoBrazoX, 3);
    Serial.print(F("|target_y="));
    Serial.print(automaticoV2.objetivoBrazoY, 3);
    Serial.print(F("|error_x="));
    Serial.print(automaticoV2.ultimoErrorX, 3);
    Serial.print(F("|error_y="));
    Serial.print(automaticoV2.ultimoErrorY, 3);
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
    if (estadoAnterior == EST_AUTOMATICO_V2 &&
        nuevoEstado != EST_AUTOMATICO_V2) {
        registrarEventoPortentaV2("SESSION_END", "salida de Automatico V2");
    }
    estadoGeneral = nuevoEstado;
    inicioEstadoGeneral = millis();
    entradaMenuYAnterior = joystickY;

    Serial.print(F("[BOOT] Estado general -> "));
    Serial.println(estadoGeneralWire());

    switch (nuevoEstado) {
        case EST_CAMERA_CALIBRATION:
            volverChecklistTrasCalibracionCamara = false;
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
        case EST_WAIT_CONTROLLER:
        case EST_FINAL_CHECKLIST:
        case EST_BOOT_SAFE:
        case EST_WAIT_I2C:
        case EST_I2C_SETTLE:
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
            break;
        case EST_AUTOMATICO_V2:
            detenerTodos();
            movimientoPosicionadoActivo = false;
            propietarioMovimiento = MOV_SIN_PROPIETARIO;
            reiniciarAutomaticoV2();
            break;
    }
    if (estadoAnterior != EST_AUTOMATICO_V2 &&
        nuevoEstado == EST_AUTOMATICO_V2) {
        ultimoLogTelemetriaV2 = 0;
        registrarEventoPortentaV2("SESSION_START", "entrada a Automatico V2");
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
    if (!enlaceI2CVigente()) return CHECK_I2C;
    if (!protocoloValido || !baseESPLista()) return CHECK_PROTOCOLO;
    if (!camaraConectada() || estadoCamara != CAMARA_LISTA || errorCamara != 0)
        return CHECK_CAMARA;
    if (!homografiaValida()) return CHECK_HOMOGRAFIA;
    if (!modeloListo()) return CHECK_MODELO;
    if (!calibracionXYValida || !escalaConfigurada()) return CHECK_XY;
    if (!calibracionZValida) return CHECK_Z;
    if (!brazoEnHome()) return CHECK_HOME;
    if (motoresEnMovimiento() || movimientoPosicionadoActivo) return CHECK_MOTORES;
    if (!btConectado) return CHECK_BT;
    if (!calibracionEncoderValida || velocidadReferencia50MmS <= 0.0f)
        return CHECK_ENCODER;
    if (faseCal == CAL_ERROR || mensajeErrorCalibracion[0] != '\0') return CHECK_CAL_ERROR;
    if (!finalesCoherentes() || algunFinalActivo()) return CHECK_FINALES;
    return CHECK_OK;
}

const char *textoChecklist(ResultadoChecklist resultado) {
    switch (resultado) {
        case CHECK_OK: return "Checklist completo";
        case CHECK_I2C: return "Comunicacion I2C inactiva";
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
void procesarMenuPrincipal() {
    if (!btConectado) {
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (movimientoPosicionadoActivo || motoresEnMovimiento()) return;

    if (joystickY != 0 && entradaMenuYAnterior == 0) {
        int nueva = static_cast<int>(opcionMenu) - joystickY;
        if (nueva < 0) nueva = 4;
        if (nueva > 4) nueva = 0;
        opcionMenu = static_cast<uint8_t>(nueva);
        Serial.print(F("[MENU] Opcion="));
        Serial.println(opcionMenu);
    }
    entradaMenuYAnterior = joystickY;

    if (!eventoBotonX) return;
    eventoBotonX = false;

    switch (opcionMenu) {
        case MENU_MODO_MANUAL:
            cambiarEstadoGeneral(EST_MANUAL);
            break;
        case MENU_MODO_AUTOMATICO:
            if (!calibracionXYValida || !calibracionZValida ||
                !camaraListaCompleta()) {
                Serial.println(F("[AUTO] No disponible: camara o brazo sin calibrar"));
                return;
            }
            cambiarEstadoGeneral(EST_AUTOMATICO);
            break;
        case MENU_CALIBRACION_BRAZO:
            cambiarEstadoGeneral(EST_USER_ARM_CALIBRATION);
            break;
        case MENU_CALIBRACION_CAMARA:
            volverChecklistTrasCalibracionCamara = false;
            cambiarEstadoGeneral(EST_USER_CAMERA_CALIBRATION);
            break;
        case MENU_MODO_AUTOMATICO_V2:
            if (!calibracionXYValida || !calibracionZValida ||
                !camaraListaCompleta()) {
                Serial.println(F("[AUTO V2] No disponible: camara o brazo sin calibrar"));
                return;
            }
            if (!encoderListoAutomaticoV2()) {
                Serial.print(F("[AUTO V2] No disponible: encoder invalido. escala="));
                Serial.print(escalaEncoderMmPorCuenta, 9);
                Serial.print(F(" flags=0x"));
                Serial.println(estadoEncoderBanda, HEX);
                Serial.println(F("[AUTO V2] Reinicie y complete la etapa 3/3 con la banda al 50 %"));
                return;
            }
            if (leerPasosZ() != V2_Z_SEGURO_PASOS) {
                Serial.println(F("[AUTO V2] No disponible: Z no esta en posicion segura"));
                return;
            }
            if (!posicionZSeguraV2(V2_Z_SEGURO_PASOS) ||
                !posicionZSeguraV2(posicionCapturaZV2())) {
                Serial.println(F("[AUTO V2] No disponible: configure Z dentro del rango seguro"));
                return;
            }
            cambiarEstadoGeneral(EST_AUTOMATICO_V2);
            if (!bandaEnMovimientoAutomaticoV2()) {
                Serial.println(F("[AUTO V2] Modo activo; esperando que la banda avance camara->brazo"));
            }
            break;
        default:
            opcionMenu = MENU_MODO_MANUAL;
            break;
    }
}

void procesarModoManual() {
    if (!btConectado) {
        detenerTodos();
        cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        return;
    }
    if (eventoBotonTriangulo) {
        eventoBotonTriangulo = false;
        detenerTodos();
        cambiarEstadoGeneral(EST_MAIN_MENU);
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

void procesarModoAutomatico() {
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
        case V2_ESPERANDO_LLEGADA: return "ESPERANDO LLEGADA";
        case V2_DISPARANDO_CATCH: return "DISPARANDO CATCH";
        case V2_BAJANDO_Z: return "BAJANDO Z";
        case V2_SUBIENDO_Z: return "SUBIENDO Z";
        case V2_ESPERANDO_CONFIRMACION: return "FASE 6 RESERVADA";
        case V2_COMPLETADO: return "COMPLETADO";
        case V2_CANCELANDO: return "CANCELANDO";
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
    automaticoV2.fase = V2_ESPERANDO_PIEZA;
    automaticoV2.inicioFase = millis();
    automaticoV2.objetivoBrazoYAnterior = posicionYmm();
    automaticoV2.ultimoConteoProcesado = conteoEncoderBanda;
    automaticoV2.instanteObjetivoAnterior = millis();
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

bool actualizarObjetivoMovilV2() {
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

float calcularUmbralDisparoYV2() {
    // El brazo permanece quieto en Y. La velocidad solo convierte el tiempo
    // mecanico de descenso en una distancia de anticipacion; no gobierna Y.
    const float anticipacion = fmaxf(0.0f, velocidadBandaMmS) *
        tiempoDescensoZV2Segundos() + V2_AJUSTE_ANTICIPACION_Z_MM;
    return posicionCatchYV2() - anticipacion;
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
    // Esta instantanea queda inmutable durante el catch. Los paquetes I2C
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
    secuenciaObjetivoEnMovimiento = secuenciaObjetivoRecibida;
    ackSecuenciaObjetivo = secuenciaObjetivoRecibida;
    codigoAckObjetivo = ACK_OBJ_ACEPTADO;

    Serial.print(F("[AUTO V2] Objetivo aceptado seq="));
    Serial.print(automaticoV2.secuencia);
    Serial.print(F(" encoder="));
    Serial.print(automaticoV2.conteoReferencia);
    Serial.print(F(" yCatch="));
    Serial.println(yCatch, 2);

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
    registrarEventoPortentaV2("ACCEPT", "objetivo aceptado");
    registrarAckPortentaV2("objetivo aceptado");
    cambiarFaseAutomaticoV2(V2_PREPOSICIONANDO);
}

void imprimirContadoresV2() {
    Serial.print(F("[AUTO V2] intentos="));
    Serial.print(intentosV2);
    Serial.print(F(" catch_automaticos="));
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
        automaticoV2.fase == V2_BAJANDO_Z && movZ < 0 && limiteZabajo;
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
        automaticoV2.fase >= V2_PREPOSICIONANDO &&
        automaticoV2.fase <= V2_DISPARANDO_CATCH;
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
        millis() - automaticoV2.inicioFase > V2_TIMEOUT_FASE_MS) {
        iniciarCancelacionAutomaticoV2("timeout de fase", false);
        return;
    }

    switch (automaticoV2.fase) {
        case V2_ESPERANDO_PIEZA:
            detenerTodos();
            // La igualdad solo bloquea la repeticion del mismo mensaje I2C.
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
                cambiarFaseAutomaticoV2(V2_ESPERANDO_LLEGADA);
            }
            break;

        case V2_ESPERANDO_LLEGADA:
            detenerY();
            if (!actualizarObjetivoMovilV2()) {
                iniciarCancelacionAutomaticoV2(
                    "prediccion de pieza no valida esperando llegada", false);
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
            if (automaticoV2.objetivoBrazoY >= automaticoV2.umbralDisparoY) {
                Serial.print(F("[AUTO V2] Disparo Z piezaY="));
                Serial.print(automaticoV2.objetivoBrazoY, 2);
                Serial.print(F(" umbral="));
                Serial.print(automaticoV2.umbralDisparoY, 2);
                Serial.print(F(" vel="));
                Serial.print(velocidadBandaMmS, 2);
                Serial.print(F(" tZ="));
                Serial.println(tiempoDescensoZV2Segundos(), 3);
                registrarEventoPortentaV2(
                    "TRIGGER", "pieza entro al umbral de descenso Z"
                );
                cambiarFaseAutomaticoV2(V2_DISPARANDO_CATCH);
            }
            break;

        case V2_DISPARANDO_CATCH: {
            detenerY();
            const long capturaZ = posicionCapturaZV2();
            if (!posicionZSeguraV2(capturaZ)) {
                iniciarCancelacionAutomaticoV2(
                    "altura Z de captura fuera de rango", false);
                return;
            }
            automaticoV2.busquedaFinalZActiva = false;
            moverZHasta(capturaZ, DIV_POSICION);
            cambiarFaseAutomaticoV2(V2_BAJANDO_Z);
            break;
        }

        case V2_BAJANDO_Z:
            detenerY();
            if (limiteZabajo) {
                detenerZ();
                fijarPasosZ(limiteMinimoZPasos());
                automaticoV2.busquedaFinalZActiva = false;
            }
            if (!objetivoZEnCurso() && movZ == 0) {
                if (!limiteZabajo) {
                    if (!automaticoV2.busquedaFinalZActiva) {
                        automaticoV2.busquedaFinalZActiva = true;
                        Serial.println(F(
                            "[AUTO V2] Buscando DIN04 debajo del conteo calibrado"
                        ));
                        registrarEventoPortentaV2(
                            "Z_SEARCH", "busqueda lenta del final inferior DIN04"
                        );
                        moverZHasta(
                            limiteMinimoZPasos() -
                                V2_BUSQUEDA_FINAL_Z_EXTRA_PASOS,
                            V2_DIV_BUSQUEDA_FINAL_Z
                        );
                        return;
                    }
                    iniciarCancelacionAutomaticoV2(
                        "DIN04 no aparecio dentro del margen de busqueda Z", false);
                    return;
                }
                ++intentosV2;
                automaticoV2.ultimoErrorX =
                    automaticoV2.objetivoBrazoX - posicionXmm();
                Serial.print(F("[AUTO V2] CAPTURA_VIRTUAL X="));
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
                Serial.println(F(" final_z_abajo=1"));
                registrarEventoPortentaV2(
                    "CAPTURE", "captura virtual en final Z abajo");
                // Desde el catch la pieza deja de pertenecer a la banda. Se
                // congela Y y la retirada es vertical hasta la altura segura.
                detenerY();
                moverZHasta(V2_Z_SEGURO_PASOS, DIV_POSICION);
                cambiarFaseAutomaticoV2(V2_SUBIENDO_Z);
            }
            break;

        case V2_SUBIENDO_Z:
            detenerY();
            if (!objetivoZEnCurso() && movZ == 0) {
                ++exitosV2;
                completarResultadoV2("CATCH_AUTOMATICO");
            }
            break;

        case V2_ESPERANDO_CONFIRMACION:
            // Recuperacion defensiva: el flujo nuevo nunca entra a fase 6.
            ++exitosV2;
            completarResultadoV2("CATCH_AUTOMATICO");
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
        if (iniciadaDesdeArranque) {
            cambiarEstadoGeneral(EST_ARM_CALIBRATION);
        } else if (volverChecklistTrasCalibracionCamara) {
            volverChecklistTrasCalibracionCamara = false;
            cambiarEstadoGeneral(btConectado
                                     ? EST_FINAL_CHECKLIST
                                     : EST_WAIT_CONTROLLER);
        } else {
            cambiarEstadoGeneral(btConectado ? EST_MAIN_MENU : EST_WAIT_CONTROLLER);
        }
        return;
    }

    if (!iniciadaDesdeArranque && eventoBotonTriangulo && btConectado) {
        eventoBotonTriangulo = false;
        volverChecklistTrasCalibracionCamara = false;
        solicitarComandoCamara(CAM_CMD_STANDBY);
        cambiarEstadoGeneral(EST_MAIN_MENU);
    }
}

void procesarCalibracionBrazoEnCurso(bool iniciadaDesdeArranque) {
    procesarCalibracionBrazo();
    if (estadoGeneral == EST_SYSTEM_ERROR) return;

    if (faseCal == CAL_COMPLETA) {
        if (iniciadaDesdeArranque) {
            cambiarEstadoGeneral(EST_WAIT_CONTROLLER);
        } else {
            cambiarEstadoGeneral(btConectado ? EST_MAIN_MENU : EST_WAIT_CONTROLLER);
        }
        return;
    }

    if (!iniciadaDesdeArranque && eventoBotonTriangulo && btConectado) {
        eventoBotonTriangulo = false;
        detenerTodos();
        calibracionXYValida = false;
        calibracionZValida = false;
        faseCal = CAL_ESPERA;
        mensajeErrorCalibracion = "";
        cambiarEstadoGeneral(EST_MAIN_MENU);
    }
}

//-------------------------------------------------------------------------------------------------
// SEGURIDAD DE ENLACE Y SESION
//-------------------------------------------------------------------------------------------------
void vigilarSeguridadComunicacion() {
    if (cambioSesionESPPendiente) {
        cambioSesionESPPendiente = false;
        if (estadoGeneral == EST_WAIT_I2C || estadoGeneral == EST_I2C_SETTLE ||
            estadoGeneral == EST_BOOT_SAFE) {
            cambiarEstadoGeneral(EST_WAIT_I2C);
        } else if (estadoGeneral != EST_SYSTEM_ERROR) {
            entrarErrorSistema(ERROR_REINICIO_ESP32,
                               "ESP32 se reinicio; objetivos anteriores descartados");
        }
        return;
    }

    if (fallosPaqueteConsecutivos >= MAX_PAQUETES_INVALIDOS_CONSECUTIVOS &&
        estadoGeneral != EST_BOOT_SAFE && estadoGeneral != EST_WAIT_I2C &&
        estadoGeneral != EST_SYSTEM_ERROR) {
        entrarErrorSistema(ERROR_PROTOCOLO_INVALIDO,
                           "Paquetes I2C invalidos persistentes");
        return;
    }

    if (estadoGeneral == EST_BOOT_SAFE || estadoGeneral == EST_WAIT_I2C ||
        estadoGeneral == EST_SYSTEM_ERROR) {
        return;
    }

    if (!enlaceI2CVigente()) {
        detenerTodos();
        if (estadoGeneral == EST_I2C_SETTLE) {
            Serial.println(F("[I2C] Enlace perdido durante espera de 5 s"));
            cambiarEstadoGeneral(EST_WAIT_I2C);
        } else {
            entrarErrorSistema(ERROR_TIMEOUT_I2C, "Timeout de comunicacion I2C");
        }
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
    checklistInicialCompletado = false;
    calibracionEncoderValida = false;
    velocidadReferencia50MmS = 0.0f;
    velocidadMaximaEstimadaMmS = 0.0f;
    signoEncoderAvance = 1;
    calibracionEncoder = {};
    ackSecuenciaObjetivo = 0;
    codigoAckObjetivo = ACK_OBJ_NINGUNO;
    secuenciaObjetivoEnMovimiento = 0;
    volverChecklistTrasCalibracionCamara = false;
    if (enlaceI2CVigente()) solicitarComandoCamara(CAM_CMD_REINICIAR_ERROR);
    cambiarEstadoGeneral(EST_BOOT_SAFE);
}

void reintentarDesdeEstadoError() {
    if (!errorActualRelacionadoConCamara()) {
        Serial.println(F("[ERROR] Reintento completo por error no relacionado con camara"));
        reiniciarSecuenciaCompleta();
        return;
    }

    if (!enlaceI2CVigente()) {
        Serial.println(F("[ERROR] Sin enlace I2C; no es posible reintentar solo la camara"));
        reiniciarSecuenciaCompleta();
        return;
    }

    const bool brazoCalibrado = calibracionXYValida &&
                                calibracionZValida &&
                                escalaConfigurada();
    const bool regresarAlChecklist =
        brazoCalibrado &&
        (volverChecklistTrasCalibracionCamara ||
         errorSistema == ERROR_CHECKLIST);

    detenerTodos();
    movimientoPosicionadoActivo = false;
    propietarioMovimiento = MOV_SIN_PROPIETARIO;
    secuenciaObjetivoEnMovimiento = 0;
    errorSistema = ERROR_NINGUNO;
    mensajeErrorSistema = "";
    ultimoResultadoChecklist = CHECK_OK;
    eventoBotonX = false;
    eventoBotonCirculo = false;
    eventoBotonTriangulo = false;
    volverChecklistTrasCalibracionCamara = regresarAlChecklist;

    if (brazoCalibrado) {
        Serial.println(F("[CAM] Reintento selectivo; calibracion del brazo conservada"));
        cambiarEstadoGeneral(EST_USER_CAMERA_CALIBRATION);
    } else {
        Serial.println(F("[CAM] Reintento de camara; el brazo aun requiere calibracion"));
        cambiarEstadoGeneral(EST_CAMERA_CALIBRATION);
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
                cambiarEstadoGeneral(EST_WAIT_I2C);
            }
            break;

        case EST_WAIT_I2C:
            detenerTodos();
            if (enlaceI2CVigente() && protocoloValido && baseESPLista()) {
                Serial.println(F("[I2C] Primer paquete completo y valido"));
                cambiarEstadoGeneral(EST_I2C_SETTLE);
            }
            break;

        case EST_I2C_SETTLE:
            detenerTodos();
            if (millis() - inicioEstadoGeneral >= TIEMPO_ESTABILIZACION_I2C_MS) {
                cambiarEstadoGeneral(EST_CAMERA_CALIBRATION);
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
                if (checklistInicialCompletado) {
                    cambiarEstadoGeneral(EST_MAIN_MENU);
                } else if (!calibracionEncoderValida) {
                    cambiarEstadoGeneral(EST_ENCODER_CALIBRATION);
                } else {
                    cambiarEstadoGeneral(EST_FINAL_CHECKLIST);
                }
            }
            break;

        case EST_ENCODER_CALIBRATION:
            procesarCalibracionEncoder();
            break;

        case EST_FINAL_CHECKLIST:
            detenerTodos();
            ultimoResultadoChecklist = evaluarChecklistFinal();
            if (ultimoResultadoChecklist == CHECK_OK) {
                checklistInicialCompletado = true;
                Serial.println(F("[BOOT] Checklist final OK"));
                cambiarEstadoGeneral(EST_MAIN_MENU);
            } else {
                const char *texto = textoChecklist(ultimoResultadoChecklist);
                Serial.print(F("[BOOT][CHECKLIST] Falla: "));
                Serial.println(texto);
                entrarErrorSistema(ERROR_CHECKLIST, texto);
            }
            break;

        case EST_MAIN_MENU:
            procesarMenuPrincipal();
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
    return estadoGeneral == EST_MAIN_MENU && enlaceI2CVigente() &&
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
               estadoGeneral == EST_AUTOMATICO_V2) {
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

    Wire.begin();
    Wire.setClock(100000);

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
    tAnteriorI2C = tiempoEncendidoSistema;
    tAnteriorEstadoESP = tiempoEncendidoSistema;
    comunicacionI2CHabilitada = false;
    comandoCamaraActual = CAM_CMD_NINGUNO;
    secuenciaComandoCamara = 0;

    Serial.println(F("[BOOT] Portenta coordinadora iniciada en estado seguro"));
    Serial.println(F("[BOOT] I2C maestro 0x40 a 100 kHz; retencion inicial 3000 ms"));
    Serial.println(F("[BOOT] I2C: control 10 ms, telemetria encoder 20 ms (~45 % de bus)"));
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
    if (!comunicacionI2CHabilitada &&
        ahora - tiempoEncendidoSistema >= RETARDO_ARRANQUE_ESP32_MS) {
        comunicacionI2CHabilitada = true;
        tAnteriorI2C = ahora;
        tAnteriorEstadoESP = ahora;
        Serial.println(F("[I2C] Inicio de sondeo versionado a ESP32"));
    }

    // if (comunicacionI2CHabilitada) {
    //     if (ahora - tAnteriorI2C >= PERIODO_CONTROL_MS) {
    //         tAnteriorI2C = ahora;
    //         leerPaqueteESP32();
    //         huboLectura = true;
    //     }
    //     // Evita solicitar y escribir al esclavo en la misma vuelta del loop.
    //     if (!huboLectura && ahora - tAnteriorEstadoESP >= PERIODO_ESTADO_ESP_MS) {
    //         tAnteriorEstadoESP = ahora;
    //         enviarPaquetePortenta();
    //     }
    // }
    if (comunicacionI2CHabilitada) {
    const bool tocaEnviar =
        ahora - tAnteriorEstadoESP >= PERIODO_ESTADO_ESP_MS;

    const bool tocaLeer =
        ahora - tAnteriorI2C >= PERIODO_CONTROL_MS;

    // El envio de telemetria tiene prioridad cada 20 ms. La lectura de control
    // pendiente se atiende en la siguiente vuelta del loop, cada 10 ms.
    if (tocaEnviar) {
        tAnteriorEstadoESP = ahora;
        enviarPaquetePortenta();
    }
    else if (tocaLeer) {
        tAnteriorI2C = ahora;
        leerPaqueteESP32();
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
    mantenerBusI2CMaestroRecuperable();

    // Los clicks son eventos de una sola iteracion, nunca quedan latched al cambiar de estado.
    eventoBotonX = false;
    eventoBotonCirculo = false;
    eventoBotonTriangulo = false;

    if (ahora - ultimoReporteI2C >= 1000UL) {
        ultimoReporteI2C = ahora;
        Serial.print(F("[I2C] rxOK="));
        Serial.print(lecturasI2COk);
        Serial.print(F(" rxError="));
        Serial.print(lecturasI2CError);
        Serial.print(F("(len="));
        Serial.print(erroresI2CLongitud);
        Serial.print(F(" crc="));
        Serial.print(erroresI2CCRC);
        Serial.print(F(" sem="));
        Serial.print(erroresI2CSemantica);
        Serial.print(')');
        Serial.print(F(" txOK="));
        Serial.print(enviosI2COk);
        Serial.print(F(" txError="));
        Serial.print(enviosI2CError);
        Serial.print(F(" ultimoTx="));
        Serial.print(ultimoCodigoErrorEnvioI2C);
        Serial.print(F(" reinicios="));
        Serial.print(reiniciosBusI2CMaestro);
        Serial.print(F(" pausaLoopMax="));
        Serial.print(maximaPausaLoopMs);
        Serial.print(F("ms encVel="));
        Serial.print(velocidadBandaMmS, 1);
        Serial.print(F(" encCps="));
        Serial.print(frecuenciaEncoderCuentasS, 0);
        Serial.print(F(" estado="));
        Serial.println(estadoGeneralWire());
    }
}


#include <Wire.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

//=================================================================================================
// BUS I2C 0: ESP32 COMO ESCLAVO DE LA PORTENTA
//=================================================================================================
#define I2C_PORTENTA_SDA        27
#define I2C_PORTENTA_SCL        14
#define DIRECCION_ESP32         0x40

//=================================================================================================
// BUS I2C 1: ESP32 COMO MAESTRO DE LA PANTALLA OLED
//=================================================================================================
#define OLED_SDA                21
#define OLED_SCL                22
#define OLED_DIRECCION          0x3C

TwoWire I2C_Pantalla = TwoWire(1);
Adafruit_SH1106G pantalla(128, 64, &I2C_Pantalla, -1);

//=================================================================================================
// PINES DE SERVOS
//=================================================================================================
#define PIN_SERVO_ROTACION      25
#define PIN_SERVO_PINZA         26

Servo servo25;
Servo servo26;

//=================================================================================================
// PAQUETES DE COMUNICACION
//=================================================================================================
// Este paquete se envia cuando la Portenta hace Wire.requestFrom().
struct __attribute__((packed)) PaqueteControl {
    uint8_t ejeX;
    uint8_t ejeY;
    uint8_t ejeZ;
    uint8_t btConectado;
    uint8_t botonX;
    uint8_t botonTri;
    uint8_t anguloServo1;
    uint8_t anguloServo2;
};

// Este paquete se recibe cuando la Portenta hace Wire.beginTransmission().
// Los cuatro valores cambian de significado segun la pantalla:
// - CALIBRACION: pasosX, pasosY, rangoXPasos, rangoYPasos.
// - POSICION XYZ: posX x10, posY x10, objetivoX x10, objetivoY x10.
struct __attribute__((packed)) PaquetePantalla {
    uint8_t magic;
    uint8_t version;
    uint8_t estado;
    uint8_t opcionMenu;
    uint8_t faseCal;
    uint8_t flags;
    uint8_t limites;
    int8_t movX;
    int8_t movY;
    int8_t movZ;
    uint8_t servoRot;
    uint8_t servoPin;
    uint8_t errorCal;
    int32_t valor1;
    int32_t valor2;
    int32_t valor3;
    int32_t valor4;
    uint8_t checksum;
};

static_assert(sizeof(PaqueteControl) == 8, "PaqueteControl debe medir 8 bytes");
static_assert(sizeof(PaquetePantalla) <= 32, "PaquetePantalla excede el buffer I2C");

const uint8_t MAGIC_PANTALLA = 0xA5;
const uint8_t VERSION_PANTALLA = 1;

// Estados recibidos desde la Portenta.
enum EstadoSistemaRemoto : uint8_t {
    MENU_PRINCIPAL_R = 0,
    MODO_MANUAL_R = 1,
    MODO_HOMING_R = 2,
    MODO_CARTESIANO_R = 3
};

// Fases recibidas desde la Portenta.
enum FaseCalibracionRemota : uint8_t {
    CAL_ESPERA_R = 0,
    CAL_X_MIN_1_R,
    CAL_X_MIN_LIBERAR_R,
    CAL_X_MIN_SEPARAR_R,
    CAL_X_MIN_2_R,
    CAL_X_MAX_1_R,
    CAL_X_MAX_LIBERAR_R,
    CAL_X_MAX_SEPARAR_R,
    CAL_X_MAX_2_R,
    CAL_Y_MIN_1_R,
    CAL_Y_MIN_LIBERAR_R,
    CAL_Y_MIN_SEPARAR_R,
    CAL_Y_MIN_2_R,
    CAL_Y_MAX_1_R,
    CAL_Y_MAX_LIBERAR_R,
    CAL_Y_MAX_SEPARAR_R,
    CAL_Y_MAX_2_R,
    CAL_HOME_R,
    CAL_COMPLETA_R,
    CAL_ERROR_R
};

portMUX_TYPE estadoMux = portMUX_INITIALIZER_UNLOCKED;
PaquetePantalla estadoPantalla = {};
volatile bool estadoPantallaNuevo = false;
bool estadoPantallaValido = false;
bool pantallaInicializada = false;
bool busPantallaIniciado = false;
bool i2cEsclavoIniciado = false;
bool sistemaBaseListo = false;
unsigned long proximoIntentoOLED = 0;
uint32_t intentosInicioOLED = 0;

//=================================================================================================
// CONTROL BLUETOOTH
//=================================================================================================
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int8_t ejeX = 0;
int8_t ejeY = 0;
int8_t ejeZ = 0;

uint8_t btConectado = 0;
uint8_t botonX = 0;
uint8_t botonTri = 0;
uint8_t dpadRaw = 0;

int anguloS1 = 90;
int anguloS2 = 90;

unsigned long ultimoUpdateServo = 0;
unsigned long ultimoEstadoPortenta = 0;
unsigned long ultimaPantalla = 0;

const unsigned long PERIODO_SERVO_MS = 5;
const unsigned long PERIODO_PANTALLA_MS = 100;
const unsigned long TIMEOUT_ESTADO_PORTENTA_MS = 1000;

// Diagnostico I2C. No se imprime dentro de los callbacks para no bloquear el bus.
volatile uint32_t rxI2CTotal = 0;
volatile uint32_t rxI2COk = 0;
volatile uint32_t rxI2CLongitudIncorrecta = 0;
volatile uint32_t rxI2CMagicIncorrecto = 0;
volatile uint32_t rxI2CChecksumIncorrecto = 0;
volatile int ultimoTamanoRecibido = -1;
unsigned long ultimoReporteI2C = 0;

//=================================================================================================
// UTILIDADES DE PAQUETES
//=================================================================================================
uint8_t calcularChecksum(const uint8_t *datos, size_t longitud) {
    uint8_t resultado = 0;

    for (size_t i = 0; i < longitud; i++) {
        resultado ^= datos[i];
    }

    return resultado;
}

void ponerControlEnNeutro() {
    ejeX = 0;
    ejeY = 0;
    ejeZ = 0;
    botonX = 0;
    botonTri = 0;
    dpadRaw = 0;
    btConectado = 0;
}

//=================================================================================================
// ARRANQUE NO BLOQUEANTE DE LA PANTALLA OLED
//=================================================================================================
// La pantalla NO se inicializa dentro de setup(). De esta manera, aunque el SH1106 tarde en
// encender o el bus quede temporalmente ocupado, Bluepad32 y el resto del ESP32 pueden arrancar.
// El loop intenta inicializar la OLED periódicamente sin detener el sistema.
void intentarInicializarOLEDNoBloqueante() {
    if (pantallaInicializada || !busPantallaIniciado) {
        return;
    }

    const unsigned long ahora = millis();

    // Dar prioridad absoluta al arranque de Bluetooth y del sistema base.
    if (!sistemaBaseListo || ahora < 1500UL) {
        return;
    }

    if ((long)(ahora - proximoIntentoOLED) < 0) {
        return;
    }

    proximoIntentoOLED = ahora + 1000UL;
    intentosInicioOLED++;

    // Primero verificar ACK. El timeout del bus evita que un SH1106 lento bloquee el programa.
    I2C_Pantalla.beginTransmission(OLED_DIRECCION);
    const uint8_t error = I2C_Pantalla.endTransmission(true);

    if (error != 0) {
        Serial.print(F("[OLED] Intento "));
        Serial.print(intentosInicioOLED);
        Serial.print(F(": sin ACK, codigo "));
        Serial.println(error);
        return;
    }

    // Solo se llama begin() cuando el controlador ya respondio.
    if (!pantalla.begin(OLED_DIRECCION, true)) {
        Serial.print(F("[OLED] Intento "));
        Serial.print(intentosInicioOLED);
        Serial.println(F(": fallo pantalla.begin()"));
        return;
    }

    pantalla.clearDisplay();
    pantalla.setTextSize(1);
    pantalla.setTextColor(SH110X_WHITE);
    pantalla.setCursor(0, 0);
    pantalla.println(F("ESP32 INICIADO"));
    pantalla.println(F("OLED CONECTADA"));
    pantalla.display();

    pantallaInicializada = true;
    ultimaPantalla = 0;

    Serial.println(F("[OLED] Inicializada correctamente sin bloquear el arranque"));
}

//=================================================================================================
// CALLBACKS I2C DEL ESP32 ESCLAVO
//=================================================================================================
void requestEvent() {
    PaqueteControl paquete;

    paquete.ejeX = (ejeX == 0) ? 0 : (ejeX > 0 ? 1 : 2);
    paquete.ejeY = (ejeY == 0) ? 0 : (ejeY > 0 ? 1 : 2);
    paquete.ejeZ = (ejeZ == 0) ? 0 : (ejeZ > 0 ? 1 : 2);
    paquete.btConectado = btConectado;
    paquete.botonX = botonX;
    paquete.botonTri = botonTri;
    paquete.anguloServo1 = (uint8_t)anguloS1;
    paquete.anguloServo2 = (uint8_t)anguloS2;

    Wire.write((const uint8_t *)&paquete, sizeof(paquete));
}

void receiveEvent(int cantidadBytes) {
    // Un sondeo I2C de direccion puede generar una recepcion de 0 bytes.
    // Se ignora porque no es un paquete de pantalla.
    if (cantidadBytes <= 0) {
        while (Wire.available()) Wire.read();
        return;
    }

    rxI2CTotal++;
    ultimoTamanoRecibido = cantidadBytes;

    uint8_t buffer[sizeof(PaquetePantalla)];
    size_t recibidos = 0;

    while (Wire.available() && recibidos < sizeof(buffer)) {
        buffer[recibidos++] = (uint8_t)Wire.read();
    }

    // Vaciar cualquier byte sobrante para dejar limpio el siguiente paquete.
    while (Wire.available()) {
        Wire.read();
    }

    if (recibidos != sizeof(PaquetePantalla)) {
        rxI2CLongitudIncorrecta++;
        return;
    }

    PaquetePantalla temporal;
    memcpy(&temporal, buffer, sizeof(temporal));

    if (
        temporal.magic != MAGIC_PANTALLA ||
        temporal.version != VERSION_PANTALLA
    ) {
        rxI2CMagicIncorrecto++;
        return;
    }

    const uint8_t checksumCalculado = calcularChecksum(
        buffer,
        sizeof(PaquetePantalla) - 1
    );

    if (checksumCalculado != temporal.checksum) {
        rxI2CChecksumIncorrecto++;
        return;
    }

    portENTER_CRITICAL(&estadoMux);
    estadoPantalla = temporal;
    estadoPantallaNuevo = true;
    portEXIT_CRITICAL(&estadoMux);

    rxI2COk++;
}

//=================================================================================================
// CALLBACKS BLUEPAD32
//=================================================================================================
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
        }
    }

    ponerControlEnNeutro();
}

void processControllers() {
    bool hayControlConectado = false;

    for (auto ctl : myControllers) {
        if (ctl == nullptr || !ctl->isConnected()) {
            continue;
        }

        hayControlConectado = true;

        if (!ctl->hasData()) {
            continue;
        }

        // Joystick izquierdo: movimiento X/Y.
        ejeX = (abs(ctl->axisX()) > 100)
            ? (ctl->axisX() > 0 ? -1 : 1)
            : 0;

        ejeY = (abs(ctl->axisY()) > 100)
            ? (ctl->axisY() > 0 ? -1 : 1)
            : 0;

        // Joystick derecho vertical: movimiento Z.
        int rY = ctl->axisRY();
        int rX = ctl->axisRX();

        ejeZ = (abs(rY) > 100)
            ? (rY > 0 ? 1 : -1)
            : 0;

        // Botones usados por el menu de la Portenta.
        botonX = ctl->a() ? 1 : 0;
        botonTri = ctl->y() ? 1 : 0;

        // D-Pad usado para el servo de la pinza.
        dpadRaw = ctl->dpad();

        // Servos actualizados independientemente de la pantalla.
        if (millis() - ultimoUpdateServo >= PERIODO_SERVO_MS) {
            ultimoUpdateServo = millis();

            if (rX > 150) {
                anguloS1 = min(180, anguloS1 + 1);
            }
            else if (rX < -150) {
                anguloS1 = max(0, anguloS1 - 1);
            }

            if (dpadRaw & 0x08) {
                anguloS2 = min(180, anguloS2 + 1);
            }

            if (dpadRaw & 0x04) {
                anguloS2 = max(0, anguloS2 - 1);
            }

            servo25.write(anguloS1);
            servo26.write(anguloS2);
        }
    }

    btConectado = hayControlConectado ? 1 : 0;

    if (!hayControlConectado) {
        ponerControlEnNeutro();
    }
}

//=================================================================================================
// TEXTO DE CALIBRACION PARA LA PANTALLA
//=================================================================================================
const char *nombreFaseRemota(uint8_t fase) {
    switch (fase) {
        case CAL_ESPERA_R: return "PRESIONA X";
        case CAL_X_MIN_1_R: return "BUSCANDO X-";
        case CAL_X_MIN_LIBERAR_R: return "LIBERANDO X-";
        case CAL_X_MIN_SEPARAR_R: return "SEPARANDO X-";
        case CAL_X_MIN_2_R: return "VERIFICANDO X-";
        case CAL_X_MAX_1_R: return "BUSCANDO X+";
        case CAL_X_MAX_LIBERAR_R: return "LIBERANDO X+";
        case CAL_X_MAX_SEPARAR_R: return "SEPARANDO X+";
        case CAL_X_MAX_2_R: return "VERIFICANDO X+";
        case CAL_Y_MIN_1_R: return "BUSCANDO Y-";
        case CAL_Y_MIN_LIBERAR_R: return "LIBERANDO Y-";
        case CAL_Y_MIN_SEPARAR_R: return "SEPARANDO Y-";
        case CAL_Y_MIN_2_R: return "VERIFICANDO Y-";
        case CAL_Y_MAX_1_R: return "BUSCANDO Y+";
        case CAL_Y_MAX_LIBERAR_R: return "LIBERANDO Y+";
        case CAL_Y_MAX_SEPARAR_R: return "SEPARANDO Y+";
        case CAL_Y_MAX_2_R: return "VERIFICANDO Y+";
        case CAL_HOME_R: return "YENDO A HOME";
        case CAL_COMPLETA_R: return "CALIBRACION OK";
        case CAL_ERROR_R: return "ERROR";
        default: return "FASE DESCONOCIDA";
    }
}

const char *textoErrorCal(uint8_t error) {
    switch (error) {
        case 1: return "TIMEOUT";
        case 2: return "RANGO X INVALIDO";
        case 3: return "RANGO Y INVALIDO";
        case 4: return "ERROR DE ESCALA";
        default: return "ERROR GENERAL";
    }
}

//=================================================================================================
// ACTUALIZACION DE PANTALLA EN EL ESP32
//=================================================================================================
void mostrarSinPortenta() {
    if (!pantallaInicializada) {
        return;
    }

    pantalla.clearDisplay();
    pantalla.setCursor(0, 0);
    pantalla.println(F("ESP32 LISTO"));
    pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);
    pantalla.setCursor(0, 20);
    pantalla.println(F("ESPERANDO DATOS"));
    pantalla.println(F("DE LA PORTENTA"));
    pantalla.setCursor(0, 52);
    pantalla.print(F("BT: "));
    pantalla.println(btConectado ? F("CONECTADO") : F("DESCONECTADO"));
    pantalla.display();
}

void actualizarPantallaESP32(const PaquetePantalla &p) {
    if (!pantallaInicializada) {
        return;
    }

    const bool calibrado = (p.flags & 0x01) != 0;
    const bool movimientoXYZ = (p.flags & 0x02) != 0;
    const bool escalaValida = (p.flags & 0x04) != 0;

    const bool limiteXmas = (p.limites & 0x01) != 0;
    const bool limiteXmenos = (p.limites & 0x02) != 0;
    const bool limiteYmas = (p.limites & 0x04) != 0;
    const bool limiteYmenos = (p.limites & 0x08) != 0;

    pantalla.clearDisplay();
    pantalla.setCursor(0, 0);

    if (p.estado == MENU_PRINCIPAL_R) {
        pantalla.println(F("--- MENU PRINCIPAL ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        pantalla.setCursor(4, 15);
        pantalla.print(p.opcionMenu == 0 ? "-> " : "   ");
        pantalla.println(F("MODO MANUAL"));

        pantalla.setCursor(4, 29);
        pantalla.print(p.opcionMenu == 1 ? "-> " : "   ");
        pantalla.println(F("CALIBRACION"));

        pantalla.setCursor(4, 43);
        pantalla.print(p.opcionMenu == 2 ? "-> " : "   ");
        pantalla.println(F("POSICION XYZ"));

        pantalla.setCursor(0, 56);
        pantalla.print(calibrado ? F("XY CALIBRADO") : F("XY SIN CALIBRAR"));
    }

    else if (p.estado == MODO_MANUAL_R) {
        pantalla.println(F("CONTROL MANUAL"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        pantalla.setCursor(0, 14);
        pantalla.print(F("X:"));
        pantalla.print(p.movX == 0 ? " 0" : (p.movX > 0 ? "+1" : "-1"));
        pantalla.print(F(" Y:"));
        pantalla.print(p.movY == 0 ? " 0" : (p.movY > 0 ? "+1" : "-1"));
        pantalla.print(F(" Z:"));
        pantalla.println(p.movZ == 0 ? " 0" : (p.movZ > 0 ? "+1" : "-1"));

        pantalla.setCursor(0, 28);
        pantalla.print(F("X+:"));
        pantalla.print(limiteXmas ? "ON " : "OFF");
        pantalla.print(F(" X-:"));
        pantalla.println(limiteXmenos ? "ON" : "OFF");

        pantalla.setCursor(0, 42);
        pantalla.print(F("Y+:"));
        pantalla.print(limiteYmas ? "ON " : "OFF");
        pantalla.print(F(" Y-:"));
        pantalla.println(limiteYmenos ? "ON" : "OFF");

        pantalla.setCursor(0, 56);
        pantalla.print(F("S1:"));
        pantalla.print(p.servoRot);
        pantalla.print(F(" S2:"));
        pantalla.print(p.servoPin);
    }

    else if (p.estado == MODO_HOMING_R) {
        pantalla.println(F("--- CALIBRACION ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        if (p.faseCal == CAL_ESPERA_R) {
            pantalla.setCursor(0, 20);
            pantalla.println(F("PRESIONA X"));
            pantalla.println(F("PARA INICIAR"));
            pantalla.println(F("TRI: SALIR"));
        }
        else if (p.faseCal == CAL_COMPLETA_R) {
            pantalla.setCursor(0, 18);
            pantalla.println(F("CALIBRACION OK"));
            pantalla.println(F("HOME X=0 Y=0"));
            pantalla.println(F("TRI: MENU"));
        }
        else if (p.faseCal == CAL_ERROR_R) {
            pantalla.setCursor(0, 18);
            pantalla.println(F("ERROR CALIBRACION"));
            pantalla.println(textoErrorCal(p.errorCal));
            pantalla.println(F("TRI: MENU"));
        }
        else {
            pantalla.setCursor(0, 16);
            pantalla.println(nombreFaseRemota(p.faseCal));

            pantalla.setCursor(0, 32);
            pantalla.print(F("X:"));
            pantalla.print(p.valor1);
            pantalla.print(F(" Y:"));
            pantalla.println(p.valor2);

            pantalla.setCursor(0, 46);
            pantalla.print(F("RX:"));
            pantalla.print(p.valor3);
            pantalla.print(F(" RY:"));
            pantalla.println(p.valor4);

            pantalla.setCursor(0, 57);
            pantalla.print(F("TRI: CANCELAR"));
        }
    }

    else if (p.estado == MODO_CARTESIANO_R) {
        pantalla.println(F("--- POSICION XYZ ---"));
        pantalla.drawLine(0, 10, 128, 10, SH110X_WHITE);

        pantalla.setCursor(0, 15);

        if (escalaValida) {
            pantalla.print(F("X:"));
            pantalla.print((float)p.valor1 / 10.0f, 1);
            pantalla.print(F(" Y:"));
            pantalla.println((float)p.valor2 / 10.0f, 1);
        }
        else {
            pantalla.println(F("SIN ESCALA MM"));
        }

        pantalla.setCursor(0, 29);
        pantalla.print(F("TX:"));
        pantalla.print((float)p.valor3 / 10.0f, 1);
        pantalla.print(F(" TY:"));
        pantalla.println((float)p.valor4 / 10.0f, 1);

        pantalla.setCursor(0, 43);
        pantalla.println(movimientoXYZ ? F("ESTADO: MOVIENDO") : F("ESTADO: LISTO"));

        pantalla.setCursor(0, 56);
        pantalla.print(F("SERIAL 115200"));
    }

    pantalla.display();
}

void procesarEstadoPantalla() {
    bool hayNuevoEstado = false;
    PaquetePantalla copia;

    portENTER_CRITICAL(&estadoMux);

    if (estadoPantallaNuevo) {
        copia = estadoPantalla;
        estadoPantallaNuevo = false;
        hayNuevoEstado = true;
    }

    portEXIT_CRITICAL(&estadoMux);

    if (hayNuevoEstado) {
        estadoPantallaValido = true;
        ultimoEstadoPortenta = millis();

        // Guardar la copia validada para refrescos posteriores.
        portENTER_CRITICAL(&estadoMux);
        estadoPantalla = copia;
        portEXIT_CRITICAL(&estadoMux);
    }

    if (millis() - ultimaPantalla < PERIODO_PANTALLA_MS) {
        return;
    }

    ultimaPantalla = millis();

    if (
        !estadoPantallaValido ||
        millis() - ultimoEstadoPortenta > TIMEOUT_ESTADO_PORTENTA_MS
    ) {
        mostrarSinPortenta();
        return;
    }

    portENTER_CRITICAL(&estadoMux);
    copia = estadoPantalla;
    portEXIT_CRITICAL(&estadoMux);

    actualizarPantallaESP32(copia);
}

//=================================================================================================
// SETUP
//=================================================================================================
void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println(F("[BOOT] Iniciando ESP32..."));

    // 1. Iniciar primero Bluetooth. No debe depender de la OLED ni de la Portenta.
    BP32.setup(&onConnectedController, &onDisconnectedController);
    Serial.println(F("[BOOT] Bluepad32 configurado"));

    // 2. Inicializar servos.
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    servo25.setPeriodHertz(50);
    servo26.setPeriodHertz(50);

    servo25.attach(PIN_SERVO_ROTACION, 500, 2400);
    servo26.attach(PIN_SERVO_PINZA, 500, 2400);

    servo25.write(anguloS1);
    servo26.write(anguloS2);
    Serial.println(F("[BOOT] Servos configurados"));

    // 3. Preparar el bus de la OLED, pero NO llamar pantalla.begin() aqui.
    I2C_Pantalla.setBufferSize(128);
    I2C_Pantalla.setTimeOut(25);
    busPantallaIniciado = I2C_Pantalla.begin(
        OLED_SDA,
        OLED_SCL,
        100000
    );

    Serial.print(F("[BOOT] Bus OLED: "));
    Serial.println(busPantallaIniciado ? F("OK") : F("ERROR"));

    // 4. Activar el ESP32 como esclavo I2C AL FINAL. Asi la Portenta no puede ejecutar
    // callbacks mientras Bluepad32 y los perifericos principales todavia se inicializan.
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Wire.setBufferSize(64);

    i2cEsclavoIniciado = Wire.begin(
        (uint8_t)DIRECCION_ESP32,
        I2C_PORTENTA_SDA,
        I2C_PORTENTA_SCL,
        100000
    );

    sistemaBaseListo = true;

    Serial.print(F("[BOOT] I2C esclavo Portenta: "));
    Serial.println(i2cEsclavoIniciado ? F("OK") : F("ERROR"));
    Serial.println(F("  SDA=27, SCL=14, direccion=0x40, 100 kHz"));
    Serial.println(F("[BOOT] Sistema base listo. La OLED se iniciara desde loop()."));
}

//=================================================================================================
// LOOP
//=================================================================================================
void loop() {
    // Bluetooth siempre tiene prioridad, incluso si la OLED no responde.
    BP32.update();
    processControllers();

    // La OLED se recupera automaticamente sin bloquear setup().
    intentarInicializarOLEDNoBloqueante();

    procesarEstadoPantalla();

    // Reporte de diagnostico fuera del callback I2C.
    if (millis() - ultimoReporteI2C >= 1000) {
        ultimoReporteI2C = millis();

        Serial.print(F("[I2C RX] total="));
        Serial.print(rxI2CTotal);
        Serial.print(F(" ok="));
        Serial.print(rxI2COk);
        Serial.print(F(" tam="));
        Serial.print(ultimoTamanoRecibido);
        Serial.print(F(" errLen="));
        Serial.print(rxI2CLongitudIncorrecta);
        Serial.print(F(" errMagic="));
        Serial.print(rxI2CMagicIncorrecto);
        Serial.print(F(" errCRC="));
        Serial.println(rxI2CChecksumIncorrecto);
    }

    // Cede tiempo al sistema Bluetooth y a las tareas internas del ESP32.
    delay(1);
}

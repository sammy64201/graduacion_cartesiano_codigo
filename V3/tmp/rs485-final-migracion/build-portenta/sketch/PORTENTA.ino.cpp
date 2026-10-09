#include <Arduino.h>
#line 1 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
// Banco sin motores, salidas, encoder ni calibraciones de arranque.
#include <Arduino_MachineControl.h>
#include "hal/trng_api.h"
#include "FinalRS485.h"
using namespace machinecontrol;
using namespace FinalRS485;
EnlaceRS485::Cliente cliente;
EnlaceRS485::Receptor receptor;
uint32_t intentos = 0, ok = 0, fallos = 0, txError = 0, timeouts = 0;
uint32_t ajenas = 0, erroresDatos = 0, rttMax = 0, rttTotal = 0;
uint32_t inicioFase = 0, ultimoEnvio = 0, inicioCaso = 0, ultimoResumen = 0;
uint32_t baseOK = 0, baseAjena = 0, baseRxError = 0, baseDatos = 0, baseTX = 0;
uint32_t baseTimeout = 0;
uint32_t maximaSeparacion = 0, inicioRun = 0;
uint8_t fase = 0;
size_t paso = 0;
bool activo = false, casoActivo = false, terminado = false, sesionESPConocida = false;
uint16_t sesionESP = 0;

#line 20 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
bool enviarBytes(const uint8_t *trama, size_t n);
#line 29 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
void enviarCaso(uint32_t numero, Caso c);
#line 55 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
void comenzarFase(uint8_t nueva);
#line 63 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
void cerrarCarga();
#line 81 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
void setup();
#line 97 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
void loop();
#line 20 "C:\\Users\\samue\\OneDrive\\Documents\\Universidad\\Tesis\\Github\\graduacion_cartesiano_codigo\\V3\\tests\\RS485_FINAL_MIGRACION\\PORTENTA\\PORTENTA.ino"
bool enviarBytes(const uint8_t *trama, size_t n) {
    comm_protocols.rs485.noReceive();
    comm_protocols.rs485.beginTransmission();
    const size_t escritos = comm_protocols.rs485.write(trama, n);
    comm_protocols.rs485.endTransmission();
    comm_protocols.rs485.receive();
    if (escritos != n) ++txError;
    return escritos == n;
}
void enviarCaso(uint32_t numero, Caso c) {
    // Una sola solicitud pendiente, igual que el firmware completo.
    const uint32_t inicioTX = millis();
    if (!cliente.puedeIniciar(inicioTX)) return;
    cliente.solicitud = numero - 1;
    cliente.iniciar(inicioTX);
    EnlaceRS485::Mensaje m = {};
    m.sesion = cliente.sesion; m.solicitud = cliente.solicitud;
    const auto p = solicitud(numero);
    memcpy(m.datos, &p, sizeof(p));
    uint8_t trama[EnlaceRS485::TRAMA];
    if (c == RUIDO) {
        memset(trama, 0x55, sizeof(trama));
        enviarBytes(trama, sizeof(trama));
        const uint8_t cero = 0; enviarBytes(&cero, 1);
    }
    const size_t n = crearTrama(m, c, trama);
    enviarBytes(trama, n);
    cliente.enviadaMs = millis(); // Timeout desde el ultimo bit de TX.
    if (intentos) {
        const uint32_t separacion = inicioTX - ultimoEnvio;
        if (separacion > maximaSeparacion) maximaSeparacion = separacion;
    }
    ultimoEnvio = inicioTX;
    ++intentos;
}
void comenzarFase(uint8_t nueva) {
    fase = nueva; intentos = ok = timeouts = ajenas = erroresDatos = 0;
    rttMax = rttTotal = maximaSeparacion = 0;
    inicioFase = ultimoEnvio = millis();
    baseRxError = receptor.crcIncorrecto + receptor.longitudIncorrecta + receptor.fragmentos;
    baseTX = txError;
    Serial.print("[FINAL PORTENTA] FASE="); Serial.println(fase);
}
void cerrarCarga() {
    const uint32_t duracion = millis() - inicioFase;
    const uint32_t total = fase == 0 ? LENTOS : RAPIDOS;
    const uint32_t limite = fase == 0 ? LIMITE_LENTOS_MS : LIMITE_RAPIDOS_MS;
    const bool aprobado = ok == total && timeouts == 0 && ajenas == 0 &&
        erroresDatos == 0 && txError == baseTX &&
        receptor.crcIncorrecto + receptor.longitudIncorrecta + receptor.fragmentos == baseRxError &&
        duracion <= limite;
    if (!aprobado) ++fallos;
    Serial.print("[FINAL PORTENTA] CARGA "); Serial.print(aprobado ? "PASS" : "FAIL");
    Serial.print(" enviados="); Serial.print(intentos); Serial.print(" ok="); Serial.print(ok);
    Serial.print(" timeout="); Serial.print(timeouts); Serial.print(" ajenas="); Serial.print(ajenas);
    Serial.print(" datos="); Serial.print(erroresDatos); Serial.print(" duracion_ms="); Serial.print(duracion);
    Serial.print(" rttMax_ms="); Serial.print(rttMax);
    Serial.print(" rttMedio_us="); Serial.print(ok ? 1000UL * rttTotal / ok : 0);
    Serial.print(" separacionMax_ms="); Serial.println(maximaSeparacion);
    comenzarFase(fase + 1);
}
void setup() {
    Serial.begin(115200);
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
    Serial.print("[FINAL PORTENTA] v17 bus");
    Serial.print(EnlaceRS485::BAUD); Serial.println("; SIN ACTUADORES; enviar T para iniciar");
    Serial.println("[FINAL PORTENTA] Cargar AMBOS sketches de RS485_FINAL_MIGRACION");
}
void loop() {
    if (!activo) {
        if (Serial.available() && Serial.read() == 'T') {
            trng_t rng; trng_init(&rng); size_t cantidad = 0;
            const int error = trng_get_bytes(&rng, reinterpret_cast<uint8_t *>(&cliente.sesion),
                sizeof(cliente.sesion), &cantidad); trng_free(&rng);
            if (error || cantidad != sizeof(cliente.sesion) || !cliente.sesion) {
                Serial.println("[FINAL PORTENTA] FAIL sesion aleatoria; enviar T para reintentar");
                return;
            }
            activo = true; inicioRun = millis(); comenzarFase(0);
        }
        return;
    }
    if (terminado) return;
    EnlaceRS485::Mensaje m;
    while (comm_protocols.rs485.available()) {
        const uint8_t dato = static_cast<uint8_t>(comm_protocols.rs485.read());
        const uint32_t recibidoMs = millis();
        cliente.registrarRecepcion(recibidoMs);
        if (!receptor.agregar(dato, recibidoMs, m)) continue;
        if (!cliente.coincide(m, millis())) { ++ajenas; continue; }
        PaqueteESPAPortenta p; memcpy(&p, m.datos, sizeof(p));
        if (!validarPaquete(p) || !p.sesionArranque) { ++erroresDatos; continue; }
        if (!sesionESPConocida) { sesionESP = p.sesionArranque; sesionESPConocida = true; }
        const auto esperado = respuesta(m.solicitud, sesionESP);
        if (memcmp(&p, &esperado, sizeof(p))) { ++erroresDatos; continue; }
        const uint32_t rtt = millis() - cliente.enviadaMs;
        rttTotal += rtt; if (rtt > rttMax) rttMax = rtt;
        ++ok; cliente.confirmar();
    }
    receptor.vencerFragmento(millis());
    if (cliente.vencer(millis())) ++timeouts;
    const uint32_t ahora = millis();
    if (fase < 2) {
        const uint32_t total = fase == 0 ? LENTOS : RAPIDOS;
        const uint32_t periodo = fase == 0 ? PERIODO_LENTOS_MS : PERIODO_RAPIDOS_MS;
        if (!cliente.pendiente && intentos == total) cerrarCarga();
        else if (!cliente.pendiente && ahora - ultimoEnvio >= periodo)
            enviarCaso((fase == 0 ? 0 : LENTOS) + intentos + 1, NORMAL);
    } else if (!casoActivo && cliente.puedeIniciar(ahora)) {
        if (paso == NUM_CASOS) {
            terminado = true;
            Serial.print("[FINAL PORTENTA] RESULTADO="); Serial.print(fallos ? "FAIL" : "PASS");
            Serial.print(" fallos="); Serial.print(fallos);
            Serial.print(" duracion_ms="); Serial.println(ahora - inicioRun);
            Serial.println("[FINAL PORTENTA] Solo enlace; pendiente validar firmware completo bajo carga");
        } else {
            baseOK = ok; baseAjena = ajenas; baseDatos = erroresDatos; baseTimeout = timeouts;
            baseTX = txError; inicioCaso = ahora;
            if (CASOS[paso] == NUEVA_SESION) { cliente.sesion ^= 0x40000000UL; if (!cliente.sesion) cliente.sesion = 1; }
            enviarCaso(numero(paso), CASOS[paso]); casoActivo = true;
        }
    } else if (casoActivo && ahora - inicioCaso >= VENTANA_FALLO_MS && !cliente.pendiente) {
        const bool responder = esperaRespuesta(CASOS[paso]);
        const uint32_t ajenasEsperadas = CASOS[paso] == AJENO || CASOS[paso] == TARDIO ? 1 : 0;
        // Tras vencer un fragmento, el receptor descarta hasta el delimitador.
        // Enviar este cero antes del siguiente paquete completo de recuperacion.
        if (CASOS[paso] == TRUNCADO) { const uint8_t cero = 0; enviarBytes(&cero, 1); }
        const bool aprobado = ok - baseOK == (responder ? 1U : 0U) &&
            timeouts - baseTimeout == (responder ? 0U : 1U) &&
            ajenas - baseAjena == ajenasEsperadas && erroresDatos == baseDatos && txError == baseTX &&
            receptor.crcIncorrecto + receptor.longitudIncorrecta + receptor.fragmentos == baseRxError;
        if (!aprobado) ++fallos;
        Serial.print("[FINAL PORTENTA] caso="); Serial.print(paso);
        Serial.print(" tipo="); Serial.print(static_cast<unsigned>(CASOS[paso]));
        Serial.print(" "); Serial.println(aprobado ? "PASS" : "FAIL");
        ++paso; casoActivo = false;
    }
    if (ahora - ultimoResumen >= 1000) {
        ultimoResumen = ahora;
        Serial.print("[FINAL PORTENTA] fase="); Serial.print(fase);
        Serial.print(" enviados="); Serial.print(intentos); Serial.print(" ok="); Serial.print(ok);
        Serial.print(" timeout="); Serial.print(timeouts); Serial.print(" ajenas="); Serial.print(ajenas);
        Serial.print(" datos="); Serial.print(erroresDatos); Serial.print(" txError="); Serial.print(txError);
        Serial.print(" crc="); Serial.print(receptor.crcIncorrecto);
        Serial.print(" len="); Serial.println(receptor.longitudIncorrecta);
    }
}


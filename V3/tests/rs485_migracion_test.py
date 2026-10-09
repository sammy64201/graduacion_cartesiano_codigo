"""Regresiones de la copia RS485: transporte real, guardas y modos heredados."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tests'))
from recuperacion_enlace_test import run_recovery

NEW = ROOT / 'automatico v2 rs485'
OLD = ROOT / 'pruebas de automatico v2'
BUILD = ROOT / 'tmp/rs485-migracion/tests'

def definition(source, marker):
    found = re.search(re.escape(marker) + ('' if marker.endswith('{') else r'[^;{]*\{'), source)
    if not found:
        raise ValueError('Definicion ausente: ' + marker)
    start, opening = found.start(), found.end() - 1
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

def compile_run(compiler, source, name):
    cpp = BUILD / (name + '.cpp')
    cpp.write_text(source, encoding='utf-8')
    exe = BUILD / (name + '.exe')
    subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-static', str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    compiler = str(Path(args.compiler).resolve())
    compiler_bin = Path(compiler).parent
    os.environ['PATH'] = str(compiler_bin) + os.pathsep + str(compiler_bin / 'cpp') + os.pathsep + os.environ.get('PATH', '')
    BUILD.mkdir(parents=True, exist_ok=True)
    for name in ['ProtocoloRS485.h', 'EnlaceRS485.h']:
        assert (NEW / 'ESP' / name).read_bytes() == (NEW / 'PORTENTA' / name).read_bytes(), name
    esp = (NEW / 'ESP/ESP.ino').read_text(encoding='utf-8-sig')
    portenta = (NEW / 'PORTENTA/PORTENTA.ino').read_text(encoding='utf-8-sig')
    assert not re.search(r'\bWire\.\w+\s*\(', esp)
    # La migracion solo reemplaza el enlace ESP-Portenta: el expansor DIN
    # interno de Machine Control conserva Wire. No permitir transporte I2C
    # de paquetes, pero exigir su inicializacion antes de leer las entradas.
    assert re.findall(r'\bWire\.(\w+)\s*\(', portenta) == ['begin', 'setClock']
    setup = definition(portenta, 'void setup()')
    assert setup.index('Wire.begin();') < setup.index('Wire.setClock(100000);') < setup.index('digital_inputs.init();')
    assert 'HardwareSerial PuertoRS485(2)' in esp
    config = (NEW / 'ESP/ConfiguracionRS485.h').read_text(encoding='utf-8')
    for declaration in ['RX = 14', 'TX = 27', 'DE = 18']:
        assert declaration in config, declaration
    for declaration in ['RS485_RX = ConfiguracionRS485::RX', 'RS485_TX = ConfiguracionRS485::TX',
                        'RS485_DIRECCION = ConfiguracionRS485::DE',
                        'OLED_SDA = 21', 'OLED_SCL = 22', 'OLED_I2C_HZ = 100000',
                        'HUSKY_RX_PIN = 32', 'HUSKY_TX_PIN = 33', 'HUSKY_UART_NUMBER = 1',
                        'PIN_SERVO_ROTACION = 25', 'PIN_SERVO_PINZA = 26']:
        assert declaration in esp, declaration
    assert 'TwoWire I2C_Pantalla(1)' in esp
    assert 'constexpr bool RS485_DIRECCION_MANUAL = true;' in esp
    # Desde 2026-10-09 RS485 es el firmware vigente. La copia I2C conservada
    # no debe obligar a duplicar nuevas mejoras del transporte o del ciclo.
    assert (OLD / 'ESP/ProtocoloI2C.h').read_bytes() == (OLD / 'PORTENTA/ProtocoloI2C.h').read_bytes()

    # Ejecutar setup() real con la dependencia que quedo fuera de la primera
    # migracion. El expansor interno no puede usarse antes de Wire.begin().
    arranque = '#include <cassert>\n#include <cstdio>\n#include <stdint.h>\n#define F(x) x\n'
    arranque += '#include "' + (NEW / 'PORTENTA/EnlaceRS485.h').as_posix() + '"\n'
    arranque += '#include "' + (NEW / 'PORTENTA/CapturaFijaV2.h').as_posix() + '"\n'
    arranque += '''
struct Terminal {
  bool iniciado=false;
  void begin(unsigned baud) { assert(baud==115200); iniciado=true; }
  template<class T> void print(T, int=0) {}
  template<class T> void println(T) {}
} Serial;
struct BusInterno {
  bool listo=false; uint32_t hz=0;
  void begin() { listo=true; }
  void setClock(uint32_t valor) { assert(listo); hz=valor; }
} Wire;
unsigned lecturasDIN=0;
struct Entradas {
  void init() { assert(Wire.listo && Wire.hz==100000); ++lecturasDIN; }
} digital_inputs;
struct Encoder { void reset() {} } encoders[1];
struct Salidas { void set(int, int) {} } digital_outputs;
struct Ticker { void attach(void (*)(), float) {} } motorTicker;
void generarPulsoMotor() {}
constexpr int LOW=0, CAM_CMD_NINGUNO=0;
constexpr int pP_X=4, pP_Y=2, pP_Z=0, pD_X=5, pD_Y=3, pD_Z=1;
constexpr float velocidadMotores=0.0001f;
constexpr float CAMARA_A_HOME_Y_MM=845, V2_AJUSTE_DISTANCIA_CATCH_MM=335;
float escalaEncoderMmPorCuenta=0.075f;
int8_t signoEncoderAvance=1;
CapturaFijaV2::Estimador estimadorCapturaFijaV2;
uint32_t tiempoEncendidoSistema=0, inicioEstadoGeneral=0, tAnteriorRS485=0, tAnteriorEstadoESP=0;
bool comunicacionRS485Habilitada=true;
uint8_t comandoCamaraActual=99, secuenciaComandoCamara=99;
uint32_t millis() { return 1234; }
void iniciarRS485Maestro() { assert(Serial.iniciado); }
void detenerTodos() {}
void leerFinalesCarrera() { assert(Wire.listo); ++lecturasDIN; }
void mostrarAyudaTerminal() {}
'''
    arranque += setup + '\n'
    arranque += '''
int main() {
  setup();
  assert(Serial.iniciado && Wire.listo && lecturasDIN==2);
  assert(!comunicacionRS485Habilitada && comandoCamaraActual==CAM_CMD_NINGUNO);
  assert(tiempoEncendidoSistema==1234 && tAnteriorRS485==1234);
  std::puts("PASS: setup real: USB serial y Wire interno antes de DIN/finales, arranque seguro RS485");
}
'''
    compile_run(compiler, arranque, 'arranque')
    compile_run(compiler, (ROOT / 'tests/rs485_transporte_test.cpp').read_text().replace(
        '../automatico v2 rs485/', NEW.as_posix() + '/'), 'transporte')

    # Ejecutar la inicializacion real de ambas placas y la respuesta ESP.
    # El mock distingue el overload begin(baud, pre, post) de UART config,
    # conserva microsegundos y exige flush + 200 us antes de liberar DE.
    perfil = '#include <cassert>\n#include <cstdio>\n#define F(x) x\n'
    perfil += '#include "' + (ROOT / 'tests/rs485_final_mocks/Arduino_MachineControl.h').as_posix() + '"\n'
    for name in ['ProtocoloRS485.h', 'EnlaceRS485.h', 'ConfiguracionRS485.h']:
        perfil += '#include "' + (NEW / 'ESP' / name).as_posix() + '"\n'
    perfil += '''
using namespace ProtocoloRS485;
uint32_t ahoraSimulado=1000, fraccionMicrosegundos=0;
bool deESP=false, flushESP=false;
uint64_t inicioTxESP=0, finTxESP=0;
std::deque<uint8_t> entradaESP, entradaPortenta;
namespace machinecontrol { Comunicacion comm_protocols; }
using namespace machinecontrol;
Monitor Serial;
EnlaceRS485::Cliente clienteRS485;
EnlaceRS485::Receptor receptorRS485;
'''
    perfil += definition(portenta, 'bool iniciarRS485Maestro()') + '\n'
    perfil += '''
namespace Esclavo {
HardwareSerial PuertoRS485(2);
EnlaceRS485::Receptor receptorRS485;
EnlaceRS485::Mensaje solicitudRS485;
PaqueteESPAPortenta paqueteTxSnapshot;
bool rs485EsclavoIniciado=false, respuestaRS485Pendiente=false;
uint32_t ultimoIntentoRS485=0, inicioInstanciaRS485=0, ultimaActividadRS485=0;
uint32_t reiniciosRS485Esclavo=0, ultimaPublicacionRS485=0;
uint32_t solicitudesLecturaRS485=0, txRS485Ok=0, txRS485Error=0;
constexpr uint32_t PERIODO_PUBLICACION_RS485_MS=10;
int txRS485Mux=0;
void portENTER_CRITICAL(int *) {} void portEXIT_CRITICAL(int *) {}
void prepararSnapshotRS485() { prepararPaquete(paqueteTxSnapshot); }
'''
    for name in ['RS485_RX', 'RS485_TX', 'RS485_DIRECCION', 'RS485_DIRECCION_MANUAL']:
        perfil += re.search(r'constexpr [^;\n]+\b' + name + r'\s*=\s*[^;]+;', esp).group() + '\n'
    perfil += definition(esp, 'bool iniciarRS485Esclavo(') + '\n'
    perfil += definition(esp, 'void responderPortentaRS485()') + '\n}\n'
    perfil += '''
void comprobarMaestro() {
  const auto &c=comm_protocols;
  assert(c.iniciada && c.habilitada && !c.rs232 && !c.fullDuplex && !c.terminacion);
  assert(c.rs485.baud==115200 && c.rs485.config==SERIAL_8N1);
  assert(c.rs485.pre==0 && c.rs485.post==2000 && c.rs485.rx && !c.rs485.tx);
  assert(clienteRS485.sesion && !clienteRS485.pendiente);
}
int main() {
  assert(iniciarRS485Maestro()); comprobarMaestro();
  const uint32_t sesion=clienteRS485.sesion;
  clienteRS485.iniciar(millis());
  comm_protocols.rs485ABTerm(true); // Una recuperacion debe restablecer el perfil.
  assert(iniciarRS485Maestro()); comprobarMaestro();
  assert(clienteRS485.sesion==sesion);
  assert(Esclavo::iniciarRS485Esclavo(false));
  assert(Esclavo::PuertoRS485.baud==115200 && !deESP);
  assert(Esclavo::iniciarRS485Esclavo(true));
  assert(Esclavo::reiniciosRS485Esclavo==1 && !deESP);
  Esclavo::solicitudRS485.sesion=sesion; Esclavo::solicitudRS485.solicitud=42;
  const uint64_t inicio=tiempoMicrosegundos();
  Esclavo::respuestaRS485Pendiente=true; Esclavo::responderPortentaRS485();
  assert(inicioTxESP-inicio==15000 && !deESP);
  assert(!Esclavo::respuestaRS485Pendiente && Esclavo::txRS485Ok==1);
  EnlaceRS485::Receptor r; EnlaceRS485::Mensaje m; bool recibido=false;
  for(auto b:entradaPortenta) recibido |= r.agregar(b,millis(),m);
  assert(recibido && m.sesion==sesion && m.solicitud==42);
  PaqueteESPAPortenta p; memcpy(&p,m.datos,sizeof(p));
  assert(validarPaquete(p));
  std::puts("PASS: perfil real 115200, inicio/recuperacion, Portenta 0/2000 us, ESP DE 200/flush/200 us y respuesta 15 ms");
}
'''
    compile_run(compiler, perfil, 'perfil')

    # RX y guardas reales: tolerancia estricta >1000 ms, CRC/correlacion,
    # snapshot congelado, errores y cancelacion conservando referencias.
    run_recovery(compiler, NEW, 'RS485', BUILD)

    # Ejecutar el envio real: una orden urgente durante una solicitud pendiente
    # debe volver a construirse con el estado nuevo, no con un paquete cacheado.
    envio = '#include <cassert>\n#include <cstdio>\n#include <vector>\n#include <deque>\n#include <cstring>\n'
    for name in ['ProtocoloRS485.h', 'EnlaceRS485.h']:
        envio += '#include "' + (NEW / 'PORTENTA' / name).as_posix() + '"\n'
    envio += '''
using namespace ProtocoloRS485;
uint32_t now=1000;
unsigned long millis() { return now; }
bool deESPTodaviaActivo=false;
struct UARTSimulada {
  bool rx=true, tx=false, corto=false;
  unsigned transmisiones=0, deshabilitacionesRX=0;
  std::vector<uint8_t> bytes;
  std::deque<uint8_t> entrada;
  int available() { return rx ? entrada.size() : 0; }
  int read() { auto b=entrada.front(); entrada.pop_front(); return b; }
  void noReceive() { assert(!deESPTodaviaActivo); ++deshabilitacionesRX; rx=false; }
  void beginTransmission() { assert(!rx && !deESPTodaviaActivo); ++transmisiones; tx=true; }
  size_t write(const uint8_t *p, size_t n) {
    assert(tx && !rx);
    if(corto) return 0;
    bytes.assign(p,p+n); return n;
  }
  void endTransmission() { tx=false; }
  void receive() { rx=true; }
};
struct Comunicaciones { UARTSimulada rs485; } comm_protocols;
EnlaceRS485::Cliente clienteRS485;
EnlaceRS485::Receptor receptorRS485;
bool envioRS485Urgente=false;
uint32_t enviosRS485Ok=0, enviosRS485Error=0;
uint32_t respuestasRS485Ajenas=0, lecturasRS485Error=0, erroresRS485Semantica=0;
uint32_t ultimaLatenciaRS485Ms=0, maximaLatenciaRS485Ms=0, lecturasRS485Ok=0, timeoutsRS485=0;
uint8_t fallosPaqueteConsecutivos=0;
unsigned paquetesValidos=0;
uint8_t ultimoCodigoErrorEnvioRS485=0, ackActual=ACK_OBJ_NINGUNO;
bool paqueteSemanticamenteValido(const PaqueteESPAPortenta &) { return true; }
void registrarPaqueteValido(const PaqueteESPAPortenta &) { ++paquetesValidos; }
void construirPaquetePortenta(PaquetePortentaAESP &p) {
  p.codigoAckObjetivo=ackActual; p.ackSecuenciaObjetivo=42; prepararPaquete(p);
}
'''
    envio += definition(portenta, 'bool leerPaqueteESP32()') + '\n'
    envio += definition(portenta, 'bool enviarPaquetePortenta()')
    envio += '''
void verificar(uint8_t ack) {
  EnlaceRS485::Receptor r; EnlaceRS485::Mensaje m; bool recibido=false;
  for(auto b:comm_protocols.rs485.bytes) recibido |= r.agregar(b,now,m);
  assert(recibido && m.sesion==clienteRS485.sesion && m.solicitud==clienteRS485.solicitud);
  PaquetePortentaAESP p; memcpy(&p,m.datos,sizeof(p));
  assert(validarPaquete(p) && p.codigoAckObjetivo==ack && p.ackSecuenciaObjetivo==42);
}
int main() {
  assert(!enviarPaquetePortenta()); // Sesion no inicializada: no transmitir.
  clienteRS485.sesion=123;
  assert(enviarPaquetePortenta()); verificar(ACK_OBJ_NINGUNO);
  auto anterior=comm_protocols.rs485.bytes;
  ackActual=ACK_OBJ_CERRAR_PINZA;
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  assert(comm_protocols.rs485.bytes==anterior && enviosRS485Ok==1);

  // Reproducir la carrera que el test maestro con delay(100) no ejercitaba:
  // el cero final ya llego, pero ESP conserva DE hasta flush + POST_TX_US.
  // Se ejecuta la funcion de envio real, tambien para la orden urgente.
  now+=24; deESPTodaviaActivo=true;
  PaqueteESPAPortenta respuesta={}; prepararPaquete(respuesta);
  EnlaceRS485::Mensaje mensaje={};
  mensaje.sesion=clienteRS485.sesion; mensaje.solicitud=clienteRS485.solicitud;
  memcpy(mensaje.datos,&respuesta,sizeof(respuesta));
  uint8_t trama[EnlaceRS485::TRAMA]; EnlaceRS485::codificar(mensaje,trama);
  comm_protocols.rs485.entrada.insert(comm_protocols.rs485.entrada.end(),trama,trama+sizeof(trama));
  assert(leerPaqueteESP32() && paquetesValidos==1 && !clienteRS485.pendiente);
  const uint32_t solicitudAnterior=clienteRS485.solicitud;
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  assert(comm_protocols.rs485.bytes==anterior && enviosRS485Ok==1 && enviosRS485Error==0);
  now+=2; deESPTodaviaActivo=false;
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  assert(clienteRS485.solicitud==solicitudAnterior && !clienteRS485.pendiente);
  assert(comm_protocols.rs485.transmisiones==1 && comm_protocols.rs485.deshabilitacionesRX==1);
  assert(comm_protocols.rs485.rx && !comm_protocols.rs485.tx);
  ++now;
  assert(enviarPaquetePortenta()); verificar(ACK_OBJ_CERRAR_PINZA);
  assert(!envioRS485Urgente && comm_protocols.rs485.rx && !comm_protocols.rs485.tx);
  clienteRS485.confirmar(); comm_protocols.rs485.corto=true;
  assert(!enviarPaquetePortenta() && !clienteRS485.pendiente && enviosRS485Error==1);
  assert(comm_protocols.rs485.rx && !comm_protocols.rs485.tx);
  comm_protocols.rs485.corto=false; ackActual=ACK_OBJ_ABRIR_PINZA;
  assert(enviarPaquetePortenta()); verificar(ACK_OBJ_ABRIR_PINZA);

  // RX parcial no valida el enlace y tampoco prolonga el plazo de 100 ms.
  // Si coincide con el timeout, el siguiente intento sigue esperando silencio.
  now+=99; comm_protocols.rs485.entrada.push_back(0x55);
  assert(!leerPaqueteESP32() && clienteRS485.pendiente && paquetesValidos==1);
  ++now; assert(!leerPaqueteESP32() && timeoutsRS485==1 && !clienteRS485.pendiente);
  assert(!enviarPaquetePortenta() && envioRS485Urgente);
  ++now; assert(!enviarPaquetePortenta());
  ++now; assert(enviarPaquetePortenta()); verificar(ACK_OBJ_ABRIR_PINZA);
  assert(paquetesValidos==1 && lecturasRS485Ok==1);
  std::puts("PASS: RX/envio reales, DE sin solapamiento, urgente diferida 3 ms, ruido sin renovar plazo y TX corto recuperable");
}
'''
    compile_run(compiler, envio, 'envio')

    # Reusar los escenarios existentes sobre los fuentes NUEVOS sin copiar ni
    # modificar el firmware anterior. Las capturas historicas siguen en OLD.
    for name in ['rs485_captura_fija_integracion_test.py', 'auto_v2_giro_test.py',
                 'mlv2_catch_test.py', 'mlv2_rectas_test.py', 'vision_modelo129_test.py']:
        original = ROOT / 'tests' / name
        code = original.read_text(encoding='utf-8-sig')
        code = code.replace("ROOT = Path(__file__).resolve().parents[1]", 'ROOT = Path(' + repr(str(ROOT)) + ')')
        code = code.replace("folder = ROOT / 'pruebas de automatico v2'", "folder = ROOT / 'automatico v2 rs485'")
        code = code.replace('pruebas de automatico v2/ESP/', 'automatico v2 rs485/ESP/')
        code = code.replace("capture = folder / 'registros_v2/", "capture = (ROOT / 'pruebas de automatico v2') / 'registros_v2/")
        code = code.replace("log = (folder / 'registros_v2/", "log = ((ROOT / 'pruebas de automatico v2') / 'registros_v2/")
        code = code.replace("(folder / 'registros_v2/", "((ROOT / 'pruebas de automatico v2') / 'registros_v2/")
        code = code.replace('I2C', 'RS485')
        code = code.replace('CRC version 18', 'CRC version 19')
        code = code.replace("protocol = protocol.replace('../ESP/ProtocoloRS485.h',", "protocol = protocol.replace('I2C', 'RS485')\n    protocol = protocol.replace('../ESP/ProtocoloRS485.h',")
        script = BUILD / name
        script.write_text(code, encoding='utf-8')
        subprocess.run([sys.executable, str(script), '--compiler', compiler], check=True)
    print('PASS: migracion RS485, OLED I2C, pines y cinco suites de regresion sobre los nuevos sketches')

if __name__ == '__main__':
    main()

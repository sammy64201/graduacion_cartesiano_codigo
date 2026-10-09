"""Ejecuta RX y recuperacion reales: tolerancia de 1 s y referencias conservadas."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def definition(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def recovery_source(folder, transport):
    """El harness usa el sketch elegido; no replica sus decisiones de enlace."""
    source = (folder / 'PORTENTA/PORTENTA.ino').read_text(encoding='utf-8-sig')
    protocol = 'Protocolo' + transport
    cpp = '#include <cassert>\n#include <cstdio>\n#include <cstdint>\n#include <cstring>\n#include <deque>\n#define F(x) x\n'
    cpp += '#include "' + (folder / ('PORTENTA/' + protocol + '.h')).as_posix() + '"\n'
    cpp += 'using namespace ' + protocol + ';\n'
    if transport == 'RS485':
        cpp += '#include "' + (folder / 'PORTENTA/EnlaceRS485.h').as_posix() + '"\n'
    # Parametros compartidos que el sketch utiliza directamente.
    for header in re.findall(r'#include "([^"]+)"', source):
        if 'Recuperacion' in header or 'Politica' in header:
            cpp += '#include "' + (folder / 'PORTENTA' / header).as_posix() + '"\n'
    cpp += definition(source, 'enum EstadoGeneral :') + ';\n'
    for name in ['TIMEOUT_' + transport + '_MS', 'TIMEOUT_SECUENCIA_ESP_MS',
                 'MAX_PAQUETES_INVALIDOS_CONSECUTIVOS']:
        found = re.search(r'(?:const|constexpr) [^;\n]+\b' + name + r'\s*=\s*[^;]+;', source)
        if found:
            cpp += found.group() + '\n'
    cpp += r'''
struct Terminal {
  template<class T> void print(T) {}
  template<class T> void println(T) {}
} Serial;
uint32_t now=1000;
uint32_t millis() { return now; }
EstadoGeneral estadoGeneral=EST_AUTOMATICO_V2, modoPendiente=EST_AUTOMATICO_V2;
bool existePaqueteValido=true, protocoloValido=true, sesionESPConocida=true;
bool secuenciaPaqueteESPConocida=true, cambioSesionESPPendiente=false;
bool movimientoPosicionadoActivo=true, calibracionXYValida=true, calibracionZValida=true;
bool moviendo=true;
uint32_t ultimoPaqueteValidoMs=1000, ultimoCambioSecuenciaESPMs=1000;
uint8_t fallosPaqueteConsecutivos=0, ultimaSecuenciaPaqueteESP=7;
uint16_t sesionArranqueESP=1, secuenciaObjetivoEnMovimiento=42, ackSecuenciaObjetivo=0;
uint8_t codigoAckObjetivo=ACK_OBJ_NINGUNO;
long rangoXPasos=28400, rangoYPasos=16200, rangoZPasos=7300;
float pasosPorMmX=100, pasosPorMmY=99.5f, escalaEncoderMmPorCuenta=0.075f;
unsigned paros=0, cancelacionesPosicionado=0;
bool motoresEnMovimiento() { return moviendo; }
void detenerTodos() { ++paros; moviendo=false; movimientoPosicionadoActivo=false; }
void cambiarEstadoGeneral(EstadoGeneral e) { estadoGeneral=e; }
void cancelarMovimientoPosicionado(const char *, bool invalidar) {
  ++cancelacionesPosicionado; detenerTodos();
  if(invalidar) calibracionXYValida=calibracionZValida=false;
}
PaqueteESPAPortenta paqueteESP={};
int8_t joystickX=0, joystickY=0, joystickZ=0;
bool btConectado=false;
uint8_t posServoRot=90, posServoPin=90;
bool botonX=false, botonCirculo=false, botonTriangulo=false, botonCuadrado=false;
bool botonXAnterior=false, botonCirculoAnterior=false, botonTrianguloAnterior=false, botonCuadradoAnterior=false;
bool eventoBotonX=false, eventoBotonCirculo=false, eventoBotonTriangulo=false, eventoBotonCuadrado=false;
enum { V2_EVALUANDO_CATCH, ML_ESPERANDO_CONFIRMACION };
struct { int fase=V2_EVALUANDO_CATCH; } automaticoV2;
struct { int fase=ML_ESPERANDO_CONFIRMACION; } entrenamientoML;
bool ajusteCatchV2Seleccionado() { return false; }
bool entrenamientoConResultadoSeleccionado() { return false; }
void registrarEventoPortentaV2(const char *, const char *) {}
uint8_t estadoCamara=0, errorCamara=0, flagsCamara=0, muestrasTag[4]={}, claseObjetivo=0;
int16_t objetivoCamaraX10=0, objetivoCamaraY10=0;
uint16_t secuenciaObjetivoRecibida=0;
int32_t conteoReferenciaObjetivoRecibido=0;
'''
    if transport == 'RS485':
        cpp += r'''
struct UART {
  std::deque<uint8_t> entrada;
  int available() { return entrada.size(); }
  int read() { const uint8_t b=entrada.front(); entrada.pop_front(); return b; }
} ;
struct { UART rs485; } comm_protocols;
EnlaceRS485::Cliente clienteRS485;
EnlaceRS485::Receptor receptorRS485;
uint32_t respuestasRS485Ajenas=0, lecturasRS485Error=0, erroresRS485Semantica=0;
uint32_t ultimaLatenciaRS485Ms=0, maximaLatenciaRS485Ms=0, lecturasRS485Ok=0, timeoutsRS485=0;
'''
    else:
        cpp += r'''
struct I2C {
  std::deque<uint8_t> entrada;
  int requestFrom(uint8_t, uint8_t) { return entrada.size(); }
  int available() { return entrada.size(); }
  int read() { const uint8_t b=entrada.front(); entrada.pop_front(); return b; }
} Wire;
uint32_t lecturasI2CError=0, erroresI2CLongitud=0, erroresI2CCRC=0, erroresI2CSemantica=0, lecturasI2COk=0;
'''
    for marker in ['bool paqueteSemanticamenteValido(', 'void registrarPaqueteValido(',
                   'bool leerPaqueteESP32()', 'bool enlace' + transport + 'Vigente()',
                   'void volverAEspera' + transport + '(', 'void vigilarSeguridadComunicacion()']:
        cpp += definition(source, marker) + '\n'
    if transport == 'RS485':
        cpp += r'''
void transmitir(PaqueteESPAPortenta p, unsigned rechazo=0) {
  clienteRS485.pendiente=true; clienteRS485.sesion=123;
  ++clienteRS485.solicitud; clienteRS485.enviadaMs=now-20;
  if(rechazo==4) clienteRS485.enviadaMs=now-EnlaceRS485::TIMEOUT_RESPUESTA_MS;
  EnlaceRS485::Mensaje m={}; m.sesion=clienteRS485.sesion; m.solicitud=clienteRS485.solicitud;
  if(rechazo==2) ++m.sesion;
  if(rechazo==3) ++m.solicitud;
  memcpy(m.datos,&p,sizeof(p));
  uint8_t trama[EnlaceRS485::TRAMA]; EnlaceRS485::codificar(m,trama);
  if(rechazo==5) { comm_protocols.rs485.entrada.push_back(0x55); return; }
  if(rechazo==6) trama[8]^=1; // CRC16/COBS corrupto, sin tocar la carga valida.
  comm_protocols.rs485.entrada.insert(comm_protocols.rs485.entrada.end(),trama,trama+sizeof(trama));
}
void resetTransporte() { clienteRS485={}; receptorRS485={}; comm_protocols.rs485.entrada.clear(); }
'''
    else:
        cpp += r'''
void transmitir(PaqueteESPAPortenta p, unsigned rechazo=0) {
  const auto b=reinterpret_cast<const uint8_t*>(&p);
  Wire.entrada.insert(Wire.entrada.end(),b,b+(rechazo==5 ? 7 : sizeof(p)));
}
void resetTransporte() { Wire.entrada.clear(); }
'''
    cpp += r'''
void preparar(uint32_t inicio=1000) {
  resetTransporte(); now=inicio; ultimoPaqueteValidoMs=ultimoCambioSecuenciaESPMs=inicio;
  existePaqueteValido=protocoloValido=sesionESPConocida=secuenciaPaqueteESPConocida=true;
  cambioSesionESPPendiente=false; fallosPaqueteConsecutivos=0;
  sesionArranqueESP=1; ultimaSecuenciaPaqueteESP=7;
  estadoGeneral=EST_AUTOMATICO_V2; modoPendiente=EST_AUTOMATICO_V2;
  moviendo=movimientoPosicionadoActivo=calibracionXYValida=calibracionZValida=true;
  secuenciaObjetivoEnMovimiento=42; ackSecuenciaObjetivo=0; codigoAckObjetivo=ACK_OBJ_NINGUNO;
  paros=cancelacionesPosicionado=0;
}
PaqueteESPAPortenta respuesta() {
  PaqueteESPAPortenta p={}; p.sesionArranque=1; p.secuenciaPaquete=7;
  prepararPaquete(p); return p;
}
void comprobarReferencias() {
  assert(calibracionXYValida && calibracionZValida);
  assert(rangoXPasos==28400 && rangoYPasos==16200 && rangoZPasos==7300);
  assert(pasosPorMmX==100 && pasosPorMmY==99.5f && escalaEncoderMmPorCuenta==0.075f);
}
void comprobarActivo() {
  assert(estadoGeneral==EST_AUTOMATICO_V2 && moviendo && movimientoPosicionadoActivo && !paros);
  assert(secuenciaObjetivoEnMovimiento==42 && codigoAckObjetivo==ACK_OBJ_NINGUNO);
  comprobarReferencias();
}
void comprobarParo() {
  assert(estadoGeneral==EST_WAIT_TRANSPORTE && !moviendo && !movimientoPosicionadoActivo);
  assert(modoPendiente==EST_MAIN_MENU && paros==1);
  assert(ackSecuenciaObjetivo==42 && codigoAckObjetivo==ACK_OBJ_CANCELADO && !secuenciaObjetivoEnMovimiento);
  comprobarReferencias();
}
int main() {
  // Fronteras solicitadas: el umbral pertenece al sistema, no a cada sondeo.
  for(uint32_t edad : {149U,150U,999U,1000U}) {
    preparar(); now+=edad; vigilarSeguridadComunicacion(); comprobarActivo();
  }
  preparar(); now+=1001; vigilarSeguridadComunicacion(); comprobarParo();
  vigilarSeguridadComunicacion(); comprobarParo(); // Recuperacion no se repite.
  // Recuperar una respuesta conserva el paro y la cancelacion; no inicia pulsos.
  transmitir(respuesta()); assert(leerPaqueteESP32()); vigilarSeguridadComunicacion(); comprobarParo();
  // La secuencia del snapshot puede congelarse con respuestas correctas frescas.
  preparar();
  for(uint32_t edad : {149U,150U,999U,1000U,1001U,5000U}) {
    now=1000+edad; transmitir(respuesta()); assert(leerPaqueteESP32());
    assert(ultimoPaqueteValidoMs==now && ultimoCambioSecuenciaESPMs==1000);
    vigilarSeguridadComunicacion(); comprobarActivo();
  }
  // Ninguna rafaga de errores anticipa la perdida si existe respuesta reciente.
  for(uint32_t edad : {149U,150U,999U,1000U}) {
    preparar(); now+=edad; fallosPaqueteConsecutivos=255;
    vigilarSeguridadComunicacion(); comprobarActivo();
  }
  preparar(); fallosPaqueteConsecutivos=255; now+=1001;
  vigilarSeguridadComunicacion(); comprobarParo();
  // Una respuesta a los 999 ms renueva la vida durante otros 1000 ms completos.
  preparar(); now+=999; transmitir(respuesta()); assert(leerPaqueteESP32());
  now+=1000; vigilarSeguridadComunicacion(); comprobarActivo();
  ++now; vigilarSeguridadComunicacion(); comprobarParo();
  // CRC de aplicacion, semantica y longitud/ruido no renuevan ultimo valido.
  for(unsigned fallo : {0U,1U,5U}) {
    preparar(); now+=999; auto p=respuesta();
    if(fallo==0) p.checksum^=1;
    if(fallo==1) { p.joystickX=2; prepararPaquete(p); }
    transmitir(p,fallo); assert(!leerPaqueteESP32()); assert(ultimoPaqueteValidoMs==1000);
    vigilarSeguridadComunicacion(); comprobarActivo();
    now=2001; vigilarSeguridadComunicacion(); comprobarParo();
  }
  // La resta uint32_t mantiene la frontera incluso cuando millis() desborda.
  preparar(UINT32_MAX-500); now+=1000; vigilarSeguridadComunicacion(); comprobarActivo();
  ++now; vigilarSeguridadComunicacion(); comprobarParo();
  // Un cambio real de sesion sigue cancelando: la CPU ESP si arranco de nuevo.
  // Esa ruta separada tambien conserva referencias y no reanuda el movimiento.
  preparar(); now+=20; auto nuevaSesion=respuesta(); nuevaSesion.sesionArranque=2;
  prepararPaquete(nuevaSesion); transmitir(nuevaSesion); assert(leerPaqueteESP32());
  vigilarSeguridadComunicacion();
  assert(estadoGeneral==EST_WAIT_TRANSPORTE && !moviendo && !movimientoPosicionadoActivo);
  assert(cancelacionesPosicionado==1 && modoPendiente==EST_MAIN_MENU);
  comprobarReferencias();
  transmitir(nuevaSesion); assert(leerPaqueteESP32()); vigilarSeguridadComunicacion();
  assert(estadoGeneral==EST_WAIT_TRANSPORTE && !moviendo && !movimientoPosicionadoActivo);
  comprobarReferencias();
'''
    if transport == 'RS485':
        cpp += r'''
  for(unsigned fallo : {2U,3U,4U,6U}) {
    preparar(); now+=999; transmitir(respuesta(),fallo);
    assert(!leerPaqueteESP32() && ultimoPaqueteValidoMs==1000);
    vigilarSeguridadComunicacion(); comprobarActivo();
    now=2001; vigilarSeguridadComunicacion(); comprobarParo();
  }
  // 100 ms vence solo el intercambio: deja sondear sin recuperar el sistema.
  preparar(); clienteRS485.sesion=123; assert(clienteRS485.iniciar(now));
  now+=100; assert(!leerPaqueteESP32() && !clienteRS485.pendiente);
  assert(timeoutsRS485>0); vigilarSeguridadComunicacion(); comprobarActivo();
  assert(clienteRS485.iniciar(now));
'''
    cpp += '  std::puts("PASS: ' + transport + ': RX real, umbral >1000 ms, secuencia/invalidos sin recuperacion temprana, referencias y cancelacion persistentes");\n}\n'
    return cpp.replace('EST_WAIT_TRANSPORTE', 'EST_WAIT_' + transport)


def run_recovery(compiler, folder, transport, build):
    path = build / ('recuperacion_' + transport.lower() + '.cpp')
    path.write_text(recovery_source(folder, transport), encoding='utf-8')
    exe = path.with_suffix('.exe')
    subprocess.run([str(compiler), '-std=c++11', '-Wall', '-Wextra', '-static',
                    str(path), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', required=True)
    args = parser.parse_args()
    compiler = Path(args.compiler).resolve()
    os.environ['PATH'] = str(compiler.parent) + os.pathsep + str(compiler.parent / 'cpp') + os.pathsep + os.environ.get('PATH', '')
    build = Path(tempfile.mkdtemp(prefix='recuperacion-enlace-', dir=ROOT / 'tmp'))
    for name, transport in [('pruebas de automatico v2', 'I2C'), ('automatico v2 rs485', 'RS485')]:
        run_recovery(compiler, ROOT / name, transport, build)


if __name__ == '__main__':
    main()

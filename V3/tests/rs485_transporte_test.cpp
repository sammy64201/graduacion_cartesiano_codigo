#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "../automatico v2 rs485/ESP/EnlaceRS485.h"
#include "../automatico v2 rs485/ESP/ProtocoloRS485.h"

using namespace EnlaceRS485;
static bool iguales(const Mensaje &a, const Mensaje &b) {
    return a.sesion == b.sesion && a.solicitud == b.solicitud &&
           std::memcmp(a.datos, b.datos, CARGA) == 0;
}
static bool recibir(Receptor &r, const uint8_t *p, size_t n, Mensaje &m, uint32_t t=100) {
    bool recibido = false;
    for (size_t i=0; i<n; ++i) recibido |= r.agregar(p[i], t, m);
    return recibido;
}
int main() {
    static_assert(sizeof(ProtocoloRS485::PaqueteESPAPortenta) == 32, "ESP 32 bytes");
    static_assert(sizeof(ProtocoloRS485::PaquetePortentaAESP) == 32, "P 32 bytes");
    static_assert(ProtocoloRS485::VERSION_PROTOCOLO == 19, "Calidad del objetivo en captura fija");
    // Perfil comprobado por el usuario: el monitor USB no demuestra la
    // velocidad del bus ni los tiempos del controlador RS485 de Portenta.
    static_assert(BAUD == 115200, "Bus comprobado a 115200");
    static_assert(PRE_TX_US == 200 && POST_TX_US == 200, "DE manual del ESP: 200 us antes y despues");
    static_assert(PORTENTA_PRE_TX_US == 0 && PORTENTA_POST_TX_US == 2000,
                  "Portenta conserva begin(115200, 0, 2000)");
    static_assert(GIRO_BUS_US == 15000 && GIRO_BUS_US > PORTENTA_POST_TX_US,
                  "Respuesta despues de liberar el transmisor Portenta");
    static_assert(GIRO_SOLICITUD_MS == 3 && GIRO_SOLICITUD_MS * 1000 > POST_TX_US,
                  "Portenta espera silencio antes de volver a tomar el bus");
    static_assert(TIEMPO_TRAMA_MS == 4, "Trama de 44 bytes 8N1 a 115200");
    static_assert(!PORTENTA_TERMINACION, "Terminacion como el montaje probado por el usuario");
    assert(crc16(reinterpret_cast<const uint8_t*>("123456789"),9) == 0x29B1);
    uint32_t rng=123456;
    for (unsigned caso=0; caso<1000; ++caso) {
        Mensaje original = {};
        original.sesion = 0x12345678;
        original.solicitud = caso+1;
        for (size_t i=0;i<CARGA;++i) {
            rng = rng*1664525+1013904223;
            original.datos[i] = caso==0 ? 0 : caso==1 ? 255 : rng>>24;
        }
        uint8_t trama[TRAMA];
        assert(codificar(original,trama) == TRAMA);
        for (size_t i=0;i<TRAMA-1;++i) assert(trama[i]!=0);
        assert(trama[TRAMA-1]==0);
        Receptor r; Mensaje resultado;
        assert(recibir(r,trama,TRAMA,resultado));
        assert(iguales(original,resultado));
        // Un error de un bit en cualquier byte de COBS/datos/CRC no se acepta.
        if (caso<10) for (size_t i=0;i<TRAMA-1;++i) for(unsigned bit=0;bit<8;++bit) {
            uint8_t mala[TRAMA]; std::memcpy(mala,trama,TRAMA); mala[i]^=1U<<bit;
            Receptor corrupto; Mensaje m;
            assert(!recibir(corrupto,mala,TRAMA,m));
            assert(recibir(corrupto,trama,TRAMA,m) && iguales(original,m));
        }
    }
    Mensaje original={}; original.sesion=23; original.solicitud=42;
    uint8_t trama[TRAMA]; codificar(original,trama);
    for(size_t omitido=0;omitido<TRAMA-1;++omitido) {
        Receptor r; Mensaje m;
        for(size_t i=0;i<TRAMA;++i) if(i!=omitido) assert(!r.agregar(trama[i],100,m));
        assert(recibir(r,trama,TRAMA,m) && iguales(original,m));
    }
    Receptor r; Mensaje m;
    for(unsigned i=0;i<500;++i) assert(!r.agregar(0x55,100,m));
    assert(!r.agregar(0,100,m));
    assert(recibir(r,trama,TRAMA,m));
    recibir(r,trama,10,m,200);
    const uint32_t finFragmento = 200 + TIMEOUT_FRAGMENTO_MS;
    r.vencerFragmento(finFragmento);
    assert(r.fragmentos==1);
    assert(!recibir(r,trama+10,TRAMA-10,m,finFragmento+1));
    assert(recibir(r,trama,TRAMA,m,finFragmento+2));

    Cliente cliente; cliente.sesion=23;
    assert(cliente.iniciar(100));
    assert(!cliente.iniciar(101)); // No superponer transmisiones.
    m.sesion=23; m.solicitud=cliente.solicitud;
    const uint32_t finRespuesta = 100 + TIMEOUT_RESPUESTA_MS;
    assert(cliente.coincide(m,finRespuesta-1));
    assert(!cliente.coincide(m,finRespuesta));
    ++m.sesion; assert(!cliente.coincide(m,110)); --m.sesion;
    ++m.solicitud; assert(!cliente.coincide(m,110)); --m.solicitud;
    assert(cliente.vencer(finRespuesta)); assert(!cliente.vencer(finRespuesta+1));
    assert(!cliente.coincide(m,141));
    assert(cliente.iniciar(142)); assert(!cliente.coincide(m,143));
    m.solicitud=cliente.solicitud; assert(cliente.coincide(m,143));
    cliente.confirmar(); assert(!cliente.coincide(m,144));
    cliente.solicitud=0xFFFFFFFF;
    assert(cliente.iniciar(0xFFFFFFF0)); assert(cliente.solicitud==1);
    m.solicitud=1; assert(cliente.coincide(m,9));
    const uint32_t finWrap = 0xFFFFFFF0U + TIMEOUT_RESPUESTA_MS;
    assert(!cliente.coincide(m,finWrap)); assert(cliente.vencer(finWrap));

    // El ultimo byte puede estar disponible mientras ESP todavia mantiene DE.
    // Confirmar la respuesta no permite transmitir inmediatamente otra orden.
    Cliente giro; giro.sesion=23;
    assert(giro.iniciar(0)); // Sin actividad RX conocida, el arranque es inmediato.
    giro.registrarRecepcion(0); giro.confirmar();
    const uint32_t solicitudAntes=giro.solicitud;
    assert(!giro.iniciar(0) && !giro.iniciar(2));
    assert(!giro.pendiente && giro.solicitud==solicitudAntes);
    assert(giro.iniciar(3)); giro.confirmar();

    // Todo byte, incluso ruido o una respuesta ajena, exige un nuevo silencio.
    // Esos bytes no renuevan el plazo de la respuesta ni la seguridad.
    Cliente ruido; ruido.sesion=23;
    assert(ruido.iniciar(1000));
    ruido.registrarRecepcion(1099);
    assert(!ruido.vencer(1099) && ruido.vencer(1100));
    assert(!ruido.iniciar(1100) && !ruido.iniciar(1101));
    ruido.registrarRecepcion(1101);
    assert(!ruido.iniciar(1102) && !ruido.iniciar(1103));
    assert(ruido.iniciar(1104));

    Cliente giroWrap; giroWrap.sesion=23;
    giroWrap.registrarRecepcion(0xFFFFFFFEU);
    assert(!giroWrap.iniciar(0xFFFFFFFEU) && !giroWrap.iniciar(0));
    assert(giroWrap.iniciar(1));

    Servidor servidor;
    assert(servidor.aceptar(original)==NUEVA);
    assert(servidor.aceptar(original)==DUPLICADA);
    Mensaje antigua=original; --antigua.solicitud;
    assert(servidor.aceptar(antigua)==DESCARTADA);
    antigua=original; antigua.datos[0]^=1;
    assert(servidor.aceptar(antigua)==DESCARTADA);
    ++original.solicitud; assert(servidor.aceptar(original)==NUEVA);
    original.sesion=24; original.solicitud=1; assert(servidor.aceptar(original)==NUEVA);
    original.sesion=0; assert(servidor.aceptar(original)==DESCARTADA);
    Servidor wrap; original.sesion=24; original.solicitud=0xFFFFFFF0;
    assert(wrap.aceptar(original)==NUEVA);
    original.solicitud=1; assert(wrap.aceptar(original)==NUEVA);

    ProtocoloRS485::PaquetePortentaAESP p={};
    ProtocoloRS485::prepararPaquete(p);
    assert(ProtocoloRS485::validarPaquete(p));
    p.version=ProtocoloRS485::VERSION_PROTOCOLO-1;
    p.checksum=ProtocoloRS485::calcularChecksumPaquete(p);
    assert(!ProtocoloRS485::validarPaquete(p));
    std::puts("PASS: 1000 roundtrips, 3440 errores de bit, truncado, ruido, fragmentos, correlacion, plazo, giro tras RX/ruido/wrap, duplicados y rechazo de version previa");
}

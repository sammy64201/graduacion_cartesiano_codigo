
#include <cassert>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <string>
#include <sstream>
#include <iostream>
#define F(x) x
class String {
    std::string value;
public:
    String(const char *text) : value(text) {}
    String(std::string text) : value(text) {}
    size_t length() const { return value.length(); }
    void trim() {
        const size_t first=value.find_first_not_of(" \t\r\n");
        if(first==std::string::npos) { value.clear(); return; }
        value=value.substr(first, value.find_last_not_of(" \t\r\n")-first+1);
    }
    void toUpperCase() {
        for(char &c:value) c=static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    bool startsWith(const char *prefix) const { return value.find(prefix)==0; }
    String substring(size_t begin) const { return String(value.substr(begin)); }
    const char *c_str() const { return value.c_str(); }
    bool operator==(const char *other) const { return value==other; }
    bool operator!=(const char *other) const { return value!=other; }
};
struct Terminal {
    std::ostringstream output;
    template<class T> void print(const T &value) { output << value; }
    template<class T> void println(const T &value) { output << value << '\n'; }
    void clear() { output.str(""); output.clear(); }
} Serial;
// Cualquier dependencia nueva de movimientos debe romper la compilacion:
// el comando solo configura la reserva siguiente.
struct Contexto { int32_t desfaseCatchMs; unsigned fase; unsigned secuencia; };
Contexto automaticoV2={99,11,42};
int32_t desfaseCatchConfiguradoMs=0;
bool ensayo=true;
bool capturaFijaV2EnPrueba() { return ensayo; }
int movX=0, movY=0, movZ=-1;
unsigned estadoGeneral=7;
bool closeTo(float actual, float expected) { return std::fabs(actual-expected)<0.00001f; }
#include "C:/Users/samue/OneDrive/Documents/Universidad/Tesis/Github/graduacion_cartesiano_codigo/V3/automatico v2 rs485/PORTENTA/AjusteTemporalCapturaV2.h"
constexpr unsigned long V2_TIEMPO_CIERRE_PINZA_MS = 450UL;
constexpr unsigned long V2_LATENCIA_ORDEN_PINZA_MS = 158UL;
float tiempoContactoNominalFijoV2() {
    const float baseS = (V2_TIEMPO_CIERRE_PINZA_MS + V2_LATENCIA_ORDEN_PINZA_MS) * 0.001f;
    return AjusteTemporalCapturaV2::horizonte(baseS,
        capturaFijaV2EnPrueba() ? automaticoV2.desfaseCatchMs : 0);
}
void procesarComandoTerminal(String comando) {
    comando.trim();
    if (comando.length() == 0) return;

    String mayuscula = comando;
    mayuscula.toUpperCase();
    if (mayuscula == "CATCH" || mayuscula.startsWith("CATCH ")) {
        if (mayuscula != "CATCH") {
            const String datos = comando.substring(6);
            int32_t propuesto = 0;
            if (!AjusteTemporalCapturaV2::analizar(datos.c_str(), propuesto)) {
                Serial.println(F("[CATCH] Usar CATCH <ms enteros de -500 a +500>; +adelanta, -retrasa"));
                return;
            }
            desfaseCatchConfiguradoMs = propuesto;
        }
        Serial.print(F("V2LOG|P|event=CATCH_OFFSET|configured_catch_offset_ms="));
        Serial.print(desfaseCatchConfiguradoMs);
        Serial.print(F("|catch_offset_ms="));
        Serial.print(capturaFijaV2EnPrueba() ? automaticoV2.desfaseCatchMs : 0);
        Serial.print(F("|test_mode=")); Serial.print(capturaFijaV2EnPrueba() ? 1 : 0);
        Serial.println(F("|message=proxima reserva; positivo adelanta; negativo retrasa"));
        Serial.print(F("[CATCH] Proxima pieza: ")); Serial.print(desfaseCatchConfiguradoMs);
        Serial.println(F(" ms (+adelanta/-retrasa). La pieza reservada conserva su ajuste."));
        Serial.println(F("[CATCH] Solo ensayo fijo V2; se restablece al reiniciar. Permanente: V2_DESFASE_CATCH_MS."));
        return;
    }
}

void run(const char *text, int32_t expected) {
    const Contexto before=automaticoV2;
    const float horizonBefore=tiempoContactoNominalFijoV2();
    Serial.clear();
    procesarComandoTerminal(String(text));
    assert(desfaseCatchConfiguradoMs==expected);
    assert(automaticoV2.desfaseCatchMs==before.desfaseCatchMs);
    assert(automaticoV2.fase==before.fase && automaticoV2.secuencia==before.secuencia);
    assert(closeTo(tiempoContactoNominalFijoV2(),horizonBefore));
    assert(movX==0 && movY==0 && movZ==-1 && estadoGeneral==7);
}
void valid(const char *text, int32_t expected) {
    run(text,expected);
    const std::string output=Serial.output.str();
    assert(output.find("event=CATCH_OFFSET")!=std::string::npos);
    assert(output.find("configured_catch_offset_ms="+std::to_string(expected))!=std::string::npos);
    assert(output.find("|catch_offset_ms=99|test_mode=1")!=std::string::npos);
}
int main() {
    valid("CATCH",0);
    valid("CATCH +100",100);
    valid("CATCH",100);
    valid("CATCH -100",-100);
    valid(" catch +100 \r\n",100);
    valid("CaTcH -100",-100);
    valid("CATCH +500",500);
    valid("CATCH -500",-500);
    valid("CATCH 0",0);
    valid("CATCH +100   ",100);
    const char *invalid[]={"CATCH +501", "CATCH -501", "CATCH texto", "CATCH 1.5",
        "CATCH +", "CATCH -", "CATCH --100", "CATCH 100 otra", "CATCH 0x10",
        "CATCH 999999999999999999999999999999", "CATCH -999999999999999999999999999999",
        "CATCH 2147483648", "CATCH -2147483649"};
    for(const char *text:invalid) {
        run(text,100);
        assert(Serial.output.str().find("Usar CATCH")!=std::string::npos);
        assert(Serial.output.str().find("event=CATCH_OFFSET")==std::string::npos);
    }
    run("",100); run(" \t\r\n",100); run("CATCHX +100",100); run("CATCH+100",100);
    int32_t parsed=99;
    assert(!AjusteTemporalCapturaV2::analizar(nullptr,parsed) && parsed==99);
    assert(!AjusteTemporalCapturaV2::analizar("",parsed) && parsed==99);
    assert(!AjusteTemporalCapturaV2::analizar("   ",parsed) && parsed==99);
    assert(AjusteTemporalCapturaV2::analizar(" \t+100\n",parsed) && parsed==100);
    assert(closeTo(AjusteTemporalCapturaV2::horizonte(0.608f,100),0.708f));
    assert(closeTo(AjusteTemporalCapturaV2::horizonte(0.608f,-100),0.508f));
    assert(closeTo(AjusteTemporalCapturaV2::horizonte(0.608f,-500),0.108f));
    const float base=(V2_TIEMPO_CIERRE_PINZA_MS+V2_LATENCIA_ORDEN_PINZA_MS)*0.001f;
    assert(closeTo(tiempoContactoNominalFijoV2(),base+0.099f));
    automaticoV2.desfaseCatchMs=-100;
    assert(closeTo(tiempoContactoNominalFijoV2(),base-0.100f));
    ensayo=false;
    assert(closeTo(tiempoContactoNominalFijoV2(),base));
    run("catch +200",200);
    assert(Serial.output.str().find("|catch_offset_ms=0|test_mode=0")!=std::string::npos);
    std::cout << "RS485 CATCH terminal: comandos, limites, reserva y horizonte verificados\n";
}

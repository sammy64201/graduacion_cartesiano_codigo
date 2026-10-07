from pathlib import Path
root = Path(__file__).resolve().parents[1]
p = root / 'tests/auto_v2_integracion_test.py'
s = p.read_text(encoding='utf-8-sig')
s = s.replace("'V2_SEGUIMIENTO_ESTABLE_MS', 'AJUSTE_CATCH_DISPARO_MANUAL'", "'V2_SEGUIMIENTO_ESTABLE_MS', 'V2_AJUSTE_DISPARO_CATCH_MS'")
s = s.replace("    cpp += '''\n// Alternar", "    cpp += '#include \\\"' + (folder / 'PORTENTA/AjusteCatchV2.h').as_posix() + '\\\"\\n'\n    cpp += '''\nAjusteCatchV2::Sesion ajusteCatchV2;\nuint8_t paginaCambiosCatch=0;\nvoid imprimirCambiosCatchV2() {}\n// Alternar")
s = s.replace("'bool iniciarTrasladoEntrega()', 'void procesarModoAutomaticoV2()'", "'bool iniciarTrasladoEntrega()', 'void llenarResumenCatchV2(', 'void procesarModoAutomaticoV2()'")
start = s.index('// Modo independiente: observa referencia')
end = s.index('// Entrega conserva la reserva', start)
s = s[:start] + '''// Modo independiente: disparo autonomo y adelanto/retardo acotado.
calMode=true; reset(); procesarModoAutomaticoV2(); now+=299;
procesarModoAutomaticoV2(); assert(!zActive);
now+=1; procesarModoAutomaticoV2();
assert(zActive && automaticoV2.disparoCatchMs==1300 && automaticoV2.ajusteProbadoMs==0);
ajusteCatchV2.evaluar(AjusteCatchV2::ANTES,0);
assert(ajusteCatchV2.offsetMs==50 && ajusteCatchV2.ensayos==1);
reset(); procesarModoAutomaticoV2(); now+=300; procesarModoAutomaticoV2(); assert(!zActive);
now+=50; procesarModoAutomaticoV2(); assert(zActive && automaticoV2.ajusteProbadoMs==50);
ajusteCatchV2.evaluar(AjusteCatchV2::DESPUES,50);
assert(ajusteCatchV2.offsetMs==25 && ajusteCatchV2.pasoMs==25);
ajusteCatchV2=AjusteCatchV2::Sesion(-100);
reset(); eventoBotonX=true; procesarModoAutomaticoV2(); assert(!zActive);
now+=199; procesarModoAutomaticoV2(); assert(!zActive);
now+=1; procesarModoAutomaticoV2();
assert(zActive && automaticoV2.referenciaCatchProyectada && automaticoV2.referenciaCatchMs==1300);
assert(static_cast<int32_t>(automaticoV2.disparoCatchMs-automaticoV2.referenciaCatchMs)==-100);
// Perder alineacion reinicia la espera y ninguna tecla omite las guardas.
reset(); procesarModoAutomaticoV2(); now+=199; ySteps=1000;
procesarModoAutomaticoV2(); assert(!zActive && !automaticoV2.inicioEstable);
ySteps=0; procesarModoAutomaticoV2(); now+=199; procesarModoAutomaticoV2(); assert(!zActive);
now+=1; procesarModoAutomaticoV2(); assert(zActive);
ajusteCatchV2=AjusteCatchV2::Sesion();
''' + s[end:]
s = s.replace('  reset(V2_SUBIENDO_FINAL); zSteps=0; eventoBotonX=true;', '  ajusteCatchV2=AjusteCatchV2::Sesion();\n  reset(V2_SUBIENDO_FINAL); zSteps=0; eventoBotonX=true;')
s = s.replace('  assert(muestrasCal==1 && resultados==1);', '''  assert(muestrasCal==1 && resultados==1 && ajusteCatchV2.ensayos==1);
  assert(ajusteCatchV2.offsetMs==(button==0 ? 0 : button==1 ? 50 : -50));
  assert(exitosV2==(button==0 ? 1 : 0));''')
s = s.replace('calMode=false;\n// Ruta fija', '''// Tres agarres al mismo valor confirman; un fallo retira esa confirmacion.
ajusteCatchV2=AjusteCatchV2::Sesion(-100);
for(int n=0;n<3;++n) ajusteCatchV2.evaluar(AjusteCatchV2::AGARRO,-100);
assert(ajusteCatchV2.confirmado() && ajusteCatchV2.offsetMs==-100);
PaquetePortentaAESP resumen={}; llenarResumenCatchV2(resumen);
resumen.estadoSistema=SISTEMA_CAMBIOS_CATCH; prepararPaquete(resumen);
assert(validarPaquete(resumen) && resumen.conteoEncoder==-100 && resumen.velocidadEncoderUmS==-100);
assert(resumen.secuenciaEncoder==3 && resumen.estadoEncoder==13);
ajusteCatchV2.evaluar(AjusteCatchV2::DESPUES,-100); assert(!ajusteCatchV2.confirmado());
ajusteCatchV2=AjusteCatchV2::Sesion(-200);
ajusteCatchV2.evaluar(AjusteCatchV2::DESPUES,-200);
assert(ajusteCatchV2.offsetMs==-200 && ajusteCatchV2.limiteAlcanzado);
ajusteCatchV2=AjusteCatchV2::Sesion(1000);
ajusteCatchV2.evaluar(AjusteCatchV2::ANTES,1000);
assert(ajusteCatchV2.offsetMs==1000 && ajusteCatchV2.limiteAlcanzado);
// Sesion persiste entre ciclos; modo normal conserva el valor configurado en codigo.
reset(); assert(ajusteCatchV2.offsetMs==1000);
calMode=false; reset(); procesarModoAutomaticoV2(); now+=300;
procesarModoAutomaticoV2(); assert(zActive);
// Ruta fija''')
s = s.replace('CRC version 15', 'CRC version 16').replace('ajuste manual independiente y etiquetas', 'ajuste autonomo, aprendizaje acotado y etiquetas')
p.write_text(s, encoding='utf-8')
p = root / 'tests/encoder_logger_test.ps1'
s = p.read_text(encoding='utf-8-sig').replace('|timing_result=CORRECTO|auto_reference_ms', '|timing_result=CORRECTO|tested_offset_ms=-100|next_offset_ms=-100|adjust_step_ms=25|adjust_trials=3|adjust_success_streak=3|adjust_confirmed=1|adjust_limit=0|auto_reference_ms')
s = s.replace("$fila.camera_y_mm -ne '5' -or", "$fila.tested_offset_ms -ne '-100' -or $fila.next_offset_ms -ne '-100' -or\n    $fila.adjust_confirmed -ne '1' -or $fila.adjust_trials -ne '3' -or\n    $fila.camera_y_mm -ne '5' -or")
p.write_text(s, encoding='utf-8')
print('Pruebas adaptadas')

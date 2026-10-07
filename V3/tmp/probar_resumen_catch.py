from pathlib import Path
root = Path(__file__).resolve().parents[1]
folder = root / 'pruebas de automatico v2'
p=root/'tests/auto_v2_integracion_test.py'
s=p.read_text(encoding='utf-8-sig')
s=s.replace("    handshake += definition(esp, 'void procesarHandshakeObjetivo(')", '''    handshake += '#define portENTER_CRITICAL(x) ((void)0)\\n#define portEXIT_CRITICAL(x) ((void)0)\\n'
    handshake += definition(esp, 'struct EstadoEncoderCompartido {') + ';\\nEstadoEncoderCompartido estadoEncoder;\\n'
    handshake += definition(esp, 'void actualizarEncoderDesdePortenta(')
    handshake += definition(esp, 'bool paquetePortentaSemanticamenteValido(')
    handshake += definition(esp, 'void procesarHandshakeObjetivo(')''')
s=s.replace('PaquetePortentaAESP wire={}; wire.codigoAckObjetivo=ACK_OBJ_ABRIR_PINZA;', '''// Resumen: semantica de OLED aceptada, sin convertir el ajuste en encoder.
PaquetePortentaAESP resumen={}; resumen.estadoSistema=SISTEMA_CAMBIOS_CATCH;
resumen.opcionMenu=MENU_CAMBIOS_CATCH; resumen.signoEncoder=1;
resumen.nmPorCuentaEncoder=50; resumen.conteoEncoder=-100;
resumen.velocidadEncoderUmS=-100;
for(int streak=0;streak<4;++streak) {
  resumen.estadoEncoder=(streak<<2)|(streak==3?1:0);
  assert(paquetePortentaSemanticamenteValido(resumen));
  actualizarEncoderDesdePortenta(resumen);
  assert(!estadoEncoder.flags && !estadoEncoder.nmPorCuenta && !estadoEncoder.conteo && estadoEncoder.velocidadMmS==0);
}
resumen.estadoEncoder=1; assert(!paquetePortentaSemanticamenteValido(resumen));
resumen.estadoEncoder=0; resumen.opcionMenu=MENU_DIAGNOSTICO;
assert(!paquetePortentaSemanticamenteValido(resumen));
// Al salir del resumen se vuelven a publicar datos reales.
resumen.estadoSistema=SISTEMA_MENU_PRINCIPAL; resumen.estadoEncoder=ENC_FLAG_HW_LISTO|ENC_FLAG_ESCALA_VALIDA;
resumen.nmPorCuentaEncoder=75000; resumen.conteoEncoder=123; resumen.velocidadEncoderUmS=50000000;
assert(paquetePortentaSemanticamenteValido(resumen)); actualizarEncoderDesdePortenta(resumen);
assert(estadoEncoder.conteo==123 && estadoEncoder.velocidadMmS==50);
PaquetePortentaAESP wire={}; wire.codigoAckObjetivo=ACK_OBJ_ABRIR_PINZA;''')
s=s.replace('wire.version=14;', 'wire.version=15;')
p.write_text(s,encoding='utf-8')
for board in ['ESP','PORTENTA']:
    p=folder/board/'ProtocoloI2C.h'
    s=p.read_text(encoding='utf-8-sig')
    s=s.replace('    int32_t conteoEncoder;\n    int32_t velocidadEncoderUmS;', '''    // SISTEMA_CAMBIOS_CATCH: conteo=proximo offset ms, velocidad=ultimo probado ms,
    // nmPorCuenta=paso ms, secuencia=ensayos, estado=confirmado bit0/limite bit1/
    // agarres bits2..3, fase=pagina. No publicar estos campos como encoder real.
    int32_t conteoEncoder;
    int32_t velocidadEncoderUmS;''')
    p.write_text(s,encoding='utf-8')
print('Prueba de transporte de resumen agregada')

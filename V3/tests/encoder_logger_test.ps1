$ErrorActionPreference = 'Stop'
$ruta = Join-Path $PSScriptRoot '../pruebas de automatico v2/registrar_v2.ps1'
$tokens = $null
$errores = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile(
    $ruta, [ref]$tokens, [ref]$errores)
if ($errores.Count) { throw ($errores | Out-String) }
# Cargar solamente constantes y funciones: nunca abrir puertos ni mover hardware.
foreach ($nodo in $ast.FindAll({ param($n)
    $n -is [System.Management.Automation.Language.AssignmentStatementAst] -and
    $n.Left.Extent.Text -in @('$script:Columnas', '$script:MapaCampos')
}, $false)) { Invoke-Expression $nodo.Extent.Text }
foreach ($nodo in $ast.FindAll({ param($n)
    $n -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
    $n.Name -in @('Analizar-Linea', 'Obtener-Dato', 'Nombre-FaseV2', 'Nombre-AckV2',
        'Describir-Evento', 'Convertir-CampoCsv', 'Escribir-Fila')
}, $false)) { Invoke-Expression $nodo.Extent.Text }

$script:EscritorSesion = New-Object System.IO.StringWriter
$script:TipoSesion = 'v2'
$eventos = @('ENCODER_TEST_START', 'ENCODER_TEST_RUN_START', 'ENCODER_TEST_REFERENCE',
    'ENCODER_TEST_OBSERVATION', 'ENCODER_TEST_SAMPLE', 'ENCODER_TEST_MARK',
    'ENCODER_TEST_RESET', 'ENCODER_TEST_END')
foreach ($evento in $eventos) {
    $linea = "V2LOG|P|mode=ENCODER_TEST|event=$evento|trial=2|mark=1|enc=-100|travel=300|vel=60|predicted_y=-5|observation_encoder=-99|observed_arm_y=-10|predicted_observation_y=-8|camera_encoder_error_y=-2"
    $analizada = Analizar-Linea $linea 'P'
    if (-not $analizada.Structured -or $analizada.Data.event -ne $evento) {
        throw "Evento no reconocido: $evento"
    }
    $envoltura = [pscustomobject]@{
        Utc = [DateTime]::UtcNow; ElapsedMs = 100; Source = 'P'; Line = $linea
    }
    Escribir-Fila $envoltura $analizada
}
$filas = $script:EscritorSesion.ToString() | ConvertFrom-Csv -Header $script:Columnas
if ($filas.Count -ne 8) { throw 'Filas perdidas' }
foreach ($fila in $filas) {
    if ($fila.mode -ne 'ENCODER_TEST' -or $fila.mark_index -ne '1' -or
        $fila.camera_encoder_error_y_mm -ne '-2' -or
        $fila.predicted_y_mm -ne '-5' -or $fila.travel_mm -ne '300' -or
        $fila.description_es -eq '' -or $fila.raw -notmatch 'V2LOG') {
        throw 'Campos de diagnostico perdidos en CSV'
    }
}
foreach ($disparador in @('X', 'ENCODER')) {
    $muestra = Analizar-Linea "ML_SAMPLE|seq=1|trigger=$disparador" 'P'
    $esperado = if ($disparador -eq 'X') { 'MANUAL' } else { 'AUTOMATICO' }
    if ($muestra.Data.catch_type -ne $esperado) { throw 'Regresion ML_SAMPLE' }
}
$script:EscritorSesion.Dispose()
$script:TipoSesion = 'encoder'
$script:EscritorSesion = New-Object System.IO.StringWriter
$script:EscritorSesion.WriteLine('ensayo,contador_pulsos,tiempo_s')
$casos = @(
    'V2LOG|P|mode=ENCODER_TEST|event=ENCODER_TEST_RESET|trial=0|pulses=0|elapsed_ms=0',
    'V2LOG|P|mode=ENCODER_TEST|event=ENCODER_TEST_RUN_START|trial=1|pulses=0|elapsed_ms=0',
    'V2LOG|P|mode=ENCODER_TEST|event=ENCODER_TEST_SAMPLE|trial=1|pulses=2048|elapsed_ms=1000',
    'V2LOG|E|mode=ENCODER_TEST|event=QUERY|pulses=999',
    'V2LOG|P|event=TELEMETRY|enc=-999',
    'V2LOG|H|event=LOGGER_STOP|message=fin',
    'V2LOG|P|mode=ENCODER_TEST|event=ENCODER_TEST_MARK|trial=1|pulses=4096|elapsed_ms=2000',
    'V2LOG|P|mode=ENCODER_TEST|event=ENCODER_TEST_MARK|trial=2|pulses=1024|elapsed_ms=1505',
    'V2LOG|P|mode=ENCODER_TEST|event=ENCODER_TEST_MARK|trial=3|pulses=nan|elapsed_ms=1000'
)
foreach ($linea in $casos) {
    $analizada = Analizar-Linea $linea 'P'
    Escribir-Fila ([pscustomobject]@{Utc=[DateTime]::UtcNow;ElapsedMs=0;
        Source=$analizada.Source;Line=$linea}) $analizada
}
$csv = $script:EscritorSesion.ToString()
$filas = @($csv | ConvertFrom-Csv)
if ($filas.Count -ne 2 -or ($filas.ensayo -join ',') -ne '1,2' -or
    ($filas.contador_pulsos -join ',') -ne '4096,1024' -or
    ($filas.tiempo_s -join ',') -ne '2.000,1.505' -or
    @($filas[0].PSObject.Properties).Count -ne 3) {
    throw 'El CSV encoder no contiene los dos ensayos completos con pulsos y segundos'
}
$script:EscritorSesion.Dispose()
foreach ($fase in @('12','13','14','15','16','17')) {
    if ([string]::IsNullOrWhiteSpace((Nombre-FaseV2 $fase))) {
        throw "Fase nueva de integracion sin nombre: $fase"
    }
}
if ((Nombre-AckV2 '6') -ne 'ABRIR_PINZA') { throw 'Apertura de entrega sin nombre' }
$sugerencia = Analizar-Linea 'V2LOG|E|event=AUTO_ANGLE_SUGGESTION|obj=42|class=6|suggested_rot=155|suggestion_source=MLV2_EJES_20261005|servo_rot_deg=155' 'E'
if ((Obtener-Dato $sugerencia 'suggested_rot') -ne '155' -or
    (Describir-Evento $sugerencia '').Contains('155') -ne $true) {
    throw 'Sugerencia automatica no reconocida por registrador'
}
Write-Output 'PASS: sintaxis, CSV por ensayo, filtrado de diagnostico y compatibilidad V2/ML'

$script:TipoSesion = 'catch_cal'
$script:EscritorSesion = New-Object System.IO.StringWriter
$linea = 'V2LOG|P|mode=CATCH_CAL|event=CAL_SAMPLE|obj=42|phase=17|timing_result=CORRECTO|tested_offset_ms=-100|next_offset_ms=-100|adjust_step_ms=25|adjust_trials=3|adjust_success_streak=3|adjust_confirmed=1|adjust_limit=0|auto_reference_ms=1000|reference_status=OBSERVADA|catch_start_ms=1200|delta_reference_ms=200|z_bottom_ms=1800|catch_command_ms=1800|trigger_encoder=-100|trigger_arm_y=10|trigger_piece_y=11|scale_mm_count=0.075165|camera_x=3|camera_y=5'
$analizada = Analizar-Linea $linea 'P'
Escribir-Fila ([pscustomobject]@{Utc=[DateTime]::UtcNow;ElapsedMs=0;Source='P';Line=$linea}) $analizada
$fila = $script:EscritorSesion.ToString() | ConvertFrom-Csv -Header $script:Columnas
if ($fila.mode -ne 'CATCH_CAL' -or $fila.timing_result -ne 'CORRECTO' -or
    $fila.delta_reference_ms -ne '200' -or $fila.trigger_arm_y_mm -ne '10' -or
    $fila.tested_offset_ms -ne '-100' -or $fila.next_offset_ms -ne '-100' -or
    $fila.adjust_confirmed -ne '1' -or $fila.adjust_trials -ne '3' -or
    $fila.camera_y_mm -ne '5' -or $fila.phase_name_es -ne 'EVALUANDO_CATCH') {
    throw 'El registrador pierde campos del ajuste de catch'
}
$script:EscritorSesion.Dispose()
Write-Output 'PASS: CSV de ajuste independiente, etiqueta, referencia, tiempos, encoder y geometria'

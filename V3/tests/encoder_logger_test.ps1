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
    $n.Name -in @('Analizar-Linea', 'Obtener-Dato', 'Nombre-FaseV2',
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
Write-Output 'PASS: sintaxis, CSV por ensayo, filtrado de diagnostico y compatibilidad V2/ML'
